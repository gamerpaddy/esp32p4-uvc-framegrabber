/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * Wi-Fi bring-up + UART0 console. See wifi_console.h.
 */

#include <string.h>
#include <stdio.h>
#include "wifi_console.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"
#include "esp_console.h"
#include "argtable3/argtable3.h"

static const char *TAG = "wifi_con";

#define JOIN_TIMEOUT_MS   20000
#define JOIN_MAX_RETRY    3

#define BIT_GOT_IP        BIT0
#define BIT_JOIN_FAILED   BIT1

static EventGroupHandle_t s_events;
static esp_netif_t       *s_sta_netif;

/* False when esp_wifi_init() could not reach the C6. The console still runs;
 * the Wi-Fi commands refuse politely instead of calling into a dead link. */
static bool s_wifi_up;

bool wifi_console_link_up(void)
{
    return s_wifi_up;
}

/*
 * Live RSSI is not obtainable on this stack, and both ways of asking are dead
 * ends in esp_hosted 3.0.1:
 *
 *   - esp_wifi_sta_get_ap_info() returns rssi = 0. esp_wifi_remote does not
 *     forward esp_wifi_sta_get_rssi, and the ap_info the coprocessor sends
 *     back over RPC leaves the field unpopulated.
 *   - eh_host_wifi_sta_get_rssi() is declared and implemented, but the v2 RPC
 *     packer has no compose function for its msg_id (341), so every call fails
 *     with "pack_req_payload failed (rc=-1 msg_id=341)". Upstream gap, not a
 *     configuration problem.
 *
 * Scan records DO carry per-AP RSSI, so `scan` remains the way to read signal
 * strength. Rather than print a fabricated 0 dBm, the commands say so.
 */
#define RSSI_UNAVAILABLE_NOTE "n/a (use 'scan'; see wifi_console.c)"

/* Guard for every command that needs a working C6. */
static bool require_link(void)
{
    if (!s_wifi_up) {
        printf("C6 link is down - Wi-Fi is unavailable.\n"
               "Run 'c6boot' to reset the coprocessor and see what it prints.\n");
        return false;
    }
    return true;
}

/* Set while a `join` command is in flight, so the event handler retries a few
 * times instead of giving up on the first (routinely spurious) disconnect. */
static int  s_join_retries_left;
static bool s_associated;

/* ------------------------------------------------ saved credentials (NVS) */

/*
 * Credentials live in the P4's own NVS rather than relying on
 * esp_wifi_set_storage(WIFI_STORAGE_FLASH): over ESP-Hosted that would
 * persist them in the *C6's* NVS, so reflashing the coprocessor would
 * silently drop the saved network. Keeping them here ties them to the board
 * we actually control.
 *
 * NOTE: NVS encryption is off in this project, so the passphrase is stored in
 * plaintext and is readable by anyone who can dump the flash. That matches
 * stock esp_wifi behaviour, but it is worth knowing.
 */
#define NVS_NAMESPACE   "wifi_cfg"
#define NVS_KEY_SSID    "ssid"
#define NVS_KEY_PASS    "pass"

#define RECONNECT_DELAY_US  (3 * 1000 * 1000)

/* True once we have credentials worth reconnecting to. Cleared by `leave`,
 * so a deliberate disconnect stays disconnected instead of fighting the user. */
static bool s_auto_reconnect;
static esp_timer_handle_t s_reconnect_timer;

static esp_err_t creds_save(const char *ssid, const char *pass)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_str(h, NVS_KEY_SSID, ssid);
    if (err == ESP_OK) {
        err = nvs_set_str(h, NVS_KEY_PASS, pass ? pass : "");
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

static esp_err_t creds_load(char *ssid, size_t ssid_len, char *pass, size_t pass_len)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        return err;      /* ESP_ERR_NVS_NOT_FOUND when nothing was ever saved */
    }
    err = nvs_get_str(h, NVS_KEY_SSID, ssid, &ssid_len);
    if (err == ESP_OK) {
        err = nvs_get_str(h, NVS_KEY_PASS, pass, &pass_len);
        /* An open network is saved with an empty passphrase; absent key is
         * equivalent and must not fail the load. */
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            pass[0] = '\0';
            err = ESP_OK;
        }
    }
    nvs_close(h);
    return err;
}

static void creds_clear(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    nvs_erase_key(h, NVS_KEY_SSID);
    nvs_erase_key(h, NVS_KEY_PASS);
    nvs_commit(h);
    nvs_close(h);
}

/* Fired off the event handler rather than reconnecting inline: esp_wifi_connect()
 * from within the event task would busy-loop against an AP that is out of range. */
static void reconnect_timer_cb(void *arg)
{
    if (s_auto_reconnect && !s_associated) {
        ESP_LOGI(TAG, "auto-reconnecting");
        esp_wifi_connect();
    }
}

static const char *authmode_str(wifi_auth_mode_t m)
{
    switch (m) {
    case WIFI_AUTH_OPEN:            return "open";
    case WIFI_AUTH_WEP:             return "WEP";
    case WIFI_AUTH_WPA_PSK:         return "WPA";
    case WIFI_AUTH_WPA2_PSK:        return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK:    return "WPA/2";
    case WIFI_AUTH_WPA3_PSK:        return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK:   return "WPA2/3";
    case WIFI_AUTH_WAPI_PSK:        return "WAPI";
    case WIFI_AUTH_ENTERPRISE:      return "EAP";
    default:                        return "?";
    }
}

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *e = data;
        s_associated = false;
        if (s_join_retries_left > 0) {
            s_join_retries_left--;
            ESP_LOGW(TAG, "disconnected (reason %d), retrying", e->reason);
            esp_wifi_connect();
        } else {
            ESP_LOGW(TAG, "disconnected (reason %d)", e->reason);
            xEventGroupSetBits(s_events, BIT_JOIN_FAILED);
            /* Keep trying in the background if this is a saved network and
             * the user did not ask to leave it. */
            if (s_auto_reconnect && s_reconnect_timer) {
                esp_timer_stop(s_reconnect_timer);   /* no-op if not running */
                esp_timer_start_once(s_reconnect_timer, RECONNECT_DELAY_US);
            }
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *e = data;
        s_associated = true;
        s_join_retries_left = 0;
        ESP_LOGI(TAG, "got IP " IPSTR, IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_events, BIT_GOT_IP);
    }
}

/* ------------------------------------------------------------------ scan */

static struct {
    struct arg_int *max;
    struct arg_end *end;
} s_scan_args;

static int cmd_scan(int argc, char **argv)
{
    int errs = arg_parse(argc, argv, (void **)&s_scan_args);
    if (errs != 0) {
        arg_print_errors(stderr, s_scan_args.end, argv[0]);
        return 1;
    }
    if (!require_link()) return 1;
    uint16_t want = s_scan_args.max->count ? (uint16_t)s_scan_args.max->ival[0] : 20;

    printf("scanning...\n");
    esp_err_t err = esp_wifi_scan_start(NULL, true);   /* blocking */
    if (err != ESP_OK) {
        printf("scan failed: %s\n", esp_err_to_name(err));
        return 1;
    }

    uint16_t found = 0;
    esp_wifi_scan_get_ap_num(&found);
    if (found == 0) {
        printf("no APs found\n");
        return 0;
    }
    uint16_t n = (found < want) ? found : want;

    wifi_ap_record_t *recs = calloc(n, sizeof(*recs));
    if (!recs) {
        printf("out of memory for %u records\n", n);
        esp_wifi_clear_ap_list();
        return 1;
    }
    err = esp_wifi_scan_get_ap_records(&n, recs);
    if (err != ESP_OK) {
        printf("get_ap_records: %s\n", esp_err_to_name(err));
        free(recs);
        return 1;
    }

    printf("\n%-32s %-18s %4s %3s %-7s\n", "SSID", "BSSID", "RSSI", "CH", "AUTH");
    printf("-------------------------------- ------------------ ---- --- -------\n");
    for (uint16_t i = 0; i < n; i++) {
        printf("%-32s %02x:%02x:%02x:%02x:%02x:%02x %4d %3d %-7s\n",
               (const char *)recs[i].ssid,
               recs[i].bssid[0], recs[i].bssid[1], recs[i].bssid[2],
               recs[i].bssid[3], recs[i].bssid[4], recs[i].bssid[5],
               recs[i].rssi, recs[i].primary, authmode_str(recs[i].authmode));
    }
    printf("\n%u AP(s) shown of %u found\n", n, found);
    free(recs);
    return 0;
}

/* ------------------------------------------------------------------ join */

static struct {
    struct arg_str *ssid;
    struct arg_str *pass;
    struct arg_end *end;
} s_join_args;

static int cmd_join(int argc, char **argv)
{
    int errs = arg_parse(argc, argv, (void **)&s_join_args);
    if (errs != 0) {
        arg_print_errors(stderr, s_join_args.end, argv[0]);
        return 1;
    }
    if (!require_link()) return 1;
    const char *ssid = s_join_args.ssid->sval[0];
    const char *pass = s_join_args.pass->count ? s_join_args.pass->sval[0] : "";

    wifi_config_t cfg = { 0 };
    strlcpy((char *)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, pass, sizeof(cfg.sta.password));
    /* Leave the auth threshold open so WEP/WPA-only APs are still reachable;
     * the AP's own advertised mode decides what actually gets negotiated. */
    cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;

    esp_err_t err = esp_wifi_set_config(WIFI_IF_STA, &cfg);
    if (err != ESP_OK) {
        printf("set_config: %s\n", esp_err_to_name(err));
        return 1;
    }

    xEventGroupClearBits(s_events, BIT_GOT_IP | BIT_JOIN_FAILED);
    s_join_retries_left = JOIN_MAX_RETRY;

    esp_wifi_disconnect();
    err = esp_wifi_connect();
    if (err != ESP_OK) {
        printf("connect: %s\n", esp_err_to_name(err));
        s_join_retries_left = 0;
        return 1;
    }

    printf("joining \"%s\"...\n", ssid);
    EventBits_t bits = xEventGroupWaitBits(s_events, BIT_GOT_IP | BIT_JOIN_FAILED,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(JOIN_TIMEOUT_MS));
    s_join_retries_left = 0;

    if (bits & BIT_GOT_IP) {
        esp_netif_ip_info_t ip;
        esp_netif_get_ip_info(s_sta_netif, &ip);
        printf("connected: ip=" IPSTR " gw=" IPSTR " mask=" IPSTR "\n",
               IP2STR(&ip.ip), IP2STR(&ip.gw), IP2STR(&ip.netmask));

        /* Only persist a network we actually reached - saving on the attempt
         * would leave a bad passphrase to retry forever after every reboot. */
        esp_err_t serr = creds_save(ssid, pass);
        if (serr == ESP_OK) {
            s_auto_reconnect = true;
            printf("saved: will rejoin \"%s\" automatically on boot ('leave' to forget)\n",
                   ssid);
        } else {
            printf("warning: could not save credentials: %s\n", esp_err_to_name(serr));
        }
        return 0;
    }
    if (bits & BIT_JOIN_FAILED) {
        printf("join failed (see disconnect reason above)\n");
    } else {
        printf("join timed out after %d ms\n", JOIN_TIMEOUT_MS);
    }
    return 1;
}

/* ----------------------------------------------------------------- leave */

static int cmd_leave(int argc, char **argv)
{
    (void)argc; (void)argv;
    if (!require_link()) return 1;

    /* Clear the saved network first: dropping auto-reconnect before the
     * disconnect event fires stops the timer from immediately undoing this. */
    s_join_retries_left = 0;
    s_auto_reconnect = false;
    if (s_reconnect_timer) {
        esp_timer_stop(s_reconnect_timer);
    }
    creds_clear();

    esp_err_t err = esp_wifi_disconnect();
    printf("disconnect: %s\n", esp_err_to_name(err));
    printf("forgot saved network - will not auto-join on next boot\n");
    return err == ESP_OK ? 0 : 1;
}

/* ---------------------------------------------------------------- status */

static int cmd_status(int argc, char **argv)
{
    (void)argc; (void)argv;

    uint8_t mac[6] = { 0 };
    if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK) {
        printf("sta mac    : %02x:%02x:%02x:%02x:%02x:%02x\n",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else {
        printf("sta mac    : <unavailable - is the C6 link up?>\n");
    }

    printf("sdio link  : CLK=%d CMD=%d D0..D3=%d,%d,%d,%d reset=%d, %d-bit @ %d kHz\n",
           CONFIG_ESP_HOSTED_HOST_SDIO_PIN_CLK, CONFIG_ESP_HOSTED_HOST_SDIO_PIN_CMD,
           CONFIG_ESP_HOSTED_HOST_SDIO_PIN_D0, CONFIG_ESP_HOSTED_HOST_SDIO_PIN_D1,
           CONFIG_ESP_HOSTED_HOST_SDIO_PIN_D2, CONFIG_ESP_HOSTED_HOST_SDIO_PIN_D3,
           CONFIG_ESP_HOSTED_HOST_RESET_GPIO,
           CONFIG_ESP_HOSTED_HOST_SDIO_BUS_WIDTH, CONFIG_ESP_HOSTED_HOST_SDIO_CLK_KHZ);

    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        printf("associated : \"%s\" ch %d %s\n",
               (const char *)ap.ssid, ap.primary, authmode_str(ap.authmode));
        printf("rssi       : %s\n", RSSI_UNAVAILABLE_NOTE);
        /* What the association negotiated - or would, if it were forwarded.
         * Like rssi above, esp_wifi_remote does not marshal the phy_* bits of
         * wifi_ap_record_t across the RPC, so they read back all-zero even on
         * a healthy 11n link. Printed anyway because an all-zero line is a
         * useful reminder of that; do NOT read it as "the link is 11b". */
        if (ap.phy_11b || ap.phy_11g || ap.phy_11n || ap.phy_11ax || ap.phy_lr) {
            printf("phy        : %s%s%s%s%s bw=%s\n",
                   ap.phy_11b  ? "11b " : "",
                   ap.phy_11g  ? "11g " : "",
                   ap.phy_11n  ? "11n " : "",
                   ap.phy_11ax ? "11ax " : "",
                   ap.phy_lr   ? "LR "  : "",
                   ap.bandwidth == WIFI_BW40 ? "40MHz" : "20MHz");
        } else {
            printf("phy        : not reported by the coprocessor "
                   "(esp_wifi_remote drops the phy_* bits, same as rssi)\n");
        }
        esp_netif_ip_info_t ip;
        if (esp_netif_get_ip_info(s_sta_netif, &ip) == ESP_OK) {
            printf("ip         : " IPSTR " gw " IPSTR "\n", IP2STR(&ip.ip), IP2STR(&ip.gw));
        }
    } else {
        printf("associated : no\n");
    }

    char ssid[33] = { 0 }, pass[65] = { 0 };
    if (creds_load(ssid, sizeof(ssid), pass, sizeof(pass)) == ESP_OK && ssid[0]) {
        printf("saved net  : \"%s\" (auto-join %s)\n",
               ssid, s_auto_reconnect ? "on" : "off");
    } else {
        printf("saved net  : none\n");
    }
    return 0;
}

/* ------------------------------------------------------------------ init */

static esp_err_t register_wifi_commands(void)
{
    s_scan_args.max = arg_int0("n", "max", "<count>", "max APs to print (default 20)");
    s_scan_args.end = arg_end(2);
    const esp_console_cmd_t scan_cmd = {
        .command = "scan", .help = "Scan for nearby Wi-Fi networks",
        .hint = NULL, .func = &cmd_scan, .argtable = &s_scan_args,
    };
    ESP_RETURN_ON_ERROR(esp_console_cmd_register(&scan_cmd), TAG, "reg scan");

    s_join_args.ssid = arg_str1(NULL, NULL, "<ssid>", "network name");
    s_join_args.pass = arg_str0(NULL, NULL, "<password>", "passphrase (omit if open)");
    s_join_args.end  = arg_end(2);
    const esp_console_cmd_t join_cmd = {
        .command = "join", .help = "Associate with an AP and get an IP via DHCP",
        .hint = NULL, .func = &cmd_join, .argtable = &s_join_args,
    };
    ESP_RETURN_ON_ERROR(esp_console_cmd_register(&join_cmd), TAG, "reg join");

    const esp_console_cmd_t leave_cmd = {
        .command = "leave", .help = "Disconnect from the current AP",
        .hint = NULL, .func = &cmd_leave, .argtable = NULL,
    };
    ESP_RETURN_ON_ERROR(esp_console_cmd_register(&leave_cmd), TAG, "reg leave");

    const esp_console_cmd_t status_cmd = {
        .command = "status", .help = "Show link state, RSSI, IP and SDIO wiring",
        .hint = NULL, .func = &cmd_status, .argtable = NULL,
    };
    ESP_RETURN_ON_ERROR(esp_console_cmd_register(&status_cmd), TAG, "reg status");

    return ESP_OK;
}

esp_err_t wifi_console_start(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "nvs_flash_init");

    s_events = xEventGroupCreate();
    ESP_RETURN_ON_FALSE(s_events, ESP_ERR_NO_MEM, TAG, "event group");

    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop");

    s_sta_netif = esp_netif_create_default_wifi_sta();
    ESP_RETURN_ON_FALSE(s_sta_netif, ESP_FAIL, TAG, "create sta netif");

    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event, NULL, NULL), TAG, "wifi evt");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &on_wifi_event, NULL, NULL), TAG, "ip evt");

    /* esp_wifi_init() is the point where esp_wifi_remote brings up the
     * ESP-Hosted SDIO link and handshakes with the C6. If the C6 is missing,
     * held in reset, or running incompatible coprocessor firmware, it fails
     * here. That must NOT stop the console from starting: without a REPL
     * there is no way to run the c6mon/c6boot diagnostics that tell you why. */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t werr = esp_wifi_init(&cfg);
    if (werr == ESP_OK) {
        werr = esp_wifi_set_mode(WIFI_MODE_STA);
    }
    if (werr == ESP_OK) {
        werr = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    }
    if (werr == ESP_OK) {
        werr = esp_wifi_start();
    }
    if (werr == ESP_OK) {
        /* Power save OFF. The IDF default is WIFI_PS_MIN_MODEM, which parks the
         * C6's radio between DTIM beacons: it only listens on the beacon
         * boundary (~100 ms here) and has to win a wake round trip before it
         * can TX. Behind ESP-Hosted that latency lands on top of an SDIO round
         * trip that is already in series with the air.
         *
         * Worth having - this bridge is bus-powered and exists to push thermal
         * frames, so there is nothing to save power for - but do NOT read this
         * as the fix for a hard TX stall. It was tried against exactly that
         * symptom (~68 KB, i.e. one send buffer, then no ACKs ever) and changed
         * nothing: the byte count came back bit-identical with PS off. A stall
         * that reproduces to the byte is a buffer that never drains, not a
         * radio that wakes too slowly. See net_bench.c for the throttle probe. */
        esp_err_t perr = esp_wifi_set_ps(WIFI_PS_NONE);
        if (perr != ESP_OK) {
            ESP_LOGW(TAG, "esp_wifi_set_ps(NONE) failed: %s - throughput will be "
                          "capped at roughly one window per beacon interval",
                     esp_err_to_name(perr));
        }
    }

    if (werr == ESP_OK) {
        s_wifi_up = true;
        ESP_LOGI(TAG, "Wi-Fi (C6 over SDIO) up in STA mode, power save off");

        const esp_timer_create_args_t targs = {
            .callback = &reconnect_timer_cb,
            .name     = "wifi_reconnect",
        };
        ESP_RETURN_ON_ERROR(esp_timer_create(&targs, &s_reconnect_timer),
                            TAG, "reconnect timer");

        /* Rejoin the last network that actually worked. Fire-and-forget: the
         * console must come up now, not after a 20 s association attempt.
         * Progress shows up in the log, and the event handler retries. */
        char ssid[33] = { 0 }, pass[65] = { 0 };
        if (creds_load(ssid, sizeof(ssid), pass, sizeof(pass)) == ESP_OK && ssid[0]) {
            wifi_config_t cfg = { 0 };
            strlcpy((char *)cfg.sta.ssid, ssid, sizeof(cfg.sta.ssid));
            strlcpy((char *)cfg.sta.password, pass, sizeof(cfg.sta.password));
            cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;

            if (esp_wifi_set_config(WIFI_IF_STA, &cfg) == ESP_OK) {
                s_auto_reconnect = true;
                ESP_LOGI(TAG, "auto-joining saved network \"%s\"", ssid);
                esp_wifi_connect();
            }
        } else {
            ESP_LOGI(TAG, "no saved network - use 'join <ssid> [password]'");
        }
    } else {
        s_wifi_up = false;
        ESP_LOGE(TAG, "C6 link down: %s - Wi-Fi commands disabled, "
                      "use 'c6boot' to see what the coprocessor is running",
                 esp_err_to_name(werr));
    }

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_cfg.prompt = "p4> ";
    repl_cfg.max_cmdline_length = 256;

    /* The REPL rides the built-in USB-Serial-JTAG port, the same one the log
     * console uses - so `idf.py monitor` gives you both. Deliberately not
     * UART0: that is GPIO37/38 on the P4, which this board wires to DVP
     * D11/D10, so a UART console would corrupt captured pixels. */
    esp_console_dev_usb_serial_jtag_config_t usj_cfg =
        ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_console_new_repl_usb_serial_jtag(&usj_cfg, &repl_cfg, &repl),
                        TAG, "repl_usb_serial_jtag");

    ESP_RETURN_ON_ERROR(esp_console_register_help_command(), TAG, "reg help");
    ESP_RETURN_ON_ERROR(register_wifi_commands(), TAG, "reg wifi cmds");
    ESP_RETURN_ON_ERROR(net_bench_register(), TAG, "reg bench cmds");
    ESP_RETURN_ON_ERROR(c6_debug_register(), TAG, "reg c6 cmds");
    ESP_RETURN_ON_ERROR(c6_flash_register(), TAG, "reg c6flash");

    ESP_RETURN_ON_ERROR(esp_console_start_repl(repl), TAG, "start repl");

    ESP_LOGI(TAG, "console ready on USB-Serial-JTAG - type 'help'");
    return ESP_OK;
}
