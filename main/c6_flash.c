/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * Flash the ESP32-C6 coprocessor from the P4, over UART.
 *
 * The JC-ESP32P4-M3 gives the C6 no USB of its own, so the only way in is the
 * P4. esp-serial-flasher speaks the ROM loader protocol; we drive the C6's
 * strapping directly, which is far more deterministic than proxying esptool's
 * reset timing across USB (see esp-dev-kits issue #134, where exactly that
 * approach produced no response from the C6).
 *
 * Wiring:
 *     C6 EN    <- P4 GPIO54   (active low; also ESP-Hosted's reset pin)
 *     C6 IO9   <- P4 GPIO11   (BOOT strap, bodge wire; 5.1k pull-up on module)
 *     C6 U0RXD <- P4 GPIO13
 *     C6 U0TXD -> P4 GPIO12
 *
 * The images come from the `c6_fw` partition rather than being linked into
 * the app: the C6 app alone is ~1.1 MB and would otherwise dominate the P4
 * binary. Build and load them with:
 *     cd c6_firmware && idf.py set-target esp32c6 && idf.py build
 *     python tools/pack_c6_fw.py
 *     idf.py partition-table-flash        (once, after the layout change)
 *     esptool write-flash 0x310000 build/c6_fw.bin
 *
 * Then on the P4 console:  c6flash
 */

#include <string.h>
#include <stdio.h>

#include "wifi_console.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_console.h"
#include "esp_partition.h"
#include "argtable3/argtable3.h"

#include "esp_loader.h"
#include "esp32_port.h"

static const char *TAG = "c6_flash";

#define C6_UART_PORT      2
#define C6_TX_IO          13
#define C6_RX_IO          12
#define C6_EN_IO          CONFIG_ESP_HOSTED_HOST_RESET_GPIO   /* 54 */
#define C6_BOOT_IO        11

#define INIT_BAUD         115200
/* The ROM stub raises this once connected. 1.1 MB at 115200 would take ~100 s;
 * at 921600 it is nearer 15 s. Backed off automatically if the target refuses. */
#define FAST_BAUD         921600

#define FLASH_BLOCK_SIZE  4096
#define CHUNK_SIZE        4096

#define C6FW_MAGIC        0x57463643u   /* "C6FW" little-endian */

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    uint32_t count;
    uint32_t reserved;
} c6fw_header_t;

typedef struct __attribute__((packed)) {
    uint32_t flash_offset;
    uint32_t size;
    uint32_t blob_offset;
    uint32_t reserved;
} c6fw_entry_t;

#define MAX_IMAGES 8

static const char *loader_err_str(esp_loader_error_t e)
{
    switch (e) {
    case ESP_LOADER_SUCCESS:                 return "success";
    case ESP_LOADER_ERROR_FAIL:              return "generic failure";
    case ESP_LOADER_ERROR_TIMEOUT:           return "timeout (no response from C6)";
    case ESP_LOADER_ERROR_IMAGE_SIZE:        return "image too large for target flash";
    case ESP_LOADER_ERROR_INVALID_MD5:       return "MD5 mismatch after write";
    case ESP_LOADER_ERROR_INVALID_PARAM:     return "invalid parameter";
    case ESP_LOADER_ERROR_INVALID_TARGET:    return "unsupported/unknown target chip";
    case ESP_LOADER_ERROR_UNSUPPORTED_CHIP:  return "unsupported chip";
    case ESP_LOADER_ERROR_UNSUPPORTED_FUNC:  return "unsupported function";
    case ESP_LOADER_ERROR_INVALID_RESPONSE:  return "malformed response";
    default:                                 return "unknown error";
    }
}

/* Write one image from the c6_fw partition into the C6's flash. */
static esp_loader_error_t flash_one(const esp_partition_t *part,
                                    const c6fw_entry_t *e,
                                    uint8_t *chunk)
{
    printf("  0x%06" PRIx32 "  %8" PRIu32 " B  ", e->flash_offset, e->size);
    fflush(stdout);

    esp_loader_error_t lerr = esp_loader_flash_start(e->flash_offset, e->size,
                                                     FLASH_BLOCK_SIZE);
    if (lerr != ESP_LOADER_SUCCESS) {
        printf("start failed: %s\n", loader_err_str(lerr));
        return lerr;
    }

    uint32_t remaining = e->size;
    uint32_t src = e->blob_offset;
    uint32_t last_pct = 0;

    while (remaining > 0) {
        uint32_t n = (remaining < CHUNK_SIZE) ? remaining : CHUNK_SIZE;

        esp_err_t rerr = esp_partition_read(part, src, chunk, n);
        if (rerr != ESP_OK) {
            printf("partition read failed at %" PRIu32 ": %s\n",
                   src, esp_err_to_name(rerr));
            return ESP_LOADER_ERROR_FAIL;
        }

        lerr = esp_loader_flash_write(chunk, n);
        if (lerr != ESP_LOADER_SUCCESS) {
            printf("write failed at 0x%06" PRIx32 ": %s\n",
                   e->flash_offset + (e->size - remaining), loader_err_str(lerr));
            return lerr;
        }

        src       += n;
        remaining -= n;

        uint32_t pct = ((e->size - remaining) * 100) / e->size;
        if (pct >= last_pct + 10) {
            printf(".");
            fflush(stdout);
            last_pct = pct;
        }
    }

#if MD5_ENABLED
    lerr = esp_loader_flash_verify();
    if (lerr != ESP_LOADER_SUCCESS) {
        printf(" verify failed: %s\n", loader_err_str(lerr));
        return lerr;
    }
    printf(" ok (md5 verified)\n");
#else
    printf(" ok\n");
#endif
    return ESP_LOADER_SUCCESS;
}

/* ---------------------------------------------------------------- c6flash */

static struct {
    struct arg_int *baud;
    struct arg_end *end;
} s_flash_args;

static int cmd_c6flash(int argc, char **argv)
{
    int errs = arg_parse(argc, argv, (void **)&s_flash_args);
    if (errs != 0) {
        arg_print_errors(stderr, s_flash_args.end, argv[0]);
        return 1;
    }
    uint32_t fast_baud = s_flash_args.baud->count
                       ? (uint32_t)s_flash_args.baud->ival[0] : FAST_BAUD;

    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_UNDEFINED, "c6_fw");
    if (!part) {
        printf("no 'c6_fw' partition found.\n"
               "Flash the partition table and the container first:\n"
               "  idf.py partition-table-flash\n"
               "  esptool write-flash 0x310000 build/c6_fw.bin\n");
        return 1;
    }

    c6fw_header_t hdr;
    ESP_ERROR_CHECK(esp_partition_read(part, 0, &hdr, sizeof(hdr)));
    if (hdr.magic != C6FW_MAGIC) {
        printf("c6_fw partition has no valid container (magic 0x%08" PRIx32 ").\n"
               "Run: python tools/pack_c6_fw.py, then flash build/c6_fw.bin "
               "to 0x%" PRIx32 ".\n", hdr.magic, (uint32_t)part->address);
        return 1;
    }
    if (hdr.count == 0 || hdr.count > MAX_IMAGES) {
        printf("container claims %" PRIu32 " images, refusing\n", hdr.count);
        return 1;
    }

    c6fw_entry_t entries[MAX_IMAGES];
    ESP_ERROR_CHECK(esp_partition_read(part, sizeof(hdr), entries,
                                       sizeof(entries[0]) * hdr.count));

    /* c6mon/c6boot may hold UART2; the flasher wants to configure it itself. */
    c6_debug_release_uart();

    const loader_esp32_config_t cfg = {
        .baud_rate         = INIT_BAUD,
        .uart_port         = C6_UART_PORT,
        .uart_rx_pin       = C6_RX_IO,
        .uart_tx_pin       = C6_TX_IO,
        .reset_trigger_pin = C6_EN_IO,
        .gpio0_trigger_pin = C6_BOOT_IO,
    };
    if (loader_port_esp32_init(&cfg) != ESP_LOADER_SUCCESS) {
        printf("serial port init failed\n");
        return 1;
    }

    printf("connecting to C6 (EN=GPIO%d BOOT=GPIO%d TX=GPIO%d RX=GPIO%d)...\n",
           C6_EN_IO, C6_BOOT_IO, C6_TX_IO, C6_RX_IO);

    esp_loader_connect_args_t cargs = ESP_LOADER_CONNECT_DEFAULT();
    esp_loader_error_t lerr = esp_loader_connect_with_stub(&cargs);
    if (lerr != ESP_LOADER_SUCCESS) {
        printf("connect failed: %s\n", loader_err_str(lerr));
        printf("Check that 'c6boot -d' shows the ROM download banner first.\n");
        loader_port_esp32_deinit();
        return 1;
    }
    printf("connected, target = %s\n",
           esp_loader_get_target() == ESP32C6_CHIP ? "ESP32-C6" : "unexpected chip!");

    if (esp_loader_get_target() != ESP32C6_CHIP) {
        printf("refusing to flash a chip that is not an ESP32-C6\n");
        loader_port_esp32_deinit();
        return 1;
    }

    /* Speed up the bulk transfer. Not fatal if the target declines. */
    if (fast_baud != INIT_BAUD) {
        lerr = esp_loader_change_transmission_rate_stub(INIT_BAUD, fast_baud);
        if (lerr == ESP_LOADER_SUCCESS &&
            loader_port_change_transmission_rate(fast_baud) == ESP_LOADER_SUCCESS) {
            printf("raised link to %" PRIu32 " baud\n", fast_baud);
        } else {
            printf("staying at %d baud (rate change refused: %s)\n",
                   INIT_BAUD, loader_err_str(lerr));
        }
    }

    uint8_t *chunk = malloc(CHUNK_SIZE);
    if (!chunk) {
        printf("out of memory\n");
        loader_port_esp32_deinit();
        return 1;
    }

    printf("\nwriting %" PRIu32 " images:\n", hdr.count);
    int64_t t0 = esp_timer_get_time();
    bool ok = true;

    for (uint32_t i = 0; i < hdr.count; i++) {
        if (flash_one(part, &entries[i], chunk) != ESP_LOADER_SUCCESS) {
            ok = false;
            break;
        }
    }
    double secs = (double)(esp_timer_get_time() - t0) / 1e6;

    free(chunk);

    if (ok) {
        printf("\nflashed in %.1f s. Resetting C6 into its new firmware.\n", secs);
        esp_loader_reset_target();
    } else {
        printf("\nflashing aborted after %.1f s - C6 flash is now incomplete.\n"
               "Re-run c6flash; a partial write is not bootable.\n", secs);
    }

    loader_port_esp32_deinit();

    if (ok) {
        printf("\nRun 'c6mon -t 5' to watch it boot, then reboot the P4 so\n"
               "ESP-Hosted can bring the SDIO link up against the new firmware.\n");
    }
    return ok ? 0 : 1;
}

/* ------------------------------------------------------------------ init */

esp_err_t c6_flash_register(void)
{
    s_flash_args.baud = arg_int0("b", "baud", "<rate>",
                                 "bulk transfer rate, default 921600");
    s_flash_args.end  = arg_end(2);
    const esp_console_cmd_t cmd = {
        .command = "c6flash",
        .help = "Flash the ESP32-C6 coprocessor from the c6_fw partition over UART",
        .hint = NULL, .func = &cmd_c6flash, .argtable = &s_flash_args,
    };
    ESP_RETURN_ON_ERROR(esp_console_cmd_register(&cmd), TAG, "reg c6flash");
    return ESP_OK;
}
