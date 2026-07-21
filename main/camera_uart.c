/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * UART command/response channel to the thermal camera. See camera_uart.h.
 */

#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <inttypes.h>
#include "camera_uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "tusb.h"   /* CDC-ACM: the command console rides the composite USB device */

static const char *TAG = "cam_uart";

#define UART_PORT     UART_NUM_1
#define UART_TX_IO    29
#define UART_RX_IO    28
#define UART_BAUD     38400

#define PKT_STX       0x02
#define PKT_ETX       0x03
#define MAX_PAYLOAD   64

static camera_ctx_t *s_cam;   /* for the local BIT (8/14-bit phase) command */

/*
 * Log ring: recent lines the web /log endpoint pulls to show TX confirmations
 * and RX replies in the browser console. Small on purpose — a monotonic seq
 * lets clients poll ?since=<seq> and get only new lines. Size (32 x 128) fits
 * easily in internal RAM and is enough to survive a slow poller (~15 s worth
 * at the highest realistic reply rate).
 */
#define LOG_MAX_LINES  32
#define LOG_MAX_CHARS  128

static struct {
    SemaphoreHandle_t mtx;
    uint32_t seq_next;               /* seq assigned to the most recent line */
    char     text[LOG_MAX_LINES][LOG_MAX_CHARS];
    uint32_t seq[LOG_MAX_LINES];     /* 0 = slot empty */
} s_log;

/* Write a line to the CDC COM port (if a host terminal is attached). Lines
 * are prefixed "cam_uart:" so the viewer's reply filter matches both the CDC
 * port and the log console. */
static void cdc_line(const char *text)
{
    if (!tud_cdc_connected()) {
        return;
    }
    tud_cdc_write_str("cam_uart: ");
    tud_cdc_write_str(text);
    tud_cdc_write_str("\r\n");
    tud_cdc_write_flush();
}

/* Append a line to the log ring. Sinks (RX, TX confirm, local replies) all
 * funnel through here so the browser sees exactly the same stream as the CDC
 * console and the ESP-IDF log. */
static void log_append(const char *text)
{
    if (!s_log.mtx || !text) {
        return;
    }
    xSemaphoreTake(s_log.mtx, portMAX_DELAY);
    uint32_t seq = ++s_log.seq_next;
    if (seq == 0) seq = ++s_log.seq_next;   /* skip 0 — reserved for "empty" */
    int slot = (int)(seq % LOG_MAX_LINES);
    s_log.seq[slot] = seq;
    /* Strip trailing CR/LF; the ring stores one plain line per slot. */
    size_t n = strlen(text);
    while (n && (text[n - 1] == '\r' || text[n - 1] == '\n')) n--;
    if (n >= LOG_MAX_CHARS) n = LOG_MAX_CHARS - 1;
    memcpy(s_log.text[slot], text, n);
    s_log.text[slot][n] = '\0';
    xSemaphoreGive(s_log.mtx);
}

/* Escape one string for JSON output. Enough for the ASCII/printable-with-dots
 * lines this module actually produces — quote and backslash are the only
 * characters that can occur in practice that need escaping. */
static size_t json_escape(const char *src, char *dst, size_t cap)
{
    size_t o = 0;
    for (const char *p = src; *p && o + 2 < cap; p++) {
        unsigned char c = (unsigned char)*p;
        if (c == '"' || c == '\\') {
            if (o + 2 >= cap) break;
            dst[o++] = '\\';
            dst[o++] = (char)c;
        } else if (c < 0x20) {
            if (o + 6 >= cap) break;
            o += (size_t)snprintf(dst + o, cap - o, "\\u%04x", c);
        } else {
            dst[o++] = (char)c;
        }
    }
    dst[o] = '\0';
    return o;
}

size_t camera_uart_log_snapshot(uint32_t since, char *dst, size_t cap)
{
    if (!dst || cap < 32) {
        return 0;
    }
    if (!s_log.mtx) {
        int n = snprintf(dst, cap, "{\"seq\":0,\"lines\":[]}");
        return (n > 0) ? (size_t)n : 0;
    }

    xSemaphoreTake(s_log.mtx, portMAX_DELAY);
    uint32_t seq_now = s_log.seq_next;

    /* Nothing new — cheap early-out matches the common polling case. */
    if (since >= seq_now) {
        xSemaphoreGive(s_log.mtx);
        int n = snprintf(dst, cap, "{\"seq\":%" PRIu32 ",\"lines\":[]}", seq_now);
        return (n > 0) ? (size_t)n : 0;
    }

    /* Walk oldest -> newest by seq so the client can just append. Older entries
     * outside the ring are silently skipped (they've fallen off). */
    uint32_t start = (seq_now > LOG_MAX_LINES) ? (seq_now - LOG_MAX_LINES + 1) : 1;
    if (start <= since) start = since + 1;

    size_t o = 0;
    int n = snprintf(dst + o, cap - o, "{\"seq\":%" PRIu32 ",\"lines\":[", seq_now);
    if (n < 0 || (size_t)n >= cap - o) { xSemaphoreGive(s_log.mtx); dst[0] = '\0'; return 0; }
    o += (size_t)n;

    bool first = true;
    for (uint32_t s = start; s <= seq_now; s++) {
        int slot = (int)(s % LOG_MAX_LINES);
        if (s_log.seq[slot] != s) continue;   /* stale/empty slot */

        char esc[LOG_MAX_CHARS * 2];
        json_escape(s_log.text[slot], esc, sizeof(esc));
        n = snprintf(dst + o, cap - o, "%s\"%s\"", first ? "" : ",", esc);
        if (n < 0 || (size_t)n >= cap - o) break;
        o += (size_t)n;
        first = false;
    }
    xSemaphoreGive(s_log.mtx);

    n = snprintf(dst + o, cap - o, "]}");
    if (n > 0 && (size_t)n < cap - o) o += (size_t)n;
    return o;
}

esp_err_t camera_uart_send(const char *cmd, const char *values)
{
    if (!cmd || strlen(cmd) != 3) {
        ESP_LOGE(TAG, "cmd must be exactly 3 chars (got '%s')", cmd ? cmd : "");
        return ESP_ERR_INVALID_ARG;
    }

    /* Payload = "CMD," [+ "value,"]; the trailing comma is always present. */
    char payload[MAX_PAYLOAD];
    int n = (values && values[0])
            ? snprintf(payload, sizeof(payload), "%s,%s,", cmd, values)
            : snprintf(payload, sizeof(payload), "%s,", cmd);
    if (n < 0 || n >= (int)sizeof(payload)) {
        ESP_LOGE(TAG, "payload too long");
        return ESP_ERR_INVALID_SIZE;
    }

    uint32_t sum = 0;
    for (int i = 0; i < n; i++) {
        sum += (uint8_t)payload[i];
    }

    uint8_t frame[3 + MAX_PAYLOAD + 2];
    int p = 0;
    frame[p++] = PKT_STX;
    frame[p++] = (uint8_t)(n & 0xFF);
    frame[p++] = (uint8_t)((n >> 8) & 0xFF);
    memcpy(&frame[p], payload, n);
    p += n;
    frame[p++] = (uint8_t)(sum & 0xFF);
    frame[p++] = PKT_ETX;

    int written = uart_write_bytes(UART_PORT, frame, p);
    if (written != p) {
        ESP_LOGE(TAG, "uart_write_bytes: wrote %d/%d", written, p);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "TX -> %s", payload);

    /* Also log the TX so the web console can echo what was sent. */
    char echo[MAX_PAYLOAD + 8];
    snprintf(echo, sizeof(echo), "TX -> %s", payload);
    log_append(echo);
    return ESP_OK;
}

/* Continuously read the camera's replies and print them (hex + ASCII). */
static void rx_task(void *arg)
{
    uint8_t buf[128];
    char hex[3 * sizeof(buf) + 1];
    char asc[sizeof(buf) + 1];
    for (;;) {
        int len = uart_read_bytes(UART_PORT, buf, sizeof(buf), pdMS_TO_TICKS(200));
        if (len <= 0) {
            continue;
        }
        int ho = 0, ao = 0;
        for (int i = 0; i < len; i++) {
            ho += snprintf(hex + ho, sizeof(hex) - ho, "%02X ", buf[i]);
            asc[ao++] = (buf[i] >= 0x20 && buf[i] < 0x7F) ? (char)buf[i] : '.';
        }
        asc[ao] = '\0';
        ESP_LOGI(TAG, "RX <- %d bytes | \"%s\" | %s", len, asc, hex);
        cdc_line(asc);   /* forward the reply to the CDC COM port */
        log_append(asc); /* and to the web log ring */
    }
}

esp_err_t camera_uart_submit_line(const char *line)
{
    if (!line) return ESP_ERR_INVALID_ARG;

    /* Split into "CMD" and "value" at the first comma; strip spaces. */
    char cmd[8] = {0}, val[40] = {0};
    const char *comma = strchr(line, ',');
    const char *cs = line;
    int ci = 0;
    while (*cs && cs != comma && ci < (int)sizeof(cmd) - 1) {
        if (!isspace((unsigned char)*cs)) cmd[ci++] = (char)toupper((unsigned char)*cs);
        cs++;
    }
    if (comma) {
        const char *vs = comma + 1;
        int vi = 0;
        while (*vs && vi < (int)sizeof(val) - 1) {
            if (!isspace((unsigned char)*vs)) val[vi++] = (char)toupper((unsigned char)*vs);
            vs++;
        }
    }
    if (!cmd[0]) {
        return ESP_ERR_INVALID_ARG;
    }

    /* "BIT" is a LOCAL command (not sent to the camera): switch which
     * interleaved phase is delivered. BIT,8 = 8-bit video (menu),
     * BIT,14 = 14-bit thermal, BIT alone = toggle. */
    if (strcmp(cmd, "BIT") == 0) {
        if (!s_cam) {
            ESP_LOGW(TAG, "BIT: no camera context");
            cdc_line("BIT: no camera context");
            log_append("BIT: no camera context");
            return ESP_ERR_INVALID_STATE;
        }
        uint32_t keep;
        if (strcmp(val, "8") == 0)        keep = 0;
        else if (strcmp(val, "14") == 0)  keep = 1;
        else                              keep = s_cam->keep_parity ? 0u : 1u;
        camera_set_keep_parity(s_cam, keep);
        const char *msg = keep ? "BIT,14 (thermal phase)" : "BIT,8 (video phase)";
        cdc_line(msg);
        log_append(msg);
        return ESP_OK;
    }
    if (strcmp(cmd, "PCK") == 0) {
        /* LOCAL command: flip the PCLK sampling edge live (GPIO-matrix
         * inversion) to hunt a clean data-valid window without a reflash.
         * PCK,0 = rising edge, PCK,1 = falling edge, PCK alone = toggle. */
        if (!s_cam) {
            ESP_LOGW(TAG, "PCK: no camera context");
            cdc_line("PCK: no camera context");
            log_append("PCK: no camera context");
            return ESP_ERR_INVALID_STATE;
        }
        bool inv;
        if (val[0] == '0')      inv = false;
        else if (val[0] == '1') inv = true;
        else                    inv = !s_cam->pclk_invert;
        camera_set_pclk_invert(s_cam, inv);
        const char *msg = inv ? "PCK,1 (falling edge)" : "PCK,0 (rising edge)";
        cdc_line(msg);
        log_append(msg);
        return ESP_OK;
    }
    /* Anything else is forwarded to the camera. camera_uart_send() logs the
     * TX confirmation itself. */
    return camera_uart_send(cmd, val[0] ? val : NULL);
}

/*
 * Read typed commands from the CDC-ACM COM port (part of the composite USB
 * device, alongside UVC), one per line, and send them to the camera. Format:
 * "CMD" or "CMD,value" (e.g. GCO, KBD,C, SSM,1). Spaces are ignored; the
 * 3-letter command is upper-cased.
 */
static void console_task(void *arg)
{
    ESP_LOGI(TAG, "CDC console ready — send a command + Enter, e.g.  KBD,C   SSM,1   GCO");
    char line[64];
    int len = 0;
    uint8_t c;
    for (;;) {
        if (!tud_cdc_connected() || tud_cdc_read(&c, 1) != 1) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (c == '\r' || c == '\n') {
            if (len == 0) {
                continue;
            }
            line[len] = '\0';
            len = 0;
            camera_uart_submit_line(line);
        } else if (len < (int)sizeof(line) - 1) {
            line[len++] = (char)c;
        }
    }
}

esp_err_t camera_uart_start(camera_ctx_t *cam)
{
    s_cam = cam;

    s_log.mtx = xSemaphoreCreateMutex();
    if (!s_log.mtx) {
        ESP_LOGE(TAG, "log mutex alloc failed");
        return ESP_ERR_NO_MEM;
    }

    const uart_config_t cfg = {
        .baud_rate  = UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t ret = uart_driver_install(UART_PORT, 512, 512, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_IO, UART_RX_IO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "UART1 up: TX=IO%d RX=IO%d @ %d 8N1", UART_TX_IO, UART_RX_IO, UART_BAUD);

    /* Typed commands arrive over the CDC-ACM COM port of the composite USB
     * device (same HS port as UVC) — see console_task. ESP_LOG output stays
     * on the USB-Serial-JTAG console as before. */
    xTaskCreate(rx_task, "cam_uart_rx", 4096, NULL, 5, NULL);
    xTaskCreate(console_task, "cam_uart_con", 4096, NULL, 4, NULL);
    return ESP_OK;
}
