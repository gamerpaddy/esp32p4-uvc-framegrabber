/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * UART command/response channel to the thermal camera. See camera_uart.h.
 */

#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "camera_uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
    }
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

            /* Split into "CMD" and "value" at the first comma; strip spaces. */
            char cmd[8] = {0}, val[40] = {0};
            char *comma = strchr(line, ',');
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
                    /* values are uppercase letters or digits */
                    if (!isspace((unsigned char)*vs)) val[vi++] = (char)toupper((unsigned char)*vs);
                    vs++;
                }
            }

            /* "BIT" is a LOCAL command (not sent to the camera): switch which
             * interleaved phase is delivered. BIT,8 = 8-bit video (menu),
             * BIT,14 = 14-bit thermal, BIT alone = toggle. */
            if (strcmp(cmd, "BIT") == 0) {
                if (s_cam) {
                    uint32_t keep;
                    if (strcmp(val, "8") == 0)        keep = 0;
                    else if (strcmp(val, "14") == 0)  keep = 1;
                    else                              keep = s_cam->keep_parity ? 0u : 1u;
                    camera_set_keep_parity(s_cam, keep);
                    cdc_line(keep ? "BIT,14 (thermal phase)" : "BIT,8 (video phase)");
                } else {
                    ESP_LOGW(TAG, "BIT: no camera context");
                    cdc_line("BIT: no camera context");
                }
            } else if (strcmp(cmd, "PCK") == 0) {
                /* LOCAL command: flip the PCLK sampling edge live (GPIO-matrix
                 * inversion) to hunt a clean data-valid window without a reflash.
                 * PCK,0 = rising edge, PCK,1 = falling edge, PCK alone = toggle. */
                if (s_cam) {
                    bool inv;
                    if (val[0] == '0')      inv = false;
                    else if (val[0] == '1') inv = true;
                    else                    inv = !s_cam->pclk_invert;
                    camera_set_pclk_invert(s_cam, inv);
                    cdc_line(inv ? "PCK,1 (falling edge)" : "PCK,0 (rising edge)");
                } else {
                    ESP_LOGW(TAG, "PCK: no camera context");
                    cdc_line("PCK: no camera context");
                }
            } else {
                camera_uart_send(cmd, val[0] ? val : NULL);
            }
        } else if (len < (int)sizeof(line) - 1) {
            line[len++] = (char)c;
        }
    }
}

esp_err_t camera_uart_start(camera_ctx_t *cam)
{
    s_cam = cam;
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
