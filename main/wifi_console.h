/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 *
 * Wi-Fi on the ESP32-P4 via ESP-Hosted: the P4 has no radio, so every
 * esp_wifi_* call is forwarded by esp_wifi_remote over SDIO to the on-module
 * ESP32-C6 coprocessor (CLK=18 CMD=19 D0..D3=14..17, reset=54).
 *
 * Exposes an esp_console REPL on the built-in USB-Serial-JTAG port - the same
 * port the log console uses, so `idf.py monitor` shows logs and takes typed
 * commands together. Commands:
 *   scan [-n <max>]              list nearby APs
 *   join <ssid> [password]       associate + DHCP, then remember the network
 *   leave                        disconnect and forget the saved network
 *   status                       link state, IP, MACs, saved network
 *   sdiospeed [-t s] [-l bytes]  host->C6 SDIO write throughput
 *   wifispeed <ip> [-p port] [-t s]  TCP throughput to a host PC sink
 *
 * Not UART0: that is GPIO37/38 on the P4, which this board wires to DVP
 * D11/D10, so a UART console would corrupt captured pixels while printing.
 * The camera's own command channel is separate (UART1, see camera_uart.h).
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Bring up NVS, esp_netif, the default event loop and the Wi-Fi station
 * interface (which starts the ESP-Hosted SDIO link to the C6), then start the
 * console REPL. Returns once the REPL task is running.
 *
 * If a previous `join` succeeded, its network is reloaded from NVS and
 * association starts in the background - this call does not block on it.
 * Otherwise nothing is joined until you run `join`.
 *
 * The REPL starts even if the C6 link fails, so the c6mon/c6boot diagnostics
 * remain reachable; in that case the Wi-Fi commands report the link is down.
 */
esp_err_t wifi_console_start(void);

/* True once esp_wifi_init()/start() succeeded against the C6. */
bool wifi_console_link_up(void);

/* Register the sdiospeed/wifispeed commands. Called by wifi_console_start();
 * exposed so the benchmarks can be registered from elsewhere if needed. */
esp_err_t net_bench_register(void);

/* Register c6mon/c6boot - raw access to the coprocessor's UART0, for finding
 * out what (if anything) the C6 is running when the SDIO link will not come
 * up. Also called by wifi_console_start(). */
esp_err_t c6_debug_register(void);

/* Release UART2 if c6mon/c6boot installed it, so the flasher can claim it. */
void c6_debug_release_uart(void);

/* Register c6flash - writes the C6 coprocessor images from the c6_fw
 * partition into the C6 over UART. Also called by wifi_console_start(). */
esp_err_t c6_flash_register(void);

#ifdef __cplusplus
}
#endif
