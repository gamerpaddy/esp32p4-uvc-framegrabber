idf_component_register(
    SRCS
        "app_main.c"
        "camera_pipeline.c"
        "uvc_streaming.c"
        "camera_uart.c"
        "settings.c"
        "wifi_console.c"
        "net_bench.c"
        "c6_debug.c"
        "c6_flash.c"
        "web_stream.c"
    INCLUDE_DIRS
        "."
    PRIV_REQUIRES
        esp_timer
        esp_hw_support
        esp_mm
        esp_driver_cam
        esp_driver_gpio
        esp_driver_uart
        espressif__tinyusb
        usb_device_uvc
        # Wi-Fi over ESP-Hosted (C6 via SDIO) + the UART0 console
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
        lwip
        console
        # Browser view of the DVP stream: HTTP + the P4's hardware JPEG encoder
        esp_http_server
        esp_driver_jpeg
        # C6 coprocessor flashing: images live in the c6_fw partition
        esp_partition
        espressif__esp-serial-flasher
)
