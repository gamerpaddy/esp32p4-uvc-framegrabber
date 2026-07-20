idf_component_register(SRCS usb_device_uvc.c
                    INCLUDE_DIRS "include"
                    REQUIRES esp_timer esp_hw_support)

idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)

target_include_directories(${tusb_lib} PUBLIC "${COMPONENT_DIR}/tusb")
target_sources(${tusb_lib} PUBLIC "${COMPONENT_DIR}/tusb/usb_descriptors.c")
