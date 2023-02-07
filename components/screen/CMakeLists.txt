
set(SCREEN_DIR "controller_driver/st7789"
            "controller_driver/st7796"
            "controller_driver/ili9488"
            "controller_driver/ili9488n"
            "controller_driver/gc9a01"
            "qmsd_rgb"
            )

idf_component_register(SRC_DIRS "${SCREEN_DIR}" "screen_utility" "interface_driver"
                        INCLUDE_DIRS "${SCREEN_DIR}" "interface_driver" "qmsd_rgb" "." "screen_utility"
                        REQUIRES bus esp_lcd
                        )

if(CONFIG_SPIRAM)
    idf_component_optional_requires(PRIVATE esp_psram)
endif()

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")

