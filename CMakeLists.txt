set(COMPONENT_ADD_INCLUDEDIRS include)
set(COMPONENT_SRCS "owb.c" "owb_gpio.c" "owb_rmt.c")
set(COMPONENT_REQUIRES "soc" "driver" "esp_rom")
register_component()
