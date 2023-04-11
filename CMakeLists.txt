if (NOT IDF_VERSION_MAJOR)
  set(IDF_VERSION_MAJOR 3)
endif()

if (IDF_VERSION_MAJOR LESS 5)
    message(FATAL_ERROR "Can't build: requires ESP-IDF version 5.0 or above.")
endif()

set(COMPONENT_ADD_INCLUDEDIRS include)
set(COMPONENT_SRCS "owb.c" "owb_gpio.c" "owb_rmt.c")
set(COMPONENT_REQUIRES "soc" "driver" "esp_rom")
register_component()
