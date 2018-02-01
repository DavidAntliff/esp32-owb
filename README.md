# esp32-owb

This is a ESP32-compatible C component for the Maxim Integrated "1-Wire" protocol.

It is written and tested for version 3.0 of the [ESP-IDF](https://github.com/espressif/esp-idf) environment, using the xtensa-esp32-elf toolchain (gcc version 5.2.0, crosstool-ng-1.22.0-80-g6c4433a).

Support for v2.1 is available on the [ESP-IDF_v2.1](https://github.com/DavidAntliff/esp32-owb/tree/ESP-IDF_v2.1) branch.

## Features

This library includes:

 * External power supply mode (parasitic mode not yet supported).
 * Static (stack-based) or dynamic (malloc-based) memory model.
 * No globals - support any number of 1-Wire buses simultaneously.
 * 1-Wire device detection and validation, including search for multiple devices on a single bus.
 * Addressing optimisation for a single (solo) device on a bus.
 * 1-Wire bus operations including multi-byte read and write operations.
 * CRC checks on ROM code.

## Documentation

Automatically generated API documentation (doxygen) is available [here](https://davidantliff.github.io/esp32-owb/index.html).

## Source Code

The source is available from [GitHub](https://www.github.com/DavidAntliff/esp32-owb).

## License

The code in this project is licensed under the MIT license - see LICENSE for details.

## Links

 * [esp32-ds18b20](https://github.com/DavidAntliff/esp32-ds18b20) - ESP32-compatible DS18B20 Digital Thermometer component for ESP32
 * [1-Wire Communication Through Software](https://www.maximintegrated.com/en/app-notes/index.mvp/id/126)
 * [1-Wire Search Algorithm](https://www.maximintegrated.com/en/app-notes/index.mvp/id/187)
 * [Espressif IoT Development Framework for ESP32](https://github.com/espressif/esp-idf)

## Acknowledgements

Parts of this code are based on references provided to the public domain by Maxim Integrated.

"1-Wire" is a registered trademark of Maxim Integrated.



