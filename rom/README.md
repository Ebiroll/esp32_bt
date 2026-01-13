This file was created to import the ESP32 ROM functions as labels with the SymbolImportScript in Ghidra

A quick regex to reformat the [ESP32 ld data](https://github.com/espressif/esp-idf/tree/master/components/esp_rom/esp32/ld) into the format expected by script

Replace `/[PROVIDE \(]*([^\s=]+)[\s=]+([^\s);]*).*/g` -> `$1 $2 l`

This is tester with ghidra 12 but add rom in overlay rom