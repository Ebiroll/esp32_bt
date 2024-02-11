
## This example shows how to use the esp32 to send iBeacon packets.

Explanation
https://github.com/JiaoXianjun/BTLE/blob/master/doc/ibeacon.pdf

# DStike V2 Pocket32

I lost the spec sheet for the DStike V2 Pocket32, so I have to reverse engineer my own software on the device.

Then I found it

https://www.tindie.com/products/lspoplove/pocket-32-esp3218650/#product-reviews


PIN 16 is the led, we will turn it on and off when not in sleep mode.

# Reverse the flash code

## Download flash
```
cd .platformio
source penv/bin/activate
 /Users/olofastrand/.platformio/packages/tool-esptoolpy@1.30300.0/esptool.py  --chip esp32 -b 921600 -p /dev/tty.usbserial-0001 --no-stub read_flash 0  0x400000 flash.bin
```
## Create elf from dump
git clone https://github.com/tenable/esp32_image_parser
cd esp32_image_parser
### Create a new penv environment
```
python3 --version
pip3 install virtualenv
cd your_project_directory
virtualenv penv
source penv/bin/activate
pip3 install -r requirements.txt
esp32_image_parser.py create_elf espwroom32.bin -partition ota_0 -output ota_0.elf

(penv) esp32_image_parser % python3 esp32_image_parser.py show_partitions /Users/olofastrand/.platformio/flash.bin 
reading partition table...
entry 0:
  label      : nvs
  offset     : 0x9000
  length     : 24576
  type       : 1 [DATA]
  sub type   : 2 [WIFI]

entry 1:
  label      : phy_init
  offset     : 0xf000
  length     : 4096
  type       : 1 [DATA]
  sub type   : 1 [RF]

entry 2:
  label      : factory
  offset     : 0x10000
  length     : 1048576
  type       : 0 [APP]
  sub type   : 0 [FACTORY]

Had to fixup the code to get it to work
https://flaviutamas.com/2021/reversing-emporia-vue-2

import sys
import json
import os, argparse
from makeelf.elf import *
from esptool import *
from esptool.bin_image import *
from esp32_firmware_reader import *
from read_nvs import *

diff --git a/esp32_image_parser.py b/esp32_image_parser.py
index 6503cf7..d5861a5 100755
--- a/esp32_image_parser.py
+++ b/esp32_image_parser.py
@@ -51,9 +51,9 @@ def image2elf(filename, output_file, verbose=False):
 
     # maps segment names to ELF sections
     section_map = {
-        'DROM'                      : '.flash.rodata',
-        'BYTE_ACCESSIBLE, DRAM, DMA': '.dram0.data',
-        'IROM'                      : '.flash.text',
+        'DROM'                 : '.flash.rodata',
+        'BYTE_ACCESSIBLE, DRAM': '.dram0.data',
+        'IROM'                 : '.flash.text',
         #'RTC_IRAM'                  : '.rtc.text' TODO
     }

When running we got the following output,
0;32mI (86) esp_image: segment 0: paddr=0x00010020 vaddr=0x3f400020 size=0x04048 ( 16456) map␛[0m
␛[0;32mI (101) esp_image: segment 1: paddr=0x00014070 vaddr=0x3ffb0000 size=0x01704 (  5892) load␛[0m
␛[0;32mI (106) esp_image: segment 2: paddr=0x0001577c vaddr=0x40080000 size=0x00400 (  1024) load␛[0m
␛[0;32mI (113) esp_image: segment 3: paddr=0x00015b84 vaddr=0x40080400 size=0x07a4c ( 31308) load␛[0m
␛[0;32mI (135) esp_image: segment 4: paddr=0x0001d5d8 vaddr=0x400c0000 size=0x00000 (     0) load␛[0m
␛[0;32mI (135) esp_image: segment 5: paddr=0x0001d5e0 vaddr=0x00000000 size=0x02a30 ( 10800) ␛[0m
␛[0;32mI (145) esp_image: segment 6: paddr=0x00020018 vaddr=0x400d0018 size=0x11aa8 ( 72360) map␛[0m
␛[0;32mI (180) boot: Loaded app from partition at offset 0x10000␛[0m
␛[0;32mI (180) boot: Disabling RNG early entropy source...␛[0m
␛[0;32mI (181) cpu_start: Pro cpu up.␛[0m
␛[0;32mI (184) cpu_start: Single core mode␛[0m
␛[0;32mI (189) heap_init: Initializing. RAM available for dynamic allocation:␛[0m
␛[0;32mI (196) heap_init: At 3FFAE6E0 len 00001920 (6 KiB): DRAM␛[0m
␛[0;32mI (202) heap_init: At 3FFB1E28 len 0002E1D8 (184 KiB): DRAM␛[0m
␛[0;32mI (208) heap_init: At 3FFE0440 len 00003BC0 (14 KiB): D/IRAM␛[0m
␛[0;32mI (214) heap_init: At 3FFE4350 len 0001BCB0 (111 KiB): D/IRAM␛[0m
␛[0;32mI (221) heap_init: At 40087E4C len 000181B4 (96 KiB): IRAM␛[0m
␛[0;32mI (227) cpu_start: Pro cpu start user code␛[0m
␛[0;32mI (245) cpu_start: Starting scheduler on PRO CPU.␛[0m
␛[0;32mI (246) gpio: GPIO[16]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0 ␛[0m
␛[0;32mI (246) gpio: GPIO[17]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0 ␛[0m


https://www.tindie.com/products/lspoplove/pocket-32-esp3218650/#product-reviews

1 extra LED could be programmed(Connected with GPIO16[D0])

To make the reversed code more readable try svd-loader and the SVD file https://github.com/Ebiroll/esp32_flash_loader/blob/master/data/esp32.svd

The official SVD file is here, but dont work so well with the parser
https://github.com/espressif/svd

When done
deactivate
```



