
# Understanfing BTLE

This repository explores and tries to undestland BLE technology.




# Sub directories

    server, We create an UART that sends single letters when connected 
    client , Requres a display(M5) as we select what device to connect to with the buttons.

    [hackrf-sniff](hack-rf) sniff. Sniffer software code from https://github.com/JiaoXianjun/BTLE
The major limitation of using HackRFOne as sniffer that it does not handle channel jumping so well.
This is another sniffer, however bandwidth might be a problem. It tunes the HACKRF to 2427.0
https://github.com/mikeryan/ice9-bluetooth-sniffer This has nice integration with wireshark. <BR>

    i_beacon, Simple iBeacon  

    The hci controller software Is from the examples direcory.
    $(HOME)/.platformio/packages/framework-espidf/examples/bluetooth/hci/controller_vhci_ble_adv

    [vci_ble_simulate](simulation), This is taken from qemu to allow simulation of the BLE controller over a socket.



# Bluetooth LE Channels
The Bluetooth LE system operates in the 2.4 GHz ISM band at 2400 - 2483.5 MHz. It uses 40 RF channels (each channel is 2 MHz wide). Channel 37,38 and 39 are advertiser channels.

![Alt text](BLEChannelHopping.png "BLE Channel Hopping")




The Idea was to go down a rabbit hole similar to this.
https://github.com/lupyuen/lupyuen.github.io/blob/master/src/wifi.md
[Read the article "Reverse Engineering WiFi on RISC-V BL602"](https://lupyuen.github.io/articles/wifi)
There are some really useful pointers and information in this.

![Alt text](frequencies-wifi.png "WiFi Frequencies")


https://docs.platformio.org/en/latest/boards/espressif32/m5stack-core2.html


https://tinygo.org/docs/reference/microcontrollers/machine/m5stack-core2/


## ChatGPT conversation.
However, as we live in a new ai-age of I thought I ask chat GPT about this.
The AI is mostly hallucinating  https://en.wikipedia.org/wiki/Hallucination_(artificial_intelligence)
[AI questions](AI.md) There might be sparks of information there.



One useful tip I got from the AI
Add internal debug logging to the Bluetooth controller software.

# Set debug level
```
#include "esp_bt.h"
extern int g_bt_plf_log_level;


// Set the logging level to debug
g_bt_plf_log_level = ESP_LOG_DEBUG;

// Release the memory used by the Bluetooth controller with the logging level set
esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
```

Logging enabled in the server program.


```
CO [29345], type 0, ts 29347, ap 0 0, cs 9, met 8
EE elt 0x3ffce2d8, ts 29352, 0
BT BB INTR enabled!
CO First, elt 0x3ffce2d8
CO [29421], type 0, ts 29423, ap 0 0, cs 9, met 8
EE elt 0x3ffce2d8, ts 29428, 0
BT BB INTR enabled!
CO First, elt 0x3ffce2d8
CO [29501], type 0, ts 29503, ap 0 0, cs 9, met 8
EE elt 0x3ffce2d8, ts 29508, 0
BT BB INTR enabled!
CO First, elt 0x3ffce2d8
CO [29577], type 0, ts 29579, ap 0 0, cs 9, met 8
EE elt 0x3ffce2d8, ts 29584, 0
BT BB INTR enabled!
CO First, elt 0x3ffce2d8
CO [29657], type 0, ts 29659, ap 0 0, cs 9, met 8
EE elt 0x3ffce2d8, ts 29664, 0
BT BB INTR enabled!
CO First, elt 0x3ffce2d8
CO [29731], type 0, ts 29733, ap 0 0, cs 9, met 8
EE elt 0x3ffce2d8, ts 29738, 0
BT BB INTR enabled!
CO First, elt 0x3ffce2d8
CO [29807], type 0, ts 29809, ap 0 0, cs 9, met 8
EE elt 0x3ffce2d8, ts 29814, 0
BT BB INTR enabled!
CO First, elt 0x3ffce2d8
CO [29875], type 0, ts 29877, ap 0 0, cs 9, met 8
EE elt 0x3ffce2d8, ts 29882, 0
BT BB INTR enabled!
CO First, elt 0x3ffce2d8
CO [29947], type 0, ts 29949, ap 0 0, cs 9, met 8
EE elt 0x3ffce2d8, ts 29954, 0
BT BB INTR enabled!
CO First, elt 0x3ffce2d8
CO [30019], type 0, ts 30021, ap 0 0, cs 9, met 8
EE elt 0x3ffce2d8, ts 30026, 0
BT BB INTR enabled!
CO First, elt 0x3ffce2d8
CO [30089], type 0, ts 30091, ap 0 0, cs 9, met 8
EE elt 0x3ffce2d8, ts 30096, 0
BT BB INTR enabled!
CO First, elt 0x3ffce2d8
CO [30153], type 0, ts 30155, ap 0 0, cs 9, met 8
EE elt 0x3ffce2d8, ts 30160, 0
BT BB INTR enabled!
CO First, elt 0x3ffce2d8
CO [30227], type 0, ts 30229, ap 0 0, cs 9, met 8
EE elt 0x3ffce2d8, ts 30234, 0
BT BB INTR enabled!
CO First, elt 0x3ffce2d8
CO [30303], type 0, ts 30305, ap 0 0, cs 9, met 8
EE elt 0x3ffce2d8, ts 30310, 0
BT BB INTR enabled!
CO First, elt 0x3ffce2d8
CO [30373], type 0, ts 30375, ap 0 0, cs 9, met 8
EE elt 0x3ffce2d8, ts 30380, 0
BT BB INTR enabled!
CO First, elt 0x3ffce2d8
CO [30451], type 0, ts 30453, ap 0 0, cs 9, met 8
EE elt 0x3ffce2d8, ts 30458, 0
BT BB INTR enabled!
CO First, elt 0x3ffce2d8
CO [30515], type 0, ts 30517, ap 0 0, cs 9, met 8
EE elt 0x3ffce2d8, ts 30522, 0
BT BB INTR enabled!
CO First, elt 0x3ffce2d8
CO [30589], type 0, ts 30591, ap 0 0, cs 9, met 8
EE elt 0x3ffce2d8, ts 30596, 0
BT BB INTR enabled!
CO First, elt 0x3ffce2d8
CO [30653], type 0, ts 30655, ap 0 0, cs 9, met 8
EE elt 0x3ffce2d8, ts 30660, 0
BT BB INTR enabled!
CO First, elt 0x3ffce2d8
CO [30721], type 0, ts 30723, ap 0 0, cs 9, met 8
EE elt 0x3ffce2d8, ts 30728, 0
BT BB INTR enabled!
```


# Separate run 
```
 *** Sent Value: 74 ***
BT BB INTR enabled!
CO First, elt 0x3ffcab3c
CO [74281], type 4, ts 74283, ap 74283 363, cs 0, met 42
EE elt 0x3ffcab3c, ts 74284, 0
BT BB INTR enabled!
CO First, elt 0x3ffcab3c
CO [74329], type 4, ts 74331, ap 74331 362, cs 0, met 42
EE elt 0x3ffcab3c, ts 74332, 0
BT BB INTR enabled!
CO First, elt 0x3ffcab3c
CO [74377], type 4, ts 74379, ap 74379 361, cs 0, met 42
EE elt 0x3ffcab3c, ts 74380, 0
BT BB INTR enabled!
CO First, elt 0x3ffcab3c
CO [74425], type 4, ts 74427, ap 74427 360, cs 0, met 42
EE elt 0x3ffcab3c, ts 74428, 0
BT BB INTR enabled!
CO First, elt 0x3ffcab3c
CO [74473], type 4, ts 74475, ap 74475 359, cs 0, met 42
EE elt 0x3ffcab3c, ts 74476, 0
BT BB INTR enabled!
CO First, elt 0x3ffcab3c
CO [74521], type 4, ts 74523, ap 74523 358, cs 0, met 42
EE elt 0x3ffcab3c, ts 74524, 0
```




## Chat gpt (4.0)
This output seems to be related to the Bluetooth subsystem, specifically the Bluetooth baseband (BT BB) and the Link Manager Protocol (LMP). It is likely generated by enabling debugging for Bluetooth components. Let's break down the log messages to understand their meaning.

CO [29345], type 0, ts 29347, ap 0 0, cs 9, met 8: This log message is related to a Connection event (CO) occurring at timestamp 29345. The event type is 0, and it has an associated timestamp (ts) of 29347. The access point (ap) values are 0 0, the channel selection (cs) is 9, and the metrics (met) value is 8.
EE elt 0x3ffce2d8, ts 29352, 0: This log message is related to an End Event (EE) with an associated element (elt) at memory address 0x3ffce2d8, timestamp 29352, and an additional value of 0.
BT BB INTR enabled!: This log message indicates that the Bluetooth Baseband (BT BB) Interrupt (INTR) is enabled.
CO First, elt 0x3ffce2d8: This log message shows that the first Connection event (CO First) is related to the element (elt) at memory address 0x3ffce2d8.
The rest of the log messages follow a similar pattern, indicating that there are recurring Connection events and End Events, with Bluetooth Baseband Interrupts being enabled.

Please note that this output is specific to the Bluetooth implementation on your system, and the logs may be different for different platforms or SDK versions. The general idea is that these log messages help you understand the events and actions taken by the Bluetooth subsystem during its operation, which can be helpful for debugging and optimization purposes.

## Chat gpt (3.5)
The output you provided appears to be debug messages generated by the Bluetooth Baseband (BB) layer in the ESP32 Bluetooth stack. Here's a brief explanation of the messages:

"CO [29345], type 0, ts 29347, ap 0 0, cs 9, met 8": This message is a debug message generated by the BB layer that provides information about a Bluetooth packet. The message includes the packet type (type 0), the timestamp (ts), the access address (ap), the channel selection algorithm (cs), and the modulation index (met).
"EE elt 0x3ffce2d8, ts 29352, 0": This message is also a debug message generated by the BB layer that provides information about a Bluetooth packet. The message includes the element (elt) that the packet is associated with, the timestamp (ts), and additional information (0).
"BT BB INTR enabled!": This message is a debug message generated by the BB layer that indicates that the Bluetooth BB interrupt has been enabled.
Overall, these messages provide information about the Bluetooth communication between devices, including details about the packets being exchanged and the status of the BB layer. The messages are typically used for debugging and troubleshooting purposes to identify issues with the Bluetooth communication.

## Open assistant
This log output indicates various elements related to the initial configuration and setup of your ESP32 device. To understand the log sequence:

"CO" stands for "Configuration Object." Whenever a new Configuration Object is created or modified in the memory of an ESP32 device, a CO entry appears in the log containing its type and timestamp (ts). These entries are usually grouped together with similar timestamps, forming log blocks, which correspond to individual firmware/device reboots. Each block has a distinctive first line labeled "First," followed by subsequent lines representing further changes to configuration objects.
Timestamp: ts: The number of clock cycles since the last system reset or bootup. This value helps track when significant events occurred during initialization or runtime operations.
"EE" represents "Event Engine." Event Engine entries indicate successful registration of event handlers for specific peripheral features or internal components. They include a timestamp and a brief description ("elt") identifying the registering entity responsible for the event handler (in this case, 0x3ffce2d8 refers to the default Event Engine instance).
Timestamp: ts: Similar to CO, indicating the time of the EE registration.
"BT BB INTR enabled!" confirms that Interrupt request inputs supporting Blueooth Baseband communication have been activated successfully. This statement doesn't contribute to understanding your particular issues, so we won't discuss it any further.
The repeated substring "CO <timestamp>, elt 0x<some hex>, ... BT BB INTR enabled!" hints at the creation of several Configuration Object instances throughout the initial phase of device startup. This process should not cause any trouble unless something went wrong during object generation or binding. Please review this section more carefully to verify if there were any errors reported along these lines.
Overall, nothing in this log snippet suggests severe problems requiring immediate attention; however, please analyze the entire set o


# Useful resources.

https://www.youtube.com/watch?v=q4CxE5P6RUE


https://github.com/espressif/esptool/blob/master/flasher_stub/ld/rom_32.ld


https://randomnerdtutorials.com/esp32-ble-server-client/









