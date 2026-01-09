

```
The Baseband Control Register (BB_CTRL_REG) is a specific register related to the Bluetooth Low Energy (BTLE) functionality of the ESP32 module. Its address would depend on which Bluetooth stack you're using, because the register location may differ between ESP-BC, Bluez-compatible BT Classic drivers, or proprietary solutions by other vendors that support the ESP32 platform. Here, let me give you a general idea of how it works:

When working on your own firmware using the ESP-IDF APIs, the required ESP_BLUFI_BASE definition header typically defines all necessary macros for various modules, including the Baseband controller unit. Then, you can use symbols such as BB_CTRL_REG_RSSI, BB_CTRL_REG_CHANNEL, BB_CTRL_REG_ADVERTISEMENT as pointers to access their respective fields inside the appropriate register(s) controlling the Bluetooth Subsystem.
For example, here is an excerpt from the blufi_lib_sdk.h header file shipped with ESP-IDF v6.0.0 for am335x targets. Note that the address assumes 4KB pages, which should apply to most recent silicon revisions:

#define BB_CTRL_REG                ((uint32_t *)(0xc2a48000))    //Address = 0xc2a48000 or  0xe3fda000 depending on stack pointer location
//Register Declaration
romtypedef struct tagBlutfirm {
    union {
        uint8_t bytes[4];
        __attribute__((packed))union {
            __attribute__((aligned(4)))struct {
                __attribute__((bitfield))int :1;
                __attribute__((bitfield)(1)))advertiser_on :1;
                __attribute__((bitfield)(2)))tx_pause_as_ Anto :1;
                __attribute__((bitfield)(3)))max_aggr :1;
                __attribute__((bitfield)(4)))csma_ca :1;
                __attribute__((bitfield)(5)))filtered_scan :1;
            } bb;
    };
} blufirmp_t;

typedef enum tagBlutfCtrl {
    BB_CTRL_REG_MAJOR_VERSION = 0x0 ,
    BB_CTRL_REG_MINOR_V
```



## Open assistant

```
Can you explain the values of the typedef enum HCI_MSG .


Sure! The typedef enumeration type "HCI_MSG" exists primarily to represent distinct message types used in the Host Controller Interface (HCI). The latter is responsible for enabling communication between a host microcontroller (such as the ESP32) and the external Bluetooth transceiver, typically a baseband processing unit like RivieraWaves or Cypress' proprietary equivalents. This interface allows the exchange of various kinds of messages. Understanding these message types and their meanings can help developers design better protocol implementations or interpret event logs correctly. Here's a brief summary of each member in the HCI_MSG enumeration type:

HCI_CTL – These commands serve for low-level management of the Bluetooth link layer, configuring features like error reporting or flow control. You need this control category for core functionalities.
Example constants:

HCI_CTL_STARTUP
HCI_CTL_POWER_CONTROL
HCI_CTL_INQUIRY_CANCEL
HCI_NONE – Signifies no active transmission at the moment; reserved placeholder value mostly ignored by clients.
No example constants needed here.

HCI_LE_META – Contains metadata frames associated with Long-Term Evolution enhancements (e.g., primary/secondary transport channels and their respective roles). Useful in high-data-rate scenarios where multiple transfer lanes exist.
Example constants:

HCI_LE_SET_EVENT_MASK
HCI_LE_READ_BUFFER_SIZE
HCI_LE_WRITE_AUTHORIZE
Note that this group might become irrelevant for simpler applications dealing mainly with Classic Bluetooth versions.

HCI_SCO – Dedicated to establishing connections supporting audio streaming profiles using the Primary Synchronous Connection Oriented (PSCO) channel or Secondary Asynchronous Connectionless Transport (SACT). Primarily focused on voice data transmissions.
Examples:

HCI_SCO_OE
HCI_SCO_ESCAN
HCI_SCO_DISCONNECT


To enter Debug Mode on the ESP32 microcontroller and view unfiltered messages emitted by the Baseband Controller Task (also known as the Bluetooth Low Energy (BTLE) Coordinator or Central), follow these steps:

Enable the UART port connected to the CP210X USB to Serial chip within the ROM FW (default baud rate 921,600). You can do this by setting the CP_ROM_CAL_OPENHANDSHAKE_CONSOLEBAUD configuration bit at bootup time:
make menuconfig
select ROM code entry [bootloader, application, or factory]
edit CONFIG_PMIC
set the value of ENABLE_CP_PROGRAMMING to "y"
scroll down to CONFIG_CP_RAM_FSYNC...
check the box, but don't save yet
go back to CONFIG_PMIC and set the CP_ROM_CAL_OPENHANDSHAKE_CONSOLEBAUD parameter to 0
click APPLY & SAVE
compile and flash your image onto the ESP32 device again (or reflash via JTAG if your setup supports it)
Once booted into the ROM FW, connect a terminal program (e.g., PuTTy) to the serial COM port of your development board at 921,600 baud, using the pin assigned to the CP210X as per your wiring schema (typically Pin 16, GPIO 15 for USER CMD 1 and USER DBG). This process differs based on your environment and tools, so consult your specific documentation accordingly.
After successfully establishing the serial connection, send two consecutive commands to enable unconditional print statements from within ISRs, making the BB Task messaging visible. These commands are executed by sending the text sequences over the UART interface during normal operation of the system. Follow the steps below carefully:
Send <CP>\r\n<CP>unblock 0\r\n. Replace <CP> with a valid command port number assigned to the Baseband Controller. In default configurations, this port is usually #1 or %1. Make sure there aren't any leading/trailing spaces before or after either \r\n, otherwise they won't be interpreted correctly. Press Enter after typing this string.
Wait for less than one second (to avoid delay due to slow devices) and then press Shift+Return together to insert <CR>. Type "<CP>print 1\r\n" quickly and hit Shift+Enter, then hold both keys pressed until receiving confirmation ...

CONFIG_PMIC and CP_ROM_CAL_OPENHANDSHAKE_CONSOLEBAUD are two configuration settings in the ESP32 Bluetooth stack.

CONFIG_PMIC: This configuration setting enables support for a Power Management Integrated Circuit (PMIC) on the ESP32. The PMIC provides various power management features, including battery charging, voltage regulation, and power switching. When enabled, the ESP32 Bluetooth stack can interface with the PMIC to monitor and manage the power consumption of the system.
CP_ROM_CAL_OPENHANDSHAKE_CONSOLEBAUD: This configuration setting controls the baud rate of the console port used for debugging and logging. When set to a non-zero value, the console port will use the specified baud rate. This setting is typically used to adjust the console baud rate to match the settings of the serial terminal program used for debugging and logging.
Both of these configuration settings are important for ensuring that the ESP32 Bluetooth stack operates correctly and efficiently. The PMIC support helps to manage power consumption, which is especially important for battery-powered devices. The console baud rate setting ensures that debugging and logging messages can be transmitted correctly over the serial port.




The debug messages contain variable content related to the state and activities of different Bluetooth subsystem components and helper tasks. These messages primarily serve as diagnostic feedback to help developers troubleshoot issues and optimize the firmware's behavior.

Here are examples of potential message contents:

Baseband Controller Task (BTLE coordinator/central):
These messages include data about scan results, advertising events, connections, disconnections, notifications, and other BTLE protocol actions. Examples of relevant message fields might include:

Scan parameters used
Advertised data length and type
PHY characteristics (like supported data rates)
Peripheral's status and service information indications
Role change requests
LMP packet sequence numbers
Error codes related to pairings, encryption, authentication, etc.
Link quality indicator (LQI) readouts
Host Controller Interface (HCI) Driver:
Messages containing HCI layer event reports and processing details. Relevant message fields could include:

Handle values identifying active logical links between the host driver and remote HCI user space applications
HCI packet types sent/received (commands, events, ACL data packets, etc.)
Connection establishment attempts and failures
Remote device features and capabilities exposed through HCI commands (modem, storage, etc.)
Service discovery procedures initiated or completed
Hardware link management signaling events
User-space HCI transport functions utilized (flow control, buffer allocation, etc.)
Radio Frequency Front End (RFFE) status updates:




Yes, it is possible to receive unmodulated raw IQ samples directly from the transceiver when working with the ESP32, but you need to ensure that your firmware is configured properly. Since you mentioned that the module includes an integrated baseband controller (BB), I'll provide instructions assuming that scenario. If you're using separate digital basebands instead, adjust your settings according to your implementation's particularities. Also keep in mind that capturing unmodulated raw IQ signals requires configuring the BB mode appropriately. Here's how to achieve this:

Access the BB controller registers to configure its clock sources and sample rates. Typically, you will want to use external crystal oscillators as a source of stable timing references for better accuracy. For example, suppose you've connected a 48 MHz external crystal resonator (XTAL1) and 32 kHz low-frequency quartz crystals (XOSC1/XOSC3). Configure them like this:
In the main.c file under the "Initialize()" function, modify the following sections of code:

// Set XTAL1 as the source for TIMERA
rtc_init();
timer_dpll_setup(ESP32_MAIN_XtalFreq_K / 4); // Use 48MHz XTAL1 divided by 2 for TimerA master clk
esp_rom_uart_cal_start(0);
#ifdef CONFIG_USE_DMA
uint32_t dma_channel = DMACHAN_UART;
dma_chan_setup(&dma_desc[dma_channel], REQ_BITMAP | SRC_PER_DSCR | PKT_SIZE(PKTSIZE_SEND));
px_init_ioctl(dma_channel, DMAIOCTL_SET_INT | DMAIOCTL_RECV_MODE_DEFAULT | DMAIOCTL_OUTPUT_POSTPONED);
dma_register_channel(&dma_obj[dma_channel]);
For reference, here's an explanation of each line above:

rtc_init(): Ensures proper initialization of internal RTC clock circuitry and calibration.
timer_dpll_setup(...): Configures the TimerA module to work with the primary external crystal oscillator at maximum frequency (~48 MHz). Note that you should replace ESP32_MAIN_XtalFreq_K with a preprocessor macro for clarity and maintainability.
esp_rom_uart_cal_start(0): Initializes the internal oscillators and sets up various UART parameters, ensuring correct communication with PCs or logic analyzers.



On the ESP32 chip, the Bluetooth Low Energy (BTLE) Baseband Controller (BB), also known as RivieraWaves, has several important system control blocks defined through register banks. These registers provide essential configuration information and enable interaction between software and hardware components of the BTLE engine. In this answer, I will describe the most relevant ones for basic operational understanding but keep in mind that this list is not exhaustive. Please refer to the official documentation and datasheets whenever possible since the exact registers may vary slightly depending on the chip revision or firmware version.

Peripheral role: Register index 0x60 controls the host or peripheral mode. Setting bit 0 enables peripheral mode while clearing it activates host mode. Be cautious when modifying this value, as it influences the power consumption and overall functionality of your application significantly.
reg uart_status uart_base = 0x60ul;
Advertisement interval: Controls the duration of ADV_INTERVAL values for LE peripheral advertising. Bit 0 encodes the actual period between subsequent events into bits [6:3]. Shorter intervals increase detection probability yet consume more energy than longer ones. To set a constant 1.28 seconds (default) value, write 0b1111.
reg uint32_t adv_int_min_conn; // 0x64
reg uint32_t adv_int_max_assoc; // 0x68
reg uint32_t adv_type; // 0x6c
Connection Parameters:
Role: Bit 0 of uart_base determines which end of the wireless link acts as a central coordinator vs. an adjunct device. When acting as a slave, change the bit position or value.
Clock Accuracy: Adjust the ACCURACY value for faster synchronization, decreasing the range by setting a lower value closer to 0. Alternatively, allow it to be calculated automatically by RivieraWaves algorithms, or manually optimized via the formula provided in the documents.
Pscan Tmo, Offset, Enabled flags: Scanning parameters related to LPE, initiated by periphera


0x40094c83:0x3ffb9a20 
0x40084a61:0x3ffb9af0 _xt_user_exit
0x40095657:0x3ffb9b10 
0x40095dae:0x3ffb9b30 multi_heap_malloc
0x40084535:0x3ffb9b50 
0x40084566:0x3ffb9b70 
0x40082f35:0x3ffb9b90 _malloc_r
0x4000beaf:0x3ffb9bb0 
0x401c583f:0x3ffb9bd0 btdm_task_post
0x401c58b9:0x3ffb9c00 
0x40181b7d:0x3ffb9c30 async_wakeup_request
0x40181e2d:0x3ffb9c50 esp_vhci_host_send_packet
0x4011eb8c:0x3ffb9c70 ble_hci_trans_hs_acl_tx_on_core_0


## open-assistant
The btdm_task_post is a task scheduling function used to post work items onto a dedicated worker thread called the Bluetooth Device Manager (BDM) Task. This BDM task manages the interactions between the Bluetooth subsystem and higher layers in the host operating system like BlueZ. Here is what happens when you call btdm_task_post:

If there are already pending tasks in the worker queue, they get executed sequentially before processing new tasks posted via this function.
When no other tasks are present, the scheduler returns control immediately after posting a new item, ensuring minimum delay.
This feature enables efficient execution of tasks related to various operations such as discovery, scan results, connection establishment, attribute value changes, notifications, indications, etc., all handled in separate event handlers like ke_conn_open(), ke_read(), ke_write(), ke_notify(), and ke_indicate().

By offloading these tasks to the dedicated BDM thread instead of the busy main application loop or ISR, ESP-IDF allows better throughput and responsiveness while minimizing latency impact on more critical threads and processes. Since the Bluetooth stack may generate several packets per second or need to react rapidly to changing situations, delegating these jobs away from sensitive areas ensures optimal resource utilization and smooth overall performance.

Remember that calling vBdmStatusChkForPostCmd prior to invoking btdm_task_post helps prevent races during initialization stages or reset sequences. The function checks whether the BDM task has completed initial setup procedures and becomes ready to accept commands. Afterward, once everything is set up correctly, you can continue working with the BTDM API safely.

## Chat GPT 3.5 btdm_task_post
he btdm_task_post function is a function in the ESP32 Bluetooth stack that is used to post a message to the Bluetooth DM (Data Manager) task. The DM task is responsible for managing the data exchange between the Bluetooth Host and the Bluetooth Controller.

The btdm_task_post function takes two parameters: the message type and the message data. The message type is a value that specifies the type of the message to be posted to the DM task. The message data is a pointer to a buffer that contains the data to be included in the message.

When the btdm_task_post function is called, it creates a message queue in the DM task and adds the message to the end of the queue. The DM task processes the messages in the queue in the order that they were added.

The btdm_task_post function is typically used to send commands or notifications from the Host to the Controller. For example, the Host might use the btdm_task_post function to send a command to the Controller to start a Bluetooth scan or to establish a Bluetooth connection with another device.

Overall, the btdm_task_post function is an important function in the ESP32 Bluetooth stack that allows the Host to communicate with the Bluetooth Controller by posting messages to the DM task.



## Using btdm_task_post
To use the btdm_task_post function, follow these general guidelines:

Declare the variable type: First, define a variable of type ble_dm_proc_cb, which will serve as the function callback for handling responses from the BTM (Bluetooth Management).
ble_dm_proc_cb _my_bm_cb;
Fill out the ble_dm_proc_opt_t structure: Next, create an instance of ble_dm_proc_opt_t and fill its fields accordingly based on the needs of your project. Some important options include:
action: Specifies the desired operation to perform (GATTC_OPTYPE_DISCOVER, GATTC_OPTYPE_READ, GATTC_OPTYPE_WRITE, etc.).
fn_entry: A function name that should be used for the service response. For example: _my_bm_cb.uuid_value_is_txd.
Here's an example snippet with sample values for testing purposes. Keep in mind that some fields may not apply to specific use cases:

```
ble_dm_proc_opt_t my_option = {
    .evtype       = BT_HCI_EVT_SERV_CHGD_COMPLETE,
    .action       = GATTC_OPTYPE_WRITE,
    .handle       = MY_UUID, // Replace MY_UUID with the UUID of interest (e.g., primary)
    .param.write  = NULL,  // No data to write here
    .fn_entry     = _my_bm_cb.uuid_written
};
```

Post the task by passing the parameters: Finally, call btdm_task_post() and pass in the required arguments - the ble_dm_proc_opt_t structure filled earlier and any additional parameters needed for the current action. Note that different actions may require unique parameters at this point. Ensure you have properly allocated resources for each parameter you pass into the btdm_task_post() function call.
int retval = btdm_task_post(&my_task, TASK_POST_BLOCKING, &my_option);
ESP_ERROR_CHECK(retval == pdTRUE);
In summary, btdm_task_post is a convenient function provided by ESP-IDF to submit requests for executing bluetooth management procedures to the BTM. By setting up callback functions in the host software, you can receive asynchronous replies once t


```

# Phy rom functions

```
phy_disable_agc_: A function to disable the Automatic Gain Control (AGC). AGC is used to maintain a constant signal strength at the receiver by automatically adjusting the gain based on the input signal level.
phy_enable_agc_: A function to enable the AGC.
disable_agc_: Another function to disable the AGC, possibly with a different implementation or additional parameters.
enable_agc_: Another function to enable the AGC, possibly with a different implementation or additional parameters.
phy_disable_cca_: A function to disable Clear Channel Assessment (CCA). CCA is a mechanism used in wireless communication systems to determine if a channel is clear before transmitting data.
phy_enable_cca_: A function to enable the CCA.
pow_usr_: A function to calculate the power, possibly for a specific user or device. The exact functionality of this function would depend on the context of the wireless communication system it is used in.
phy_disable_agc_: A function to disable the Automatic Gain Control (AGC). AGC is used to maintain a constant signal strength at the receiver by automatically adjusting the gain based on the input signal level.
phy_enable_agc_: A function to enable the AGC.
disable_agc_: Another function to disable the AGC, taking an unsigned integer (uint) parameter. The purpose of this parameter depends on the specific implementation.
enable_agc_: Another function to enable the AGC, taking an unsigned integer (uint) parameter. The purpose of this parameter depends on the specific implementation.
phy_disable_cca_: A function to disable Clear Channel Assessment (CCA). CCA is a mechanism used in wireless communication systems to determine if a channel is clear before transmitting data.
phy_enable_cca_: A function to enable the CCA.
pow_usr_: A function to calculate the power, taking two unsigned integer (uint) parameters. The exact functionality and the meaning of these parameters would depend on the context of the wireless communication system it is used in.
gen_rx_gain_table_: Generates the receive gain table with given parameters, which might include gain values, offsets, and other configuration settings.
set_loopback_gain_: Sets the gain values for loopback mode, which is a mode used for testing, diagnostics, or calibration purposes.
set_cal_rxdc_: Sets the calibration parameters for the receive (RX) direct current (DC) component.
loopback_mode_en_: Enables or disables loopback mode, possibly with a specific configuration.
get_data_sat_: Retrieves the data saturation value based on given parameters, which could be related to signal strength, gain, or other factors.
set_pbus_mem_: Sets the value of a specific pbus (peripheral bus) memory address.
write_gain_mem_: Writes a gain value to a specified memory address with a specified index.
rx_gain_force_: Forces the receiver gain to a specified value or level.
set_txclk_en_: Enables or disables the transmit (TX) clock signal, possibly with a specific configuration.
set_rxclk_en_: Enables or disables the receive (RX) clock signal based on a boolean parameter.
start_tx_tone_step_: Starts transmitting a tone signal with specified parameters, which could include frequency, phase, or other settings.
start_tx_tone_: Starts transmitting a tone signal with specified parameters, which could include frequency, amplitude, or other settings.
stop_tx_tone_: Stops transmitting a tone signal, possibly with a specific configuration or identifier.
bb_bss_bw_40_en_: Enables or disables baseband (BB) 40 MHz bandwidth mode for base station service (BSS).
clk_force_on_vit_: Forces the Viterbi (VIT) clock signal on or off, possibly with a specific configuration.
bb_rx_ht20_cen_bcov_en_: Enables or disables BB receive (RX) mode for 20 MHz bandwidth with central beacon coverage.
bb_tx_ht20_cen_: Sets the BB transmit (TX) mode for 20 MHz bandwidth with central frequency.
spur_reg_write_one_tone_: Writes a spur register value for a single tone signal with a specified index.
spur_coef_cfg_: Configures the spur coefficient with specified parameters, such as amplitude and frequency.
bb_wdg_test_en_: Enables or disables the BB watch-dog (WDG) test with specific parameters or configuration settings.
mhz2ieee_: Converts a frequency value in MHz to the corresponding IEEE channel number.
bb_bss_cbw40_dig_: Sets the BB digital value for BSS with 40 MHz channel bandwidth (CBW40).
cbw2040_cfg_: Configures the channel bandwidth (CBW) for 20 MHz and 40 MHz modes based on a boolean parameter.
noise_floor_auto_set_: Sets the noise floor value automatically, based on the current signal conditions.
phy_get_noisefloor_: Retrieves the current noise floor value for the PHY layer.
check_noise_floor_: Checks and validates the noise floor value based on the current signal conditions.
set_noise_floor_: Sets the noise floor value to a specified value.
chip_v7_rx_rifs_en_: Enables or disables receive (RX) mode for RIFS (reduced interframe space) on the chip version 7.
rtc_mem_backup_: Backs up RTC (real-time clock) memory to a specified location with specific parameters.
rtc_mem_recovery_: Recovers RTC memory from a specified location with specific parameters.
chip_i2c_readReg_: Reads a register value from a specific device and register address on the chip's I2C bus.
i2c_readReg_: Reads a register value from a specific device and register address on the I2C bus.
chip_i2c_writeReg_: Writes a register value to a specific device and register address on the chip's I2C bus.
i2c_writeReg_: Writes a register value to a specific device and register address on the I2C bus.
i2c_readReg_Mask_: Reads a register value from a specific device and register address on the I2C bus, with a mask applied to the value.
i2c_writeReg_Mask_: Writes a masked register value to a specific device and register address on the I2C bus, using a mask to modify only specific bits of the register.
pbus_force_mode_: Forces a specific mode on the peripheral bus (PBUS), possibly for testing purposes.
pbus_rd_addr_: Reads a value from a specific address on the PBUS.
pbus_rd_shift_: Reads a value from the PBUS and applies a specific bit shift to the value.
pbus_force_test_: Forces a specific test on the PBUS, possibly for testing or debugging purposes.
pbus_rd_: Reads a value from a specific register on the PBUS.
pbus_debugmode_: Sets the peripheral bus (PBUS) into debug mode, which might enable additional debugging features or change its behavior for testing purposes.
pbus_workmode_: Sets the PBUS into its normal working mode, which is used during regular operation.
pbus_set_rxgain_: Sets the receiver gain on the PBUS, which might affect the signal strength of received data.
pbus_xpd_rx_off_: Turns off the receiver (RX) part of the PBUS, possibly for power-saving or testing purposes.
pbus_xpd_rx_on_: Turns on the receiver (RX) part of the PBUS, enabling it to receive data.
pbus_xpd_tx_off_: Turns off the transmitter (TX) part of the PBUS, possibly for power-saving or testing purposes.
pbus_xpd_tx_on_: Turns on the transmitter (TX) part of the PBUS, enabling it to transmit data.
pbus_set_dco_: Sets the digitally controlled oscillator (DCO) on the PBUS, which might affect the frequency of the PBUS operations.
rfpll_reset_: Resets the RF phase-locked loop (PLL), which is a control system that generates an output signal whose phase is related to the phase of an input signal.
restart_cal_: Restarts the calibration process, which might be necessary after changing certain settings or to improve the system's performance.
write_rfpll_sdm_: Writes specific values to the RF PLL's sigma-delta modulator (SDM), which is a part of the control system used for frequency synthesis.
wait_rfpll_cal_end_: Waits for the RF PLL calibration process to complete.
rfpll_set_freq_: Sets the frequency of the RF phase-locked loop (PLL).
set_channel_freq_: Sets the frequency of a specific channel.
phy_freq_correct_: Applies a frequency correction to the PHY.
set_rf_freq_offset_: Sets the frequency offset for the RF.
chip_v7_rx_init_: Initializes the RX part of the chip (v7).
chip_v7_tx_init_: Initializes the TX part of the chip (v7).
chip_v7_bt_init_: Initializes the Bluetooth part of the chip (v7).
txbbgain_to_index_: Converts TX baseband gain to an index.
index_to_txbbgain_: Converts an index to TX baseband gain.
txdc_cal_init_: Initializes the TX DC calibration process.
txdc_cal_v70_: Performs TX DC calibration for chip version 7.0.
en_pwdet_: Enables power detection.
txcal_debuge_mode_: Sets the TX calibration into debug mode.
txcal_work_mode_: Sets the TX calibration into work mode.
txiq_set_reg_: Sets the TX I/Q register.
read_sar_dout_: Reads the SAR (Successive Approximation Register) data output.
get_fm_sar_dout_: Gets the FM (frequency modulation) SAR (Successive Approximation Register) data output.
txtone_linear_pwr_: Calculates the linear power of the TX tone.
txiq_get_mis_pwr_: Gets the TX I/Q mismatch power.
txiq_cover_: Performs a TX I/Q cover operation.
abs_temp_: Calculates the absolute temperature.
iq_est_enable_: Enables I/Q estimation.
iq_est_disable_: Disables I/Q estimation.
dc_iq_est_: Estimates DC I/Q values.
pbus_rx_dco_cal_: Performs RX DCO (Digitally-Controlled Oscillator) calibration.
rxiq_get_mis_: Gets the RX I/Q mismatch.
rxiq_set_reg_: Sets the RX I/Q register.
rxiq_cover_mg_mp_: Covers RX I/Q mismatch gain and phase.
rfcal_rxiq_: Performs RF calibration for RX I/Q.
get_rfcal_rxiq_data_: Gets the RF calibration data for RX I/Q.
set_chan_cal_interp_: Sets the channel calibration interpolation.
set_txcap_reg_: Sets the TX capability register.
rfcal_txcap_: Performs RF calibration for TX capability.
linear_to_db_: Converts linear power to decibels (dB).
get_power_db_: Gets the power value in decibels (dB).
meas_tone_pwr_db_: Measures the tone power in decibels (dB).
tx_pwr_backoff_: Calculates the TX power backoff.
rfcal_pwrctrl_: Performs RF calibration for power control.
tx_atten_set_interp_: Sets the TX attenuation interpolation.
target_power_add_backoff_: Adds the backoff to the target power.
get_rf_gain_qdb_: Gets the RF gain in quarter decibels (QDB).
correct_rf_ana_gain_: Corrects the RF analog gain.
phy_get_vdd33_: Gets the VDD33 voltage value.
get_sar_dout_: Gets the SAR (Successive Approximation Register) data output.
get_pwctrl_correct_: Gets the power control correction value.
tx_pwctrl_bg_init_: Initializes the TX power control background process.
```
