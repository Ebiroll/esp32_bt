#include <Arduino.h>
//#include <M5UnitLCD.h>
//#include <M5UnitOLED.h>
//#include <M5Unified.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include <vector>


#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" 
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


extern int g_bt_plf_log_level;
extern int rxiq_print_en;


// https://github.com/nkolban/ESP32_BLE_Arduino/blob/master/examples/BLE_uart/BLE_uart.ino
// https://www.youtube.com/watch?v=oCMOYS71NIU

//public static final UUID RX_SERVICE_UUID = UUID.fromString("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
//public static final UUID RX_CHAR_UUID = UUID.fromString("6e400002-b5a3-f393-e0a9-e50e24dcca9e");
//public static final UUID TX_CHAR_UUID = UUID.fromString("6e400003-b5a3-f393-e0a9-e50e24dcca9e");
    

// NRF UART Service
//static BLEUUID serviceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
//This service exposes two characteristics: one for transmitting and one for receiving (as seen from the peer).

//RX Characteristic (UUID: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E)
//The peer can send data to the device by writing to the RX Characteristic of the service. ATT Write Request or ATT Write Command can be used. The received data is sent on the UART interface.
//TX Characteristic (UUID: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E)
// If the peer has enabled notifications for the TX Characteristic, the application can send data to the peer as notifications. The application will transmit all data received over UART as notifications.


// Microchip UART Service 
//static BLEUUID serviceUUID("49535343-FE7D-4AE5-8FA9-9FAFD205E455");
// The characteristic of the remote service we are interested in.
//static BLEUUID    charUUID_RX("49535343-8841-43F4-A8D4-ECBE34729BB3");
//static BLEUUID    charUUID_TX("49535343-1E4D-4BD9-BA61-23C647249616");
//Transparent UART TX	49535343-1E4D-4BD9-BA61-23C647249616	Notify, Write, Write without response
//Transparent UART RX	49535343-8841-43F4-A8D4-ECBE34729BB3	Write, Write without response

#include "esp_bt.h"




int scanTime = 5; //In seconds
BLEScan* pBLEScan;

bool debug_print=true;

// Global variables

std::vector<BLEAdvertisedDevice> gAdvertisedDevices;
bool gDisplayUpdate=false;
int times_delayed=0;
bool deviceConnected=false;
bool oldDeviceConnected = false;

static BLECharacteristic * pTXCharecteristics=NULL;
static BLECharacteristic * pRXCharecteristics=NULL;

BLEServer *pServer = NULL;

int txValue=64;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("onConnect");
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("onDisConnect");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      Serial.println("Wr...");
      Serial.print(rxValue.c_str());
    }
    void onRead(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      Serial.println("Rd...");
      Serial.print(rxValue.c_str());
    }
    void onNotify(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      Serial.println("Notify...");
      Serial.print(rxValue.c_str());
    }
};




extern "C" esp_err_t esp_task_wdt_reset(void);

void setup(void)
{
  //auto cfg = M5.config();
  //M5.begin(cfg);
  Serial.begin(115200);

  Serial.println("Setup...");


  esp_task_wdt_reset();

/*
  { /// custom setting
    auto spk_cfg = M5.Speaker.config();
    /// Increasing the sample_rate will improve the sound quality instead of increasing the CPU load.
    spk_cfg.sample_rate = 96000; // default:64000 (64kHz)  e.g. 48000 , 50000 , 80000 , 96000 , 100000 , 128000 , 144000 , 192000 , 200000
    spk_cfg.task_pinned_core = APP_CPU_NUM;
    // spk_cfg.task_priority = configMAX_PRIORITIES - 2;
    spk_cfg.dma_buf_count = 20;
    // spk_cfg.dma_buf_len = 512;
    M5.Speaker.config(spk_cfg);
  }  
*/
  Serial.println("Speaker...");


  //M5.Speaker.begin();

  //gfxSetup(&M5.Display);

  // Initialized from M5.begin()
  //Serial.begin(115200);
  //Serial.begin(9600);

  rxiq_print_en=1;

  BLEDevice::init("ESP32 UART");

  // Set the logging level to debug
  g_bt_plf_log_level = ESP_LOG_VERBOSE;  // ESP_LOG_DEBUG;

  rxiq_print_en=1;


// Release the memory used by the Bluetooth controller with the logging level set
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);



  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);  
                      

  pRXCharecteristics = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );


  pRXCharecteristics->setCallbacks(new MyCallbacks());

  if (debug_print) Serial.println("Descriptor...");



  pTXCharecteristics = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );


  pTXCharecteristics->addDescriptor(new BLE2902());

  if (debug_print) Serial.println("Start...");



  // No need for our own callbacks 
  // pTXCharecteristics->setCallbacks(new MyCallbacks());

  pService->start();

  pServer->getAdvertising()->start();


  //pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);

  Serial.println("Waiting a client connection to notify...");
}




static void notifyCallbackRX(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    if (debug_print) Serial.print("Notify callback for RX characteristic ");
    if (debug_print) Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    if (debug_print) Serial.print(" of data length ");
    if (debug_print) Serial.println(length);
    if (debug_print) Serial.print("type: ");
}

static void notifyCallbackTX(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    if (debug_print) Serial.print("Notify callback for TX characteristic ");
    if (debug_print) Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    if (debug_print) Serial.print(" of data length ");
    if (debug_print) Serial.println(length);

    //if (debug_print) Serial.println((char*)pData);
}




void loop(void)
{


  if(Serial.available()) {
    String input = Serial.readString();

    if (pTXCharecteristics) {
      pTXCharecteristics->setValue((uint8_t *)input.c_str(),input.length()); 
      pTXCharecteristics->notify(); 
    }
  } else {

  }



  //if (times_delayed==0) {
  //  if (debug_print) Serial.print("Loop:\n");
  //}


  
  times_delayed++;  
  if (times_delayed>2000) {
    if (deviceConnected) { 
      Serial.printf("*** Sent Value: %d ***\n", 
      txValue);
      pTXCharecteristics->setValue((uint8_t *)&txValue,1); 
      pTXCharecteristics->notify(); 
      txValue++; 
      if ( txValue>110 ) txValue=64;
    }
    times_delayed=0;
  } 

  vTaskDelay(1);

  

   // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
		  // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}

#if !defined ( ARDUINO )
extern "C" {
  void loopTask(void*)
  {
    setup();
    for (;;) {
      loop();
    }
    vTaskDelete(NULL);
  }

  void app_main()
  {
    xTaskCreatePinnedToCore(loopTask, "loopTask", 8192, NULL, 1, NULL, 1);
  }
}
#endif