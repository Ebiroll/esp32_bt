#include <Arduino.h>
#include <M5UnitLCD.h>
#include <M5UnitOLED.h>
#include <M5Unified.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <vector>
bool authorized=false;


int scanTime = 5; //In seconds
BLEScan* pBLEScan;

bool debug_print=false;

// Global variables

std::vector<std::string> gFoundDevices;
std::vector<BLEAdvertisedDevice> gAdvertisedDevices;
int gSelectedRow=0;
bool gDisplayUpdate=false;
int times_delayed=0;
bool connected=false;
static BLERemoteCharacteristic* pRemoteCharacteristic=NULL;
static BLERemoteCharacteristic* pRemoteRXCharecteristics=NULL; 
unsigned short int Calc_CRC (unsigned char *ptr, unsigned short len);



typedef struct Authorization_Answer_Type {
    uint8_t answer;  // enum authorization_answer
    uint8_t delay;   // When a code is rejected or in delay, remaining delay is returned in seconds.
} Authorization_Answer_Type;


unsigned char gFullBuffer[256];
int gFullBufferLen=0;
int gMessLen=0;


bool receive_partial_Frame(char *data,int len) {
  int i=0; 

 if  (data[i++]==0xAA) {
    gFullBufferLen=data[i++]+9;
    gMessLen=0;   
 }
 if (gFullBufferLen>0) {
      memcpy(&gFullBuffer[gMessLen],data,len);
      gMessLen+=len;
      if (gMessLen>=gFullBufferLen) {
        return true;
      }
  }

  return false;
}


// 
// 0x00: Code accepted
bool receive_Auth_Frame(char *data,int len) {
 int i=0;
 if (debug_print) if (debug_print) Serial.print("Check Auth:"); 
 if  (data[i++]==0xAA) {
    int len2=data[i++];
    if (len2==len-9) {
      if (debug_print) if (debug_print) Serial.print("check crc\n");
      if (debug_print) if (debug_print) Serial.print(data[len-1],HEX);
      if (debug_print) if (debug_print) Serial.print(data[len-2],HEX);
      if (debug_print) if (debug_print) Serial.print("--\n");
      int crc=Calc_CRC((unsigned char *)&data[1],len-3);
      if (debug_print) if (debug_print) Serial.print(crc,HEX);
      // From, to
      i+=4;
      if (data[len-1]==(crc&0xff) && data[len-2]==crc/256) {
          if (data[i++]==29) {  // kNet_BT_Authorize
            Authorization_Answer_Type *answer=(Authorization_Answer_Type *)&data[i];
            if (debug_print) if (debug_print) Serial.print("Authorization_Answer_Type answer=");
            if (debug_print) if (debug_print) Serial.print(answer->answer);
            if (debug_print) if (debug_print) Serial.print(" delay=");
            if (debug_print) if (debug_print) Serial.println(answer->delay);
            if (answer->answer==0) {
              authorized=true;
            }
            return true;
          }
      }
    }
  }
  return false;
}

bool receive_Console_Frame(char *data,int len) {
 int i=0; 
//if (debug_print) if (debug_print) Serial.print("check console frame");

 if  (data[i++]==0xAA) {
    int len2=data[i++];
    if (len2==len-9) {
      int crc=Calc_CRC((unsigned char *)&data[1],len-3);
      //if (debug_print) if (debug_print) Serial.print("check crc");
      i+=4;
      if (data[len-1]==(crc&0xff) && data[len-2]==crc/256) {
          if (data[i++]==0x14) {  // 
            if (debug_print) if (debug_print) Serial.print("Console Frame:");
            for (int j=0;j<len2-1;j++) {
              Serial.print(data[i+j]);
              //if (debug_print) if (debug_print) Serial.print(" ");
            }
            if (debug_print) if (debug_print) Serial.println();
            return true;
          }
      }
    }
 }
  return false;
}


// MyAdvertisedDeviceCallbacks class
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      if (advertisedDevice.getAddressType()==0) {
        gAdvertisedDevices.push_back(advertisedDevice);
        if (debug_print) if (debug_print) Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
        gFoundDevices.push_back(advertisedDevice.toString().c_str());
        gDisplayUpdate = true;
      } else {
        if (debug_print) if (debug_print) Serial.printf("Device: %s \n", advertisedDevice.toString().c_str());
      }
    }
};


void gfxSetup(LGFX_Device* gfx)
{
  if (gfx == nullptr) { return; }
  if (gfx->width() < gfx->height())
  {
    gfx->setRotation(gfx->getRotation()^1);
  }
  gfx->setFont(&fonts::lgfxJapanGothic_12);
  gfx->setEpdMode(epd_mode_t::epd_fastest);
  gfx->setCursor(0, 8);
  gfx->print("BT GDB : ");
  //gfx->println(bt_device_name);
  gfx->setTextWrap(false);
  gfx->fillRect(0, 6, gfx->width(), 2, TFT_BLACK);

}

void gfxLoop(LGFX_Device* gfx)
{
  if (gfx == nullptr) { return; }


  if (times_delayed==100 || gDisplayUpdate) {
    gfx->startWrite();

    for(int i = 0; i < gFoundDevices.size(); i++)
    {
      gfx->setCursor(4, 8+  11 * i);
      // Clead the line
      gfx->fillRect(0, 8 + 11*i, gfx->width(), 12, gfx->getBaseColor());      
      if (gSelectedRow==i) 
      {
        gfx->fillRect(0, 8 + 11*i, gfx->width(), 12, 0xBBBBBB);            
      }
      gfx->print(gFoundDevices[i].c_str());
      gfx->print("  "); // Garbage data removal when UTF8 characters are broken in the

    }
    gDisplayUpdate=false;
    gfx->display();
    gfx->endWrite();
  }

#if 0
    tx = gfx->getCursorX();
    gfx->width();
    taskYIELD();


  if (fft_enabled && !gfx->displayBusy())
  {
          gfx->writeFastVLine(px, i * 3, 2, TFT_WHITE);
      }
      gfx->display();

      // draw FFT level meter
 
      uint32_t bar_color[2] = { 0x000033u, 0x99AAFFu };

          gfx->fillRect(x, y, bw - 1, py - y, bar_color[(y < py)]);
          gfx->writeFastHLine(x, py - 1, bw - 1, bgcolor(gfx, py - 1));
          gfx->writeFastHLine(x, py, bw - 1, TFT_WHITE);

          gfx->setAddrWindow(i, y, 1, h);
          gfx->writeColor(bg, 1);
      }
      gfx->display();
      gfx->endWrite();
    }
  }
#endif
}

extern "C" esp_err_t esp_task_wdt_reset(void);

void setup(void)
{


  auto cfg = M5.config();

  // If you want to play sound from ModuleDisplay, write this
//  cfg.external_speaker.module_display = true;

  // If you want to play sound from ModuleRCA, write this
//  cfg.external_speaker.module_rca     = true;

  // If you want to play sound from HAT Speaker, write this
  //cfg.external_speaker.hat_spk        = false;

  // If you want to play sound from ATOMIC Speaker, write this
  //cfg.external_speaker.atomic_spk     = false;

  M5.begin(cfg);

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

  M5.Speaker.begin();

  gfxSetup(&M5.Display);

  //Serial.begin(115200);
  //Serial.begin(9600);
  if (debug_print) if (debug_print) Serial.println("Scanning...");


  BLEDevice::init("ESP32");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); //active scan uses more power, but get results faster
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value
  

}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }
 
  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

static BLEUUID serviceUUID("49535343-FE7D-4AE5-8FA9-9FAFD205E455");
// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID_RX("49535343-8841-43F4-A8D4-ECBE34729BB3");
static BLEUUID    charUUID_TX("49535343-1E4D-4BD9-BA61-23C647249616");


//Transparent UART TX	49535343-1E4D-4BD9-BA61-23C647249616	Notify, Write, Write without response
//Transparent UART RX	49535343-8841-43F4-A8D4-ECBE34729BB3	Write, Write without response


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
    if (debug_print) Serial.println(gFullBuffer[6], HEX);

    if (receive_partial_Frame((char *)pData, length)) {
      if (receive_Auth_Frame((char *)gFullBuffer, gMessLen))
      {
        if (debug_print) Serial.println("Auth Frame");
      }
      else      
      {
        if  (gFullBuffer[0]==0xAA) {
                  for (int j=0;j<gMessLen;j++) {
                    Serial.print(gFullBuffer[j]);
                  }
        }
      }
    }
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
    if (debug_print) Serial.print("type: ");
    if (debug_print) Serial.println(gFullBuffer[6], HEX);
    if (receive_partial_Frame((char *)pData, length)) {
      if (debug_print) Serial.println("Received full frame\n");
      if (receive_Auth_Frame((char *)gFullBuffer, gMessLen))
      {
        if (debug_print) Serial.println("Auth Frame");
      }
      else
      if (receive_Console_Frame((char *)gFullBuffer, gMessLen))
      {
        if (debug_print) Serial.println("Data Frame");
      }
    }

    //if (debug_print) Serial.println((char*)pData);
}



bool connectToServer(BLEAdvertisedDevice *myDevice) {
    if (debug_print) Serial.print("Forming a connection to ");
    if (debug_print) Serial.println(myDevice->getAddress().toString().c_str());
   
    BLEClient*  pClient  = BLEDevice::createClient();
    if (debug_print) Serial.println(" - Created client");
 
    pClient->setClientCallbacks(new MyClientCallback());
 
    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    if (debug_print) Serial.println(" - Connected to server");
 
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      if (debug_print) Serial.print("Failed to find our service UUID: ");
      if (debug_print) Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }

    //auto servicemap= pClient->getServices();
    // std::map<std::string, BLERemoteService*> *map
    // Print all services
    //for (auto &myPair : *pClient->getServices()) {
    //  if (debug_print) Serial.print("Service UUID: ");
    //  if (debug_print) Serial.println(myPair.first.c_str());
    // }

    if (debug_print) Serial.println(" - Is Service");
 
    esp_task_wdt_reset();

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    if (pRemoteService != nullptr) {
      pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID_TX);
      if (pRemoteCharacteristic == nullptr) {
        if (debug_print) Serial.print("Failed to find our characteristic UUID: ");
        if (debug_print) Serial.println(charUUID_TX.toString().c_str());
        pClient->disconnect();
        return false;
      }

      pRemoteRXCharecteristics = pRemoteService->getCharacteristic(charUUID_RX);
      if (pRemoteRXCharecteristics == nullptr) {
        if (debug_print) Serial.print("Failed to find our characteristic UUID: ");
        if (debug_print) Serial.println(charUUID_RX.toString().c_str());
        pClient->disconnect();
        return false;
      }


      if (debug_print) Serial.println(" - Found our characteristic");
 
      // Read the value of the characteristic.
      
      if(pRemoteCharacteristic->canRead()) {
        std::string value = pRemoteCharacteristic->readValue();
        if (debug_print) Serial.print("The characteristic value was: ");
        if (debug_print) Serial.println(value.c_str());
      }
    }

    if(pRemoteCharacteristic->canNotify()) {
      if (debug_print) Serial.print("Can Notify");      
      pRemoteCharacteristic->registerForNotify(notifyCallbackTX);
    } else {
      if (debug_print) Serial.print("Can't Notify");
    }

    connected = true;
    return true;
}


void printAdvertisedInfo(BLEAdvertisedDevice *myDevice) {
  if (debug_print) Serial.print("BLE Advertised Device found: ");
  if (debug_print) Serial.println(myDevice->toString().c_str());
  if (debug_print) Serial.print("Address Type: ");
  if (debug_print) Serial.println(myDevice->getAddressType());
  if (debug_print) Serial.print("RSSI: ");
  if (debug_print) Serial.println(myDevice->getRSSI());
  if (debug_print) Serial.print("Name: ");
  if (debug_print) Serial.println(myDevice->getName().c_str());
  //if (debug_print) Serial.print("TX Power: ");
  //if (debug_print) Serial.println(myDevice->getTXPower());
  if (debug_print) Serial.print("Service UUID: ");
  if (debug_print) Serial.println(myDevice->getServiceUUID().toString().c_str());
  if (debug_print) Serial.print("Payload: ");
  if (debug_print) Serial.println(myDevice->getPayloadLength());
}


unsigned short int Calc_CRC (unsigned char *ptr, unsigned short len)
{
 unsigned short int i, k, crc;


 crc = 0xFFFF;
 for (k = 0; k < len; k++)
    {
     crc = crc ^ ptr[k];
     for (i = 0; i < 8; i++)
        {
         if (crc & 0x0001)
            {
             crc = crc >> 1;
             crc = crc ^ 0xA001;
            }
         else
            crc = crc >> 1;
        }
    }
 return crc;
}

void write_Ping_Frame() {
  int full_len=0;
  if (connected) {
    if (debug_print) Serial.println("Sending Ping Frame");    
    int i=0;    
    char mess[100];

    mess[i++]=0xAA;
    // Size
    mess[i++]=0x02;
    
    //From
    mess[i++]=0x00;
    mess[i++]=0x00;
    
    //To
    mess[i++]=0xF0;
    mess[i++]=0x00;
    
    //Type
    mess[i++]=7;  // kNet_BT_Authorize
    mess[i++]=7;  // data 0
    mess[i++]=0;  // data 1

            
    // Checksum 2 bytes
    mess[i++]=0xE7;
    mess[i++]=0xBD;

    // size
    mess[1]=i-9;
    full_len=i;
    
    int crc=Calc_CRC((unsigned char *)&mess[1],full_len-3);
    // data[len2-1]!=(crc&0xff) && (data[len2-2]!=crc/256)
    mess[full_len-1]=crc&0xff;
    mess[full_len-2]=crc/256;

    if (pRemoteCharacteristic) {
      pRemoteCharacteristic->writeValue((uint8_t*)mess, full_len);
    }
  }
}



void write_Auth_Frame(uint32_t code) {
  int full_len=0;
  if (connected) {
    if (debug_print) Serial.println("Sending Auth Frame");    
    int i=0;    
    char mess[100];

    mess[i++]=0xAA;
    // Size
    mess[i++]=0x06;
    
    //From
    mess[i++]=0x00;
    mess[i++]=0x00;
    
    //To
    mess[i++]=0x00;
    mess[i++]=0x00;
    
    //Type
    mess[i++]=29;  // kNet_BT_Authorize

    mess[i++]=0;  // data 0
    mess[i++]=0;  // data 1


    // Set code
    mess[i++]=code&0xff;    // data 2
    mess[i++]=code>>8;      // data 3
    mess[i++]=code>>16;     // data 4
    mess[i++]=code>>24;     // data 5


/*
                received_code = pak->data[5];
                received_code <<= 8;
                received_code += pak->data[4];
                received_code <<= 8;
                received_code += pak->data[3];
                received_code <<= 8;
                received_code += pak->data[2];
*/

            
    // Checksum 2 bytes
    mess[i++]=0xE7;
    mess[i++]=0xBD;

    // size
    mess[1]=i-9;
    full_len=i;
    
    int crc=Calc_CRC((unsigned char *)&mess[1],full_len-3);
    // data[len2-1]!=(crc&0xff) && (data[len2-2]!=crc/256)
    mess[full_len-1]=crc&0xff;
    mess[full_len-2]=crc/256;

    if (pRemoteCharacteristic) {
      pRemoteCharacteristic->writeValue((uint8_t*)mess, full_len);
    }
  }
}






void writeD3_Frame(char *data,int len) {
  int full_len=0;
  if (connected) {
    if (debug_print) Serial.println("Sending D3 Frame");    
    int i=0;
    
    char mess[100];

    mess[i++]=0xAA;
    // Size
    mess[i++]=0x1C;
    
    //From
    mess[i++]=0x00;
    mess[i++]=0x00;
    
    //To
    mess[i++]=0xF0;
    mess[i++]=0x00;
    
    //Type
    mess[i++]=20;  // kConsole
    
        
    int j=0;
    // j<strlen(data)
    while(j<len) {
        mess[i+j]=data[j];
        j++;
    }
    i=i+j;
    
    // Checksum 2 bytes
    mess[i++]=0xE7;
    mess[i++]=0xBD;

    // size
    mess[1]=i-9;
    full_len=i;
    
    int crc=Calc_CRC((unsigned char *)&mess[1],full_len-3);
    // data[len2-1]!=(crc&0xff) && (data[len2-2]!=crc/256)
    mess[full_len-1]=crc&0xff;
    mess[full_len-2]=crc/256;

    if (pRemoteCharacteristic) {
      pRemoteCharacteristic->writeValue((uint8_t*)mess, full_len);
    }
  }
}

void writeRawData(char *data,int len) {
  int full_len=0;
    if (pRemoteCharacteristic) {
      pRemoteCharacteristic->writeValue((uint8_t*)data, full_len);
    }
}



int times_skipped=0;

void loop(void)
{

  M5GFX* gfx=&M5.Display;

  if(Serial.available()) {
    times_skipped=0;
    String input = Serial.readString();


    if (!authorized) {
        write_Auth_Frame(26220);
    }
    if (input[0]=='p') {
      if (authorized)
        write_Ping_Frame();
    } else {
      if (authorized)
        writeRawData((char *)input.c_str(),input.length());
    }
  } else {
    times_skipped++;
    if (times_skipped>2000) {
      times_skipped=0;
      if (authorized)
        write_Ping_Frame();
    }
  }


  gfxLoop(&M5.Display);

  //if (times_delayed==0) {
  //  if (debug_print) Serial.print("Loop:\n");
  //}


  if (M5.BtnA.isHolding() && M5.BtnB.isHolding()) {
     gfx->print("Scanning...\n");
    gFoundDevices.clear();
    gAdvertisedDevices.clear();
    if (debug_print) Serial.print("Rescan:\n");
    gSelectedRow = 0;
    BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
    if (debug_print) Serial.print("Devices found: ");
    if (debug_print) Serial.println(foundDevices.getCount());
    if (debug_print) Serial.println("Scan done!");
    pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
  }
  
  times_delayed++;  
  if (times_delayed>5000) times_delayed=0;

  vTaskDelay(1);

  M5.update();
  if (gSelectedRow > gFoundDevices.size() - 1) {
    gSelectedRow = gFoundDevices.size() - 1;
    gDisplayUpdate = true;
  }


  if (M5.BtnA.wasPressed())
  {
    //M5.Speaker.tone(440, 100);
    if (gSelectedRow > 0) {
      gSelectedRow--;
      gDisplayUpdate = true;
    }
  }
  if (M5.BtnB.wasPressed())
  {
    //M5.Speaker.tone(440, 100);
    if (gSelectedRow < gFoundDevices.size() - 1) {
      gSelectedRow++;
      gDisplayUpdate = true;
    }
  }

  if (M5.BtnC.wasPressed()) {
    if (gSelectedRow<gAdvertisedDevices.size()) {
      gfx->print("Connecting...\n");
      M5.Speaker.tone(440, 100);
      BLEAdvertisedDevice *myDevice = &gAdvertisedDevices[gSelectedRow];
        esp_task_wdt_reset();
        authorized=false;
        if (debug_print) Serial.print("BLE Advertised Device found: ");
        if (debug_print) Serial.println(myDevice->toString().c_str());
        if (debug_print) Serial.print("Address Type: ");
        if (debug_print) Serial.println(myDevice->getAddressType());
        if (debug_print) Serial.print("RSSI: ");
        if (debug_print) Serial.println(myDevice->getRSSI());
        if (debug_print) Serial.print("Name: ");
        if (debug_print) Serial.println(myDevice->getName().c_str());
        //if (debug_print) Serial.print("TX Power: ");
        //if (debug_print) Serial.println(myDevice->getTXPower());
        if (debug_print) Serial.print("Service UUID: ");
        if (debug_print) Serial.println(myDevice->getServiceUUID().toString().c_str());
        if (debug_print) Serial.print("Payload: ");
        if (debug_print) Serial.println(myDevice->getPayloadLength());

        if (debug_print) Serial.print("Num Services: ");
        if (debug_print) Serial.println(myDevice->getPayloadLength());
        esp_task_wdt_reset();
        connectToServer(myDevice);
    }
      //printAdvertisedInfo(&gAdvertisedDevices[gSelectedRow]);
  }

  if (M5.BtnA.wasDeciedClickCount())
  {
    switch (M5.BtnA.getClickCount())
    {
    case 1:
      M5.Speaker.tone(1000, 100);
      break;

    case 2:
      M5.Speaker.tone(800, 100);
      break;
    }
  }
  if (M5.BtnA.isHolding() || M5.BtnB.isPressed() || M5.BtnC.isPressed())
  {
    /*
    size_t v = M5.Speaker.getVolume();
    int add = (M5.BtnB.isPressed()) ? -1 : 1;
    if (M5.BtnA.isHolding())
    {
      add = M5.BtnA.getClickCount() ? -1 : 1;
    }
    v += add;
    if (v <= 255)
    {
      M5.Speaker.setVolume(v);
    }
    */
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