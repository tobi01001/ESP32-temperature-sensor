#include <WiFi.h>   // all the WiFi stuff on ESP32
#include <SimpleDHT.h>  // for earsy reading DHT sensors
#include <rom/crc.h>    // CRC calculation

#include <Arduino.h> // generic Arduino libraries
#include <HTTPClient.h> // HTTP client used for OTA update
#include <HTTPUpdate.h> // used for OTA via HTTP
#include "driver/adc.h" // used for voltage monitoring
#include <esp_wifi.h>   // esp32 low level interface functions for WiFi
#include <esp_bt.h>     // esp32 low level interface functions for WiFi
#ifdef USE_WIFIMANAGER
  #include <WifiManager.h>
#endif

#include "config.h"


// Defines the firmware version (the OTA checks if the version of a file is newer, i.e. old < new)
const uint32_t FW_VERSION = DFW_VERSION; // year_month_day_counternumber 

// This is the URL where the OTA binaries and version files can be found.
// Currently, each device looks after <MAC_address in hex>.verion and .bin
const char* fwUrlBase = FWURLBASE; 

// instance of the sensor class/library with the 1wire data pin
SimpleDHT22 dht22(PINDHT22);

// This is the adress (host) where the data is sent to.
const IPAddress host = IPAddress(192,168,2,18);
// This is the TelnetPort where the data is sent to.
const uint16_t  TelNetPort = 7072;

// The GPIO pin where the battery voltage is measured.
const uint16_t Analog_channel_pin = 35;
// current ADC reading (volatge raw value)
uint16_t adc_value = 0;
// current voltage reading in millivolts
uint16_t voltage_value = 0;

enum sleepReason {
  SR_NOTHING,
  SR_NO_NEW_DATA,
  SR_SEND_DATA,
  SR_SENSOR_ERROR,
  SR_WIFI_ERROR,
  SR_CONN_ERROR,
  SR_OTHER
};

// to quickly check if we are connected to WiFi
bool wifiConnected = false;

WiFiClient *client = NULL;

#ifdef SENSOR_MEASURE_PERFORMANCE
// current time it took to connect to WiFi
int16_t connect_time = 0;



// just for some performance meaurements - see how long the sketch was run.
// Reading and sending of sensor values takes (in my environment) less than 700 ms 
// while majority is used for setting up the WiFi connection....
RTC_DATA_ATTR uint16_t    runtimes_milliseconds[SENSOR_MIN_UPDATE];
RTC_DATA_ATTR uint16_t    runtime_milliseconds_mean;
RTC_DATA_ATTR uint8_t     rt_counter;
// for performance measurements: How long does it taker to connect to the WiFi
// lets compare different environments ans using WiFimanager vs. predefined ssid and pass
RTC_DATA_ATTR  int16_t    last_time_to_connect = 0;
// Sensor Error counter (+10 on error, -1 on success)
RTC_DATA_ATTR uint16_t    sens_err_count = 0;
// The last Sensor err seen (gets reset on sensor Erro counter = 0?)
RTC_DATA_ATTR int         last_sens_error = 0;
// WiFi   Error counter (+10 on error, -1 on success)
RTC_DATA_ATTR uint16_t    wifi_conn_err_count = 0;
RTC_DATA_ATTR uint16_t    wifi_send_err_count = 0;

RTC_DATA_ATTR sleepReason SR = SR_NOTHING;
RTC_DATA_ATTR sleepReason SRs[SENSOR_MIN_UPDATE];

#endif

// The RTC_DATA_ATTR attrribute is used to tell the compiler/linker that these variables 
// are put into the RTC (real time clock) memory. So they keep there values during deep sleep - amazing.

// The previous "NUM_VALUES" temperature values.
// The last value represents the last "weighted mean value". 
// this is used to check if the temperature changed.
RTC_DATA_ATTR int16_t     pTemp[NUM_VALUES];
// The previous "NUM_VALUES" humidity values.
// The last value represents the last "weighted mean value". 
// this is used to check if the humidity changed.
RTC_DATA_ATTR uint16_t    pHumi[NUM_VALUES];
// The previous "NUM_VALUES" voltage values.
// The last value represents the last "weighted mean value". 
// this is used to check if the voltage changed.
RTC_DATA_ATTR uint16_t    pVolt[NUM_VALUES];

RTC_DATA_ATTR int16_t     lastSentTemp = 0;
RTC_DATA_ATTR uint16_t    lastSentHumi = 0;

// Check if started the first time /i.e. RTC variables not initialized)
// This could be checked with the "boot reason" as well.
RTC_DATA_ATTR bool        firstrun = true;
// Counter to check when the next OTA check is due.
RTC_DATA_ATTR uint8_t     checkOTA_count = SENSOR_OTA_INTERVAL;  // to only check once every x minutes
// Counter to check when the next update needs to be send regardless if values have changed.
RTC_DATA_ATTR uint16_t    sendMinCounter = SENSOR_MIN_UPDATE;

// the mac address as bytes
RTC_DATA_ATTR byte        chipID[6]; 
// the mac address as characters
RTC_DATA_ATTR char        macAddr [14];
// the CRC16 calculated over the mac address (used as Sensor name / ID)
RTC_DATA_ATTR uint16_t    chip_crc;

// Declarations
void  
  disconnect_WiFi(bool),
  connect_WiFi(void),
  getMACAddress(void),
  checkForUpdates(void),
  print_wakeup_reason(void),
  goto_sleep(uint16_t seconds, sleepReason sleep), 
  round_05_precision(float *value),
  readADCValue(void);
int16_t 
  map16(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max),
  roundToPrecision(int16_t * value, int8_t precision),
  roundFloatToInt10(float * value, int8_t precision);

// saturates at s (i.e. value will not be higher)
uint8_t addsat8( uint8_t a, uint8_t b, uint8_t s) {
  uint16_t c = (uint16_t)a + (uint16_t)b;
  if(c > 255 || c > s) return s;
  return (uint8_t) c;
}

// Will diconnect the WiFi and set wifiConnected = false
void disconnect_WiFi(bool wifi_off = true)
{
  // lets do this twice.... seems to be not working all the time
  // and if it did not work it seems we miss the update...
  uint8_t disconnect_counter = 3;
  bool disconnected = false;
  while(disconnected && disconnect_counter--)
  {
    disconnected = WiFi.disconnect(false);
    delay(5);
  }
  if(wifi_off) 
  {
    WiFi.disconnect(wifi_off);
    delay(5);
    WiFi.mode(WIFI_OFF);  //redundant but who cares....
  }
  
  // We are still not sure if disconnect worked but we consider it "done"
  wifiConnected = false;
}

// will connect to the WiFi.
// Depending on Configuration this will either use 
// - WiFiManager stored value (or configuration AP on first start)
// - or predefined SSID and Password
void connect_WiFi()
{
  if(wifiConnected && WiFi.status() == WL_CONNECTED) return; // Already connected.
  
  #ifdef USE_WIFIMANAGER
  // The first testing with Wifimanager revealed connect timings of around 1000 ms (almost double) with variations down to 400 ms.
  WiFiManager wifiManager;
  
  wifiManager.setTimeout(90);
  
  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if(!wifiManager.autoConnect()) {
    DEBUGPRNTLN("failed to connect and hit timeout");
    wifiConnected = false;
    return;
  } 
  #else
  // Apparently it does not make a difference if we use static IP or not. 
  // Time to connect in my environment is about 550 ms
  // short delay
  if(!WiFi.enableSTA(true)) {
    wifiConnected = false;
    return;
  }
  delay(5);
  WiFi.begin(SECRET_SSID, SECRET_PASS);
  uint16_t count = 500; //Timeout roughly .5 seconds.
  while (WiFi.status() != WL_CONNECTED) {
      // 1 ms delay to proceed quite fast in case we are connected...
      delay(1);
      if(!count--) 
      {
          wifiConnected = false;
          #ifdef DEBUG
          DEBUGPRNTLN("WiFi connection failed..");
          #endif
          return;
      }
  }
  #endif
  delay(5);
  wifiConnected = true;

  DEBUGPRNTLN("WiFi connected");
  DEBUGPRNT("IP address: ");
  DEBUGPRNTLN(WiFi.localIP());
  DEBUGPRNTLN("Wifi Setup done.");
}

// Arduino "map" is of data type long. We just need 16 bit int.
// A template could work as well but I just did a 16 bit version...
int16_t map16(int16_t x, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max) {
    // input width
    int16_t divisor = (in_max - in_min);
    // sanity to avoid DIV/0
    if(divisor == 0){
        return -1; //AVR returns -1, SAM returns 0
    }
    // return mapped value
    return (x - in_min) * (out_max - out_min) / divisor + out_min;
}

// Will round the given float value (must be positive)
// to 0.5 precision which avoids unneccessary updates
void round_05_precision(float *value)
{    
    float hum = roundf(*value);
    DEBUGPRNTLN("\n\tRounding Value " + String(*value) + " to " + String(hum));
    float delta_hum = *value - hum;
    DEBUGPRNTLN("\tRounding Diff  " + String(delta_hum));
    *value = hum;
    if(delta_hum < -0.25f)
    {
      *value = hum - 0.5;
    }
    else if(delta_hum > 0.25f)
    {
      *value = hum + 0.5f;
    }
    else
    {
      *value = hum;  
    }
    DEBUGPRNTLN("\tRounded Value is now " + String(*value) + "\n");
}

// Will convert the given float value to an int (times 10) and round to the precision given (integer)
// e.g. 10.3 will convert to 103
// with precision 5 it will return 105. 
int16_t roundFloatToInt10(float * value, int8_t precision)
{
  int16_t retVal = (int16_t)(*value * 10);
  if(precision > 1)
  {
    retVal = ((retVal + (precision / 2)) / precision) * precision;
  }
  return retVal;
}


// Will round an int to the precision given (integer)
// e.g. 103
// with precision 5 it will return 105. 
int16_t roundToPrecision(int16_t * value, int8_t precision)
{
  int16_t retVal = *value;
  if(precision > 1)
  {
    retVal = ((retVal + (precision / 2)) / precision) * precision;
  }
  return retVal;
}

// Helper function to write the MAC_Address
// in both chipID as well as maxAddr variables.
// finally the CRC over the chipID is calculatred and stored in chip_crc.
void getMACAddress()
{
  // write WiFi adapter mac address in chipID
  WiFi.macAddress( chipID );
  // write the mac address as hex characters.
  sprintf(macAddr, "%02X%02X%02X%02X%02X%02X", chipID[0], chipID[1], chipID[2], chipID[3], chipID[4], chipID[5]);  
  // calculate and store the crc
  chip_crc = crc16_le(0,(uint8_t*)(&chipID), 6);
}

/// Start of main function that performs HTTP OTA /// 
void checkForUpdates() {

  if(!wifiConnected) return; // No need to check if there is no WiFi

  // Holds the URL and (Base)-Filename to check for OTA-Updates
  String fwURL = String( fwUrlBase );
  // ...we look for the MAC Address in the respective folder (to be able to provide different updates per device)
  fwURL.concat( macAddr );
  // URL for the file which holds the version of the OTA ( .version extension)
  // This file will be checked for the version number (FW_VERSION as uint32_t)
  String fwVersionURL = fwURL;
  fwVersionURL.concat( ".version" );
  DEBUGPRNTLN( "Checking for firmware updates." );
  DEBUGPRNT( "MAC address: " );
  DEBUGPRNTLN( macAddr );
  
  DEBUGPRNT( "Firmware version URL: " );
  DEBUGPRNTLN( fwVersionURL );

  // WifiClient to check the server for available updates
  WiFiClient client;
  // HTTP client to GET the available FW version via WiFi Client
  HTTPClient httpClient;
  httpClient.begin( client, fwVersionURL );
  // read the URL
  int httpCode = httpClient.GET();
  // if reading was OK
  if( httpCode == 200 ) {
    // OTA version as String
    String newFWVersion = httpClient.getString();
    DEBUGPRNT( "Current firmware version: " );
    DEBUGPRNTLN( FW_VERSION );
    DEBUGPRNT( "Available firmware version: " );
    DEBUGPRNTLN( newFWVersion );
    // OTA version as integer
    uint32_t newVersion = (uint32_t)newFWVersion.toInt();
    // check if OTA version is newer than current version
    // Attention: If the version file holds a newer (higher) version
    // than in the firmware file, this will end in an "update loop"
    if( newVersion > FW_VERSION ) {
      DEBUGPRNTLN( "Preparing to update" );
      // firmware image file URL
      String fwImageURL = fwURL;
      fwImageURL.concat( ".bin" );
      DEBUGPRNTLN("Trying to load "+ fwImageURL);
      // and now the magic, where the firmware gets simply read and updated
      t_httpUpdate_return ret = httpUpdate.update( client, fwImageURL ); /// FOR ESP32 HTTP FOTA
      
      switch(ret) {
        case HTTP_UPDATE_FAILED:
          #ifdef DEBUG
          Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str()); /// FOR ESP32
          #endif
          break;

        case HTTP_UPDATE_NO_UPDATES:
          DEBUGPRNTLN("HTTP_UPDATE_NO_UPDATES");
          break;
        default: 
          break;
      }
    }
    else {
      DEBUGPRNTLN( "Already on latest version" );
    }
  }
  else {
    DEBUGPRNT( "Firmware version check failed, got HTTP response code " );
    DEBUGPRNTLN( httpCode );
  }
  httpClient.end();
  DEBUGPRNTLN("\nEnd - Checking for FW Updates\n");
}


// check the wakeup reason.
// currently we only print it in DEBUG mode to the serial.
// could be used for deciding how to start up as well.
void print_wakeup_reason()
{
#ifdef DEBUG
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch(wakeup_reason){
    case ESP_SLEEP_WAKEUP_EXT0     : DEBUGPRNTLN("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1     : DEBUGPRNTLN("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER    : DEBUGPRNTLN("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : DEBUGPRNTLN("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP      : DEBUGPRNTLN("Wakeup caused by ULP program"); break;
    default                        : DEBUGPRNTLN("Wakeup was not caused by deep sleep: " + String(wakeup_reason)); break;
  }
  DEBUGPRNTLN("\n");
#endif
}

// function used to send the esp to deep sleep.
// parameter:
// uint16_t seconds -> How many seconds to sleep.
void goto_sleep(uint16_t seconds, sleepReason sleepreason=SR_OTHER)
{
  // Strange but the ESP will fail to reconnect on the next wake if WiFi is not 
  // explicitely turned off.
  // further, this will save additional energy.

  disconnect_WiFi(true);

  // we also explicitely turn off BlueTooth
  btStop();

  // we also turn off the adc (just in case).
  adc_power_off();

  // and also low level ESP framework functions to turn off BT finally
  esp_bt_controller_disable();

  // setup the sleep time
  esp_sleep_enable_timer_wakeup(seconds * uS_TO_S_FACTOR);

  #ifdef SENSOR_MEASURE_PERFORMANCE
  if(++rt_counter >= SENSOR_MIN_UPDATE) rt_counter=0;
  runtimes_milliseconds[rt_counter] = millis();
  uint32_t sum = 0;
  for(uint8_t i=0; i<SENSOR_MIN_UPDATE; i++)
  {
    sum+=runtimes_milliseconds[i];
  }
  runtime_milliseconds_mean = (uint16_t)(sum/SENSOR_MIN_UPDATE);
  

  last_time_to_connect = connect_time;
  SR = sleepreason;
  SRs[rt_counter] = sleepreason;
  #ifdef DEBUG
  DEBUGPRNTLN("\nI was active for " + String(runtime_microseconds) + " microseconds.\n");
  #endif
  #endif
  
  // Go to sleep now.
  #ifdef DEBUG
  DEBUGPRNTLN("Going to sleep for " + String(seconds) + " seconds.\n");
  #endif
  esp_deep_sleep_start();
  // this line will never be reached!
}

void readADCValue()
{
  adc_power_on();
  adc_value = analogRead(Analog_channel_pin); 
  adc_power_off();

  voltage_value = ((map16(adc_value /*& 0xFFE0*/, 0, 4095, 0, 6600))); // 11.06.2020 Removed this cap: cap the lower bits for more stable readings ~50 mV capped
  DEBUGPRNTLN("adc_value: " + String(adc_value));
  DEBUGPRNTLN("voltage_value: " + String(voltage_value));
}

void setup()
{
    #ifdef SENSOR_MEASURE_PERFORMANCE
    uint32_t starttime = millis();
    #endif
    bool sendData[3] = {false, false, false};
    #ifdef DEBUG
    delay(100);
    #else
    //delay(10);
    #endif
    disconnect_WiFi(true);
    adc_power_off();
    
    #ifdef DEBUG
    Serial.begin(115200);
    DEBUGPRNTLN("\n");
    #endif
    print_wakeup_reason();

    readADCValue();

    int16_t temperature = 0;
    int16_t humidity = 0;
    int err = SimpleDHTErrSuccess;
    if ((err = dht22.read2(&temperature, &humidity, NULL)) != SimpleDHTErrSuccess) {
        #ifdef DEBUG
        DEBUGPRNT("Read DHT22 failed, err="); DEBUGPRNTLN(err);delay(2000);
        #endif
        #ifdef SENSOR_MEASURE_PERFORMANCE
        last_sens_error = err;
        sens_err_count+= SENSOR_MIN_UPDATE;
        #endif
        goto_sleep(SENSOR_ERROR_INTERVAL, SR_SENSOR_ERROR);
    }
    else
    {
      #ifdef SENSOR_MEASURE_PERFORMANCE
      if(sens_err_count > 0)
      {
        sens_err_count--;
      }
      else
      {
        last_sens_error = 0;
      }
      #endif
    }

    if(firstrun)
    {
        #ifdef DEBUG
            DEBUGPRNTLN("\nFirst run. We init with the current readings.");
            DEBUGPRNTLN("\t" + String(NUM_VALUES) + 
                           " values will be written with T " +
                           String((float)temperature/10.0) + 
                           ", H " + String((float)humidity/10.0) + " and V " + String(voltage_value) + "!\n");
        #endif
        //firstrun = false;

        getMACAddress();

        checkOTA_count = SENSOR_OTA_INTERVAL;
        sendMinCounter = SENSOR_MIN_UPDATE;

        sendData[0] = sendData[1] = sendData[2] = true;

        for(uint8_t i=0; i < NUM_VALUES-1; i++)
        {
            pTemp[i] = roundToPrecision(&temperature, SENSOR_T_PRECISION);
            pHumi[i] = roundToPrecision(&humidity, SENSOR_H_PRECISION);
            pVolt[i] = (uint16_t)(voltage_value);
        }
        #ifdef SENSOR_MEASURE_PERFORMANCE
        for(uint8_t i=0; i<SENSOR_MIN_UPDATE; i++)
        {
          runtimes_milliseconds[i] = 500; // init with 500 ms
        }
        rt_counter=0;
        #endif
    }


    // Shift old values down.
    for(uint8_t i=0; i<NUM_VALUES-2; i++)
    {
        pTemp[i] = pTemp[i+1];
        pHumi[i] = pHumi[i+1];
        pVolt[i] = pVolt[i+1];
    }
    // write new values to the end
    pTemp[NUM_VALUES-2] = roundToPrecision(&temperature, SENSOR_T_PRECISION); 
    pHumi[NUM_VALUES-2] = roundToPrecision(&humidity, SENSOR_H_PRECISION);
    pVolt[NUM_VALUES-2] = voltage_value;
    #ifdef DEBUG
    DEBUGPRNTLN("\nSensor was read:");
    DEBUGPRNT("\tValues \t  T: " + String((float)temperature/10.0));
    DEBUGPRNT("\t  H: " + String((float)humidity/10.0));
    DEBUGPRNT("\t  V: " + String(voltage_value));
    DEBUGPRNTLN("\t  Raw: " + String(adc_value));
    DEBUGPRNTLN("");
    #endif
    // calculate wighted mean value
     int16_t temp = 0;
    uint16_t humi = 0;
    uint32_t volt = 0;
    uint16_t weight_div = 0;
    for(uint8_t i=0; i<NUM_VALUES-1; i++)
    {
        temp        += ((i+1)*2)*pTemp[i]; // oldest entry with the lowest weight
        humi        += ((i+1)*2)*pHumi[i];
        volt        += ((i+1)*2)*pVolt[i];
        weight_div  += ((i+1)*2);
    }
    temp = (temp / weight_div);
    humi = (humi / weight_div);
    volt = (volt / weight_div);
    #ifdef DEBUG
    for(uint8_t i=0; i<NUM_VALUES; i++)
    {
        DEBUGPRNT("\tItem: " + String(i));
        DEBUGPRNT("\t  T: "  + String(pTemp[i]));
        DEBUGPRNT("\t  H: "  + String(pHumi[i]));
        DEBUGPRNTLN("\t  V: "  + String(pVolt[i]));
    }
    DEBUGPRNT("\n\tCurr \t  T: " + String(String((float)((float)temp/10.0))));
    DEBUGPRNT(  "\t  H: " + String(String((float)((float)humi/10.0))));
    DEBUGPRNTLN(  "\t  V: " + String(String((float)((float)volt/1000.0))) + "\n");
    #endif

    uint16_t dT = abs((int)temp - (int)lastSentTemp);
    uint16_t dH = abs((int)humi - (int)lastSentHumi);

//    if(temp               != pTemp[NUM_VALUES-1]) { sendData[0] = true; DEBUGPRNTLN("Need to send data because of new Temperature");}
//    if(humi               != pHumi[NUM_VALUES-1]) { sendData[1] = true; DEBUGPRNTLN("Need to send data because of new Humidity");}
    if(dT >= SENSOR_MIN_DELTA_T) { sendData[0] = true; DEBUGPRNTLN("Need to send data because new Temperature and the delta is " + String(dT));}
    if(dH >= SENSOR_MIN_DELTA_H) { sendData[1] = true; DEBUGPRNTLN("Need to send data because new Humidity and the delta is " + String(dH));}
    if(++checkOTA_count   >  SENSOR_OTA_INTERVAL) { sendData[0] = sendData[1] = sendData[2] = true; DEBUGPRNTLN("Need to send data because of FW Update Interval");}
    if(++sendMinCounter   >  SENSOR_MIN_UPDATE)   { sendData[0] = sendData[1] = sendData[2] = true; DEBUGPRNTLN("Need to send data because of Sensor Min Interval");} 
  
    if(!(sendData[0] | sendData[1] | sendData[2]) )
    {
      DEBUGPRNTLN("No need to send because no new Data. Will sleep 3 times longer...");
      checkOTA_count = addsat8(checkOTA_count, 3, SENSOR_OTA_INTERVAL);
      sendMinCounter = addsat8(sendMinCounter, 3, SENSOR_MIN_UPDATE);
      DEBUGPRNTLN("\n\tOTA Count  " + String(checkOTA_count) + " of " + String(SENSOR_OTA_INTERVAL));
      DEBUGPRNTLN(  "\tMin Update " + String(sendMinCounter) + " of " + String(SENSOR_MIN_UPDATE));
      goto_sleep(SENSOR_READ_INTERVAL*3, SR_NO_NEW_DATA);
    }
    #ifdef DEBUG
    else
    {
      DEBUGPRNTLN("\n\tOTA Count  " + String(checkOTA_count) + " of " + String(SENSOR_OTA_INTERVAL));
      DEBUGPRNTLN(  "\tMin Update " + String(sendMinCounter) + " of " + String(SENSOR_MIN_UPDATE));
    }
    #endif

    DEBUGPRNTLN("\n\n");

    DEBUGPRNTLN("Going to connect to WiFi:");
    #ifdef SENSOR_MEASURE_PERFORMANCE
    uint32_t current_ms = millis();
    #endif
    
    uint8_t wifi_retry = SENSOR_WIFI_RETRY_COUNT;
    while(!wifiConnected && wifi_retry--)
    {
      connect_WiFi();
      if(!wifiConnected) // lets try once more
      {
        disconnect_WiFi(false);
        // delay(5); // delay now in disconnect_WiFi
      }
    }
    
    
    if(!wifiConnected) // this did not work as expected....
    {
      DEBUGPRNTLN("WiFi connection failed!");
      #ifdef SENSOR_MEASURE_PERFORMANCE
      connect_time = -100;
      
      wifi_conn_err_count += 10; 
      #endif
      goto_sleep(SENSOR_ERROR_INTERVAL, SR_WIFI_ERROR);
    }
    else
    {
      DEBUGPRNTLN("WiFi connection successful!");
      #ifdef SENSOR_MEASURE_PERFORMANCE
      connect_time = (int16_t)(millis() - current_ms);
      if(wifi_conn_err_count > 0) wifi_conn_err_count--;
      #endif
    }

    if(checkOTA_count > SENSOR_OTA_INTERVAL)
    {
        checkOTA_count = 0;
        checkForUpdates();
    }
    
    #ifdef SENSOR_MEASURE_PERFORMANCE
    current_ms = millis();
    
    int16_t srv_connect_time = 0;
    #endif
    uint8_t server_retry = SENSOR_SERVER_RETRY_COUNT;
    bool server_connected = false;
    while(!server_connected && server_retry--)
    {
      client = new WiFiClient();
      server_connected = client->connect(host, TelNetPort);
      if(!server_connected)
      {
        client->stop();
        delete client;
      }
      delay(5);
    }

    if (!server_connected || !client) {
        #ifdef DEBUG
        DEBUGPRNTLN("connection failed");
        #endif
        #ifdef SENSOR_MEASURE_PERFORMANCE
        srv_connect_time = -100;
        wifi_send_err_count += 10;
        #endif
        goto_sleep(SENSOR_ERROR_INTERVAL, SR_CONN_ERROR);
    }
    #ifdef SENSOR_MEASURE_PERFORMANCE
    else
    {
      
      if(wifi_send_err_count > 0) wifi_send_err_count--;
      
    }
    srv_connect_time = (int16_t) (millis() - current_ms);
    #endif

    pTemp[NUM_VALUES-1] = temp;
    pHumi[NUM_VALUES-1] = humi;
    pVolt[NUM_VALUES-1] = volt;

    // Think this takes awhile as well but not that much as compared to the WiFi
    String payload = "{handleTelnet(\"TH_Sens_" + String(chip_crc) + "\", \"minUpdateCount "   + String(sendMinCounter) + 
                     #ifdef SENSOR_MEASURE_PERFORMANCE
                     ": start_time_ms "         + String(starttime) +
                     ": last_runtime_ms "       + String(runtimes_milliseconds[rt_counter]) +
                     ": mean_runtime_ms "       + String(runtime_milliseconds_mean) +
                     ": wifi_connect_time_ms "  + String(connect_time) +
                     ": serv_connect_time_ms "  + String(srv_connect_time) +
                     ": last_wifi_conn_time_ms "+ String(last_time_to_connect) +
                     ": sensor_error_count "    + String(sens_err_count) +
                     ": last_sensor_error "     + String(last_sens_error) +
                     ": wifi_error_count "      + String(wifi_conn_err_count) +
                     ": wifi_send_err_count "   + String(wifi_send_err_count) +
                     ": wifi_rssi "             + String(WiFi.RSSI()) +
                     ": last_sleep_reason "     + String(SR) +
                     ": volt_raw "              + String((float)((float)pVolt[NUM_VALUES-2]/1000.0)) + 
                     ": adc_raw_value "         + String(adc_value) +
                     #endif
                     ": time_to_next_update "   + String(sendMinCounter<=SENSOR_MIN_UPDATE?((SENSOR_MIN_UPDATE + 1 - sendMinCounter)*SENSOR_READ_INTERVAL):0) +
                     ": time_to_FW_update "     + String((SENSOR_OTA_INTERVAL + 1 - checkOTA_count)*SENSOR_READ_INTERVAL) +
                     ": sensor_voltage "        + String((float)((float)volt/1000.0));
    if(sendData[0])
    {
      payload = payload +  ": temperature "     + String((float)((float)temp/10.0));
      lastSentTemp = temp;
      #ifdef SENSOR_MEASURE_PERFORMANCE
      payload = payload +  ": temp_raw "        + String((float)((float)pTemp[NUM_VALUES-2]/10.0));
      #endif                     
    }
    if(sendData[1])
    {
      payload = payload + ": humidity "         + String((float)((float)humi/10.0));
      lastSentHumi = humi;
      #ifdef SENSOR_MEASURE_PERFORMANCE
      payload = payload + ": humi_raw "         + String((float)((float)pHumi[NUM_VALUES-2]/10.0));
      #endif
    }
    if(sendData[2])
    {
      #ifdef SENSOR_MEASURE_PERFORMANCE
      String rts = "";
      String srs = "";
      for(uint8_t i=0; i<SENSOR_MIN_UPDATE; i++)
      {
        rts = rts + String(runtimes_milliseconds[i]);
        srs = srs + String(SRs[i]);
        if(i<SENSOR_MIN_UPDATE-1) 
        {
          rts = rts+",";
          srs = srs+",";
        }
      }
      #endif
      // if we did send, the next enforced sending counter starts fresh
      sendMinCounter = 0; 
    }
    // only need to send this once (i.e. after update or reset)
    if(firstrun)
    {
      firstrun = false;
      payload = payload + ": dev_name " + "TH_Sens_" + String(chip_crc) +
                          ": FW_VERSION "       + String(FW_VERSION) +
                          ": sensor_type "      + String(SENSOR_TYPE) +
                          ": sensor_interval "  + String(SENSOR_READ_INTERVAL) +
                          ": sensor_temp_prec " + String(SENSOR_T_PRECISION) +
                          ": sensor_humi_prec " + String(SENSOR_H_PRECISION) +
                          #ifdef SENSOR_MEASURE_PERFORMANCE
                          ": last_runtimes_ms " + rts +
                          ": last_sleep_reasons " + srs +
                          #endif
                          ": MAC_ADDRESS "      + String(macAddr);
    }
    payload += "\")}\r\n";
    #ifdef DEBUG
    DEBUGPRNT("Sending: ");
    DEBUGPRNTLN(payload);
    #endif

    // This will send the request to the server
    client->print(payload);
    client->print("exit\r\n");
    client->flush();
    client->stop();
    delete client;
    goto_sleep(SENSOR_READ_INTERVAL, SR_SEND_DATA);
}

void loop()
{
    // nothing, we won't go that far
}
