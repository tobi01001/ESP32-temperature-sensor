#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h> // generic Arduino libraries

#ifndef DFW_VERSION
#warning "FW_VERSION not defined. using default"
#define DFW_VERSION 2020052801
#endif

#ifndef FWURLBASE
#warning "FWURLBASE is not defined. using default value http://192.168.2.40/ESPOTA/"
#define FWURLBASE "http://192.168.2.40/ESPOTA/"
#endif

#ifdef USE_SECRET_SSID
#include "secrets.h"
    #ifndef SECRET_SSID
    #warning "SSID not defined. Going to use WiFiManager instead!"
    #define USE_WIFIMANAGER
    #endif

    #ifndef SECRET_PASS
    #warning "WIFI passphrase not defined. Going to use WiFiManager instead!"
    #define USE_WIFIMANAGER
    #endif
#else
    #define USE_WIFIMANAGER    
#endif

// the date pin for the one wire temperature sensor.
// if not set via buid_flag (platformio.ini), pls set it here
// #define PINDHT22 23
#ifndef PINDHT22
#warning "Need to define the DHT22 Pin. Using default #23 now"
#define PINDHT22 23
#endif

// how many values for history / wighted mean value
#define NUM_VALUES          5

#ifndef SENSOR_READ_INTERVAL
    #warning "Using default SENSOR_READ_INTERVAL (60)"
    // how many seconds to sleep
    #define SENSOR_READ_INTERVAL  60
#endif
#ifndef SENSOR_ERROR_INTERVAL
    #warning "Using default SENSOR_ERROR_INTERVAL (30)"
    // how many seconds to sleep in case of error  
    #define SENSOR_ERROR_INTERVAL 30
#endif
#ifndef SENSOR_MIN_UPDATE
    #warning "Using default SENSOR_MIN_UPDATE (10)"
    // after how many sleep cycles an update will be send regardless the values
    #define SENSOR_MIN_UPDATE     10
#endif
#ifndef SENSOR_OTA_INTERVAL
    #warning "Using default SENSOR_OTA_INTERVAL (200)"
    // after how many sleep cycles to look if a new version is available
    // this can of course be set to once a day or so. during debug, a shorter cycle is easier....  
    #define SENSOR_OTA_INTERVAL   200  
#endif


// used to convert the deep sleep call to seconds.
#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */ 

// helper defines for DEBUG (either with or without monitoring)
#ifdef DEBUG
    #define DEBUGPRNT(x)   (Serial.print(x))
    #define DEBUGPRNTLN(x) (Serial.println(x))
#else
    #define DEBUGPRNT(x)    
    #define DEBUGPRNTLN(x) 
#endif


#endif // #ifndef CONFIG_H 
