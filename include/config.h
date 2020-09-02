#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h> // generic Arduino libraries

// measures and transmits runtimes and other statistics...

#ifdef SENSOR_MEASURE_PERFORMANCE
    #pragma message "SENSOR_MEASURE_PERFORMANCE defined: will include diagnostic and performance data."
#else
    #pragma message "SENSOR_MEASURE_PERFORMANCE not defined: Performance and general diagnostics not included."
#endif


#ifndef DFW_VERSION
#pragma message "FW_VERSION not defined. using default"
#define DFW_VERSION 2020052801
#endif

#ifndef FWURLBASE
#pragma message "FWURLBASE is not defined. using default value http://192.168.2.40/ESPOTA/"
#define FWURLBASE "http://192.168.2.40/ESPOTA/"
#endif

#ifdef USE_SECRET_SSID
#include "secrets.h"
    #ifndef SECRET_SSID
    #pragma message "SSID not defined. Going to use WiFiManager instead!"
    #define USE_WIFIMANAGER
    #endif

    #ifndef SECRET_PASS
    #pragma message "WIFI passphrase not defined. Going to use WiFiManager instead!"
    #define USE_WIFIMANAGER
    #endif
#else
    #define USE_WIFIMANAGER
    #pragma message "USE_SECRET_SSID is not defined. Using WiFiManager."    
#endif

// the date pin for the one wire temperature sensor.
// if not set via buid_flag (platformio.ini), pls set it here
// #define PINDHT22 23
#ifndef PINDHT22
#pragma message "Need to define the DHT22 Pin. Using default #23 now"
#define PINDHT22 23
#endif

// how many values for history / wighted mean value
#define NUM_VALUES          5

#ifndef SENSOR_READ_INTERVAL
    #pragma message "Using default SENSOR_READ_INTERVAL (60)"
    // how many seconds to sleep
    #define SENSOR_READ_INTERVAL  60
#endif
#ifndef SENSOR_ERROR_INTERVAL
    #pragma message "Using default SENSOR_ERROR_INTERVAL (30)"
    // how many seconds to sleep in case of error  
    #define SENSOR_ERROR_INTERVAL 30
#endif
#ifndef SENSOR_MIN_UPDATE
    #pragma message "Using default SENSOR_MIN_UPDATE (10)"
    // after how many sleep cycles an update will be send regardless the values
    #define SENSOR_MIN_UPDATE     10
#endif
#ifndef SENSOR_OTA_INTERVAL
    #pragma message "Using default SENSOR_OTA_INTERVAL (200)"
    // after how many sleep cycles to look if a new version is available
    // this can of course be set to once a day or so. during debug, a shorter cycle is easier....  
    #define SENSOR_OTA_INTERVAL   200  
#endif

#ifndef SENSOR_WIFI_RETRY_COUNT
    #pragma message "Using default SENSOR_WIFI_RETRY_COUNT (3)"
    #define SENSOR_WIFI_RETRY_COUNT 3
#endif
#if SENSOR_WIFI_RETRY_COUNT < 1
    #error "SENSOR_WIFI_RETRY_COUNT needs to be > 0"
#endif 

#ifndef SENSOR_SERVER_RETRY_COUNT
    #pragma message "Using default SENSOR_SERVER_RETRY_COUNT (3)"
    #define SENSOR_SERVER_RETRY_COUNT 3
#endif
#if SENSOR_SERVER_RETRY_COUNT < 1
    #error "SENSOR_SERVER_RETRY_COUNT needs to be > 0"
#endif 

#ifndef SENSOR_T_PRECISION
    #pragma message "Using default SENSOR_T_PRECISION (5) equals 0.5 degrees"
    #define SENSOR_T_PRECISION 5
#endif
#if SENSOR_T_PRECISION < 1
    #error "SENSOR_T_PRECISION needs to be > 0"
#endif 

#ifndef SENSOR_H_PRECISION
    #pragma message "Using default SENSOR_H_PRECISION (10) equals 1.0 percent"
    #define SENSOR_H_PRECISION 10
#endif
#if SENSOR_H_PRECISION < 1
    #error "SENSOR_H_PRECISION needs to be > 0"
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
