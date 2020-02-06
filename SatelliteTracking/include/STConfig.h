#ifndef SAT_TRACK_CONFIG_H
#define SAT_TRACK_CONFIG_H

#include <Arduino.h>
#include "STAccelGyroConfig.h"

namespace SatTrack
{
    // ---------------------------------------------------------
    // WiFi configuration --------------------------------------
    const char* ST_SSID     = "your-ssid"; //Wi-Fi SSID
    const char* ST_PASSWORD = "your-password"; //Wi-Fi Password
    // ---------------------------------------------------------

    // ---------------------------------------------------------
    //Server IP or domain name ---------------------------------
    const char* HOST = "celestrak.com";
    //File or Server page you want to communicate with. along with data
    const String URL = "https://celestrak.com/NORAD/elements/amateur.txt"; 
    // ---------------------------------------------------------

    // ---------------------------------------------------------
    // Accelerometer and gyroscope orientation configuration
    // Find this info at STAccelGyroConfig.h
    const double ROLL_OFFSET = X_AXIS_RIGHT;
    // ---------------------------------------------------------



    // ---------------------------------------------------------
    // Messages language configuration -------------------------
    // Select one value from:
    #define LANG_ENG  // English
    // #define LANG_ESP  // Spanish
    // ---------------------------------------------------------


    // ---------------------------------------------------------
    // Debug serial line messages configuration ----------------
    // ESP8266 writes data and debug information into the Serial Line.
    // When you were sending data to the MEGA 2560 or to build a release
    // these parameters MUST be false.
    #define ESP8266_TRACE_TEST false
    #define ESP8266_TRACE_INFO false
    #define ESP8266_TRACE_WARNING false
    #define ESP8266_TRACE_ERROR false

    // It is possible to show MEGA2560 messages into the serial data.
    // These parameters can be true or false to create a release .
    #define MEGA2560_TRACE_TEST false
    #define MEGA2560_TRACE_INFO true
    #define MEGA2560_TRACE_WARNING true
    #define MEGA2560_TRACE_ERROR true
    // ---------------------------------------------------------
}
#endif // end SAT_TRACK_CONFIG_H
