#ifndef SAT_TRACK_CONFIG_H
#define SAT_TRACK_CONFIG_H

#include <Arduino.h>
#include "STAccelGyroConfig.h"

//// #include "STCompassConfig.h"

namespace SatTrack
{
    
    // ---------------------------------------------------------
    // Local UTC -----------------------------------------------
    //// #define UTC_LOCAL +2
    //// #define UTC_CITY "MADRID"
    // ---------------------------------------------------------

    #ifdef THIS_PROJECT_HAS_ESP8266
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
    #endif

    #ifdef THIS_PROJECT_HAS_ACCEL_GYRO
    // ---------------------------------------------------------
    // Accelerometer and gyroscope orientation configuration
    // Find this info at STAccelGyroConfig.h
    const gyroXposition ACC_GYRO_X_AXIS_POSITION = ACC_GYRO_X_AXIS_LEFT;
    // ---------------------------------------------------------
    #endif

    #ifdef THIS_PROJECT_HAS_COMPASS
    // ---------------------------------------------------------
    // Magnetometer compass orientation configuration
    // Find this info at QMC5883l_st.h
    const compassXposition COMPASS_X_AXIS_POSITION = COMPASS_X_AXIS_BACKWARD;
    const byte compassInterruptionPin = 2;
    // Spain: https://www.ign.es/web/gmt-declinacion-magnetica
    const float DECLINATION_BILBAO = -19/60; // 0º 19' West
    const float DECLINATION_DONOSTIA_SS = -9/60; // 0º 9' West
    const float DECLINATION_VITORIA_GASTEIZ = -21/60; // +0º 21' West
    // World: http://www.magnetic-declination.com/
    const float DECLINATION_BAYONE = 28/60; // +0º 28' East
    const float DECLINATION_BAORDEAUX = 26/60; // +0º 26' East

    // http://www.magnetic-declination.com/
    const float LOCAL_DECLINATION = DECLINATION_BILBAO;
    // ---------------------------------------------------------
    #endif


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
