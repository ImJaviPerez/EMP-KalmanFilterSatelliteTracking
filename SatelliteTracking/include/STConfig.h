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
    const accGyroXposition ACC_GYRO_X_AXIS_POSITION = ACC_GYRO_X_AXIS_LEFT;
    //
    // Accelerometer raw values standard deviation
    const float ACC_SD_X = 12.0F; //25.0F; //35.0F; //25.0F; //20.0F; //12.29109F; //6.0F; //12.29109F; //16.38812F; //25.0F; //49.164355950F;
    const float ACC_SD_Y = 12.0F; //25.0F; //35.0F; //25.0F; //20.0F; //13.96672F; //6.0F; //13.96672F; //18.62230F; //26.0F; //55.866890392F;
    const float ACC_SD_Z = 20.0F; //41.0F; //55.0F; //41.0F; //35.0F; //20.56029F; //6.0F; //20.56029F; //27.41372F; //41.0F; //82.241174857F;
    // Gyroscope standard deviation (rad/sec)
    const float GYRO_SD_X = 0.005F; //0.002F; //0.005F; //0.001F; //0.0006F; //0.0010465581F; //0.002093116F; //0.0010465581F; //0.02691271F;
    const float GYRO_SD_Y = 0.005F; //0.002F; //0.005F; //0.001F; //0.0006F; //0.0008671617F; //0.001734323F; //0.0008671617F; //0.01265696F;
    const float GYRO_SD_Z = 0.005F; //0.002F; //0.005F; //0.001F; //0.0006F; //0.0008187369F; //0.001637474F; //0.0008187369F; //0.01905207F;
    // ---------------------------------------------------------
    #endif

    #ifdef THIS_PROJECT_HAS_COMPASS
    // ---------------------------------------------------------
    // Magnetometer compass orientation configuration
    // Find this info at QMC5883l_st.h
    const compassXposition COMPASS_X_AXIS_POSITION = COMPASS_X_AXIS_BACKWARD;
    //
    // Magnetometer X, Y an Z raw values standard deviation configuration
    const float COMPASS_SD_X = 10.0F; //14.0F;//8.0F; //11.0F;// 20.0F; //11.369459546F; //5.6847297729F; //11.369459546F;
    const float COMPASS_SD_Y = 14.0F; //14.0F;//8.0F; //11.0F;// 20.0F; //12.118513032F; //6.0592565162F; //12.118513032F;
    const float COMPASS_SD_Z = 14.0F; //16.0F;//9.0F; //13.0F;// 6.0F; //13.846676811F; //6.9233384053F; //13.846676811F;

    // Magnetometer interruption pin number
    const byte compassInterruptionPin = 2;
    // Local magnetic declination (degrees). Year 2020
    // Spain: https://www.ign.es/web/gmt-declinacion-magnetica
    const float DECLINATION_BILBAO = -15.0f/60.0f; // 0º 15' West
    const float DECLINATION_DONOSTIA_SS = -9.0f/60.0f; // 0º 9' West
    const float DECLINATION_VITORIA_GASTEIZ = -21.0f/60.0f; // +0º 21' West
    // World: http://www.magnetic-declination.com/
    const float DECLINATION_BAYONE = 28.0f/60.0f; // +0º 28' East
    const float DECLINATION_BAORDEAUX = 26.0f/60.0f; // +0º 26' East

    const float INCLINATION_BILBAO = 58.4764; //58º28'35''

    // Local declination in radians
    const float LOCAL_DECLINATION = DECLINATION_BILBAO * DEG_TO_RAD;
    //  Local inclination in radians, 
    // CHANGE SIGN: Positive values of inclination indicate
    // that the magnetic field of the Earth is pointing downward,
    // into the Earth, at the point of measurement, 
    // and negative values indicate that it is pointing upward.
    const float LOCAL_INCLINATION = -INCLINATION_BILBAO * DEG_TO_RAD;
    
    // ---------------------------------------------------------
    #endif

    // Kalman state standard deviation
    const float ANGLE_SD_X = 1.0F * DEG_TO_RAD;// 0.4165443F; //0.01F * DEG_TO_RAD;
    const float ANGLE_SD_Y = 1.0F * DEG_TO_RAD;// 0.4165443F; //0.01F * DEG_TO_RAD;
    const float ANGLE_SD_Z = 1.0F * DEG_TO_RAD;// 0.4165443F; //0.01F * DEG_TO_RAD;
    const float ANGLE_RATE_SD_X = 1.1E-2F * DEG_TO_RAD;// 0.006444372F; //1.1E-5F * DEG_TO_RAD;
    const float ANGLE_RATE_SD_Y = 1.1E-2F * DEG_TO_RAD;// 0.002441045F; //1.1E-5F * DEG_TO_RAD;
    const float ANGLE_RATE_SD_Z = 1.1E-2F * DEG_TO_RAD;// 0.03900503F; //1.1E-5F * DEG_TO_RAD;


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
