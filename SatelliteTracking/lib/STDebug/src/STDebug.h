#ifndef STDEBUG_H
#define STDEBUG_H

//// #include "STDebug.h"
//// #include "STSerialCommands.h"
//// #include "STWriteSerial.h"
#include "STSerialMsg.h"

#include <Arduino.h>

namespace SatTrack
{
   class STDebug : public STSerialMsg
    {
    private:
        ///STSerialMsg m_traceMsg; //(STSerialMsg::PORT_NUMBER portNumber);
    public:
        //// const unsigned long DBG_BAUD_RATE = 115200;
        //// const unsigned long ESP_BAUD_RATE = 115200;
        //// const unsigned long GPS_BAUD_RATE = 9600;
        
        // Type of info to show
        static const unsigned char TYPE_TRACE_TEST = 100;
        static const unsigned char TYPE_TRACE_INFO = 101;
        //// static const unsigned char TYPE_TRACE_DEBUG = 110;
        static const unsigned char TYPE_TRACE_WARNING = 120;
        static const unsigned char TYPE_TRACE_ERROR = 130;

        // Hardware origin info
        static const unsigned char ORIGIN_MEGA2560 = 1;
        static const unsigned char ORIGIN_ESP8266 = 2;

        // Messages to show
        static constexpr const char* DBG_MSG_CONNECT_2 = "Connecting to ";
        // static const char* DBG_MSG_CONNECT_2 = "Connecting to ";
        static constexpr const char* DBG_MSG_CL_TIMEOUT = ">>> Client Timeout !";
        static constexpr const char* DBG_MSG_IP_ADDRS = "WiFi connected. IP address: ";
        static constexpr const char* DBG_MSG_HOST = "Host: ";
        static constexpr const char* DBG_MSG_CONNECT_FAIL = "connection failed";
        static constexpr const char* DBG_MSG_HOST_CLOSE = "closing connection";
        static constexpr const char* DBG_MSG_TLE_LINE_NR = " TLE line number = ";
        static constexpr const char* DBG_MSG_TLE_LINE_LEN = " TLE line length = ";
        static constexpr const char* DBG_MSG_TLE_LINE_0 = "TLE line 0 ";
        static constexpr const char* DBG_MSG_TLE_LINE_1 = "TLE line 1 ";
        static constexpr const char* DBG_MSG_TLE_LINE_2 = "TLE line 2 ";
        static constexpr const char* DBG_MSG_TLE0_MISSING = "TLE line 0. Missing:";
        static constexpr const char* DBG_MSG_TLE1_MISSING = "TLE line 1. Missing:";
        static constexpr const char* DBG_MSG_TLE2_MISSING = "TLE line 2. Missing:";

        static constexpr const char* DBG_MSG_TLE_SEEK_L1_L0 = "Seeking line number 0. Do not found line1";
        static constexpr const char* DBG_MSG_TLE_SEEK_L2_L0 = "Seeking line number 0. Do not found line2";

        static constexpr const char* DBG_MSG_ESP_STARTING_UP = "ESP8266 starting up. Waiting for Serial3.available()";
        static constexpr const char* DBG_MSG_ESP_STARTUP_TIME = "ESP8266 startup time (secs) = ";

        static constexpr const char* DBG_MSG_TLE_FILE_CONTENT = "TLE file:";

        static constexpr const char* DBG_MSG_SPIFFS_MOUNT_OK = "ESP8266 SPIFFS file system mounted with success";
        static constexpr const char* DBG_MSG_SPIFFS_WRITE_LN_OK = "ESP8266 SPIFFS file written line";

        static constexpr const char* DBG_MSG_PUT_SAT_NAMES = "ESP8266 Sent satellite names list";
        static constexpr const char* DBG_MSG_SAT_NAMES = "Satellite list names";

        static constexpr const char* DBG_MSG_ERR_SPIFFS_MOUNT = "ESP8266 Error mounting SPIFFS file system";
        static constexpr const char* DBG_MSG_ERR_SPIFFS_OPEN_W = "ESP8266 Error opening file for writing";
        static constexpr const char* DBG_MSG_ERR_SPIFFS_OPEN_R = "ESP8266 Error opening file for reading";
        static constexpr const char* DBG_MSG_ERR_SPIFFS_WRITE = "ESP8266 Error writing file";

        static constexpr const char* DBG_MSG_ERR_SPIFFS_SNF = "ESP8266. SPIFFS. Satellite not found in file";


        // Error mounting SPIFFS file system
        static const byte ERR_SPIFFS_MOUNTING = 200;
        static const byte ERR_SPIFFS_OPEN_F_W = 201;
        static const byte ERR_SPIFFS_OPEN_F_R = 202;
        static const byte ERR_SPIFFS_WRITE_FILE = 203;

        // TLE server connection fails
        static const byte ERR_SERVER_CONN_FAIL = 220;
        // TLE server connection time out
        static const byte ERR_SERVER_CONN_T_OUT = 221;

        // SPIFFS. Satellite not found in file
        static const byte ERR_SPIFFS_SAT_NF = 230;

        STDebug();
        ~STDebug();


        /**
         * @author Javi Perez
         * @brief Print information in the Serial Line.
         * @version 1.0.1
         * 
         * @param typeOfInfo numeric code showing the kind of info. Please @see STDebug.h.
         * @param hwOrigin numeric code showing the hardware origin of this info. Please @see STDebug.h.
         * @param degubInfo String pointer describing information.  Please @see STDebug.h.
        */
       void printDebug(const unsigned char typeOfInfo, const unsigned char hwOrigin, String &degubInfo);
    };    
} // end namespace SatTrack
#endif // STDEBUG_H