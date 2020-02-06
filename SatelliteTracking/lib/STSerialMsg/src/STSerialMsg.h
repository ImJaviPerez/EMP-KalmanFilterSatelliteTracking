// STSerialMsg.h
#ifndef STSERIALMSG_H
#define STSERIALMSG_H

#include <HardwareSerial.h>
#include <Arduino.h>


// TO-DO:
///     BORRAME desde aqui
#define SHOW_SERIAL_TRACE_READ true
#define SHOW_SERIAL_TRACE_WRITE false
///     BORRAME hasta aqui


namespace SatTrack
{
    class STSerialMsg
    {
    public:
        // Return error messages -----------------------------
        // Return OK
        const int OK_ST_SERIAL = 1;
        // MEGA2560 serial communication baud rate
        static const unsigned long MEGA_BAUD_RATE = 115200;
        // ESP8266 serial communication baud rate
        static const unsigned long ESP_BAUD_RATE = 115200;
        // Serial port communication baud rate. To show traces
        static const unsigned long DBG_BAUD_RATE = 115200;
        // GPS 6MV2 serial communication baud rate
        static const unsigned long GPS_BAUD_RATE = 9600;

        
        // Number of Serial port to use
        enum PORT_NUMBER : int{
            PORT_ZERO,
            PORT_ONE,
            PORT_TWO,
            PORT_THREE
        };
        /*
       // Number of Serial port to use
       const int PORT_ZERO = 0;
       const int PORT_ONE = 1;
       const int PORT_TWO = 2;
       const int PORT_THREE = 3;
       */

        // Defining separators -----------------------------------
        // It is a MUST to use only one char for commands and separators
        static const char MSG_HEADER_BEGIN = '[';
        // const char MSG_HEADER_END = '>';
        // const char MSG_TAIL_BEGIN = '<';
        static const char MSG_TAIL_END = ']';

        // Separator for satellite names
        static const char ST_SEP = ',';

        // Defining commands -------------------------------------
        // List of commands between ESP8266 to MEGA2560 or vice versa
        // Every command is surrounded by a symbol at the beggining and at the end
        // Example:
        //      When MEGA2560 requests to ESP8266 for "OSCAR 7 (AO-7)" satellite TLE,
        //      it sends the command:
        //      [SOSCAR 7 (AO-7)]
        // It is a MUST to use only one char for commands and separators

        // Request from MEGA2560 to ESP8266 for all satellite names
        static const char REQ_TLE_SAT_NAMES = 'N';
        // Request from MEGA2560 to ESP8266 for one satellite TLE; 3 lines
        static const char REQ_TLE_SATELLITE = 'S';
        // Request from MEGA2560 to ESP8266 requestRefreshTLE satellite TLE; 3 lines
        static const char REQ_REFRESH_SAT_TLE = 'W';
        // Send from ESP8266 to MEGA2560 all satellites names
        static const char PUT_SAT_NAMES = 'P';
        // Send from ESP8266 to MEGA2560 one satellite TLE: Line 0
        static const char PUT_TLE0_SATELLITE = 'A';
        // Send from ESP8266 to MEGA2560 one satellite TLE: Line 1
        static const char PUT_TLE1_SATELLITE = 'B';
        // Send from ESP8266 to MEGA2560 one satellite TLE: Line 2
        static const char PUT_TLE2_SATELLITE = 'C';
        
        // Debug message
        static const char DBG_MSG = '#';


        // TO-DO
        //      Use a reference to a HardwareSerial::Serial as a constructor argument.
        //      Do not use PORT_NUMBER as constructor argument.
        STSerialMsg(int portNumber);
        // STSerialMsg(HardwareSerial &SerialPort);
        ~STSerialMsg();

        // Initialize serial port
        void init();

        // Indicates if the specified Serial port is ready.
        bool ready();
        // Return if there is data available for reading on Serial line
        int available();
        
        #if defined(ARDUINO_AVR_MEGA2560)
        // Methods FROM_MEGA_TO_ESP8226
            bool requestSatelliteNames();
            bool requestSatelliteTLE(String &satelliteName);
            bool requestRefreshTLE(String &satelliteName);
        #elif defined(ESP8266)
            // Methods FROM_ESP8266_TO_MEGA
            bool putSatelliteNames(String &satelliteListNames);
            bool putSatelliteTLE(int lineNumber, String &line);
            bool putSatelliteTLE(String &line);
        #endif

        // #if defined(ESP8266_TRACE) || defined(MEGA2560_TRACE)
        bool printTrace(const byte typeOfInfo, const byte hwOrigin, String &degubInfo);
        // #endif

        unsigned long baudRate();

        //// bool readSerialMsg(char &command, String &info);
        bool readSerialMsg();
        //// bool writeSerialMsg(String &msg);
        bool writeSerialMsg();

        // Member functions to read info from *m_pSerial
        bool newCommandRead();
        char commandRead();
        String commandInfoRead();
        // Reset last command and info Read
        void resetCommandRead();

        // Member functions to write info to *m_pSerial
        //// bool newCommandWrite();
        //// char commandWrite();
        //// String commandInfoWrite();
        
        // FIX-ME ############# ESTAS AQUI ##########################################
        //  Create m_newCommandRead, m_newCommandWrite, m_commandRead, m_commandWrite
        //      resetCommandRead(), private: m_resetCommandWrite()
        //      newCommandRead(), (NO: newCommandWrite()), commandRead(), (NO: commandWrite()), commandInfoRead(), (NO: commandInfoWrite())
        //  And modify readSerialMsg(), writeSerialMsg() and SatTrack.cpp and ST_ESP8266.cpp
        //

    private:
        // m_pSerial is a pointer to Serial, or Serial1 or Serial2 or Serial3
        HardwareSerial *m_pSerial;
        // Baud rate
        unsigned long m_baudRate;
        // It is a new command read from *m_pSerial
        bool m_newCommandRead;
        // Command value read from *m_pSerial
        char m_commandRead;
        // Information attached to the command read from *m_pSerial
        String m_infoRead;
        // It is a new command that we have to write to *m_pSerial
        bool m_newCommandWrite;
        // Command value that we have to write to *m_pSerial
        char m_commandWrite;
        // Information attached to the command  that we have to write to *m_pSerial
        String m_infoWrite;
        
        // Reset last command and info Written
        void m_resetCommandWrite();
    };

} // end namespace SatTrack
    
#endif // STSERIALMSG_H
