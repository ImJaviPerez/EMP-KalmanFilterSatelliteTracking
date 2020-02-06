// #include <HardwareSerial.h>

#include "STSerialMsg.h"

// #include <Arduino.h>

using namespace SatTrack;


// Constructor
STSerialMsg::STSerialMsg(int portNumber)
{
    switch (portNumber)
    {
    case PORT_ZERO:
        m_pSerial = &Serial;
        m_baudRate = DBG_BAUD_RATE;
        break;
    case PORT_ONE:
        m_pSerial = &Serial1;
        m_baudRate = ESP_BAUD_RATE;
        break;

    case PORT_TWO:
    #ifdef ARDUINO_AVR_MEGA2560
        m_pSerial = &Serial2;
        m_baudRate = GPS_BAUD_RATE;
        break;
    #endif
    case PORT_THREE:
    #ifdef ARDUINO_AVR_MEGA2560
        m_pSerial = &Serial3;
        m_baudRate = MEGA_BAUD_RATE;
        break;
    #endif

    default:
        break;
    }
    resetCommandRead();
    m_resetCommandWrite();

    // init();
}


// Destructor
STSerialMsg::~STSerialMsg()
{
}

/**
 * @brief Initialize serial port
 * 
 */
void STSerialMsg::init()
{
    // m_pSerial->begin(m_baudRate);
    m_pSerial->begin(m_baudRate);
    delay(900);
    //// m_pSerial->println();
    m_pSerial->println();
}

int STSerialMsg::available()
{
    // Get the number of bytes (characters) available for reading from the serial port.
    // This is data that’s already arrived and stored in the serial receive buffer (which holds 64 bytes).
    return m_pSerial->available();
}

bool STSerialMsg::ready()
{
    // Indicates if the specified Serial port is ready.
    return (*m_pSerial);
}

unsigned long STSerialMsg::baudRate()
{
    return m_baudRate;
}

#if defined(ARDUINO_AVR_MEGA2560)
bool STSerialMsg::requestSatelliteNames()
{
    m_commandWrite = REQ_TLE_SAT_NAMES;
    m_infoWrite = "";
    return writeSerialMsg();

    //// String msg = String(MSG_HEADER_BEGIN) + String(m_command) + String(MSG_TAIL_END);
    //// bool result = writeSerialMsg(msg);
    //// return result;
}

bool STSerialMsg::requestSatelliteTLE(String &satelliteName)
{
    m_commandWrite = REQ_TLE_SATELLITE;
    m_infoWrite = satelliteName;
    return writeSerialMsg();
    //// String msg = String(MSG_HEADER_BEGIN) + String(m_command) + String(satelliteName) + String(MSG_TAIL_END);
    //// bool result = writeSerialMsg(msg);
    //// return result;
}


bool STSerialMsg::requestRefreshTLE(String &satelliteName)
{
    m_commandWrite = REQ_REFRESH_SAT_TLE;
    m_infoWrite = satelliteName;
    return writeSerialMsg();
    //// String msg = String(MSG_HEADER_BEGIN) + String(m_command) + String(satelliteName) + String(MSG_TAIL_END);
    //// bool result = writeSerialMsg(msg);
    //// return result;
}
// #end if defined(ARDUINO_AVR_MEGA2560)
#elif defined(ESP8266)
bool STSerialMsg::putSatelliteNames(String &satelliteListNames)
{
    m_commandWrite = PUT_SAT_NAMES;
    m_infoWrite = satelliteListNames;
    return writeSerialMsg();
    //// String msg = String(MSG_HEADER_BEGIN) + String(m_command) + String(satelliteListNames) + String(MSG_TAIL_END);
    //// bool result = writeSerialMsg(msg);
    //// return result;
}

bool STSerialMsg::putSatelliteTLE(int lineNumber, String &line)
{
    switch (lineNumber)
    {
    case 0:
        m_commandWrite = PUT_TLE0_SATELLITE;
        break;
    case 1:
        m_commandWrite = PUT_TLE1_SATELLITE;
        break;
    case 2:
        m_commandWrite = PUT_TLE2_SATELLITE;
        break;
    
    default:
        // TO-DO
        //      Send  error message
        break;
    }

    m_infoWrite = line;
    return writeSerialMsg();
}

// FIX-ME ####################
//      Check this funtion
bool STSerialMsg::putSatelliteTLE(String &TLEline)
{
    char firstChar = TLEline.charAt(1);
    switch (firstChar)
    {
    case '1':
        m_commandWrite = PUT_TLE1_SATELLITE;
        break;
    case '2':
        m_commandWrite = PUT_TLE2_SATELLITE;
        break;
    default:
        m_commandWrite = PUT_TLE0_SATELLITE;
        break;
    }

    m_infoWrite = TLEline;
    return writeSerialMsg();
}
#endif // defined(ESP8266)
bool STSerialMsg::printTrace(const byte typeOfInfo, const byte hwOrigin, String &degubInfo)
{
    m_commandWrite = DBG_MSG;
    //// m_info = degubInfo;
    //// String msg = String(MSG_HEADER_BEGIN) + String(m_command) +
    ////         String(typeOfInfo) + 
    ////         String("|") + String(hwOrigin) + 
    ////         String("|") + degubInfo +
    ////         String(MSG_TAIL_END);

    // Write the message to the Serial Port. 
    // It will be sent to Serial port to be shown by user
    m_infoWrite = String(typeOfInfo) + String("|") + String(hwOrigin) + String("|") + degubInfo;
    return writeSerialMsg();
    //// return writeSerialMsg(msg);
}


bool STSerialMsg::newCommandRead()
{
    return m_newCommandRead;
}

char STSerialMsg::commandRead()
{
    return m_commandRead;
}

String STSerialMsg::commandInfoRead()
{
    return m_infoRead;
}

void STSerialMsg::resetCommandRead()
{
    m_newCommandRead = false;
    m_commandRead = ' ';
    m_infoRead = "";
}

void STSerialMsg::m_resetCommandWrite()
{
    m_newCommandWrite = false;
    m_commandWrite = ' ';
    m_infoWrite = "";
}

//// bool STSerialMsg::readSerialMsg(char &command, String &info)
bool STSerialMsg::readSerialMsg()
{
    //// return stReadSerial((*m_pSerial), command, info);

    // Read (*m_pSerial) and fetch a command and info
    // Every message read from the (*m_pSerial) port have:
    // MSG_HEADER_BEGIN + command + information + MSG_TAIL_END
    // MSG_HEADER_BEGIN, command and MSG_TAIL_END have one char each of them
    // For example:
    //      To request TLE lines from "OSCAR 7 (AO-7)" satellite
    //      we will read:
    //          MSG_HEADER_BEGIN + REQ_TLE_SATELLITE + OSCAR 7 (AO-7) + MSG_TAIL_END
    //          "<SOSCAR 7 (AO-7)]"
    //      where: MSG_HEADER_BEGIN = '<', 
    //              REQ_TLE_SATELLITE = 'S',
    //              MSG_TAIL_END = ']'
    //      so the command and information are between MSG_HEADER_BEGIN and MSG_TAIL_END

    #if defined(ESP8266)
    #if SHOW_SERIAL_TRACE_READ
        #if defined(HAVE_HWSERIAL0)
            Serial.println("HAVE_HWSERIAL0");
        #endif
        #if defined(HAVE_HWSERIAL1)
            Serial.println("HAVE_HWSERIAL1");
        #endif
        #if defined(HAVE_HWSERIAL2)
            Serial.println("HAVE_HWSERIAL2");
        #endif
        #if defined(HAVE_HWSERIAL3)
            Serial.println("HAVE_HWSERIAL3");
        #endif
    #endif
    #endif

    enum READING_STATUS {
        NOT_FOUND_MSG_HEADER_BEGIN,
        FOUND_MSG_HEADER_BEGIN,
        FOUND_COMMAND,
        FOUND_INFO,
        FOUND_MSG_TAIL_END,
        FOUND_DBG_MSG
    };
    READING_STATUS readingStatus = NOT_FOUND_MSG_HEADER_BEGIN;

    #if SHOW_SERIAL_TRACE_READ
        String debugMessage = "";
    #endif
    #if false // SHOW_SERIAL_TRACE_READ
        if (m_pSerial->available() > 0)
        {
            Serial.print("readSerialMsg().m_pSerial->available()=");
            Serial.println(m_pSerial->available());
            if(m_pSerial == &Serial){
                Serial.println("readSerialMsg().m_pSerial == &Serial");
            }else if(m_pSerial == &Serial1){
                Serial.println("readSerialMsg().m_pSerial == &Serial1");
            }
            #ifdef ARDUINO_AVR_MEGA2560
                if(m_pSerial == &Serial2){
                    Serial.println("readSerialMsg().m_pSerial == &Serial2");
                }else if(m_pSerial == &Serial3){
                    Serial.println("readSerialMsg().m_pSerial == &Serial3");
                }
            #endif
        }else
        {
            //// Serial.print("NOT-readSerialMsg(). m_pSerial->available()=");
            //// Serial.println(m_pSerial->available());
        }
        
    #endif

    //// while (m_pSerial->available()) {
    //// while ((m_pSerial->available() > 0) && ((readingStatus != FOUND_MSG_TAIL_END) || (readingStatus != FOUND_DBG_MSG))  && (!m_newCommandRead)) {
    while ((m_pSerial->available() > 0) && (readingStatus != FOUND_MSG_TAIL_END) && (!m_newCommandRead)) {
        #if SHOW_SERIAL_TRACE_READ
            //// debugMessage = String("m_pSerial->available()=") + String(m_pSerial->available());
            //// Serial.println(debugMessage);
        #endif
        // Get next char:
        char inputChar = (char)m_pSerial->read();

        switch (readingStatus)
        {
        case NOT_FOUND_MSG_HEADER_BEGIN:
            // if the incoming character is a MSG_HEADER_BEGIN:
            if (inputChar == MSG_HEADER_BEGIN) {
                // Change reading status
                readingStatus = FOUND_MSG_HEADER_BEGIN;
            }
            #if false // SHOW_SERIAL_TRACE_READ
                debugMessage = String(NOT_FOUND_MSG_HEADER_BEGIN) + String(".readingStatus=") + String(readingStatus);
                Serial.println(debugMessage);
            #endif
            break;
        case FOUND_MSG_HEADER_BEGIN:
            // Find command request
            switch (inputChar)
            {
            case (REQ_TLE_SAT_NAMES):
            case (REQ_TLE_SATELLITE):
            case (REQ_REFRESH_SAT_TLE):
            case (PUT_SAT_NAMES):
            case (PUT_TLE0_SATELLITE):
            case (PUT_TLE1_SATELLITE):
            case (PUT_TLE2_SATELLITE):

                // Change reading status
                readingStatus = FOUND_COMMAND;
                // Save the command
                //// command = inputChar;
                m_commandRead = inputChar;
                break;
            
            case (DBG_MSG):
                // Do not do anything with debug messages
                readingStatus = FOUND_DBG_MSG;
                m_commandRead = inputChar;
                break;
            
            default:
                // Not found command after MSG_HEADER_BEGIN. Return error
                return false;
                break;
            }
            #if SHOW_SERIAL_TRACE_READ
                debugMessage = String(FOUND_MSG_HEADER_BEGIN) + String(".readingStatus=") + String(readingStatus);
                Serial.println(debugMessage);
            #endif
            // In other case do nothing
            break;

        case FOUND_COMMAND:
        case FOUND_DBG_MSG:
        case FOUND_INFO:
            if (inputChar != MSG_TAIL_END) {
                // INFO
                //      In this case it is no neccesary to change the readingStatus
                // // Change reading status
                // readingStatus = FOUND_INFO; 
                // Append to the info String:
                //// info += inputChar;
                m_infoRead += inputChar;
                readingStatus = FOUND_INFO;
            }else{  // inputChar == MSG_TAIL_END
                // Change reading status
                readingStatus = FOUND_MSG_TAIL_END;
                // Found new command
                m_newCommandRead = true;
            }
            #if false //SHOW_SERIAL_TRACE_READ
                debugMessage = String(FOUND_COMMAND) + String(".readingStatus=") + String(readingStatus);
                Serial.println(debugMessage);
            #endif
            break;

        //// case FOUND_DBG_MSG:
        case FOUND_MSG_TAIL_END:
            break;
        
        default:
            break;
        }

        #if SHOW_SERIAL_TRACE_READ
            debugMessage = String("END.m_commandR=") + String(m_commandRead) + String(".m_infoR=") + String(m_infoRead);
            Serial.println(debugMessage);
            debugMessage = String("END.readingStatus=") + String(readingStatus);
            Serial.println(debugMessage);
        #endif

        //// if (readingStatus == FOUND_MSG_TAIL_END) {
        ////     return true;
        //// }
    } // end (m_pSerial->available())
    //// return false;


    #if SHOW_SERIAL_TRACE_READ
        // if ((readingStatus == FOUND_MSG_TAIL_END) || (readingStatus == FOUND_DBG_MSG))
        if ((readingStatus == FOUND_MSG_TAIL_END))
        {
            debugMessage = String("E2.readingStatus=") + String(readingStatus) + 
                String(".newCommandR=") + String(m_newCommandRead) + 
                String(".commandR=") + String(m_commandRead) + String(".infoR=") + String(m_infoRead);
            Serial.println(debugMessage);
        }
    #endif
    //// m_newCommand = (readingStatus == FOUND_MSG_TAIL_END);

    return m_newCommandRead;

    /*
    if ((readingStatus == FOUND_MSG_TAIL_END) || (readingStatus == FOUND_DBG_MSG))
    {
        return true;
    }
    else
    {
        return false;
    }
    */
    // return (readingStatus == FOUND_MSG_TAIL_END);
}


//// bool STSerialMsg::writeSerialMsg (String &msg)
bool STSerialMsg::writeSerialMsg ()
{
    // Every message written into the (*m_pSerial) port will have:
    // MSG_HEADER_BEGIN + command + info + MSG_TAIL_END
    // MSG_HEADER_BEGIN, command and MSG_TAIL_END have one char each of them
    // For example:
    //      To request TLE lines from "OSCAR 7 (AO-7)" satellite
    //      we will write:
    //          MSG_HEADER_BEGIN + REQ_TLE_SATELLITE + OSCAR 7 (AO-7) + MSG_TAIL_END
    //          "[SOSCAR 7 (AO-7)]"
    //      where: MSG_HEADER_BEGIN = '[', 
    //              REQ_TLE_SATELLITE = 'S',
    //              MSG_TAIL_END = ']'

    String msg = String(MSG_HEADER_BEGIN) + String(m_commandWrite) + String(m_infoWrite) + String(MSG_TAIL_END);

    //// // msg.length() <= 0 then do not write anything
    //// if (msg.length() <= 0)
    //// {
    ////     Serial.println("ERROR: msg.length() <= 0");
    ////     return true;
    //// }
    
    bool msgSent = false;
    //// // We can write if there is nothing to read
    //// while ((m_pSerial->available() <= 0) & !msgSent)
    while (!msgSent)
    {       
        // Write the msg to the (*m_pSerial) Port.
        unsigned int nextStartIdx = 0;

        while (nextStartIdx < msg.length())
        {
            int safw = m_pSerial->availableForWrite();
            // We can write someting if m_pSerial->availableForWrite() > 0
            if ((safw > 0) & (msg.length() > 0))
            {
                size_t strLen, msgLen;

                // Check if there is enough (*m_pSerial) available buffer to write
                if (nextStartIdx + safw >= msg.length())
                {   
                    // Prepare message to send
                     String strAux = String(msg.substring(nextStartIdx));
                    // write remainder msg
                    // strAux.length() + '\0' + '\n'
                    strLen = strAux.length() + 2;
                    // Send message
                    msgLen = m_pSerial->println(strAux);
                    // Test if message has been sent
                    if (strLen != msgLen)
                    {
                        #if SHOW_SERIAL_TRACE_WRITE
                            if (m_commandWrite != DBG_MSG)
                            {
                                String strAux2 = String("if.strLen=") + String(strLen) + String(".msgLen=") + String(msgLen) + strAux;
                                Serial.println(strAux2);
                                Serial.flush();
                            }
                        #endif
                        // Error sending message
                        return false;
                    }
                    #if SHOW_SERIAL_TRACE_WRITE
                        if (m_commandWrite != DBG_MSG)
                        {
                            Serial.print("writeSerialMsg().safw=");
                            Serial.print(safw);
                            Serial.print(". msgII=");
                            Serial.println(msg.substring(nextStartIdx));
                            if(m_pSerial == &Serial){
                                Serial.println("writeSerialMsg().m_pSerial == &Serial");
                            }else if(m_pSerial == &Serial1){
                                Serial.println("writeSerialMsg().m_pSerial == &Serial1");
                            }
                            #ifdef ARDUINO_AVR_MEGA2560
                                if(m_pSerial == &Serial2){
                                    Serial.println("writeSerialMsg().m_pSerial == &Serial2");
                                }else if(m_pSerial == &Serial3){
                                    Serial.println("writeSerialMsg().m_pSerial == &Serial3");
                                }
                            #endif                        
                        }
                    #endif
                }else{  
                    // write a portion of the msg
                    //// m_pSerial->print(msg.substring(nextStartIdx, nextStartIdx + safw) );
                    // Select a substring of the msg
                    String strAux = String(msg.substring(nextStartIdx, nextStartIdx + safw));
                    // strAux.length() + '\0' + '\n'
                    strLen = strAux.length() + 2; //// sizeof(strAux);
                    // write a substring msg
                    msgLen = m_pSerial->println(strAux);
                    // 
                    if ((strLen) != msgLen)
                    {
                        #if SHOW_SERIAL_TRACE_WRITE
                            if (m_commandWrite != DBG_MSG)
                            {
                                String strAux2 = String("else.strLen=") + String(strLen) + String(".msgLen=") + String(msgLen) + strAux;
                                Serial.println(strAux2);
                                Serial.flush();
                            }
                        #endif
                        // Error sending message
                        return false;
                    }
                    #if SHOW_SERIAL_TRACE_WRITE
                        if (m_commandWrite != DBG_MSG)
                        {
                            Serial.print("writeSerialMsg().safw=");
                            Serial.print(safw);
                            Serial.print(". msgI=");
                            Serial.println(msg.substring(nextStartIdx, nextStartIdx + safw));
                        }
                    #endif
                }
                // Wait until message has been sent
                m_pSerial->flush();
            }
            nextStartIdx += safw;
        }
        msgSent = true; 
    }
    //// // Wait until message has been sent
    //// m_pSerial->flush();
    #if SHOW_SERIAL_TRACE_WRITE
        if (m_commandWrite != DBG_MSG)
        {
            Serial.println("m_newCommandW=");
            Serial.println(m_newCommandWrite);
            Serial.print("msgSent=");
            if (msgSent)
            {
                Serial.println("true");
            }else{
                Serial.println("false");
            }
            Serial.flush();
        }
    #endif

    if (msgSent)
    {
        m_resetCommandWrite();
    }
    return msgSent;
} 


