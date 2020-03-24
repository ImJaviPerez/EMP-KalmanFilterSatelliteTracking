/**
 * @brief 
 * @version 0.1.0
 * 
 */

// This piece of code  is based on
// https://github.com/oksbwn/GET-Call-Arduino-ESP8266-using-WifiClient/blob/master/WifiClientCode.ino
//
// RobotDyn MEGA-WiFi R3 configuration:
// 
// Board Generic ESP8266
// https://docs.platformio.org/en/latest/boards/espressif8266/esp12e.html
//
// While uploading
// Press "Mode" button while uploading
// DIM switches:
// 1  OFF
// 2  OFF
// 3  OFF
// 4  OFF
// 5  ON
// 6  ON
// 7  ON
// 8  OFF
//
//
// When uploading is finished
// DIM switches:
// 1  OFF
// 2  OFF
// 3  OFF
// 4  OFF
// 5  ON
// 6  ON
// 7  OFF
// 8  OFF
//
#define THIS_PROJECT_HAS_ESP8266

#include <ESP8266WiFi.h>
#include <FS.h>

#include "STConfig.h"
#include <STSerialMsg.h>
#include <STDebug.h>

#include <Arduino.h>


using namespace SatTrack;


// -------------------------------------------------------
// Configuring user messages in an appropriate language
#ifdef LANG_ENG
    #include "STUserMessagesENG.h"
#else
  #ifdef LANG_ESP
    #include "STUserMessagesESP.h"
  #endif
#endif
// -------------------------------------------------------

#define ESP8266_TRACE (ESP8266_TRACE_TEST || ESP8266_TRACE_INFO || ESP8266_TRACE_WARNING || ESP8266_TRACE_ERROR)

// ----------------------------------------------------
// Global variables -----------------------------------

#if ESP8266_TRACE
  // Declare auxiliar variable
  String debugMessage;
  // Debug trace messages
  STDebug serialMsgDBG;
#endif
// Serial messages FROM_ESP8226_TO_MEGA
STSerialMsg serialMsgMEGA(STSerialMsg::PORT_ZERO);
// STSerialMsg serialMsgMEGA(STSerialMsg::PORT_ONE);


String satListNames = "";
//// String serialMsgRead = "";
//// char msgCommandRead = ' ';
//// String msgInfoRead = "";
char msgCommandWrite = ' ';
String msgInfoWrite = "";


bool stringComplete = false;  // whether the string is complete
// SPFFIS file name
const char* fileName = "/fileTLE.txt";
    

// -------------------------------------------------------
// ----------------------------------------------------


// ----------------------------------------------------
// Forward declarations -------------------------------
inline int ConnectWiFi();
inline int DownloadTLEFile();
int ReadSpiffsFile();
int putTLE(String &satelliteName);
// ----------------------------------------------------


void setup() {

  // Initialize Serial ports
  #if ESP8266_TRACE
    serialMsgDBG.init();
  #endif
  serialMsgMEGA.init();
  
  #if ESP8266_TRACE
    // Waiting for serialMsgMEGA and serialMsgDBG avalability
    while(!serialMsgMEGA.ready() & !serialMsgDBG.ready()){};
  #else
    // Waiting for serialMsgMEGA avalability
    while(!serialMsgMEGA.ready()){};
  #endif


  #if ESP8266_TRACE_INFO
    unsigned long initTime = millis();
  #endif

  #if ESP8266_TRACE_INFO
    debugMessage = "Before WiFi connection";
    serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_ESP8266, debugMessage);
  #endif
  // Connect to a WiFi network
  ConnectWiFi();

   // Starting TLE download
  // delay(5000);

  // Downloan TLE file
  DownloadTLEFile();

  #if ESP8266_TRACE_INFO
    debugMessage = String(STDebug::DBG_MSG_ESP_STARTUP_TIME) + String((millis() - initTime)/1000);
    serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_ESP8266, debugMessage);
  #endif

  /// SPIFFS.format();
  //// //// ReadSpiffsFile();

  /*
    String getTLESatellite = String("LUSAT (LO-19)");
    putTLE(getTLESatellite);
  */
}

void loop() 
{
  // Read Serial data coming from MEGA2560
  serialMsgMEGA.readSerialMsg();

  //// if (result)
  if (serialMsgMEGA.newCommandRead())
  {
    #if ESP8266_TRACE_INFO
      // debugMessage = String("satListNames=") + satListNames;
      // serialMsgDBG.printDebug(STDebug::TYPE_TRACE_INFO, STDebug::ORIGIN_ESP8266, debugMessage);
      debugMessage = String("msgCommandR=") + String(serialMsgMEGA.commandRead()) + String(".msgInfoR=") + String(serialMsgMEGA.commandInfoRead());
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_INFO, STDebug::ORIGIN_ESP8266, debugMessage);
    #endif
    String satName;
    bool result;

    // There is a new command
    //// switch (msgCommandRead)
    switch (serialMsgMEGA.commandRead())
    {
    case STSerialMsg::REQ_TLE_SAT_NAMES:
      // There is a request of all Satellite names.
      // So, we are going to send all Sateline names.
      result = serialMsgMEGA.putSatelliteNames(satListNames);
      if (result)
      {
        serialMsgMEGA.resetCommandRead();
        #if ESP8266_TRACE_INFO
          // debugMessage = String("satListNames=") + satListNames;
          // serialMsgDBG.printDebug(STDebug::TYPE_TRACE_INFO, STDebug::ORIGIN_ESP8266, debugMessage);
          debugMessage = String(STDebug::DBG_MSG_PUT_SAT_NAMES);
          serialMsgDBG.printDebug(STDebug::TYPE_TRACE_INFO, STDebug::ORIGIN_ESP8266, debugMessage);
        #endif
      }
      #if ESP8266_TRACE_INFO
        if (!result){
          // debugMessage = String("satListNames=") + satListNames;
          // serialMsgDBG.printDebug(STDebug::TYPE_TRACE_INFO, STDebug::ORIGIN_ESP8266, debugMessage);
          debugMessage = "Not sent satListNames";
          serialMsgDBG.printDebug(STDebug::TYPE_TRACE_INFO, STDebug::ORIGIN_ESP8266, debugMessage);
        }
      #endif

      break;
    
    case STSerialMsg::REQ_TLE_SATELLITE:
      // TO-DO
      //    Do something with function returned info
      //// putTLE(msgInfoRead);
      satName = serialMsgMEGA.commandInfoRead();
      if (putTLE(satName))
      {
        serialMsgMEGA.resetCommandRead();
      }
      break;
    
    case STSerialMsg::REQ_REFRESH_SAT_TLE:
      // TO-DO
      //    Do something with function returned info
      DownloadTLEFile();
      // TO-DO
      //    Do something with function returned info
      ///// putTLE(msgInfoRead);
      satName  = serialMsgMEGA.commandInfoRead();
      if (putTLE(satName))
      {
        serialMsgMEGA.resetCommandRead();
      }      break;
    
    default:
      break;
    }

    //// msgCommandRead = ' ';
    //// msgInfoRead = "";
  }
}


inline int ConnectWiFi()
{
  //Only used if using Static IP
  /*
  IPAddress ip(192, 168, 0, 6); //IP
  IPAddress gatewayDNS(192, 168, 0, 1);//DNS and Gewateway
  IPAddress netmask(255, 255, 255,0); //Netmask
  */

  #if ESP8266_TRACE_TEST
    debugMessage = String(STDebug::DBG_MSG_CONNECT_2) + String(ST_SSID);
    serialMsgDBG.printDebug(STDebug::TYPE_TRACE_INFO, STDebug::ORIGIN_ESP8266, debugMessage);
  #endif
  // WiFi.config(ip,gatewayDNS,netmask,gatewayDNS); //Only used if using Static IP 
  WiFi.begin(ST_SSID, ST_PASSWORD); //Connecting to the network
  
  while (WiFi.status() != WL_CONNECTED) {
    //Wait till connects
    delay(500);
    // TO-DO
    //    Show an error if there is too much delay
    #if ESP8266_TRACE_INFO
      //// Serial.print(".");
    #endif
  }

  #if ESP8266_TRACE_INFO
    // Show IP address
    debugMessage = String(STDebug::DBG_MSG_IP_ADDRS) + WiFi.localIP().toString();
    serialMsgDBG.printDebug(STDebug::TYPE_TRACE_INFO, STDebug::ORIGIN_ESP8266, debugMessage);
  #endif

  return 1;
}

/**
 * @brief This function make next jobs:
 * 1. Connect with a HOST server and download a file from an URL
 * 2. Save that file in the ESP8266 file system (SPIFFS)
 * 3. Create a list with all satellites names
 * 
 * @author ImJaviPerez
 * @version 1.0.0 - 2019/11/21
 * 
 * @return int 1 if evereythig goes OK
 * @return int STDebug::ERR_SERVER_CONN_FAIL if there is a server connection fail
 * @return int STDebug::ERR_SERVER_CONN_T_OUT if there is a server connection time out
 * @return int STDebug::ERR_SPIFFS_OPEN_F_W if there is an error opening the SPIFFS file
 */
inline int DownloadTLEFile(){

  const byte TLE_LINE_0_LENGTH = 24;
  const byte TLE_LINE_1_LENGTH = 69;
  const byte TLE_LINE_2_LENGTH = 69;

  // To count number of lines in TLE file
  int fileLineNumber = 0;
  // To save last TLE line position in the file
  int tleLastLineNumber = 0;

  int bytesWritten = 0;

  // Connect to host -------------------------------------
  #if ESP8266_TRACE_INFO
    // Connecting to HOST
    debugMessage = String(STDebug::DBG_MSG_CONNECT_2) + String(HOST);
    serialMsgDBG.printDebug(STDebug::TYPE_TRACE_INFO, STDebug::ORIGIN_ESP8266, debugMessage);
  #endif
  //Client to handle TCP Connection
  WiFiClient client;
  const int httpPort = 80;
  // Connect to server using port httpPort
  if (!client.connect(HOST, httpPort)) {
    #if ESP8266_TRACE_WARNING
      debugMessage = String(STDebug::ERR_SERVER_CONN_FAIL) + String(STDebug::DBG_MSG_HOST) + 
      String(HOST) + String(STDebug::DBG_MSG_CONNECT_FAIL);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_WARNING, STDebug::ORIGIN_ESP8266, debugMessage);
      return STDebug::ERR_SERVER_CONN_FAIL;
    #endif
  }
  
  // This will send the request to the server ------------
  // Asks for a file
  client.print(String("GET ") + URL + " HTTP/1.1\r\n" +
               "Host: " + HOST + "\r\n" + 
               "Connection: close\r\n\r\n");
  unsigned long timeout = millis();
  while (client.available() == 0) {
    //Try to fetch response for 25 seconds
    if (millis() - timeout > 25000) {
      client.stop();
      #if ESP8266_TRACE_WARNING
        debugMessage = String(STDebug::ERR_SERVER_CONN_T_OUT) + String(STDebug::DBG_MSG_CL_TIMEOUT);
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_WARNING, STDebug::ORIGIN_ESP8266, debugMessage);
        return STDebug::ERR_SERVER_CONN_T_OUT;
      #endif      
    }
  }
  
  // Mount SPIFFS ----------------------------------------
  bool succesSPIFFS = SPIFFS.begin();
  #if ESP8266_TRACE_WARNING
    if (!succesSPIFFS){
      // ESP8266 Error mounting SPIFFS file system
      debugMessage = String(STDebug::ERR_SPIFFS_MOUNTING) + String(STDebug::DBG_MSG_ERR_SPIFFS_MOUNT);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_WARNING, STDebug::ORIGIN_ESP8266, debugMessage);
    }
  #endif
  #if ESP8266_TRACE_INFO
    if (succesSPIFFS){
      // ESP8266 SPIFFS file system mounted with success
      debugMessage = String(STDebug::DBG_MSG_SPIFFS_MOUNT_OK);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_INFO, STDebug::ORIGIN_ESP8266, debugMessage);
    }
  #endif

  // Open SPIFFS file ------------------------------------
  File fileTLE = SPIFFS.open(fileName, "w");
  if (!fileTLE) {
    #if ESP8266_TRACE_WARNING
      // Error opening file for writing
      debugMessage = String(STDebug::ERR_SPIFFS_OPEN_F_W) + String(STDebug::DBG_MSG_ERR_SPIFFS_OPEN_W);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_INFO, STDebug::ORIGIN_ESP8266, debugMessage);
      return STDebug::ERR_SPIFFS_OPEN_F_W;
    #endif
    
  }
  // SPIFFS file is ready to write
  
  // Read file from server -------------------------------
  // Find ordered TLE lines that stars with: "", 1, 2
  // and 
  // - Save those lines to a file in the ESP8266 file system (SPIFFS)
  // - Create a list with all satellites names
  int seekingTLElineNumber = 0;
  satListNames = "";

  // String array with 3 TLE lines
  String tleLine[3];

  // FIX-ME. TO-DO: Sometimes (not always) there are missing satellite lines at the end of the file. Why?
  while(client.available()){
    // Count line number
    fileLineNumber++;
    // Seeks next TLE line
    switch (seekingTLElineNumber) {
      case 0:
        // In this file every line ends with '\r\n'
        // Read de line content until '\r'
        tleLine[0] = client.readStringUntil('\r');
        // Discard '\n' char
        client.read();
        // Check if this line is a satellite header line
        if (tleLine[0].length() == TLE_LINE_0_LENGTH) {
          tleLastLineNumber = fileLineNumber;
          seekingTLElineNumber++;
          #if ESP8266_TRACE_TEST
            debugMessage = String(DBG_MSG_TLE_LINE_0) + 
            String(DBG_MSG_TLE_LINE_NR) + String(fileLineNumber) + 
            String(DBG_MSG_TLE_LINE_LEN) + String(tleLine[0].length());
            serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_ESP8266, debugMessage);

            debugMessage = String(tleLine[0]);
            serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_ESP8266, debugMessage);
          #endif
          }else{
            seekingTLElineNumber = 0;
            #if ESP8266_TRACE_INFO
              // Show missing line
              debugMessage = String(STDebug::DBG_MSG_TLE0_MISSING) + String(tleLine[0]);
              serialMsgDBG.printDebug(STDebug::TYPE_TRACE_INFO, STDebug::ORIGIN_ESP8266, debugMessage);
            #endif
          }
          
        break;

      case 1:
        // In this file every line ends with '\r\n'
        // Read de line content until '\r'
        tleLine[1] = client.readStringUntil('\r');
        // Discard '\n' char
        client.read();
        // Check if this line has appropiate length
        if ((tleLine[1].length() == TLE_LINE_1_LENGTH) & tleLine[1].startsWith("1")) {
          #if ESP8266_TRACE_TEST
            debugMessage = String(DBG_MSG_TLE_LINE_1) + 
            String(DBG_MSG_TLE_LINE_NR) + String(fileLineNumber) + 
            String(DBG_MSG_TLE_LINE_LEN) + String(tleLine[1].length());
            serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_ESP8266, debugMessage);

            debugMessage = String(tleLine[1]);
            serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_ESP8266, debugMessage);
          #endif
          // Check if the later line was a tleLine[0]
          if (fileLineNumber == (tleLastLineNumber + 1)){
            tleLastLineNumber = fileLineNumber;
            seekingTLElineNumber++;
          }else{
            seekingTLElineNumber = 0;
            #if ESP8266_TRACE_INFO
              debugMessage = String(STDebug::DBG_MSG_TLE_SEEK_L1_L0);
              serialMsgDBG.printDebug(STDebug::TYPE_TRACE_INFO, STDebug::ORIGIN_ESP8266, debugMessage);
            #endif
          }
        }else{
            seekingTLElineNumber = 0;
            #if ESP8266_TRACE_INFO
              // Show missing line
              debugMessage = String(STDebug::DBG_MSG_TLE1_MISSING) + String(tleLine[1]);
              serialMsgDBG.printDebug(STDebug::TYPE_TRACE_INFO, STDebug::ORIGIN_ESP8266, debugMessage);
            #endif
        }
        break;
      case 2:
        // In this file every line ends with '\r\n'
        // Read de line content until '\r'
        tleLine[2] = client.readStringUntil('\r');
        // Discard '\n' char
        client.read();
        // Check if the length of tleLine[2] and starts with "2"
        if ((tleLine[2].length() == TLE_LINE_2_LENGTH) & tleLine[2].startsWith("2")) {
          #if ESP8266_TRACE_TEST
            debugMessage = String(DBG_MSG_TLE_LINE_2) + 
            String(DBG_MSG_TLE_LINE_NR) + String(fileLineNumber) + 
            String(DBG_MSG_TLE_LINE_LEN) + String(tleLine[2].length());
            serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_ESP8266, debugMessage);

            debugMessage = String(tleLine[2]);
            serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_ESP8266, debugMessage);
          #endif
          // Check if the later line was a tleLine[1]
          if (fileLineNumber == (tleLastLineNumber + 1)){
            tleLastLineNumber = fileLineNumber;
            // Send 3 TLE lines throw Serial line
            // It will be cathed by MEGA2560 at Serial port

            // Write 3 TLE lines into the file -----------
            for (int i=0; i<3; i++){
              // Add EOL '\n'
              tleLine[i] += "\n";
              // Write lines to file
              bytesWritten = fileTLE.print(tleLine[i]);
              #if ESP8266_TRACE_ERROR
              // ESP8266 Error writing file
                if (bytesWritten != tleLine[i].length()){
                  debugMessage = String(STDebug::ERR_SPIFFS_WRITE_FILE) +
                  String(STDebug::DBG_MSG_ERR_SPIFFS_WRITE) + 
                  String(" Line ") + String(i) + String(": ") + String(tleLine[i]);
                  serialMsgDBG.printDebug(STDebug::TYPE_TRACE_ERROR, STDebug::ORIGIN_ESP8266, debugMessage);
                }
              #endif

              #if ESP8266_TRACE_TEST
                // Show new line written in ESP8266 SPIFFS file
                if (bytesWritten == tleLine[i].length()){
                  debugMessage = String(DBG_MSG_SPIFFS_WRITE_LN_OK) + 
                  String(" Line ") + String(i) + String(": ") + String(tleLine[i]);
                  serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_ESP8266, debugMessage);
                }
                // TO-DO: Do not send these Strings throw Serial Line
                //        Send a list with the names of the satellites
              #endif
            } // end: for (int i=0; i<3; i++)

            // Create the list of satellites -------------
            // Add separator
            satListNames += ((satListNames.length() == 0)? "" : String(STDebug::ST_SEP));
            // Add new satellite
            satListNames += tleLine[0];
            // Remove any leading and trailing whitespace
            satListNames.trim();
          }else{ // ! (fileLineNumber == (tleLastLineNumber + 1))
            // The the later line was NOT a tleLine[1]
            #if ESP8266_TRACE_INFO
              debugMessage = String(STDebug::DBG_MSG_TLE_SEEK_L2_L0);
              serialMsgDBG.printDebug(STDebug::TYPE_TRACE_INFO, STDebug::ORIGIN_ESP8266, debugMessage);
            #endif
          }
        }else{ // ! (tleLine[2].length() == TLE_LINE_2_LENGTH) & tleLine[2].startsWith("2")
            #if ESP8266_TRACE_INFO
              // Show missing line
              debugMessage = String(STDebug::DBG_MSG_TLE2_MISSING) + String(tleLine[2]);
              serialMsgDBG.printDebug(STDebug::TYPE_TRACE_INFO, STDebug::ORIGIN_ESP8266, debugMessage);
            #endif
        }
        // Look for line 0 again
        seekingTLElineNumber = 0;
        
        break;
      default:
        #if ESP8266_TRACE_ERROR
          debugMessage = String(MSG_ERROR_TLE_LINE_NUMBER);
          serialMsgDBG.printDebug(STDebug::TYPE_TRACE_ERROR, STDebug::ORIGIN_ESP8266, debugMessage);
        #endif
      break;
    }
  } // end: while(client.available())

  //Close Connection
  client.stop();
  // Close opened file before return
  fileTLE.close();
  return 1;
}

/**
 * @brief This function writes a TLE satellite into the Serial port.
 * 
 * Reads the SPIFFS file containing the TLE satellite list.
 * Find the satelliteName at line 0.
 * Write into the Serial Port: line 0, line 1 and line 2.
 * It will be catched by de MEGA2560 at Serial3 port.
 * 
 * @param satelliteName The name of the satellite as it 
 * appears in the original file.
 * @return STDebug::ERR_SPIFFS_OPEN_F_R if there is an ESP8266 error opening file for reading
 * @return 1 if it finish OK
 */
int putTLE(String &satelliteName)
{
  bool satelliteFound = false;
  String lastLine;

  // Open SPIFFS file
  File fileTLE = SPIFFS.open(fileName, "r");
  if (!fileTLE) {
    #if ESP8266_TRACE_ERROR
      // Error opening file for reading
      debugMessage = String(STDebug::ERR_SPIFFS_OPEN_F_R) + String(STDebug::DBG_MSG_ERR_SPIFFS_OPEN_R);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_ERROR, STDebug::ORIGIN_ESP8266, debugMessage);
      return STDebug::ERR_SPIFFS_OPEN_F_R;
    #endif
    
  }
  // SPIFFS file is ready for reading

  #if ESP8266_TRACE_INFO
      debugMessage = String("fileTLE.available()=") + String(fileTLE.available());
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_ESP8266, debugMessage);
  #endif

  // Read the file line by line searching satelliteName
  #if ESP8266_TRACE_INFO
      int i = 0;
  #endif
  while((fileTLE.available() > 0) & (!satelliteFound)){
    // This reads the whole file
    // fileTLE.readString();

    // This reads the file line by line
    lastLine = fileTLE.readStringUntil('\n');
    String trimmedLine = lastLine;
    // Remove any leading and trailing whitespace
    trimmedLine.trim();
    // Test if this is the searched line
    satelliteFound = (trimmedLine == satelliteName);
    #if ESP8266_TRACE_INFO
        i++;
    #endif
  }
  #if ESP8266_TRACE_INFO
      debugMessage = String("Line number=") + String(i);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_ESP8266, debugMessage);
  #endif

  if (satelliteFound){
    // Send 3 lines: one TLE    
    for (int i = 0; i < 3; i++)
    {
      // Add an EOL at the end
      // lastLine += '\n';

      // Send line to MEGA2560
      //    Use a function to send each line sourrounded by a command:
      //    MSG_HEADER_BEGIN + PUT_TLE0_SATELLITE + lastLine + MSG_TAIL_END
      //    MSG_HEADER_BEGIN + PUT_TLE1_SATELLITE + lastLine + MSG_TAIL_END
      //    MSG_HEADER_BEGIN + PUT_TLE2_SATELLITE + lastLine + MSG_TAIL_END
      serialMsgMEGA.putSatelliteTLE(i, lastLine);
      
      // Read next line from SPIFFS file
      lastLine = fileTLE.readStringUntil('\n');
    }
    fileTLE.close();
    return 1;
  }
  fileTLE.close();
  // !(satelliteFound) 
  #if ESP8266_TRACE_ERROR
      // SPIFFS. Satellite not found in file
      debugMessage = String(STDebug::ERR_SPIFFS_SAT_NF) + String(". ") + String(STDebug::DBG_MSG_ERR_SPIFFS_SNF) +
      String(". satelliteName=") + String(satelliteName);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_ERROR, STDebug::ORIGIN_ESP8266, debugMessage);
  #endif
  return STDebug::ERR_SPIFFS_SAT_NF;;
}

int ReadSpiffsFile()
{
  // Open file -------------------------------------------
  File fileTLE = SPIFFS.open(fileName, "r");
  if (!fileTLE) {
    #if ESP8266_TRACE_ERROR
      // Error opening file for reading
      debugMessage = String(STDebug::ERR_SPIFFS_OPEN_F_R) + String(STDebug::DBG_MSG_ERR_SPIFFS_OPEN_R);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_ERROR, STDebug::ORIGIN_ESP8266, debugMessage);
      return STDebug::ERR_SPIFFS_OPEN_F_R;
    #endif
    
  }
  // SPIFFS file is ready for reading

  Serial.println("START READING FILE ------------");
  int n = 1;
  while(fileTLE.available()){
    Serial.print(n++);
    Serial.print(": ");

    // This reads the whole file
    // Serial.println(fileTLE.readString());

    // This reads the file line by line
    Serial.println(fileTLE.readStringUntil('\n'));
  }
  Serial.println("END READING FILE --------------");
  
  fileTLE.close();
  return 1;
}

