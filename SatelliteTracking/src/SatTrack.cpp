/**
 * @file SatTrack.cpp 
 * @author ImJaviPerez
 * @brief 
 * @version 0.1.004
 * @date 2019-12-20 18:21
 * 
 * 
 * @copyright Copyright (c) 2019
 * 
 */

// RobotDyn MEGA-WiFi R3 configuration:
// 
// Board Generic MEGA 2560
// https://docs.platformio.org/en/latest/boards/espressif8266/esp12e.html
//
// While uploading
// DIM switches:
// 1  ON
// 2  ON
// 3  ON
// 4  ON
// 5  OFF
// 6  OFF
// 7  OFF
// 8  OFF
//
//
// When uploading is finished
// DIM switches:
// 1  ON
// 2  ON
// 3  ON
// 4  ON
// 5  OFF
// 6  OFF
// 7  OFF
// 8  OFF
//
#include <STSerialMsg.h>
#include <STDebug.h>
#include <TinyGPS.h>  // GPS NEO V2
#include <QMC5883L_ST.h> // Digital compass
#include <Wire.h>     // Digital compass I2C communication
#include <I2Cdev.h>   // MPU6050 accelerometer Gyroscope communication
#include <Kalman_ST.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <LiquidCrystalDisplay.h> // H35B-IC touch screen display

#include "SatTrack.h" // SatTrack.cpp header files
#include "STConfig.h" // App configuration
#include "LCD_config.h" // H35B-IC touch screen display



// =====================================================
// Configuring user messages in an appropriate language
#ifdef LANG_ENG
    #include "STUserMessagesENG.h"
#else
  #ifdef LANG_ESP
    #include "STUserMessagesESP.h"
  #endif
#endif
// =======================================================

#include <STDebug.h>

// Plan13 calculates satellite attitude
#include <Plan13.h>

#include <Arduino.h>


using namespace SatTrack;

#define MEGA2560_TRACE (MEGA2560_TRACE_TEST | MEGA2560_TRACE_INFO | MEGA2560_TRACE_WARNING | MEGA2560_TRACE_ERROR)

// =======================================================
// Global variables --------------------------------------

// Variables to sen and receive messages -----------------
#if MEGA2560_TRACE
  // Debug trace messages
  STDebug serialMsgDBG;
  // Declare auxiliar variable
  String debugMessage = "";
#endif
// Messages between MEGA2560 and ESP8266
STSerialMsg serialMsgESP(STSerialMsg::PORT_THREE);

// Variables to smart-delay messages ---------------------
#if MEGA2560_TRACE_INFO
  unsigned long lastTimeTrace = millis();
  // TRACE_TIME_PERIOD must be greater or equal than ANGLES_TIME_PERIOD
  const unsigned long TRACE_TIME_PERIOD = ANGLES_TIME_PERIOD + 0; // milliseconds
#endif

// Global varibles ---------------------------------------
String strSatelliteListNames = "";
char *psSatelliteTLE[] = {"NUSAT-1 (FRESCO)        ",
"1 41557U 16033B   20062.16981994  .00001218  00000-0  47510-4 0  9995",
"2 41557  97.4050 153.1162 0015235 106.3023 253.9893 15.27771467209398"};
//// bool newSatelliteName = true;

String askTLEsatelliteName = "NUSAT-1 (FRESCO)";

//// String msgInfoRead = "";
////  char msgCommandRead = ' ';

// GPS variables -----------------------------------------
const int DEFAULT_YEAR = 2020;
const byte DEFAULT_MONTH = 03;
const byte DEFAULT_DAY = 02;
const byte DEFAULT_HOUR = 12;
const byte DEFAULT_MINUTE = 0;
const byte DEFAULT_SECOND = 0;
bool gpsReceiving = false;

TinyGPS gps;
long lat, lon;
float flat, flon;
unsigned long age = 0, date = 0, time = 0, chars = 0;
int year = DEFAULT_YEAR;
byte month = DEFAULT_MONTH, day = DEFAULT_DAY;
byte hour = DEFAULT_HOUR, minute = DEFAULT_MINUTE, second = DEFAULT_MINUTE, hundredths = 0;

// Digital compass variables -----------------------------
QMC5883L_ST compass(COMPASS_X_AXIS_POSITION);

// After the device is powered on, some time periods are 
// required for the device fully functional. 
// The external power supply requires a time period for 
// voltage to ramp up (PSUP), it is typically 
// 50 milli-second. However it isn’t controlled by the device.
// The power on/off time related to the device are:
// POR Completion Time: Time Period After VDD and VDDIO at Operating Voltage to Ready for I2C Commend and Analogy Measurement
// // const unsigned long PORCT = 1; // 350 micro seconds
// PINT: Power on Interval. Time Period Required for Voltage Lower Than SDV (0.2 volts) to Enable Next PORCT
// // const unsigned long PINT = 1; // 100 micro seconds
// Compass heading
/// int heading, horizontalHeadingAngle, horizontalHeadingAngle2, horizontalHeadingAngle3;
int16_t heading, horizontalHeadingAngle, horizontalHeadingAngle2, horizontalHeadingAngle3;
int16_t rollXcompass = 0, pitchYcompass = 0, yawZcompass = 0;


// MPU6050 accelerometer Gyroscope -----------------------
// Create the Kalman instance
Kalman_ST kalmanX, kalmanY, kalmanZ;

// IMU Data
// Accelerometers: raw angles
double accX, accY, accZ;
// Gyroscopes: raw angles
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
// rollAccX: angle around X axis.
// pitchAccY: angle around Y axis.
// yawAccZ: angle around Z axis.
double rollAccX = 0, pitchAccY = 0, yawAccZ = 0;
double gyroXrate, gyroYrate, gyroZrate;
#if MEGA2560_TRACE_TEST
  // Angle calculated using the gyro only
  double gyroXangle, gyroYangle, gyroZangle;
#endif

// Calculated angle using a Kalman filter
double kalAngleX, kalAngleY, kalAngleZ; 

uint32_t timer;
#if MEGA2560_TRACE_INFO
  uint32_t lastTimeRefreshTrace = 0;
#endif
double dt;
// Buffer for I2C data
uint8_t i2cDataMPU6050[14];
// MPU6050 Communication channel
// imjaviperez
I2Cdev i2cDevice;

// TODO: Make MPU6050  calibration routine


// Satellite attitude variables. Plan13 ------------------
double satAzimuth, satElevation;
#if MEGA2560_TRACE_INFO
  char asciiDateTime[20] ;
#endif
Satellite sat = Satellite("QWERTY_QWERTY           ", //"RADIO ROSTO (RS-15)     ",
"1 23439U 94085A   19345.83261864 -.00000019  00000-0  66113-3 0  9998",
"2 23439  64.8132 191.2812 0145849  78.7161 283.0113 11.27569313 27759");
DateTime  myDateTime ;

char DEFAULT_PLACE[] = "Bilbao"; // "Leioa";
const double DEFAULT_LATITUDE = 43.3000; // 43.328955;
const double DEFAULT_LONGITUDE = 2.9333; // -2.966181;
const double DEFAULT_ALTITUDE = 37.0; // 40.0; // meters

//   Observer postition. Plan13
char *observerPlace = DEFAULT_PLACE;
double observerLat = DEFAULT_LATITUDE;
double observerLong = DEFAULT_LONGITUDE;
double observerAlt = DEFAULT_ALTITUDE;
const Observer &observationPlace = Observer(observerPlace, observerLat, observerLong, observerAlt) ;
  


// Touch screen display variables (H35B-IC) ---------------
const uint8_t strMsgButtonUp[]="Button PREVIOUS!";
const uint8_t strMsgButtonDown[]="Button NEXT!";
const uint8_t strMsgFinishedSetup[]="Finished setup";

uint8_t  strSatName[25] = "QWERTY :(";
uint8_t  strPagenumber[] = "pp/ff";

uint8_t  checkSat01 = 0;
uint8_t  checkSat02 = 0;
uint8_t  checkSat03 = 0;

// LCD variables
uint8_t   identifier_lcd,cnt_lcd;
uint8_t   cmd_buffer_lcd[CMD_MAX_SIZE];
uint8_t   data_size_lcd;
uint8_t   update_lcd;
uint8_t   command_cmd_lcd;
uint8_t   command_status_lcd;
uint8_t   command_length_lcd;
uint8_t   page_id_lcd = 0;
uint8_t   targe_Id = 0;

LiquidCrystal TFTlcd(13);//RST pin13

// -------------------------------------------------------


// Status for asking arguments ---------------------------
bool askSatelliteNames = false;
bool askOneSatelliteTLE = false;
/// bool fakeDateTime = true;

// Variables for smart delay sampling ====================
// Period of time to get GPS Date and Time (in millisecons)
unsigned long lastTimeGetGpsDT = millis();
unsigned long lastTimeGps = millis();
unsigned long lastTimeCompass = millis();
unsigned long lastTimeAngles = millis();
unsigned long lastTimeSatAttitude = millis();
unsigned long lastTimeLcdRefresh = millis();


// Variables to ask only one device per time, to do one task per time
class CircularArray
{
public:
  CircularArray();
  ~CircularArray();
  enum devicesToComunicateWith {GPS_DATE_TIME, GPS_POSITION, COMPASS, ACC_GYRO, LCD};
  uint8_t next();
  uint8_t current();
private:
  uint8_t m_currentDevice;
  devicesToComunicateWith m_listOfDevices[5] = {GPS_DATE_TIME, GPS_POSITION, COMPASS, ACC_GYRO, LCD};
};

CircularArray::CircularArray()
{
  // First device to comunicate with
  m_currentDevice = devicesToComunicateWith::GPS_DATE_TIME;
}

CircularArray::~CircularArray()
{
}

uint8_t CircularArray::next()
{
  if ( (m_currentDevice+1) >= ((uint8_t)(sizeof(m_listOfDevices)/sizeof(m_listOfDevices[0]))) )
  {
    m_currentDevice = 0;
  }
  else
  {
    m_currentDevice++;
  }
  
  return m_listOfDevices[m_currentDevice];
}

uint8_t CircularArray::current()
{
  return m_listOfDevices[m_currentDevice];
}

CircularArray deviceToComunicateWith;




// Only for testing purpouses
bool anotherPeriodOfTime = true;
// End of Global variables -------------------------------
// =======================================================


void setup() {
  // =====================================================
  // Initialize Serial ports -----------------------------
  #if MEGA2560_TRACE
    serialMsgDBG.init();
  #endif  
  serialMsgESP.init();
  // Serial2 GPS
  Serial2.begin(STSerialMsg::GPS_BAUD_RATE);
  delay(1000);

  // Wait for Serial ports to be connetted ---------------
  #if MEGA2560_TRACE
  // Waiting for Serial3 and Serial avalability
  // ESP8266 sends TLE throw Serial3 port
    while(!serialMsgDBG.ready()){};
  #endif
  // ESP8266 Serial port
  while(!serialMsgESP.ready()){
      #if MEGA2560_TRACE_TEST
        if (millis() - lastTimeTrace >= TRACE_TIME_PERIOD){
          lastTimeTrace = millis();
          debugMessage = String(DBG_MSG_ESP_STARTING_UP);
          serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, ORIGIN_MEGA2560, debugMessage);
        }
      #endif
  };
  // GPS Serial port
  while (!(Serial2)) {}
  // =====================================================

  // =====================================================
  // I2C communication setup
   // Compass and MPU6050 (accelerometer gyroscope) communication
  #if ARDUINO >= 157
    Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
    TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif

  i2cDataMPU6050[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cDataMPU6050[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cDataMPU6050[2] = GYRO_FULL_SCALE_RANGE_0250; // Set Gyro Full Scale Range to ±250deg/s
  i2cDataMPU6050[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  //// while (i2cDevice.i2cWrite(0x19, i2cDataMPU6050, 4, false)); // Write to all four registers at once
  while (!i2cDevice.writeBytes(IMU_MPU6050_ADDRS, 0x19, 4,i2cDataMPU6050)); // Write to all four registers at once

  //// while (i2cDevice.i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (!i2cDevice.writeByte(IMU_MPU6050_ADDRS,0x6B, 0x01)); // PLL with X axis gyroscope reference and disable sleep mode

  //// while (i2cDevice.i2cRead(0x75, i2cDataMPU6050, 1));
  while (!i2cDevice.readByte(IMU_MPU6050_ADDRS, 0x75, i2cDataMPU6050));
  if (i2cDataMPU6050[0] != IMU_MPU6050_ADDRS) { // Read "WHO_AM_I" register
    #if MEGA2560_TRACE_ERROR
      debugMessage = String("Error reading MPU6050 sensor");
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_ERROR, STDebug::ORIGIN_MEGA2560, debugMessage);
    #endif
    //  TO-DO
    //    Change this error behaviour
    while (true);
  }

  // Time for VDD to rise from 10% to 90% of its final value
  delay(100); // Wait for sensor to stabilize
  // =====================================================


  // =====================================================
  // Initialize acelerometer gyroscope:
  getAccGyroDeviceInfo();
  /*
  // Wait to receive data
  while (!i2cDevice.readBytes(IMU_MPU6050_ADDRS, 0x3B, 6, i2cDataMPU6050));
  // Initialize acelerometer
  accX = (int16_t)((i2cDataMPU6050[0] << 8) | i2cDataMPU6050[1]);
  accY = (int16_t)((i2cDataMPU6050[2] << 8) | i2cDataMPU6050[3]);
  accZ = (int16_t)((i2cDataMPU6050[4] << 8) | i2cDataMPU6050[5]);
  */


  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees

  /// #ifdef RESTRICT_PITCH // Eq. 25 and 26
  ///   double rollAccX  = atan2(accY, accZ) * RAD_TO_DEG;
  ///   double pitchAccY = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  /// #else // Eq. 28 and 29
  ///   double rollAccX  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  ///   double pitchAccY = atan2(-accX, accZ) * RAD_TO_DEG;
  /// #endif

  transformAccelGyroInfo2Angles();

  timer = micros();
  // =====================================================
  
  // =====================================================
  #if MEGA2560_TRACE_TEST
    #ifdef ARDUINO_AVR_MEGA2560
        debugMessage = "DEFINED ARDUINO_AVR_MEGA2560";
        serialMsgDBG.printDebug(STDebug::STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
    #else
        debugMessage = "NOT DEFINED ARDUINO_AVR_MEGA2560";
        serialMsgDBG.printDebug(STDebug::STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
    #endif
  #endif

  // =====================================================
  // Initialize Compass ----------------------------------
  // Compass communication
	compass.init();
	compass.setSamplingRate(COMPASS_SAMPLING_RATE);

  // We can set initial angles after compass.init()
  while (!compass.ready()){}
  // Set starting angle after initialize compass
  compass.readRotatedAngles(rollXcompass, pitchYcompass, yawZcompass);
  // Set kalman and gyro starting angle
  kalmanX.setAngle(rollAccX);
  kalmanY.setAngle(pitchAccY);
  kalmanZ.setAngle(yawZcompass);
  #if MEGA2560_TRACE_TEST
    gyroXangle = rollAccX;
    gyroYangle = pitchAccY;
    gyroZangle = yawZcompass;
  #endif
  // =====================================================




  // =====================================================
  // Initialize LCD ----------------------------------
   update_lcd = 0;
   data_size_lcd = 0;
   TFTlcd.queue_reset();

   // TO-DO :
   //  In order to manage many pages and buttons, create an interruption like this:
   //   attachInterrupt(digitalPinToInterrupt(compassInterruptionPin),LcdIICInterrupt,FALLING);//Interrupt 0 is D2 PIN

   TFTlcd.SetPage(ATTITUDE_PAGE);//main page id
  // =====================================================


  // =====================================================
  // Force reading from ESP8266 Serial3 port before loop() statement
  mySerialEvent3();
    
  #if MEGA2560_TRACE_TEST
    debugMessage = "End of setup()";
    serialMsgDBG.printDebug(STDebug::STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
  #endif
  // =====================================================

  // Show Message strMsgFinishedSetup
  TFTlcd.Display_Message(0X18,3,(unsigned char *)strMsgFinishedSetup);
} // End of setup()


void loop() {
  // Change status periodically ==========================
  // TO-DO
  //    Change askSatelliteNames and askOneSatelliteTLE from LCD

  // Only for testing purpouses --------------------------
  anotherPeriodOfTime = true;
  // Change status periodically
  #if MEGA2560_TRACE_INFO
    if (millis() - lastTimeTrace >= TRACE_TIME_PERIOD){
      lastTimeTrace = millis();
      // debugMessage = String("anotherPeriodOfTime=") + String(anotherPeriodOfTime);
      // serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
    }else{
      anotherPeriodOfTime = false;
    }
  #endif
  if (anotherPeriodOfTime)
  {
      askSatelliteNames = false;
      askOneSatelliteTLE = false;
  }
  // End of Only for testing purpouses -------------------
  // End of Change status periodically ===================

  
  // =====================================================
  // Smart delays and get information from devices =======
  // If there is no LCD interruption we comunicate next device
  if (update_lcd == 0)
  {
    deviceToComunicateWith.next();
  }
  #if MEGA2560_TRACE_TEST
    debugMessage = String("deviceToComunicateWith.current()=") + String(deviceToComunicateWith.current())
    + String(".update_lcd=") + String(update_lcd);
    serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
  #endif

  // GPS smart delay -------------------------------------
  if ((update_lcd == 0) && 
    (deviceToComunicateWith.current() == CircularArray::GPS_DATE_TIME) &&
    (millis() - lastTimeGps >= GPS_TIME_PERIOD)){
    lastTimeGps = millis();
    // Read info from GPS: date, time and observer position
    getGPSInfo();
    observerLat = flat;
    observerLong = flon;
    observerAlt = gps.f_altitude();
    //// askGPSDateTime = true;
  }
  //Compass smart delay ----------------------------------
  // Get device compass orientation every while
  if ((update_lcd == 0) && 
    (deviceToComunicateWith.current() == CircularArray::COMPASS) &&
    (millis() - lastTimeCompass >= COMPASS_TIME_PERIOD)){
    lastTimeCompass = millis();
    // Calculate device North-South orientation ----------
    // Get compass value
    compass.readRotatedAngles(rollXcompass, pitchYcompass, yawZcompass);

    // Transform into positive angles
    rollXcompass = positiveAngle(rollXcompass);
    pitchYcompass = positiveAngle(pitchYcompass);
    yawZcompass = positiveAngle(yawZcompass);
    /// heading = compass.readHeading();
    ////heading = yawZcompass;
    heading = yawZcompass;
    
    // Add local declination correction
    heading += (int16_t) LOCAL_DECLINATION;

    // The member function readHeading() gets North heading in the device XY plane, 
    // but we need a projection of heading in the ground horizontal plane.
    // horizontalHeading() creates a projection of this vector in the ground plane
    horizontalHeading();
  }

  // Accelerometer-Gyroscope smart delay -----------------
  if ((update_lcd == 0) && 
    (deviceToComunicateWith.current() == CircularArray::ACC_GYRO) &&
    (millis() - lastTimeAngles >= ANGLES_TIME_PERIOD)){
    lastTimeAngles = millis();
    // Get device angles attitude
    getAccGyroDeviceInfo();
    // Calculate angles, gyro rate and kalman filter
    calculateRotationAngles();
    //// askDeviceAngles = true;
  }

  // Calculate satellite attitude smart delay ------------
  if ((update_lcd == 0) && 
    (deviceToComunicateWith.current() == CircularArray::GPS_POSITION) &&
    (millis() - lastTimeSatAttitude >= SAT_ATTITUDE_TIME_PERIOD))
  {
    lastTimeSatAttitude = millis();
    getSatelliteAttitude();     
    //// calculateSatelliteAttitude = false;
  }
  // Refresh LCD Smart delay -----------------------------
  if ((update_lcd == 1) || 
    ((deviceToComunicateWith.current() == CircularArray::LCD) && (millis() - lastTimeLcdRefresh >= SAT_LCD_TIME_PERIOD)))
  {
    lastTimeLcdRefresh = millis();
    update_lcd = 0;

    // TO-DO :
    // Create three pages: main, atttude, select_satellites
    // Update only attitude page
    page_id_lcd = ATTITUDE_PAGE;
    UpdateUI();
  }
  // End of Smart delays =================================

  // Ask ESP8266 for information =========================
  // Request Satellite Names -----------------------------
  // TO-DO
  //    Change value of askSatelliteNames = true from LCD
  if (askSatelliteNames)
  {
    bool result = serialMsgESP.requestSatelliteNames();
    if (result)
    {
      askSatelliteNames = false;
    }else{
      #if MEGA2560_TRACE_WARNING
        debugMessage = String("ERROR rSN=");
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_WARNING, STDebug::ORIGIN_MEGA2560, debugMessage);
      #endif
    }
  }

  // Request one Satellite TLE ---------------------------
  // TO-DO
  //    Change value of askOneSatelliteTLE = true from LCD
  if (askOneSatelliteTLE)
  {
    if (!serialMsgESP.requestSatelliteTLE(askTLEsatelliteName))
    {
      #if MEGA2560_TRACE_WARNING
        debugMessage = String("ERROR rSTLE Name=") + askTLEsatelliteName;
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_WARNING, STDebug::ORIGIN_MEGA2560, debugMessage);
      #endif
    }else{
      askOneSatelliteTLE = false;
    }
  }
  // End of Ask ESP8266 for information ==================


  // =====================================================
  // Find LCD commands: touch button, change screen, etc
  data_size_lcd = TFTlcd.queue_find_cmd(cmd_buffer_lcd,CMD_MAX_SIZE);
  if(data_size_lcd>0)//receive command
  {
    // Serial.println(data_size_lcd, HEX);
    // Serial.println(F("ProcessMessage"));
    ProcessMessage((PCTRL_MSG)cmd_buffer_lcd, data_size_lcd);//command process
  } 

  // =====================================================


  // Force reading from Serial3 port during loop() statement
  //// serialEvent3();
  mySerialEvent3();

  // 
  #if MEGA2560_TRACE_INFO
    if (millis() - lastTimeRefreshTrace >= TRACE_TIME_PERIOD) // TRACE_TIME_PERIOD
    {
      showTraceInfo();
      lastTimeRefreshTrace = millis();
    }
  #endif

 } // end: loop()

// =======================================================
// Functions =============================================

// Checking an event on the Serial3 port
// SerialEvent occurs whenever a new data comes in the hardware serial RX. 
// This routine is run between each time loop() runs
//// void serialEvent3() 
void mySerialEvent3() 
{ 
  const int TLE_LINE_0_LENGTH = 24;
  const int TLE_LINE_1_LENGTH = 69;
  const int TLE_LINE_2_LENGTH = 69;

  #if MEGA2560_TRACE_TEST
    // Show data on serial monitor
    if (serialMsgESP.available() > 0) 
    {
      debugMessage = String("serialMsgESP.available()=") + String(serialMsgESP.available());
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
    }
  #endif
  //// serialMsgESP.readSerialMsg(msgCommandRead, msgInfoRead);
  serialMsgESP.readSerialMsg();
  if (serialMsgESP.newCommandRead())
  {
    String tmpCommandInfo = serialMsgESP.commandInfoRead();
    switch (serialMsgESP.commandRead())
    {
      case STSerialMsg::PUT_SAT_NAMES:
        // The ESP8266 has PUT_SAT_NAMES. So let's read them
        //// strSatelliteListNames = serialMsgESP.commandInfo();
        strSatelliteListNames = tmpCommandInfo; //serialMsgESP.commandInfo();
        #if MEGA2560_TRACE_TEST // MEGA2560_TRACE_INFO
          // Show data on serial monitor
          debugMessage = String(STDebug::DBG_MSG_SAT_NAMES) + strSatelliteListNames;
          serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
        #endif
        break;
      
      case STSerialMsg::PUT_TLE0_SATELLITE:
        tmpCommandInfo = serialMsgESP.commandInfoRead();
        tmpCommandInfo.toCharArray(psSatelliteTLE[0], TLE_LINE_0_LENGTH);
        #if MEGA2560_TRACE_TEST
          // Show data on serial monitor
          debugMessage = String(STDebug::DBG_MSG_TLE_LINE_0) + String(psSatelliteTLE[0]);
          serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
        #endif
        break;
      
      case STSerialMsg::PUT_TLE1_SATELLITE:
        tmpCommandInfo = serialMsgESP.commandInfoRead();
        tmpCommandInfo.toCharArray(psSatelliteTLE[1], TLE_LINE_1_LENGTH);
        #if MEGA2560_TRACE_TEST
          // Show data on serial monitor
          debugMessage = String(STDebug::DBG_MSG_TLE_LINE_1) + String(psSatelliteTLE[1]);
          serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
        #endif
        break;
      
      case STSerialMsg::PUT_TLE2_SATELLITE:
        tmpCommandInfo = serialMsgESP.commandInfoRead();
        tmpCommandInfo.toCharArray(psSatelliteTLE[2], TLE_LINE_2_LENGTH);
        #if MEGA2560_TRACE_TEST
          // Show data on serial monitor
          debugMessage = String(STDebug::DBG_MSG_TLE_LINE_2) + String(psSatelliteTLE[2]);
          serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
        #endif
        break;
      
      case STSerialMsg::DBG_MSG:
      // Do not do anything with DBG_MSG
      default:
        break;
    }

    // After reading and saving message, make a reset serialMsgESP message
    // to let Serial read more information next loop.
    serialMsgESP.resetCommandRead();

  }else{
      #if MEGA2560_TRACE_TEST
        // Show data on serial monitor
        debugMessage = "No message read";
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
      #endif

  }
}

inline void showTraceInfo()
{
  
  #if MEGA2560_TRACE_INFO
  
  /*
   debugMessage = String(timer) + String("\t") + String(dt) 
    + String("\t") + String(rollAccX) + String("\t") + String(pitchAccY) + String("\t") + String(yawAccZ)
    #if MEGA2560_TRACE_TEST
    + String("\t")  + String(gyroXangle) + String("\t") + String(gyroYangle)
    #endif
    + String("\t")  + String(gyroXrate) + String("\t") + String(gyroYrate) + String("\t") + String(gyroZrate)
    + String("\t")  + String(rollXcompass)+ String("\t")  + String(pitchYcompass) + String("\t") + String(yawZcompass)
    + String("\t") + String(kalAngleX) + String("\t") + String(kalAngleY) + String("\t") + String(kalAngleZ)
    + String("\t") + String(heading)  
    + String("\t") + String(horizontalHeadingAngle) 
    + String("\t") + String(horizontalHeadingAngle2) + String("\t") + String(horizontalHeadingAngle3);
*/
    // Raw info acc, gyro, compass
    int16_t compassX, compassY, compassZ;
    compass.readRaw(&compassX, &compassY, &compassZ,false);
    debugMessage = String(accX) + String("\t") +  String(accY) + String("\t") +  String(accZ)
     + String("\t") +  String(gyroXrate) + String("\t") + String(gyroYrate) + String("\t") + String(gyroZrate)
     + String("\t") + String(compassX) + String("\t") + String(compassY) + String("\t") + String(compassZ);

    // debugMessage = String(gyroXrate) + String("\t") + String(gyroYrate) + String("\t") + String(gyroZrate)
    //  + String("\t") + String(rollAccX) + String("\t") + String(pitchAccY) + String("\t") + String(yawAccZ);
    
    // debugMessage = String(rollAccX) + String("\t") + String(pitchAccY) + String("\t") + String(yawAccZ);
    // debugMessage +=  String("\t") + String(accX) + String("\t") + String(accY) + String("\t") + String(accZ);
    // debugMessage += String("\t") + String(gyroX) + String("\t") + String(gyroY) + String("\t") + String(gyroZ);
    #if MEGA2560_TRACE_TEST
      // debugMessage += String(gyroXangle) + String("\t") + String(gyroYangle) + String("\t") + String(gyroZangle);
    #endif
    
    // int16_t cx,cy,cz;
    // compass.readRaw(&cx,&cy,&cz,true);
    // debugMessage = String(cx) + String("\t") + String(cy) + String("\t") + String(cz)
    //  + String("\t") +   String(rollXcompass) + String("\t") + String(pitchYcompass) + String("\t") + String(yawZcompass);

    // debugMessage = String(rollAccX) + String("\t") + String(pitchAccY) + String("\t") + String(yawAccZ)
    //  + String("\t") +   String(yawZcompass) 
    //  + String("\t") + String(horizontalHeadingAngle) + String("\t") + String(horizontalHeadingAngle2) + String("\t") + String(horizontalHeadingAngle3)
    //  + String("\t") + String(kalAngleX) + String("\t") + String(kalAngleY) + String("\t") + String(kalAngleZ);
    serialMsgDBG.printDebug(STDebug::TYPE_TRACE_INFO, STDebug::ORIGIN_MEGA2560, debugMessage);
  #endif
}

inline void getGPSInfo()
{
  // Every TIME_PERIOD_GPS_DT milliseconds we ask for an update
  if (millis() - lastTimeGetGpsDT <= TIME_PERIOD_GPS_DT)
  {
    bool newdata = false;
    // unsigned long start = millis();
    #if MEGA2560_TRACE_TEST
      debugMessage = String("S2.available()=") + String(Serial2.available());
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
    #endif
    while (Serial2.available()) 
    {
      char c = Serial2.read();
      #if MEGA2560_TRACE_TEST
        // To see raw GPS data
        // Serial.print(c);
        // debugMessage = String(c);
        // serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
      #endif
      if (gps.encode(c)) 
      {
        newdata = true;
        #if MEGA2560_TRACE_TEST
          debugMessage = "NEW gps DATA";
          serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
        #endif
        // break; // uncomment to print new data immediately!
      }
    }

    if (newdata) 
    {
      gps.get_position(&lat, &lon, &age);
      // gps.f_get_position(&flat, &flon, &age);
      gps.f_get_position(&flat, &flon, &age);
      gps.get_datetime(&date, &time, &age);
      gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
      // gps.stats(&chars, &sentences, &failed);
      #if MEGA2560_TRACE_TEST
        debugMessage = String("GPS OK");
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
      #endif

      //// fakeDateTime = false;
      //// askGPSDateTime = false;
    }else{
      // Create a fake DateTime
      /*
      int secsLapse = (millis() - lastTimeGetGpsDT)/1000;
      // Remainder of (second + ellapsedTime)/60
      second = (byte) ( ((int)second + secsLapse) % 60);

      int modSecs2Minute = ((int)second + secsLapse) / 60;
      minute = (byte) (((int)minute + modSecs2Minute % 60);

      int modMin2Hour = ((int)minute + modSecs2Minute) / 60;
      hour = (byte) (((int)hour + ( ((int)hour + modMin2Hour) / 60) % 60);
      

      fakeDateTime = true;
      */
    }
    lastTimeGetGpsDT = millis();
    gpsReceiving = newdata;
  }
}

/**
 * @brief Get the Acc Gyro Device Info object
 * X axis will be a virtual forward axis and
 * Y axis will be a virtual left axis.
 * Z axis is a real upward axis.
 * 
 * This device originally gets positive accZ values when 
 * it is stabilized on a desk, but we whish to obtain 
 * negative values in that position, so we change the
 * obtaind signs.
 * 
 */
inline void getAccGyroDeviceInfo()
{
  // Update all the values
  while (!i2cDevice.readBytes(IMU_MPU6050_ADDRS, 0x3B, 14, i2cDataMPU6050));

  // Get values and change sign
  //accX = (int16_t)((i2cDataMPU6050[0] << 8) | i2cDataMPU6050[1]);
  accX = -(int16_t)((i2cDataMPU6050[0] << 8) | i2cDataMPU6050[1]);
  //accY = (int16_t)((i2cDataMPU6050[2] << 8) | i2cDataMPU6050[3]);
  accY = -(int16_t)((i2cDataMPU6050[2] << 8) | i2cDataMPU6050[3]);
  // accZ = (int16_t)((i2cDataMPU6050[4] << 8) | i2cDataMPU6050[5]);
  accZ = -(int16_t)((i2cDataMPU6050[4] << 8) | i2cDataMPU6050[5]);
  tempRaw = (int16_t)((i2cDataMPU6050[6] << 8) | i2cDataMPU6050[7]);
  // gyroX = (int16_t)((i2cDataMPU6050[8] << 8) | i2cDataMPU6050[9]);
  gyroX = -(int16_t)((i2cDataMPU6050[8] << 8) | i2cDataMPU6050[9]);
  // gyroY = (int16_t)((i2cDataMPU6050[10] << 8) | i2cDataMPU6050[11]);
  gyroY = -(int16_t)((i2cDataMPU6050[10] << 8) | i2cDataMPU6050[11]);
  // gyroZ = (int16_t)((i2cDataMPU6050[12] << 8) | i2cDataMPU6050[13]);
  gyroZ = -(int16_t)((i2cDataMPU6050[12] << 8) | i2cDataMPU6050[13]);

  // Rotate accelerometer and gyroscope readings to get a coordinate system
  // with X forward.
  // Rotate X and Y angles
  if (ACC_GYRO_X_AXIS_POSITION == ACC_GYRO_X_AXIS_FORWARD) // 0
  {// X = X', Y = Y'
    // Do not do anything
  }
  else if (ACC_GYRO_X_AXIS_POSITION == ACC_GYRO_X_AXIS_LEFT) //90
  {
    int16_t tmpX = accX;
    accX = -accY; // X = - Y'
    accY = tmpX; // Y = X'

    tmpX = gyroX;
    gyroX = -gyroY;
    gyroY = tmpX;
  }
  else if (ACC_GYRO_X_AXIS_POSITION == ACC_GYRO_X_AXIS_BACKWARD) // 180
  {
    accX = -accX; // X = -X'
    accY = -accY; // Y = - Y'

    gyroX = -gyroX;
    gyroY = -gyroY;
  }
  else if (ACC_GYRO_X_AXIS_POSITION == ACC_GYRO_X_AXIS_RIGHT) // 270 
  {
    int16_t tmpX = accX;
    accX = accY; // X = Y'
    accY = -tmpX; // Y = -X'

    tmpX = gyroX;
    gyroX = gyroY;
    gyroY = -tmpX;
  }
}

inline void calculateRotationAngles()
{
  // TO-DO FIX-ME :
  // a - Change Kalman filter to include compass info.
  // b - Then use this function after reading AccGyro and afeter reading compass info.

  // Calculate delta time in seconds
  dt = (double)(micros() - timer) / 1000000;
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees

  transformAccelGyroInfo2Angles();

  // Calculate gyro X Y Z rates
  gyroXrate = gyroX / GYRO_SENSITIVITY_SCALE_FACTOR_0250; // Convert to deg/s
  gyroYrate = gyroY / GYRO_SENSITIVITY_SCALE_FACTOR_0250; // Convert to deg/s 
  gyroZrate = gyroZ / GYRO_SENSITIVITY_SCALE_FACTOR_0250; // Convert to deg/s 

  // Calculate angles using Kalman filter
  kalAngleX = kalmanX.getAngle(rollAccX, gyroXrate, dt);
  kalAngleY = kalmanY.getAngle(pitchAccY, gyroYrate, dt);
  kalAngleZ = kalmanZ.getAngle(yawAccZ, gyroZrate, dt);

  #if MEGA2560_TRACE_TEST
    // Calculate gyro angle without any filter
    gyroXangle += gyroXrate * dt;
    gyroYangle += gyroYrate * dt;
    gyroZangle += gyroZrate * dt;
    //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    //gyroYangle += kalmanY.getRate() * dt;

    // Reset the gyro angle when it has drifted too much
    gyroXangle = (int16_t)angle_pm180(gyroXangle);
    gyroYangle = (int16_t)angle_pm180(gyroYangle);
    gyroZangle = (int16_t)angle_pm180(gyroZangle);
  #endif
}

inline void getSatelliteAttitude()
{
  // TO-DO
  //    Usar string proveniente del ESP8266 para crear la variable sat
  //// char *tle0, *tle1, *tle2;
  //// psSatelliteTLE[0].toCharArray(tle0, psSatelliteTLE[0].length());
  //// psSatelliteTLE[1].toCharArray(tle1, psSatelliteTLE[1].length());
  //// psSatelliteTLE[2].toCharArray(tle2, psSatelliteTLE[2].length());

  #if false // MEGA2560_TRACE_TEST
      debugMessage = String("psSatelliteTLE[0]=") + String(psSatelliteTLE[0]) + 
                    String("\tpsSatelliteTLE[1]=") + String(psSatelliteTLE[1]) + 
                    String("\tpsSatelliteTLE[2]=") + String(psSatelliteTLE[2]);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
  #endif

  sat = Satellite(psSatelliteTLE[0], psSatelliteTLE[1], psSatelliteTLE[2]);
  //// sat = Satellite("RADIO ROSTO (RS-15)     ",
  //// "1 23439U 94085A   19311.42076594 -.00000039  00000-0  83545-4 0  9991",
  //// "2 23439  64.8160 246.9267 0145361  85.5007 276.2506 11.27568548 23874") ;

  // Prediction time
  myDateTime.settime(year, month, day, hour, minute, second);
  // Create prediction
  sat.predict(myDateTime);
  // satElevation and satAzimuth from observer place
  sat.altaz(observationPlace, satElevation, satAzimuth);  
}

/**
 * @brief This function transforms accelerometer and 
 * gyroscope information into angles.
 * Every calculated angle is positive between [0,360]
 * 
 */
inline void transformAccelGyroInfo2Angles()
{
  rollAccX = atan2(accZ, accY) * RAD_TO_DEG;
  pitchAccY = atan2(accX,accZ) * RAD_TO_DEG;
  yawAccZ = atan2(accY, accX) * RAD_TO_DEG;

  // atan2() returns angles between [-180,180] 
  // but we will use angles between [0,360]
  rollAccX = positiveAngle(rollAccX);
  pitchAccY = positiveAngle(pitchAccY);
  yawAccZ = positiveAngle(yawAccZ);
}



/**
 * @brief Writes horizontalHeadingAngle variable.
 * The member function readHeading() gets North heading in
 * the device XY plane, but we need a projection of 
 * heading in the ground horizontal plane.
 * horizontalHeading() create a projection of this vector
 * in the ground plane writing the angle variable horizontalHeadingAngle. 
 */
inline void horizontalHeading()
{
  double mx, my, mz;
  // Read compas values adding compass orientation offset.
  if (compass.readNormalizedRotated(mx,my,mz))
  {
    // 1. Abyarjoo2015
    double xh = mx * cos(kalAngleY) + my * sin(kalAngleY) * sin(kalAngleX) + mz * sin(kalAngleY) * cos(kalAngleX);
    double yh = my * cos(kalAngleX) + mz * sin(kalAngleX);
    horizontalHeadingAngle = 180.0 * atan2(-yh,xh) / M_PI;
    // Add local declination correction
    horizontalHeadingAngle += (int16_t) LOCAL_DECLINATION;
    // Allow only positive values
    horizontalHeadingAngle = positiveAngle(horizontalHeadingAngle);
    
    // 2. Bingaman 2010 [pag.74]
    xh = mx * cos(kalAngleY) + my * sin(kalAngleY) * sin(kalAngleX) - mz * sin(kalAngleY) * cos(kalAngleX);
    yh = my * cos(kalAngleX) + mz * sin(kalAngleX);
    // double zh = mx * sin(kalAngleY) - my * sin(kalAngleX) * cos(kalAngleY) + mz *cos(kalAngleX) * cos(kalAngleY);
    horizontalHeadingAngle2 = 180.0 * atan2(yh,xh) / M_PI;
    // Add local declination correction
    horizontalHeadingAngle2 += (int16_t) LOCAL_DECLINATION;
    // Allow only positive values
    horizontalHeadingAngle2 = positiveAngle(horizontalHeadingAngle2);
   

    // Grygorenko 2014
    xh = mx * cos(kalAngleY) + my * sin(kalAngleY) * sin(kalAngleX) + mz * sin(kalAngleY) * cos(kalAngleX);
    yh = my * cos(kalAngleX) - mz * sin(kalAngleX);
    horizontalHeadingAngle3 = 180.0 * atan2(yh,xh) / M_PI;
    // Add local declination correction
    horizontalHeadingAngle3 += (int16_t) LOCAL_DECLINATION;
    // Allow only positive values
    horizontalHeadingAngle3 = positiveAngle(horizontalHeadingAngle3);

  }
}

void LcdIICInterrupt()
{
  command_cmd_lcd = TFTlcd.I2C_Read();
  TFTlcd.queue_push(command_cmd_lcd);
  for(cnt_lcd =0;cnt_lcd <2;cnt_lcd++)
  {
    identifier_lcd = TFTlcd.I2C_Read();
    TFTlcd.queue_push(identifier_lcd);
    // Serial.print(command_cmd_lcd, HEX);
    // Serial.print('\t');
    // Serial.println(identifier_lcd, HEX);
  }
  command_status_lcd = TFTlcd.I2C_Read();
  TFTlcd.queue_push(command_status_lcd);
  //// Serial.println(command_status_lcd, HEX);

  identifier_lcd = TFTlcd.I2C_Read();
  TFTlcd.queue_push(identifier_lcd);

  command_length_lcd = TFTlcd.I2C_Read();
  TFTlcd.queue_push(command_length_lcd);

  /* We do not use EDIT_VALUE
  if((command_cmd_lcd == GET_EDIT_VALUE && command_status_lcd == SUCCESS)||
      (command_cmd_lcd == GET_TOUCH_EDIT_VALUE && command_status_lcd == SUCCESS))
  {
      for(cnt_lcd =0;cnt_lcd <command_length_lcd;cnt_lcd++)
    {
      identifier_lcd = TFTlcd.I2C_Read();
      TFTlcd.queue_push(identifier_lcd);
      //Serial.println(identifier_lcd, HEX);
    }
  }
  */
}

void ProcessMessage( PCTRL_MSG msg, uint16_t dataSize )
{
    uint8_t cmd_type    = msg->cmd_type;
    uint8_t control_id  = msg->control_id;
    uint8_t page_id_lcd     = msg->page_id;
    uint8_t _status     = msg->status;
    uint8_t key_type    = msg->key_type;
    uint8_t key_value   = msg->key_value;
    #if MEGA2560_TRACE_INFO
      debugMessage = String("ProcessMessage()")
      + String("\t") + String(cmd_type, HEX) 
      + String("\t") + String(page_id_lcd) 
      + String("\t") + String(control_id) 
      + String("\t") + String(_status, HEX) 
      + String("\t") + String(key_type, HEX) 
      + String("\t") + String(key_value, HEX);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
    #endif

    switch(cmd_type)
    {
    case NOTIFY_TOUCH_BUTTON:
      NotifyTouchButton(page_id_lcd,control_id,_status,key_type,key_value);
      break;
  
    case NOTIFY_TOUCH_CHECKBOX:
      NotifyTouchCheckbox(page_id_lcd,control_id,_status,key_type,key_value);
      break;
  
    case NOTIFY_TOUCH_SLIDER:
      NotifyTouchSlider(page_id_lcd,control_id,_status,key_type,key_value);
      break;

    case NOTIFY_TOUCH_EDIT:
    NotifyTouchEdit(page_id_lcd,control_id,_status,key_type,key_value);
    break;
    
    case NOTIFY_GET_EDIT:
     NotifyGetEdit((PEDIT_MSG)cmd_buffer_lcd);
      break;

    case NOTIFY_GET_TOUCH_EDIT:
    NotifyGetTouchEdit((PEDIT_MSG)cmd_buffer_lcd);
    break;
  
    case NOTIFY_GET_PAGE:
      NotifyGetPage(page_id_lcd,_status);
      break;
  
    case NOTIFY_GET_CHECKBOX:
      NotifyGetCheckbox(page_id_lcd,control_id,_status,key_type,key_value);
      break;
  
    case NOTIFY_GET_SLIDER:
      NotifyGetSlider(page_id_lcd,control_id,_status,key_type,key_value);
      break;
      
    default:
      #if MEGA2560_TRACE_ERROR
        debugMessage = String("ProcessMessage()")
        + String("\t") + String(cmd_type, HEX) 
        + String("\t") + String(page_id_lcd) 
        + String("\t") + String(control_id) 
        + String("\t") + String(_status, HEX) 
        + String("\t") + String(key_type, HEX) 
        + String("\t") + String(key_value, HEX);
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_ERROR, STDebug::ORIGIN_MEGA2560, debugMessage);
      #endif
      break;
  }
}

void UpdateUI()
{
  #if MEGA2560_TRACE_TEST
    debugMessage = String("UpdateUI()=") + String(page_id_lcd);
    serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
  #endif
  if (page_id_lcd == MAIN_PAGE)
  {
    #if MEGA2560_TRACE_TEST
      debugMessage = String("UpdateUI().MAIN_PAGE") + String(page_id_lcd);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
    #endif
    uint8_t newStrSatName[25] = "MI NUEVO SAT ;-))";
    strcpy((char *)strSatName, (char *) newStrSatName);
    TFTlcd.SetLableValue(page_id_lcd,MAIN_LABEL_SAT_NAME,strSatName);      
    ///TFTlcd.SetLableValue(page_id_lcd,MAIN_LABEL_SAT_NAME,(unsigned char *)strSatName);

    // Get GPS time and write variables
    TFTlcd.SetNumberValue(page_id_lcd,MAIN_NUMBER_HH,hour);
    TFTlcd.SetNumberValue(page_id_lcd,MAIN_NUMBER_MM,minute);
    TFTlcd.SetNumberValue(page_id_lcd,MAIN_NUMBER_SS,second);
    #if MEGA2560_TRACE_TEST
      debugMessage = String("newStrSatName=") + String((char *)newStrSatName);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
    #endif
  }
  else if (page_id_lcd == ATTITUDE_PAGE)
  {
    #if MEGA2560_TRACE_TEST
      debugMessage = String("UpdateUI().ATTITUDE_PAGE") + String(page_id_lcd);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
    #endif
    // LCD local variables
    int16_t satelliteAzimut = satAzimuth;
    int16_t antennaAzimut = 270 - kalAngleY;
    int16_t satelliteAscension =  satElevation;
    satelliteAscension = positiveAngle(satelliteAscension);
    int16_t antennaAscension = -(int16_t) heading + 90;
    antennaAscension = positiveAngle(antennaAscension);   

    // Set numbers
    // Info:
    //  The member function SetNumberValue(...) writes only positive numbers.
    //  The new member function SetLabelValueInt(...) writes positive and negative numbers,
    //  but overlaps other fields.
    // So, first I will paint the absolute value number and secondly its sign.

    // Size and color of anegative sign
    const uint16_t ATT_SIGN_WIDTH = 10;
    const uint16_t ATT_SIGN_HEIGTH = 3;
    // Paint number
    // if(satelliteAzimut<0) satelliteAzimut +=360;    
    // TFTlcd.SetValueInt(page_id_lcd,74,44,satelliteAzimut);
    TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_AZIMUT_SATELLITE,abs(satelliteAzimut));
    // Paint sign
    const uint16_t ATT_SIGN_AZIMUT_SATELLITE_X = 18;
    const uint16_t ATT_SIGN_AZIMUT_SATELLITE_Y = 53;
    uint16_t thisSign = (satelliteAzimut < 0) ? COLOR_WHITE : COLOR_BLUE_SYSTEM;
    TFTlcd.RectangleFill(ATT_SIGN_AZIMUT_SATELLITE_X,ATT_SIGN_AZIMUT_SATELLITE_Y,ATT_SIGN_WIDTH,ATT_SIGN_HEIGTH,thisSign);

    // Paint number
    // if(antennaAzimut<0) antennaAzimut +=360;    
    // TFTlcd.SetValueInt(page_id_lcd,75,45,antennaAzimut);
    TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_AZIMUT_ANTENNA,abs(antennaAzimut));
    // Paint sign
    const uint16_t ATT_SIGN_AZIMUT_ANTENNA_X = 18;
    const uint16_t ATT_SIGN_AZIMUT_ANTENNA_Y = 90;
    thisSign = (antennaAzimut < 0) ? COLOR_WHITE : COLOR_BLUE_SYSTEM;
    TFTlcd.RectangleFill(ATT_SIGN_AZIMUT_ANTENNA_X,ATT_SIGN_AZIMUT_ANTENNA_Y,ATT_SIGN_WIDTH,ATT_SIGN_HEIGTH,thisSign);

    // Paint number
    // if(satelliteAscension<0) satelliteAscension +=360;    
    // TFTlcd.SetValueInt(page_id_lcd,76,46,satelliteAscension);
    TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_ASCENSION_SATELLITE,abs(satelliteAscension));
    // Paint sign
    const uint16_t ATT_SIGN_ASCENSION_SATELLITE_X = 200;
    const uint16_t ATT_SIGN_ASCENSION_SATELLITE_Y = 53;
    thisSign = (satelliteAscension < 0) ? COLOR_WHITE : COLOR_BLUE_SYSTEM;
    TFTlcd.RectangleFill(ATT_SIGN_ASCENSION_SATELLITE_X,ATT_SIGN_ASCENSION_SATELLITE_Y,ATT_SIGN_WIDTH,ATT_SIGN_HEIGTH,thisSign);

    // Paint number
    //// TFTlcd.SetLabelValueInt(page_id_lcd,ATT_NUMBER_ASCENSION_ANTENNA,antennaAscension);
    // Set numbers
    // if(antennaAscension<0) antennaAscension +=360;    
    // TFTlcd.SetValueInt(page_id_lcd,77,47,antennaAscension);
    TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_ASCENSION_ANTENNA,abs(antennaAscension));
    // antennaAscension is a positive value yet
    // TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_ASCENSION_ANTENNA,antennaAscension);
    const uint16_t ATT_SIGN_ASCENSION_ANTENNA_X = 200;
    const uint16_t ATT_SIGN_ASCENSION_ANTENNA_Y = 90;
    thisSign = (antennaAscension < 0) ? COLOR_WHITE : COLOR_BLUE_SYSTEM;
    TFTlcd.RectangleFill(ATT_SIGN_ASCENSION_ANTENNA_X,ATT_SIGN_ASCENSION_ANTENNA_Y,ATT_SIGN_WIDTH,ATT_SIGN_HEIGTH,thisSign);

    // Heading info
    TFTlcd.SetNumberValue(page_id_lcd,ATT_HEADING,heading);

    // Complementary trace info
    //TFTlcd.SetLabelValueInt(page_id_lcd,ATT_LABEL_TEST_01,yawZcompass);
    TFTlcd.SetLabelValueInt(page_id_lcd,ATT_LABEL_TEST_02,horizontalHeadingAngle);
    TFTlcd.SetLabelValueInt(page_id_lcd,ATT_LABEL_TEST_03,horizontalHeadingAngle2);
    TFTlcd.SetLabelValueInt(page_id_lcd,ATT_LABEL_TEST_04,horizontalHeadingAngle3);


    // Set figures
    // Set compass
    // Set sattellite ascension red line.
    // TFTlcd circle angles is left handed, but ascension is right handed from East position.
    // TFTlcd heading suplementary angle = 360 - heading
    // TFTlcd East = (360 - heading) + 90
    // TFTlcd circle angle for satellite = East - satelliteAscension
    // int lcdSatAscension = (360 - heading + 90) - satelliteAscension;
    int lcdSatAscension = (90 - heading) - satelliteAscension;
    lcdSatAscension = positiveAngle(lcdSatAscension);

    TFTlcd.SetCircleGaugeValue(page_id_lcd,ATT_COMPASS,(uint16_t) lcdSatAscension);

    // Azimut bar graph constants
    const uint16_t ATT_AZIMUT_BAR_GRAPH_WIDTH = 36;
    const uint16_t ATT_AZIMUT_BAR_GRAPH_HEIGHT = 238;
    // Set satelite azimut
    const uint16_t ATT_AZIMUT_SATELLITE_BAR_GRAPH_X = 9;
    const uint16_t ATT_AZIMUT_SATELLITE_BAR_GRAPH_Y = 156;
    // Set satellite azimut
    if (satelliteAzimut < 0)
    {
      satelliteAzimut = 0;
    }
    else if (satelliteAzimut > 90)
    {
      satelliteAzimut = 90;
    }
    // Escale satelliteAzimut to BarGraph from 0 to 100
    satelliteAzimut =(int) round((satelliteAzimut*10)/9); // * 100/90
    TFTlcd.SetBarGraph(ATT_AZIMUT_SATELLITE_BAR_GRAPH_X,
                      ATT_AZIMUT_SATELLITE_BAR_GRAPH_Y,
                      ATT_AZIMUT_BAR_GRAPH_WIDTH/2,
                      ATT_AZIMUT_BAR_GRAPH_HEIGHT,
                      satelliteAzimut,COLOR_RED);
    // Set antenna azimut
    const uint16_t ATT_AZIMUT_ANTENNA_BAR_GRAPH_X = 86;
    const uint16_t ATT_AZIMUT_ANTENNA_BAR_GRAPH_Y = 156;
    if (antennaAzimut < 0)
    {
      antennaAzimut = 0;
    }
    else if (antennaAzimut > 90)
    {
      antennaAzimut = 90;
    }
    // Escale antennaAzimut to BarGraph from 0 to 100
    antennaAzimut =(int) round((antennaAzimut*10)/9); // * 100/90
    // Set smart color
    uint16_t antennaBargraphColor = COLOR_BLUE;
    if (((satelliteAzimut - 1) <= antennaAzimut) && (antennaAzimut <= (satelliteAzimut + 1)))
      antennaBargraphColor = COLOR_GREEN;
    TFTlcd.SetBarGraph(ATT_AZIMUT_ANTENNA_BAR_GRAPH_X,
                      ATT_AZIMUT_ANTENNA_BAR_GRAPH_Y,
                      ATT_AZIMUT_BAR_GRAPH_WIDTH,
                      ATT_AZIMUT_BAR_GRAPH_HEIGHT,
                      antennaAzimut,antennaBargraphColor);
    
    // Set satellite name
    //// uint8_t *newStrSatName = (uint8_t *) psSatelliteTLE[0];
    //// strcpy((char *)strSatName, (char *) newStrSatName);
    
    ////if (newSatelliteName)
    ////{
    strcpy((char *)strSatName, (char *) psSatelliteTLE[0]);    
    TFTlcd.SetLableValue(page_id_lcd,ATT_LABEL_SAT_NAME,strSatName);
    ////  newSatelliteName = false;
   //// }

    // Get GPS time and write variables
    TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_HH,hour);
    TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_MM,minute);
    TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_SS,second);

    // Set GPS signal ON-OFF
    const uint16_t ATT_GPS_ON_X = 153;
    const uint16_t ATT_GPS_ON_Y = 388;
    const uint16_t ATT_GPS_WIDTH = 17;
    const uint16_t ATT_GPS_HEIGTH = 17;
    uint16_t gpsRectangleColor = (gpsReceiving) ? COLOR_DARKGREEN : COLOR_GREY;
    TFTlcd.RectangleFill(ATT_GPS_ON_X,ATT_GPS_ON_Y,ATT_GPS_WIDTH,ATT_GPS_HEIGTH,gpsRectangleColor);
  }
  else  if(page_id_lcd == SATELLITES_PAGE)
  {
    // TO-DO
    // Show list of satellites

    #if MEGA2560_TRACE_TEST
      debugMessage = String("UpdateUI().SATELLITES_PAGE") + String(page_id_lcd);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
    #endif
    uint8_t newStrPagenumber[6] = "01/33";
    strcpy((char *)strPagenumber, (char *) newStrPagenumber);
    TFTlcd.SetLableValue(page_id_lcd,SAT_LABEL_PAGE_NUMB,strPagenumber);

    checkSat01 = 0;
    TFTlcd.SetCheckboxValue(page_id_lcd,SAT_CHECK_SAT_01,checkSat01);
    checkSat02 = 1;
    TFTlcd.SetCheckboxValue(page_id_lcd,SAT_CHECK_SAT_02,checkSat02);

    TFTlcd.SetCheckboxValue(page_id_lcd,SAT_CHECK_SAT_03,1);
    ///strcpy(checkSat01, "Satelite num 01");
    //TFTlcd.SetCheckboxValue(page_id_lcd,SAT_CHECK_SAT_01,(unsigned char *)checkSat01);
    ///strcpy(checkSat02, "Satelite num 02");
    //TFTlcd.SetCheckboxValue(page_id_lcd,SAT_CHECK_SAT_02,(unsigned char *)checkSat02);
    ///strcpy(checkSat03, "Peperrrrllllll 02");
    //TFTlcd.SetCheckboxValue(page_id_lcd,SAT_CHECK_SAT_03,(unsigned char *)checkSat03);
  }
}

void NotifyTouchButton(uint8_t page_id, uint8_t control_id, uint8_t  state,uint8_t type,uint8_t value)
{
  //TODO:
  if(type == CHANGE_PAGE&& state == KEY_RELEASE)
  {
    page_id_lcd = value;
    update_lcd = 1;
    //UpdateUI();
  }
  else if(type == ENTER&& state == KEY_RELEASE)
  {
    targe_Id = value;//targe Edit Id
    TFTlcd.GetEditValue(page_id_lcd,targe_Id);
  }
  else if(type == CHAR)
  {
  }
  else if(type == UPLOAD_CONTROL_ID && state == KEY_RELEASE)
  {
    // BEGIN imjaviperez
    if (control_id == SAT_BUTTON_OK)
    {
      // TO-DO: Save selected satellite name
      page_id_lcd = MAIN_PAGE;
      TFTlcd.SetPage(MAIN_PAGE);
      update_lcd = 1;
    }  
    else if(control_id == SAT_BUTTON_UP)
    {
      // TO-DO: Show previous page
      TFTlcd.Display_Message(0X18,2,(unsigned char *)strMsgButtonUp);
      update_lcd = 1;
    }
    else if(control_id == SAT_BUTTON_DOWN)
    {
      // TO-DO: Show next page
      TFTlcd.Display_Message(0X18,2,(unsigned char *)strMsgButtonDown);
      update_lcd = 1;
    }
    // END imjaviperez
  }
  else if(type == CLEAR)
  {
  }

}

void NotifyTouchCheckbox(uint8_t page_id, uint8_t control_id, uint8_t  state,uint8_t type,uint8_t value)
{
  //TODO:

  // BEGIN imjaviperez
  // TFTlcd.SetCheckboxValue(page_id_lcd,control_id, 0);
  // BEGIN imjaviperez


  if(state == SELECT)
    update_lcd = 1;

  //UpdateUI();
}

void NotifyTouchSlider(uint8_t page_id, uint8_t control_id, uint8_t  state,uint8_t type,uint8_t value)
{
  //TODO:
  //if(update_lcd != 1)
  //  TFTlcd.SetNumberValue(page_id_lcd,28,(uint16_t)value);
  //UpdateUI();
}

void NotifyTouchEdit(uint8_t page_id, uint8_t control_id, uint8_t  state,uint8_t type,uint8_t value)
{
  //TODO:
  if(update_lcd != 1)
    TFTlcd.GetTouchEditValue(page_id_lcd,control_id);
  
}

void NotifyGetEdit(PEDIT_MSG msg)
{
  uint8_t cmd_type    = msg->cmd_type;  //command
  uint8_t control_id  = msg->control_id;//object Id
  uint8_t page_id     = msg->page_id;   //page Id
  uint8_t _status     = msg->status;


  //The test passward number 1 2 3 4,ASCII code is 0x31 0x32 0x33 0x34
  if(msg->param[0] == 0x31 && msg->param[1] == 0x32 && msg->param[2] == 0x33 && msg->param[3] == 0x34)
  {
    //// TFTlcd.Display_Message(0X18,2,(unsigned char *)String01);  
  }
  else
  {
    //// TFTlcd.Display_Message(0X18,2,(unsigned char *)String02);  
  }
 
}

void NotifyGetTouchEdit(PEDIT_MSG msg)
{
  uint8_t cmd_type    = msg->cmd_type;  //command
  uint8_t control_id  = msg->control_id;//object Id
  uint8_t page_id     = msg->page_id;   //page Id
  uint8_t _status     = msg->status;


  //The test passward number 1 2 3 4,ASCII code is 0x31 0x32 0x33 0x34
  if(msg->param[0] == 0x31 && msg->param[1] == 0x32 && msg->param[2] == 0x33 && msg->param[3] == 0x34)
  {
    //// TFTlcd.Display_Message(0X18,2,(unsigned char *)String04);  
  }
  else
  {
    //// TFTlcd.Display_Message(0X18,2,(unsigned char *)String05);  
  }
 
}

void NotifyGetPage(uint8_t page_id,uint8_t status)
{
  //TODO:
  if(status == SUCCESS)
    page_id_lcd = page_id;
}


void NotifyGetCheckbox(uint8_t page_id, uint8_t control_id, uint8_t  state,uint8_t type,uint8_t value)
{
  //TODO:
  if(state == SELECT)
  {
    update_lcd = 1;
  }
  //UpdateUI();
}

void NotifyGetSlider(uint8_t page_id, uint8_t control_id, uint8_t  state,uint8_t type,uint8_t value)
{
  //TODO:
  if(state == SUCCESS)
  {
    //success get value
  }
  update_lcd = 1;
}



