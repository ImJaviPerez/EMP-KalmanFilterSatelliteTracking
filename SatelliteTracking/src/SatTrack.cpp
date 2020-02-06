/**
 * @file SatTrack.cpp 
 * @author your name (ImJaviPerez)
 * @brief 
 * @version 0.1.004
 * @date 2019-12-20 18:21
 * 
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include "SatTrack.h"
#include <STSerialMsg.h>
#include <STDebug.h>
#include <TinyGPS.h>  // GPS NEO V2
#include <QMC5883L_ST.h> // Digital compass
#include <Wire.h>     // Digital compass I2C communication
#include <I2Cdev.h>   // MPU6050 accelerometer Gyroscope communication
#include <Kalman_ST.h> // Source: https://github.com/TKJElectronics/KalmanFilter


#include "STConfig.h"


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
#endif
// Messages between MEGA2560 and ESP8266
STSerialMsg serialMsgESP(STSerialMsg::PORT_THREE);


#if MEGA2560_TRACE
  // Declare auxiliar variable
  String debugMessage = "";
#endif

// Variables to smart-delay messages ---------------------
#if MEGA2560_TRACE_INFO
  unsigned long lastTimeTrace = millis();
  const unsigned long TRACE_TIME_PERIOD = 3000; // milliseconds

  unsigned long traceTimeCompass = 0;
  unsigned long traceTimeAngles = 0;
  unsigned long traceTimeSatAtt = 0;
#endif

// Global varibles ---------------------------------------
String strSatelliteListNames = "";

// FIX-ME
//  Change variable "String strSatelliteTLE[]" with "char *psSatelliteTLE[]"
//// // String strSatelliteTLE[3];
//// String strSatelliteTLE[] = {"RADIO ROSTO (RS-15)     ",
//// "1 23439U 94085A   19345.83261864 -.00000019  00000-0  66113-3 0  9998",
//// "2 23439  64.8132 191.2812 0145849  78.7161 283.0113 11.27569313 27759"};

char *psSatelliteTLE[] = {"BEESAT-4                ",
"1 41619U 16040W   19353.83704974  .00001170  00000-0  51786-4 0  9995",
"2 41619  97.3458  52.4343 0012909  95.7965 264.4744 15.23549211181893"};

String satelliteName = "NUSAT-1 (FRESCO)";

//// String msgInfoRead = "";
////  char msgCommandRead = ' ';

// GPS variables -----------------------------------------
const int DEFAULT_YEAR = 2019;
const byte DEFAULT_MONTH = 12;
const byte DEFAULT_DAY = 25;
const byte DEFAULT_HOUR = 12;
const byte DEFAULT_MINUTE = 0;
const byte DEFAULT_SECOND = 0;

TinyGPS gps;
long lat, lon;
float flat, flon;
unsigned long age = 0, date = 0, time = 0, chars = 0;
int year = DEFAULT_YEAR;
byte month = DEFAULT_MONTH, day = DEFAULT_DAY;
byte hour = DEFAULT_HOUR, minute = DEFAULT_MINUTE, second = DEFAULT_MINUTE, hundredths = 0;

#define UTC_LOCAL +2
#define UTC_CITY "MADRID"

// Digital compass variables -----------------------------
QMC5883L_ST compass;

// After the device is powered on, some time periods are 
// required for the device fully functional. 
// The external power supply requires a time period for 
// voltage to ramp up (PSUP), it is typically 
// 50 milli-second. However it isn’t controlled by the device.
const unsigned long COMPASS_SAMPLING_RATE = 50; // milliseconds
// The power on/off time related to the device are:
// POR Completion Time: Time Period After VDD and VDDIO at Operating Voltage to Ready for I2C Commend and Analogy Measurement
// // const unsigned long PORCT = 1; // 350 micro seconds
// PINT: Power on Interval. Time Period Required for Voltage Lower Than SDV (0.2 volts) to Enable Next PORCT
// // const unsigned long PINT = 1; // 100 micro seconds

// MPU6050 accelerometer Gyroscope -----------------------
// Create the Kalman instance
Kalman_ST kalmanX;

// IMU Data
// Accelerometers: raw angles
double accX, accY, accZ;
// Gyroscopes: raw angles
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
// Angle calculated using the gyro only
double gyroXangle, gyroYangle, gyroZangle;
// Calculated angle using a Kalman filter
double kalAngleX; 

uint32_t timer;
// Buffer for I2C data
uint8_t i2cData[14];
// MPU6050 Communication channel
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

const char DEFAULT_PLACE[] = "Bilbao"; // "Leioa";
const double DEFAULT_LATITUDE = 43.3000; // 43.328955;
const double DEFAULT_LONGITUDE = 2.9333; // -2.966181;
const double DEFAULT_ALTITUDE = 37.0; // 40.0; // meters

//   Observer postition. Plan13
char *observerPlace = DEFAULT_PLACE;
double observerLat = DEFAULT_LATITUDE;
double observerLong = DEFAULT_LONGITUDE;
double observerAlt = DEFAULT_ALTITUDE;
const Observer &observationPlace = Observer(observerPlace, observerLat, observerLong, observerAlt) ;
  

// Status for asking arguments ---------------------------
bool askSatelliteNames = true;
bool askOneSatelliteTLE = false;
/// bool fakeDateTime = true;

bool putSatelliteNames = false;
bool putDeviceOrientation = true;

bool showDeviceInfo = true;

// Variables for smart delay sampling ====================
// Period of time to get GPS Date and Time (in millisecons)
unsigned long lastTimeGetGpsDT = millis();
const unsigned long TIME_PERIOD_GPS_DT = 3000; // milliseconds
unsigned long lastTimeGps = millis();
const unsigned long GPS_TIME_PERIOD = 50; // milliseconds
unsigned long lastTimeCompass = millis();
const unsigned long COMPASS_TIME_PERIOD = COMPASS_SAMPLING_RATE + 5; // milliseconds
unsigned long lastTimeAngles = millis();
const unsigned long ANGLES_TIME_PERIOD = 40; // milliseconds
unsigned long lastTimeSatAttitude = millis();
const unsigned long SAT_ATTITUDE_TIME_PERIOD = 40; // milliseconds

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

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = GYRO_FULL_SCALE_RANGE_0250; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  //// while (i2cDevice.i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (!i2cDevice.writeBytes(IMU_MPU6050_ADDRS, 0x19, 4,i2cData)); // Write to all four registers at once
  //// while (i2cDevice.i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (!i2cDevice.writeByte(IMU_MPU6050_ADDRS,0x6B, 0x01)); // PLL with X axis gyroscope reference and disable sleep mode

  //// while (i2cDevice.i2cRead(0x75, i2cData, 1));
  while (!i2cDevice.readByte(IMU_MPU6050_ADDRS, 0x75, i2cData));
  //// if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
  if (i2cData[0] != IMU_MPU6050_ADDRS) { // Read "WHO_AM_I" register
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
  // Initialize acelerometer gyroscope
  // Set kalman and gyro starting angle
  //// while (i2cDevice.i2cRead(0x3B, i2cData, 6));
  while (!i2cDevice.readBytes(IMU_MPU6050_ADDRS, 0x3B, 6, i2cData));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees

  double roll, pitch;
  /// #ifdef RESTRICT_PITCH // Eq. 25 and 26
  ///   double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  ///   double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
  /// #else // Eq. 28 and 29
  ///   double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  ///   double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  /// #endif

  if (ROLL_OFFSET == X_AXIS_RIGHT)
  {
    roll  = atan2(accY, accZ) * RAD_TO_DEG;
    pitch = atan2(-accX, accZ) * RAD_TO_DEG;
  }
  else if (ROLL_OFFSET == X_AXIS_LEFT)
  {
    roll  = atan2(accY, accZ) * RAD_TO_DEG;
    pitch = atan2(-accX, accZ) * RAD_TO_DEG;
    roll = -roll;
    pitch = -pitch;
  }
  else if (ROLL_OFFSET == X_AXIS_FORWARD)
  {
    roll  = atan2(-accX, accZ) * RAD_TO_DEG;
    pitch = atan2(accY, accZ) * RAD_TO_DEG;
    roll = -roll;
  }
  else if (ROLL_OFFSET == X_AXIS_BACKWARD)
  {
    roll  = atan2(-accX, accZ) * RAD_TO_DEG;
    pitch = atan2(accY, accZ) * RAD_TO_DEG;
    pitch = -pitch;
  }
  
 // Set starting angles
  kalmanX.setAngle(roll);
  gyroXangle = roll;
  gyroYangle = pitch;
  gyroZangle = 0;
  //// compAngleX = roll;
  //// compAngleY = pitch;

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

  
  // Force reading from ESP8266 Serial3 port before loop() statement
  mySerialEvent3();
    
  // TO-DO
  // Show list of satellites

  #if MEGA2560_TRACE_TEST
    debugMessage = "End of setup()";
    serialMsgDBG.printDebug(STDebug::STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
  #endif

}


void loop() {
  // Compass heading
  int heading;

  // Change status periodically ==========================
  // TO-DO FIX-ME
  //    Change status values from LCD
  // Only for testing purpouses --------------------------
  anotherPeriodOfTime = true;
  // Change status periodically
  #if MEGA2560_TRACE_INFO
    //// String satelliteName = "TECHSAT 1B (GO-32)";
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
      askSatelliteNames = true;
      askOneSatelliteTLE = false;

      showDeviceInfo  =false;

      putSatelliteNames = false;
      putDeviceOrientation = false;
  }
  // End of Only for testing purpouses -------------------
  // End of Change status periodically ===================

  
  // =====================================================
  // Smart delays and get information from devices =======
  // GPS -------------------------------------------------
  if (millis() - lastTimeGps >= GPS_TIME_PERIOD){
    lastTimeGps = millis();
    // Read info from GPS: date, time and observer position
    getGPSInfo();
    observerLat = flat;
    observerLong = flon;
    observerAlt = gps.f_altitude();
    //// askGPSDateTime = true;
  }
/* ###################################################
  //Compass ----------------------------------------------
  // Get device compass orientation every while
  if (millis() - lastTimeCompass >= COMPASS_TIME_PERIOD){
    lastTimeCompass = millis();
    // Calculate device North-South orientation ----------
    // TO-DO FIX-ME
    //    readHeading() gets North heading in the device XY plane, 
    //    but we need a projection of heading in the ground horizontal plane.
    //    So create a funtion readHeading(headXY, headYZ, headZX) to obtain a vector 
    //    and create a projection of this vector in the ground plane
    // Get compass value
    heading = compass.readHeading();
    // While QMC5883L_ST compass is calibrating heading == 0
    #if MEGA2560_TRACE_INFO
      traceTimeCompass += COMPASS_TIME_PERIOD;
      if (traceTimeCompass > TRACE_TIME_PERIOD)
      {
        debugMessage = String("heading=") + String(heading);
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
        traceTimeCompass = 0;
      }
    #endif
    //// askNorthDirection = true;
  }

  // Accelerometer-Gyroscope -----------------------------
  if (millis() - lastTimeAngles >= ANGLES_TIME_PERIOD){
    lastTimeAngles = millis();
    // Get device angles attitude
    getDeviceAngles();
    //// askDeviceAngles = true;
  }

  // Calculate satellite attitude ------------------------
  if (millis() - lastTimeSatAttitude >= SAT_ATTITUDE_TIME_PERIOD)
  {
    lastTimeSatAttitude = millis();
    getSatelliteAttitude();     
    //// calculateSatelliteAttitude = false;
  }
  // End of Smart delays =================================
  ################################################### */

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
    if (!serialMsgESP.requestSatelliteTLE(satelliteName))
    {
      #if MEGA2560_TRACE_WARNING
        debugMessage = String("ERROR rSTLE Name=") + satelliteName;
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_WARNING, STDebug::ORIGIN_MEGA2560, debugMessage);
      #endif
    }else{
      askOneSatelliteTLE = false;
    }
  }
  // End of Ask ESP8266 for information ==================



  // =====================================================
  // Send messages to move the device
  // TO-DO
  //    Show how to move the device
  if (showDeviceInfo)
  {
    // FIX-ME
    //    Revisar estos calculos
    double diffAzimuth = satAzimuth - heading;
    double diffElevation = satElevation - kalAngleX;
    #if MEGA2560_TRACE_INFO
      debugMessage = String("Differences:Azimuth=") + String(diffAzimuth) + String(".Elevation=") + String(diffElevation);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);

    #endif
    showDeviceInfo = false;
  }
  // =====================================================


  // Force reading from Serial3 port during loop() statement
  //// serialEvent3();
  mySerialEvent3();
 }

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
    //// if (Serial3.available() > 0) 
    //// {
    ////   debugMessage = String("Serial3.available()=") + String(Serial3.available());
    ////   serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
    //// }
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
      #if MEGA2560_TRACE_INFO /// MEGA2560_TRACE_TEST
        // Show data on serial monitor
        debugMessage = String(STDebug::DBG_MSG_SAT_NAMES) + strSatelliteListNames;
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
      #endif
      break;
    
    case STSerialMsg::PUT_TLE0_SATELLITE:
      //// strSatelliteTLE[0] = serialMsgESP.commandInfo();
      tmpCommandInfo = serialMsgESP.commandInfoRead();
      tmpCommandInfo.toCharArray(psSatelliteTLE[0], TLE_LINE_0_LENGTH);
      #if MEGA2560_TRACE_TEST
        // Show data on serial monitor
        //// debugMessage = String(STDebug::DBG_MSG_TLE_LINE_0) + strSatelliteTLE[0];
        debugMessage = String(STDebug::DBG_MSG_TLE_LINE_0) + String(psSatelliteTLE[0]);
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
      #endif
      break;
    
    case STSerialMsg::PUT_TLE1_SATELLITE:
      //// strSatelliteTLE[1] = serialMsgESP.commandInfo();
      tmpCommandInfo = serialMsgESP.commandInfoRead();
      tmpCommandInfo.toCharArray(psSatelliteTLE[1], TLE_LINE_1_LENGTH);
      #if MEGA2560_TRACE_TEST
        // Show data on serial monitor
        //// debugMessage = String(STDebug::DBG_MSG_TLE_LINE_1) + strSatelliteTLE[1];
        debugMessage = String(STDebug::DBG_MSG_TLE_LINE_1) + String(psSatelliteTLE[1]);
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
      #endif
      break;
    
    case STSerialMsg::PUT_TLE2_SATELLITE:
      //// strSatelliteTLE[2] = serialMsgESP.commandInfo();
      tmpCommandInfo = serialMsgESP.commandInfoRead();
      tmpCommandInfo.toCharArray(psSatelliteTLE[2], TLE_LINE_2_LENGTH);
      #if MEGA2560_TRACE_TEST
        // Show data on serial monitor
        //// debugMessage = String(STDebug::DBG_MSG_TLE_LINE_2) + strSatelliteTLE[2];
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
          debugMessage = "NEW DATA!!";
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
  }
}

inline void getDeviceAngles()
{
  // Update all the values
  while (!i2cDevice.readBytes(IMU_MPU6050_ADDRS, 0x3B, 14, i2cData));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  // Calculate delta time in seconds
  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
  double roll, pitch;
  double gyroXrate, gyroYrate, gyroZrate;

  if (ROLL_OFFSET == X_AXIS_RIGHT)
  {
    roll  = atan2(accY, accZ) * RAD_TO_DEG;
    pitch = atan2(-accX, accZ) * RAD_TO_DEG;
    gyroXrate = gyroX / GYRO_SENSITIVITY_SCALE_FACTOR_0250; // Convert to deg/s
    gyroYrate = gyroY / GYRO_SENSITIVITY_SCALE_FACTOR_0250; // Convert to deg/s
    gyroZrate = gyroZ / GYRO_SENSITIVITY_SCALE_FACTOR_0250; // Convert to deg/s
  }
  else if (ROLL_OFFSET == X_AXIS_LEFT)
  {
    roll  = atan2(accY, accZ) * RAD_TO_DEG;
    pitch = atan2(-accX, accZ) * RAD_TO_DEG;
    roll = -roll;
    pitch = -pitch;
    gyroXrate = -gyroX / GYRO_SENSITIVITY_SCALE_FACTOR_0250; // Convert to deg/s
    gyroYrate = -gyroY / GYRO_SENSITIVITY_SCALE_FACTOR_0250; // Convert to deg/s
    gyroZrate = gyroZ / GYRO_SENSITIVITY_SCALE_FACTOR_0250; // Convert to deg/s
  }
  else if (ROLL_OFFSET == X_AXIS_FORWARD)
  {
    roll  = atan2(-accX, accZ) * RAD_TO_DEG;
    pitch = atan2(accY, accZ) * RAD_TO_DEG;
    roll = -roll;
    gyroXrate = -gyroY / GYRO_SENSITIVITY_SCALE_FACTOR_0250; // Convert to deg/s
    gyroYrate = gyroX / GYRO_SENSITIVITY_SCALE_FACTOR_0250; // Convert to deg/s
    gyroZrate = gyroZ / GYRO_SENSITIVITY_SCALE_FACTOR_0250; // Convert to deg/s
  }
  else if (ROLL_OFFSET == X_AXIS_BACKWARD)
  {
    roll  = atan2(-accX, accZ) * RAD_TO_DEG;
    pitch = atan2(accY, accZ) * RAD_TO_DEG;
    pitch = -pitch;
    gyroXrate = gyroY / GYRO_SENSITIVITY_SCALE_FACTOR_0250; // Convert to deg/s
    gyroYrate = -gyroX / GYRO_SENSITIVITY_SCALE_FACTOR_0250; // Convert to deg/s
    gyroZrate = gyroZ / GYRO_SENSITIVITY_SCALE_FACTOR_0250; // Convert to deg/s
  }

  // Calculate angles using Kalman filter
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);

  // Calculate gyro angle without any filter
  gyroXangle += gyroXrate * dt;
  gyroYangle += gyroYrate * dt;
  gyroZangle += gyroZrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;


  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180)
    gyroXangle += 360;
  if (gyroXangle > 180)
    gyroXangle -= 360;
  if (gyroYangle < -180)
    gyroYangle += 360;
  if (gyroYangle > 180)
    gyroYangle -= 360;
  if (gyroZangle < -180)
    gyroZangle += 360;
  if (gyroZangle > 180)
    gyroZangle -= 360;

  #if MEGA2560_TRACE_INFO //### MEGA2560_TRACE_TEST
    traceTimeAngles += ANGLES_TIME_PERIOD;
    if (traceTimeAngles > TRACE_TIME_PERIOD)
    {
      #if MEGA2560_TRACE_TEST
      // Show data on serial monitor
        debugMessage = String("accX=") + String(accX) + String("\taccY=") + String(accY) + String("\taccZ=") + String(accZ);
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
        debugMessage = String("gyroX=") + String(gyroX) + String("\tgyroY=") + String(gyroY) + String("\tgyroZ=") + String(gyroZ);
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
        debugMessage = String("\tkalAngleX=") + String(kalAngleX);
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
        debugMessage = String("pitch=") + String(pitch) + String("\tgyroYangle=") + String(gyroYangle);
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
        debugMessage = String("roll=") + String(roll) + String("\tgyroXangle=") + String(gyroXangle);
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);

        // Show the temperature
        double temperature = (double)tempRaw / 340.0 + 36.53;
        debugMessage = String("temperature=") + String(temperature); 
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
      #endif

      #if MEGA2560_TRACE_INFO
        debugMessage = String("accX=") + String(accX) + String("\tgyroX=") + String(gyroX) + String("\tkalAngleX=") + String(kalAngleX);
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
        debugMessage = String("accY=") + String(accY) + String("\tgyroY=") + String(gyroY);
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
        debugMessage = String("accZ=") + String(accZ) + String("\tgyroZ=") + String(gyroZ);
        serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
      #endif
      //// delay(2);
      traceTimeAngles = 0;
    }
  #endif

}

inline void getSatelliteAttitude()
{
  // TO-DO
  //    Usar string proveniente del ESP8266 para crear la variable sat
  //// char *tle0, *tle1, *tle2;
  //// strSatelliteTLE[0].toCharArray(tle0, strSatelliteTLE[0].length());
  //// strSatelliteTLE[1].toCharArray(tle1, strSatelliteTLE[1].length());
  //// strSatelliteTLE[2].toCharArray(tle2, strSatelliteTLE[2].length());

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
  
  #if MEGA2560_TRACE_INFO
    traceTimeSatAtt += SAT_ATTITUDE_TIME_PERIOD;
    if (traceTimeSatAtt > TRACE_TIME_PERIOD)
    {
      // debugMessage = String("tle0=") + String(psSatelliteTLE[0]) + 
      //               String("\ttle1=") + String(psSatelliteTLE[1]) + 
      //               String("\ttle2=") + String(psSatelliteTLE[2]);
      // serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);

      // myDateTime in ASCII for printing purpouses
      myDateTime.ascii(asciiDateTime);
      debugMessage = String(sat.name) +  
                    String("\t") + String(observationPlace.name) + 
                    String("\t") +String(asciiDateTime);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
      debugMessage = String("Altitude=") + String(satElevation) + 
                    String("\tAzimuth=") + String(satAzimuth);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
      traceTimeSatAtt = 0;
    }
  #endif
}