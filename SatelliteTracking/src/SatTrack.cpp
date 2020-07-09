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
#include <Kalman_AGC.h> // Accelerometer, Gyroscope, Compass
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
  const unsigned long TRACE_TIME_PERIOD = ANGLES_TIME_PERIOD; // milliseconds
#endif

// Global varibles ---------------------------------------
String strSatelliteListNames = "";
String askTLEsatelliteName = "OSCAR 7 (AO-7)";

//// // 22-05-2020 5:15:00
//// char *psSatelliteTLE[] = {"ITUPSAT1                ",
//// "1 35935U 09051E   20142.79780545  .00000077  00000-0  28797-4 0  9995",
//// "2 35935  98.6206 326.2921 0007353 213.7759 146.2970 14.55613020565827"};
//// // 01-06-2020 9:0:30
//// char *psSatelliteTLE[] = {"BEESAT-3                ",
//// "1 39135U 13015F   20152.90529515  .00000809  00000-0  48747-4 0  9998",
//// "2 39135  64.8657 354.8565 0037014 250.6817 109.0305 15.18149603392706"};

//// // 01-06-2020 9:0:30
//// char *psSatelliteTLE[] = {"BEESAT-3                ",
//// "1 39135U 13015F   20149.80919899  .00000684  00000-0  42744-4 0  9993",
//// "2 39135  64.8659   4.8562 0037220 251.2881 108.4205 15.18143310392626"};


//// // Leioa
//// // 10-07-2020 9:16:0 - 9:26:0
//// char *psSatelliteTLE[] = {"DELFI-C3 (DO-64)        ",
//// "1 32789U 08021G   20181.67857140  .00000907  00000-0  63259-4 0  9995",
//// "2 32789  97.4170 224.0528 0010279 274.3717  85.6337 15.07761691663754"};

// Leioa
// 10-07-2020 9:14:0 - 9:25:0
char *psSatelliteTLE[] = {"DELFI-C3 (DO-64)        ",
"1 32789U 08021G   20184.86410528  .00001163  00000-0  79979-4 0  9995",
"2 32789  97.4169 227.1267 0010324 261.3421  98.6638 15.07768695664137"};

//// // Leioa
//// // 10-07-2020 9:11:0 - 9:24:0
//// char *psSatelliteTLE[] = {"CAS-2T & KS-1Q          ",
//// "1 41847U 16066G   20181.66127519  .00000040  00000-0  16486-4 0  9995",
//// "2 41847  98.6528 232.3017 0365488 192.3805 166.8218 14.38073886191173"};




// GPS variables -----------------------------------------
const int DEFAULT_YEAR = 2020;
const byte DEFAULT_MONTH = 7;
const byte DEFAULT_DAY = 10;
const byte DEFAULT_HOUR = 8;
const byte DEFAULT_MINUTE = 55;
const byte DEFAULT_SECOND = 00;

uint32_t uGpsReceiving = 0x00;

TinyGPS gps;
long lat, lon;
float flat, flon;
unsigned long age = 0, date = 0, time = 0, chars = 0;
int year = DEFAULT_YEAR;
byte month = DEFAULT_MONTH, day = DEFAULT_DAY;
byte hour = DEFAULT_HOUR, minute = DEFAULT_MINUTE, second = DEFAULT_SECOND, hundredths = 0;

// Digital compass variables -----------------------------
// compass variances of roll X angle, pitch Y angle and yaw Z angle 
float sigma2compassX = 2*M_PI*2*M_PI, sigma2compassY = 2*M_PI*2*M_PI, sigma2compassZ = 2*M_PI*2*M_PI;
QMC5883L_ST compass(COMPASS_X_AXIS_POSITION, LOCAL_DECLINATION, LOCAL_INCLINATION, COMPASS_SD_X, COMPASS_SD_Y, COMPASS_SD_Z);
///QMC5883L_ST compass(COMPASS_X_AXIS_POSITION, LOCAL_DECLINATION, 0, COMPASS_SD_X, COMPASS_SD_Y, COMPASS_SD_Z);

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
float heading, horizontalHeadingAngle;

float headingXZ, headingZY;
// float headingXZ2, headingZY2;

float rollXcompass = 0.0F, pitchYcompass = 0.0F, yawZcompass = 0.0F;
float rollXcompassInit = 0.0F, pitchYcompassInit = 0.0F, yawZcompassInit = 0.0F;
float rollXCompassOffset, pitchYCompassOffset;


// MPU6050 accelerometer Gyroscope -----------------------
// IMU Data
// Accelerometers: raw measurements values
double accX, accY, accZ;
// Gyroscopes: raw angle rates
double gyroX, gyroY, gyroZ;
// accelerometer gyroscope variances of roll X angle, pitch Y angle and yaw Z angle 
double sigma2accX , sigma2accY, sigma2accZ;
double sigma2gyroX, sigma2gyroY, sigma2gyroZ;
int16_t tempRaw;
// rollAccX: angle around X axis.
// pitchAccY: angle around Y axis.
// yawAccZ: angle around Z axis.
double rollAccX = 0.0F, pitchAccY = 0.0F, yawAccZ = 0.0F;
float yawAccZOffset;
double rollAccXInit, pitchAccYInit, yawAccZInit;

double gyroXrate = 0.0F, gyroYrate = 0.0F, gyroZrate = 0.0F;
//float yawGyroZOffset;
//double rollGyroXInit, pitchGyroYInit, yawGyroZInit;
#if MEGA2560_TRACE_TEST
  // Angle calculated using the gyro only
  double gyroXangle, gyroYangle, gyroZangle;
#endif
// Create the Kalman instance
Kalman_AGC kalmanX, kalmanY, kalmanZ;

// Create covariance matrices: H_k, Q_k
float H_diagonalX[3] = {M_PI*M_PI, GYRO_SD_X*GYRO_SD_X, M_PI*M_PI};
float H_diagonalY[3] = {M_PI*M_PI, GYRO_SD_Y*GYRO_SD_Y, M_PI*M_PI};
float H_diagonalZ[3] = {M_PI*M_PI, GYRO_SD_Z*GYRO_SD_Z, M_PI*M_PI};
float Q_diagonalX[2] = {ANGLE_SD_X*ANGLE_SD_X, ANGLE_RATE_SD_X*ANGLE_RATE_SD_X};
float Q_diagonalY[2] = {ANGLE_SD_Y*ANGLE_SD_Y, ANGLE_RATE_SD_Y*ANGLE_RATE_SD_Y};
float Q_diagonalZ[2] = {ANGLE_SD_Z*ANGLE_SD_Z, ANGLE_RATE_SD_Z*ANGLE_RATE_SD_Z};

// Define Kalman filtered state vector: [ANGLE, ANGLE_RATE]
float auxKalmanState[2];
//float auxKalmanState;

// Calculated angle using a Kalman filter
double kalAngleX, kalAngleY, kalAngleZ; 
// Calculated angle rate (velocity) using a Kalman filter
double kalVelocityX, kalVelocityY, kalVelocityZ; 

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
double satAzimut, satElevation;
#if MEGA2560_TRACE_INFO
  char asciiDateTime[20] ;
#endif
Satellite sat = Satellite("OSCAR 7 (AO-7)          ",
"1 07530U 74089B   20112.89879479 -.00000047  00000-0 -97882-5 0  9992",
"2 07530 101.7949  82.9674 0012534 112.0012 304.6453 12.53643112 78903");
DateTime  myDateTime ;

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
    // Create ordered list
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

  // MPU6050 (accelerometer gyroscope) configuration
  i2cDataMPU6050[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cDataMPU6050[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cDataMPU6050[2] = GYRO_FULL_SCALE_RANGE_0250; // Set Gyro Full Scale Range to ±250deg/s
  i2cDataMPU6050[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (!i2cDevice.writeBytes(IMU_MPU6050_ADDRS, 0x19, 4,i2cDataMPU6050)); // Write to all four registers at once

  while (!i2cDevice.writeByte(IMU_MPU6050_ADDRS,0x6B, 0x01)); // PLL with X axis gyroscope reference and disable sleep mode

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
  rollAccXInit = atan2(accZ, accY) + M_PI_2;
  rollAccXInit = positiveAngle(rollAccXInit);
  pitchAccYInit = atan2(accX,accZ) - M_PI;
  pitchAccYInit = positiveAngle(pitchAccYInit);
  yawAccZInit = atan2(accY, accX);
  yawAccZInit = positiveAngle(yawAccZInit);


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

  // We can get initial angles after compass.init()
  while (!compass.ready()){}
  // Set starting angle after initialize compass
  compass.readRotatedAngles(rollXcompass, pitchYcompass, yawZcompass);
  // Transform into positive angles
  rollXcompass = positiveAngle(rollXcompass);
  pitchYcompass = positiveAngle(pitchYcompass);
  yawZcompass = positiveAngle(yawZcompass);
  // Save starting offset angles
  rollXcompassInit = rollXcompass;
  pitchYcompassInit = pitchYcompass;
  yawZcompassInit = yawZcompass;
  // Save initial offsets
  rollXCompassOffset = (float) (rollAccXInit - rollXcompassInit);
  rollXCompassOffset = getPositiveAngle(rollXCompassOffset);
  pitchYCompassOffset = (float) (pitchAccYInit - pitchYcompassInit);
  pitchYCompassOffset = getPositiveAngle(pitchYCompassOffset);

  yawAccZOffset = (yawZcompassInit - yawAccZInit);
  yawAccZOffset = getPositiveAngle(yawAccZOffset);

  //yawGyroZOffset = (yawZcompassInit - yawGyroZInit);
  //yawGyroZOffset = getPositiveAngle(yawGyroZOffset);

  #if MEGA2560_TRACE_TEST
    debugMessage = String(rollXcompassInit*RAD_TO_DEG) + String("\t") + String(pitchYcompassInit*RAD_TO_DEG) + String("\t") + String(yawZcompassInit*RAD_TO_DEG);
    float locAngX, locAngY, locAngZ;
    compass.getLocalAngles(locAngX, locAngY, locAngZ);
    debugMessage += String("\t") + String(locAngX*RAD_TO_DEG) + String("\t") + String(locAngY*RAD_TO_DEG) + String("\t") + String(locAngZ*RAD_TO_DEG);
    debugMessage += String("\t") + String(rollAccXInit*RAD_TO_DEG) + String("\t") + String(pitchAccYInit*RAD_TO_DEG) + String("\t") + String(yawAccZInit*RAD_TO_DEG);
    debugMessage += String("\t") + String(rollXCompassOffset*RAD_TO_DEG) + String("\t") + String(pitchYCompassOffset*RAD_TO_DEG);
    serialMsgDBG.printDebug(STDebug::TYPE_TRACE_INFO, STDebug::ORIGIN_MEGA2560, debugMessage);
  #endif
  
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

  // =====================================================
  // Kalman filters initialization variables
  // a_1 = {ANGLE, ANGLE_RATE}
  float a1X[2] = {rollAccX,0.0F};
  float a1Y[2] = {pitchAccY,0.0F};
  float a1Z[2] = {yawZcompass,0.0F};
  // P_1 {Variance(angle), Variance(AngleRate)}
  float P1_diagonalX[2] = {M_PI*M_PI, 2.0F*M_PI*2.0F*M_PI};
  float P1_diagonalY[2] = {M_PI*M_PI, 2.0F*M_PI*2.0F*M_PI};
  float P1_diagonalZ[2] = {M_PI*M_PI, 2.0F*M_PI*2.0F*M_PI};

  // Calculate Accelerometer variance
  getAccVarAngles();
  // Gyroscope variance is constant: sigma2gyroX, sigma2gyroY, sigma2gyroZ
  getGyroVariance();
  // Calculate Compass variance
  compass.getVarianceAngle(sigma2compassX, sigma2compassY, sigma2compassZ);

  // H: observation error density covariance matrices
  H_diagonalX[Kalman_AGC::ACCELER] = sigma2accX;
  H_diagonalX[Kalman_AGC::GYROSPE] = sigma2gyroX;
  H_diagonalX[Kalman_AGC::COMPASS] = sigma2compassX;
  //
  H_diagonalY[Kalman_AGC::ACCELER] = sigma2accY;
  H_diagonalY[Kalman_AGC::GYROSPE] = sigma2gyroY;
  H_diagonalY[Kalman_AGC::COMPASS] = sigma2compassY;
  //
  H_diagonalZ[Kalman_AGC::ACCELER] = sigma2accZ;
  H_diagonalZ[Kalman_AGC::GYROSPE] = sigma2gyroZ;
  H_diagonalZ[Kalman_AGC::COMPASS] = sigma2compassZ;
  //
  // Q: state error density covariance matrices
  Q_diagonalX[Kalman_AGC::ANGLE] = ANGLE_SD_X*ANGLE_SD_X;
  Q_diagonalX[Kalman_AGC::ANGLE_RATE] = ANGLE_RATE_SD_X*ANGLE_RATE_SD_X;
  //
  Q_diagonalY[Kalman_AGC::ANGLE] = ANGLE_SD_Y*ANGLE_SD_Y;
  Q_diagonalY[Kalman_AGC::ANGLE_RATE] = ANGLE_RATE_SD_Y*ANGLE_RATE_SD_Y;
  //
  Q_diagonalZ[Kalman_AGC::ANGLE] = ANGLE_SD_Z*ANGLE_SD_Z;
  Q_diagonalZ[Kalman_AGC::ANGLE_RATE] = ANGLE_RATE_SD_Z*ANGLE_RATE_SD_Z;
  
  // pass pointer to the array as an argument.
  kalmanX.initialize(H_diagonalX, Q_diagonalX, a1X, P1_diagonalX);
  kalmanY.initialize(H_diagonalY, Q_diagonalY, a1Y, P1_diagonalY);
  kalmanZ.initialize(H_diagonalZ, Q_diagonalZ, a1Z, P1_diagonalZ);
  // =====================================================


  // Show Message strMsgFinishedSetup
  TFTlcd.Display_Message(0X18,3,(unsigned char *)strMsgFinishedSetup);
} // End of setup()


void loop() {
  // Change status periodically ==========================
  // TO-DO
  //    Change askSatelliteNames and askOneSatelliteTLE from LCD touch screen action

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
    observerLat = (double) flat;
    observerLong = (double) flon;
    observerAlt = (double) gps.f_altitude();
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

    // Add offset to these angles
    rollXcompass += rollXCompassOffset;
    pitchYcompass += pitchYCompassOffset;
    
    // Transform into positive angles
    rollXcompass = positiveAngle(rollXcompass);
    pitchYcompass = positiveAngle(pitchYcompass);
    yawZcompass = positiveAngle(yawZcompass);
    /// heading = compass.readHeading();
    heading = yawZcompass;
    
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
  }

  // Calculate satellite attitude smart delay ------------
  if ((update_lcd == 0) && 
    (deviceToComunicateWith.current() == CircularArray::GPS_POSITION) &&
    (millis() - lastTimeSatAttitude >= SAT_ATTITUDE_TIME_PERIOD))
  {
    lastTimeSatAttitude = millis();
    getSatelliteAttitude();     
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
  /// serialEvent3();
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
/// void serialEvent3() 
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
  serialMsgESP.readSerialMsg();
  if (serialMsgESP.newCommandRead())
  {
    String tmpCommandInfo = serialMsgESP.commandInfoRead();
    switch (serialMsgESP.commandRead())
    {
      case STSerialMsg::PUT_SAT_NAMES:
        // The ESP8266 has PUT_SAT_NAMES. So let's read them
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
  // Number of decimals
  const byte N_DEC = 8;
  #if MEGA2560_TRACE_INFO
    float vk[3];
  
    debugMessage = String(timer) + String("\t") + String(dt);
    debugMessage += String("\t") + String(rollAccX, N_DEC) + String("\t") + String(pitchAccY, N_DEC) + String("\t") + String(yawAccZ, N_DEC);
    #if MEGA2560_TRACE_TEST
    + String("\t")  + String(gyroXangle) + String("\t") + String(gyroYangle)
    #endif
    debugMessage += String("\t")  + String(gyroXrate, N_DEC) + String("\t") + String(gyroYrate, N_DEC) + String("\t") + String(gyroZrate, N_DEC);
    debugMessage += String("\t")  + String(rollXcompass, N_DEC)+ String("\t")  + String(pitchYcompass, N_DEC) + String("\t") + String(yawZcompass, N_DEC);
    debugMessage += String("\t") + String(kalAngleX, N_DEC) + String("\t") + String(kalAngleY, N_DEC) + String("\t") + String(kalAngleZ, N_DEC);
    debugMessage += String("\t") + String(kalVelocityX, N_DEC) + String("\t") + String(kalVelocityY, N_DEC) + String("\t") + String(kalVelocityZ, N_DEC);

    kalmanX.innovations(vk);
    debugMessage += String("\t") + String(vk[kalmanX.ACCELER], N_DEC) + String("\t") + String(vk[kalmanX.GYROSPE], N_DEC) + String("\t") + String(vk[kalmanX.COMPASS], N_DEC);
    kalmanY.innovations(vk);
    debugMessage += String("\t") + String(vk[kalmanY.ACCELER], N_DEC) + String("\t") + String(vk[kalmanY.GYROSPE], N_DEC) + String("\t") + String(vk[kalmanY.COMPASS], N_DEC);
    kalmanZ.innovations(vk);
    debugMessage += String("\t") + String(vk[kalmanZ.ACCELER], N_DEC) + String("\t") + String(vk[kalmanZ.GYROSPE], N_DEC) + String("\t") + String(vk[kalmanZ.COMPASS], N_DEC);

    debugMessage += String("\t") + String(sigma2accX, N_DEC) + String("\t") + String(sigma2accY, N_DEC) + String("\t") + String(sigma2accZ, N_DEC);
    debugMessage += String("\t") + String(sigma2gyroX, N_DEC) + String("\t") + String(sigma2gyroY, N_DEC) + String("\t") + String(sigma2gyroZ, N_DEC);
    debugMessage += String("\t") + String(sigma2compassX, N_DEC) + String("\t") + String(sigma2compassY, N_DEC) + String("\t") + String(sigma2compassZ, N_DEC);
    debugMessage += String("\t") + String(heading, N_DEC);
    debugMessage += String("\t") + String(horizontalHeadingAngle, N_DEC);

    // debugMessage += String("\t") + String(accX) + String("\t") +  String(accY) + String("\t") +  String(accZ);
    debugMessage += String("\t") + String(compass.rawValueX()) + String("\t") +  String(compass.rawValueY()) + String("\t") +  String(compass.rawValueZ());
    debugMessage += String("\t") + String(headingXZ, N_DEC) + String("\t") +  String(headingZY, N_DEC);


    // // Compass Variance
    // debugMessage = String(sigma2compassX, N_DEC) + String("\t") + String(sigma2compassY, N_DEC) + String("\t") + String(sigma2compassZ, N_DEC); 

    // // GPS receiving
    // debugMessage = String(uGpsReceiving) 
    // + String("\t") + String(year) + String("\t") + String(month) + String("\t") + String(day)
    // + String("\t") + String(hour) + String("\t") + String(minute) + String("\t") + String(second);

    // // Kalman and sd Acc
    // debugMessage = String(kalAngleX*RAD_TO_DEG) + String("\t") + String(kalAngleY*RAD_TO_DEG) + String("\t") + String(kalAngleZ*RAD_TO_DEG);
    // //debugMessage += String("\t") + String(sigma2accX) + String("\t") + String(sigma2accY) + String("\t") + String(sigma2accZ);
    // debugMessage += String("\t") + String(rollAccX*RAD_TO_DEG) + String("\t") + String(pitchAccY*RAD_TO_DEG) + String("\t") + String(yawAccZ*RAD_TO_DEG);
    // // Raw compass
    // int16_t compassX, compassY, compassZ;
    // compass.m_readRaw(&compassX, &compassY, &compassZ,false);
    // debugMessage = String(compassX) + String("\t") + String(compassY) + String("\t") + String(compassZ);

    // Raw info acc, gyro, compass. Measures with device in stable position
    // int16_t compassX, compassY, compassZ;
    // compass.m_readRaw(&compassX, &compassY, &compassZ,false);
    // debugMessage = String(accX) + String("\t") +  String(accY) + String("\t") +  String(accZ)
    //  + String("\t") +  String(gyroXrate, NUM_DECIMALS) + String("\t") + String(gyroYrate, NUM_DECIMALS) + String("\t") + String(gyroZrate, NUM_DECIMALS)
    //  + String("\t") + String(compassX) + String("\t") + String(compassY) + String("\t") + String(compassZ);

    // debugMessage = String(gyroXrate) + String("\t") + String(gyroYrate) + String("\t") + String(gyroZrate)
    //  + String("\t") + String(rollAccX) + String("\t") + String(pitchAccY) + String("\t") + String(yawAccZ);
    
    // debugMessage = String(rollAccX) + String("\t") + String(pitchAccY) + String("\t") + String(yawAccZ);
    // debugMessage +=  String("\t") + String(accX) + String("\t") + String(accY) + String("\t") + String(accZ);
    // debugMessage += String("\t") + String(gyroX) + String("\t") + String(gyroY) + String("\t") + String(gyroZ);
    #if MEGA2560_TRACE_TEST
      // debugMessage += String(gyroXangle) + String("\t") + String(gyroYangle) + String("\t") + String(gyroZangle);
    #endif
    
    // int16_t cx,cy,cz;
    // compass.m_readRaw(&cx,&cy,&cz,true);
    // debugMessage = String(cx) + String("\t") + String(cy) + String("\t") + String(cz)
    //  + String("\t") +   String(rollXcompass) + String("\t") + String(pitchYcompass) + String("\t") + String(yawZcompass);

    // debugMessage = String(rollAccX) + String("\t") + String(pitchAccY) + String("\t") + String(yawAccZ)
    //  + String("\t") +   String(yawZcompass) 
    //  + String("\t") + String(horizontalHeadingAngle)
    //  + String("\t") + String(kalAngleX) + String("\t") + String(kalAngleY) + String("\t") + String(kalAngleZ);

    // // horizontalHeadingAngle
    // debugMessage = String(heading*RAD_TO_DEG) + String("\t") + String(horizontalHeadingAngle*RAD_TO_DEG);
    // // Kalman
    // //debugMessage = String("\t") + String(kalAngleX*RAD_TO_DEG) + String("\t") + String(kalAngleY*RAD_TO_DEG) + String("\t") + String(kalAngleZ*RAD_TO_DEG);

    // 
    // int lcdAntennaAzimut = (int) ((M_PI_2 + M_PI - heading) * RAD_TO_DEG);
    // lcdAntennaAzimut = positiveAngleDegrees(lcdAntennaAzimut);
    // debugMessage += String("\t") + String(lcdAntennaAzimut);
    // 
    //  + String("\t") + String(kalAngleX) + String("\t") + String(kalAngleY) + String("\t") + String(kalAngleZ);
    // float locRoll, locPitch, locYaw;
    // compass.getLocalAngles(locRoll, locPitch, locYaw);
    // debugMessage += String("\t") + String(locRoll*RAD_TO_DEG) + String("\t") + String(locPitch*RAD_TO_DEG) + String("\t") + String(locYaw*RAD_TO_DEG);

    // Angles
    // debugMessage = String(timer) 
    // + String("\t") + String(rollAccX * RAD_TO_DEG) + String("\t") + String(pitchAccY * RAD_TO_DEG) + String("\t") + String(yawAccZ * RAD_TO_DEG);
    // + String("\t")  + String(gyroXrate, N_DEC) + String("\t") + String(gyroYrate, N_DEC) + String("\t") + String(gyroZrate, N_DEC)
    // debugMessage += String("\t")  + String(rollXcompass * RAD_TO_DEG)+ String("\t")  + String(pitchYcompass * RAD_TO_DEG) + String("\t") + String(yawZcompass * RAD_TO_DEG);
    // debugMessage += String("\t") + String(kalAngleX * RAD_TO_DEG) + String("\t") + String(kalAngleY * RAD_TO_DEG) + String("\t") + String(kalAngleZ * RAD_TO_DEG);
    // float locRoll, locPitch, locYaw;
    // compass.getLocalAngles(locRoll, locPitch, locYaw);
    // debugMessage += String("\t") + String(locRoll*RAD_TO_DEG) + String("\t") + String(locPitch*RAD_TO_DEG) + String("\t") + String(locYaw*RAD_TO_DEG);

    serialMsgDBG.printDebug(STDebug::TYPE_TRACE_INFO, STDebug::ORIGIN_MEGA2560, debugMessage);
  #endif
}

inline void getGPSInfo()
{
  // Every TIME_PERIOD_GPS_DT milliseconds we ask for an update
  if (millis() - lastTimeGetGpsDT >= TIME_PERIOD_GPS_DT)
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
    }
    lastTimeGetGpsDT = millis();
    // Shift left 1 position
    uGpsReceiving = uGpsReceiving << 1;
    // Add new state at first position
    uGpsReceiving |= (uint16_t) newdata;
    #if SIMULATE_GPS      
      if (!uGpsReceiving)
      {
        // increase second
        second += (byte) (TIME_PERIOD_GPS_DT/1000);
        int nextGap = second / 60;
        second = ((nextGap > 0)? (second % 60): second);
        // increase minute
        if ((nextGap > 0))
        {
          minute += nextGap;
          nextGap = minute / 60;
          minute = ((nextGap > 0)? (minute % 60): minute);
        }
        else
        {
          nextGap = 0;
        }
        // increase hour
        if ((nextGap > 0))
        {
          hour += nextGap;
          nextGap = hour / 24;
          hour = ((nextGap > 0)? (hour % 24): hour);
        }
        else
        {
          nextGap = 0;
        }
      }
    #endif

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

/**
 * @brief Get the Acceelerometer Variance  Angles: 
 * rollX, pitchY, yawZ in (radians)^2
 * 
 */
inline void getAccVarAngles()
{
  // Calculate standard deviation
  double sigmaAccEast =  sdAngleAtanDiff(accZ, accY, ACC_SD_Z, ACC_SD_Y);
  double sigmaAccNorth = sdAngleAtanDiff(accX, accZ, ACC_SD_X, ACC_SD_Z);
  double sigmaAccZ =     sdAngleAtanDiff(accY, accX, ACC_SD_Y, ACC_SD_X);

  // SD angle is between [0, PI]
  sigmaAccEast = min(abs(sigmaAccEast), 2*M_PI);
  sigmaAccNorth = min(abs(sigmaAccNorth), 2*M_PI);
  sigmaAccZ = min(abs(sigmaAccZ), 2*M_PI);

  // Calculate variance
  sigma2accX =  sigmaAccEast * sigmaAccEast;
  sigma2accY =  sigmaAccNorth * sigmaAccNorth;
  sigma2accZ =  sigmaAccZ * sigmaAccZ;
}


/**
 * @brief Get the Gyro Variance 
 * It is easy to calculate gyroscope variance because it is constant
 * 
 */
inline void getGyroVariance()
{
  sigma2gyroX = GYRO_SD_X * GYRO_SD_X;
  sigma2gyroY = GYRO_SD_Y * GYRO_SD_Y;
  sigma2gyroZ = GYRO_SD_Z * GYRO_SD_Z;
}


inline void calculateRotationAngles()
{
  // Calculate delta time in seconds
  dt = (double)(micros() - timer) / 1000000;
  timer = micros();

  transformAccelGyroInfo2Angles();

  // Calculate gyro X Y Z rates
  gyroXrate = gyroX / GYRO_SENSITIVITY_SCALE_FACTOR_0250_RADS; // Convert to rad/s
  gyroYrate = gyroY / GYRO_SENSITIVITY_SCALE_FACTOR_0250_RADS; // Convert to rad/s 
  gyroZrate = gyroZ / GYRO_SENSITIVITY_SCALE_FACTOR_0250_RADS; // Convert to rad/s 

  // Set values of H matrix: H_diagonalX[3] = {Variance(AccelerometerX), Variance(GyroscopeX), Variance(CompassX)}
  // 1. Calculate Accelerometer variance
  getAccVarAngles();
  // 2. Gyroscope variance is constant: sigma2gyroX, sigma2gyroY, sigma2gyroZ
  // 3. Calculate Compass variance
  compass.getVarianceAngle(sigma2compassX, sigma2compassY, sigma2compassZ);
  // 
  H_diagonalX[Kalman_AGC::ACCELER] = sigma2accX;
  H_diagonalX[Kalman_AGC::GYROSPE] = sigma2gyroX;
  H_diagonalX[Kalman_AGC::COMPASS] = sigma2compassX;

  H_diagonalY[Kalman_AGC::ACCELER] = sigma2accY;
  H_diagonalY[Kalman_AGC::GYROSPE] = sigma2gyroY;
  H_diagonalY[Kalman_AGC::COMPASS] = sigma2compassY;

  H_diagonalZ[Kalman_AGC::ACCELER] = sigma2accZ;
  H_diagonalZ[Kalman_AGC::GYROSPE] = sigma2gyroZ;
  H_diagonalZ[Kalman_AGC::COMPASS] = sigma2compassZ;

  // Calculate angles using Kalman filter
  kalmanX.filteredState(rollAccX, gyroXrate, rollXcompass, dt, H_diagonalX, auxKalmanState);
  kalAngleX = (double) auxKalmanState[kalmanX.ANGLE];
  kalVelocityX = (double) auxKalmanState[kalmanX.ANGLE_RATE];
  /// kalAngleX = auxKalmanState;
  kalmanY.filteredState(pitchAccY, gyroYrate, pitchYcompass, dt, H_diagonalY, auxKalmanState);
  kalAngleY = (double) auxKalmanState[kalmanY.ANGLE];
  kalVelocityY = (double) auxKalmanState[kalmanY.ANGLE_RATE];
  /// kalAngleY = auxKalmanState;
  kalmanZ.filteredState(yawAccZ, gyroZrate, yawZcompass, dt, H_diagonalZ, auxKalmanState);
  kalAngleZ = (double) auxKalmanState[kalmanZ.ANGLE];
  kalVelocityZ = (double) auxKalmanState[kalmanZ.ANGLE_RATE];
  /// kalAngleZ = auxKalmanState;


  #if MEGA2560_TRACE_TEST
    // Calculate gyro angle without any filter
    gyroXangle += gyroXrate * dt;
    gyroYangle += gyroYrate * dt;
    gyroZangle += gyroZrate * dt;
    //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    //gyroYangle += kalmanY.getRate() * dt;

    // Reset the gyro angle when it has drifted too much
    gyroXangle = (float)angle_pm180Radians(gyroXangle);
    gyroYangle = (float)angle_pm180Radians(gyroYangle);
    gyroZangle = (float)angle_pm180Radians(gyroZangle);
  #endif
}

inline void getSatelliteAttitude()
{
  // TO-DO
  //    Usar string proveniente del ESP8266 para crear la variable sat
  /// char *tle0, *tle1, *tle2;
  /// psSatelliteTLE[0].toCharArray(tle0, psSatelliteTLE[0].length());
  /// psSatelliteTLE[1].toCharArray(tle1, psSatelliteTLE[1].length());
  /// psSatelliteTLE[2].toCharArray(tle2, psSatelliteTLE[2].length());

  #if false // MEGA2560_TRACE_TEST
      debugMessage = String("psSatelliteTLE[0]=") + String(psSatelliteTLE[0]) + 
                    String("\tpsSatelliteTLE[1]=") + String(psSatelliteTLE[1]) + 
                    String("\tpsSatelliteTLE[2]=") + String(psSatelliteTLE[2]);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
  #endif

  sat = Satellite(psSatelliteTLE[0], psSatelliteTLE[1], psSatelliteTLE[2]);

  // Prediction time
  myDateTime.settime(year, (int)month, (int)day, (int)hour, (int)minute, (int)second);
  // Create prediction
  sat.predict(myDateTime);
  // satElevation and satAzimut from observer place
  sat.altaz(observationPlace, satElevation, satAzimut);
}

/**
 * @brief This function transforms accelerometer and 
 * gyroscope information into angles.
 * Every calculated angle is positive between [0,2*M_PI]
 * 
 */
inline void transformAccelGyroInfo2Angles()
{
  const float OFFSET_ROLL_X = -M_PI_2;
  const float OFFSET_PITCH_Y = M_PI;
  const float OFFSET_YAW_Z = 0.0F - yawAccZOffset ;
  
  rollAccX = atan2(accZ, accY) - OFFSET_ROLL_X;
  pitchAccY = atan2(accX,accZ) - OFFSET_PITCH_Y;
  yawAccZ = atan2(accY, accX) - OFFSET_YAW_Z;

  // atan2() returns angles between [-PI,PI] 
  // but we will use angles between [0,2*M_PI]
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
 * It has been inspired by Bingaman 2010 [pag.74], Grygorenko 2014 and Abyarjoo 2015
 */
inline void horizontalHeading()
{
  //// const float OFFSET_ROLL_X = -M_PI_2;
  //// const float OFFSET_PITCH_Y = M_PI;
  //// const float OFFSET_YAW_Z = 0.0F - yawAccZOffset ;
  //// double aux_rollAccX = rollAccX + OFFSET_ROLL_X;
  //// double aux_pitchAccY = atan2(accX,accZ) + OFFSET_PITCH_Y;
  //// double aux_yawAccZ = atan2(accY, accX) + OFFSET_YAW_Z;

  // Read compass raw values
  double mx = compass.rawValueX();
  double my = compass.rawValueY();
  double mz = compass.rawValueZ();

  // Euler rotations
  // Accelerometer angles
  double xh = mx * cos(pitchAccY) - mz * sin(pitchAccY);
  double yh = mx * sin(pitchAccY)*sin(rollAccX) + my * cos(rollAccX) + mz * sin(rollAccX)*cos(pitchAccY);
  double zh = mx * sin(pitchAccY)*cos(rollAccX) - my * sin(rollAccX) + mz * cos(pitchAccY)*cos(rollAccX);
  /// // Kalman angles
  /// double xh = mx * cos(kalAngleY) - mz * sin(kalAngleY);
  /// double yh = mx * sin(kalAngleY)*sin(kalAngleX) + my * cos(kalAngleX) + mz * sin(kalAngleX)*cos(kalAngleY);
  /// // double zh = mx * sin(kalAngleY)*cos(kalAngleX) - my * sin(kalAngleX) + mz * cos(kalAngleY)*cos(kalAngleX);

  // Calculate yaw angle. Local declination correction was added yet
  /// horizontalHeadingAngle = atan2(yh,xh);
  horizontalHeadingAngle = atan2(-xh,yh) + M_PI_2;
  // Allow only positive values
  horizontalHeadingAngle = positiveAngle(horizontalHeadingAngle);


  // Calculate Pitch projection
  headingXZ = atan2(xh,zh);
  // Allow only positive values
  headingXZ = positiveAngle(headingXZ);

  // Calculate Roll projection
  headingZY = atan2(zh,yh);
  // Allow only positive values
  headingZY = positiveAngle(headingZY);


  // (RotationMatrixX * RotationMatrixY)^(-1)
  xh= mx*cos(pitchAccY) + my*sin(rollAccX)*sin(pitchAccY) - mz*sin(pitchAccY)*cos(rollAccX);
  yh= my*cos(rollAccX) + mz*sin(rollAccX);
  zh= mx*sin(pitchAccY) - my*sin(rollAccX)*cos(pitchAccY) + mz*cos(rollAccX)*cos(pitchAccY);
  
  //// // Calculate Pitch projection
  //// headingXZ2 = atan2(xh,zh);
  //// // Allow only positive values
  //// headingXZ2 = positiveAngle(headingXZ2);
  //// 
  //// // Calculate Roll projection
  //// headingZY2 = atan2(zh,yh);
  //// // Allow only positive values
  //// headingZY2 = positiveAngle(headingZY2);

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

/**
 * @brief Update LCD touch screen
 * 
 */
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
    TFTlcd.SetNumberValue(page_id_lcd,MAIN_NUMBER_HH,(uint16_t)hour);
    TFTlcd.SetNumberValue(page_id_lcd,MAIN_NUMBER_MM,(uint16_t)minute);
    TFTlcd.SetNumberValue(page_id_lcd,MAIN_NUMBER_SS,(uint16_t)second);
    #if MEGA2560_TRACE_TEST
      debugMessage = String("newStrSatName=") + String((char *)newStrSatName);
      serialMsgDBG.printDebug(STDebug::TYPE_TRACE_TEST, STDebug::ORIGIN_MEGA2560, debugMessage);
    #endif
  }
  else if (page_id_lcd == ATTITUDE_PAGE)
  {
    // LCD local variables
    int16_t lcdSatelliteAzimut = (int16_t) satAzimut; // degrees

    int16_t lcdAntennaAzimut = (int16_t) ((horizontalHeadingAngle) * RAD_TO_DEG);
    lcdAntennaAzimut = positiveAngleDegrees(lcdAntennaAzimut);

    int16_t lcdAntennaElevation = (int16_t) ((kalAngleY) * RAD_TO_DEG);
    lcdAntennaElevation = positiveAngleDegrees(lcdAntennaElevation);
    
    int16_t lcdSatelliteElevation = (int16_t) satElevation; // degrees
    lcdSatelliteElevation = positiveAngleDegrees(lcdSatelliteElevation);


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
    // TFTlcd.SetValueInt(page_id_lcd,74,44,lcdSatelliteAzimut);
    TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_ELEVATION_SATELLITE,abs(lcdSatelliteElevation));
    // Paint sign
    const uint16_t ATT_SIGN_ELEVATION_SATELLITE_X = 18;
    const uint16_t ATT_SIGN_ELEVATION_SATELLITE_Y = 53;
    uint16_t thisSign = (lcdSatelliteElevation < 0) ? COLOR_WHITE : COLOR_BLUE_SYSTEM;
    TFTlcd.RectangleFill(ATT_SIGN_ELEVATION_SATELLITE_X,ATT_SIGN_ELEVATION_SATELLITE_Y,ATT_SIGN_WIDTH,ATT_SIGN_HEIGTH,thisSign);

    // Paint number
    // TFTlcd.SetValueInt(page_id_lcd,75,45,lcdAntennaElevation);
    TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_ELEVATION_ANTENNA,abs(lcdAntennaElevation));
    // Paint sign
    const uint16_t ATT_SIGN_ELEVATION_ANTENNA_X = 18;
    const uint16_t ATT_SIGN_ELEVATION_ANTENNA_Y = 90;
    thisSign = (lcdAntennaElevation < 0) ? COLOR_WHITE : COLOR_BLUE_SYSTEM;
    TFTlcd.RectangleFill(ATT_SIGN_ELEVATION_ANTENNA_X,ATT_SIGN_ELEVATION_ANTENNA_Y,ATT_SIGN_WIDTH,ATT_SIGN_HEIGTH,thisSign);

    // Paint number
    // TFTlcd.SetValueInt(page_id_lcd,76,46,lcdSatelliteElevation);
    TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_AZIMUT_SATELLITE,abs(lcdSatelliteAzimut));
    // Paint sign
    const uint16_t ATT_SIGN_AZIMUT_SATELLITE_X = 200;
    const uint16_t ATT_SIGN_AZIMUT_SATELLITE_Y = 53;
    thisSign = (lcdSatelliteAzimut < 0) ? COLOR_WHITE : COLOR_BLUE_SYSTEM;
    TFTlcd.RectangleFill(ATT_SIGN_AZIMUT_SATELLITE_X,ATT_SIGN_AZIMUT_SATELLITE_Y,ATT_SIGN_WIDTH,ATT_SIGN_HEIGTH,thisSign);

    // Paint number
    // Set numbers
    // TFTlcd.SetValueInt(page_id_lcd,77,47,lcdAntennaAzimut);
    TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_AZIMUT_ANTENNA,abs(lcdAntennaAzimut));
    // lcdAntennaAzimut is a positive value yet
    // TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_AZIMUT_ANTENNA,lcdAntennaAzimut);
    const uint16_t ATT_SIGN_AZIMUT_ANTENNA_X = 200;
    const uint16_t ATT_SIGN_AZIMUT_ANTENNA_Y = 90;
    thisSign = (lcdAntennaAzimut < 0) ? COLOR_WHITE : COLOR_BLUE_SYSTEM;
    TFTlcd.RectangleFill(ATT_SIGN_AZIMUT_ANTENNA_X,ATT_SIGN_AZIMUT_ANTENNA_Y,ATT_SIGN_WIDTH,ATT_SIGN_HEIGTH,thisSign);

    // Heading info
    ///int16_t lcdHeading = (int16_t) ((horizontalHeadingAngle + M_PI_2) * RAD_TO_DEG);
    int16_t lcdHeading = (int16_t) ((2 * M_PI - horizontalHeadingAngle) * RAD_TO_DEG);
    lcdHeading = positiveAngleDegrees(lcdHeading);
    TFTlcd.SetNumberValue(page_id_lcd,ATT_HEADING,lcdHeading);

    // Set figures -----------------------------
    // Set compass
    // Set sattellite ascension red line.
    // TFTlcd circle angles is left handed, but ascension is right handed from East position.
    // TFTlcd heading suplementary angle = 360 - heading
    // TFTlcd East = (360 - heading) + 90 = 90 - heading
    // TFTlcd circle angle for satellite = East - lcdSatelliteElevation
    // int lcdCircleGaugeSatAzimut = (90 - heading) - lcdSatelliteElevation;
    int16_t lcdCircleGaugeSatAzimut = (int16_t) (lcdSatelliteAzimut - lcdAntennaAzimut);
    lcdCircleGaugeSatAzimut = positiveAngleDegrees(lcdCircleGaugeSatAzimut);
    TFTlcd.SetCircleGaugeValue(page_id_lcd,ATT_COMPASS,(uint16_t) lcdCircleGaugeSatAzimut);


    // Azimut bar graph constants
    const uint16_t ATT_ELEVATION_BAR_GRAPH_WIDTH = 36;
    const uint16_t ATT_ELEVATION_BAR_GRAPH_HEIGHT = 238;
    // Set satelite azimut
    const uint16_t ATT_ELEVATION_SATELLITE_BAR_GRAPH_X = 9;
    const uint16_t ATT_ELEVATION_SATELLITE_BAR_GRAPH_Y = 156;
    // Set satellite elevation
    lcdSatelliteElevation = angle_90Degrees(lcdSatelliteElevation);

    // Escale lcdSatelliteElevation to BarGraph from 0 to 100
    lcdSatelliteElevation =(int16_t) round((lcdSatelliteElevation*10)/9); // * 100/90
    TFTlcd.SetBarGraph(ATT_ELEVATION_SATELLITE_BAR_GRAPH_X,
                      ATT_ELEVATION_SATELLITE_BAR_GRAPH_Y,
                      ATT_ELEVATION_BAR_GRAPH_WIDTH/2,
                      ATT_ELEVATION_BAR_GRAPH_HEIGHT,
                      lcdSatelliteElevation,COLOR_RED);
    // Set antenna azimut
    const uint16_t ATT_ELEVATION_ANTENNA_BAR_GRAPH_X = 86;
    const uint16_t ATT_ELEVATION_ANTENNA_BAR_GRAPH_Y = 156;
    int16_t lcdAntennaElevationBarGraph =  angle_pm180Degrees(lcdAntennaElevation);
    lcdAntennaElevationBarGraph = angle_90Degrees(lcdAntennaElevationBarGraph);

    // Escale lcdAntennaElevationBarGraph to BarGraph from 0 to 100
    lcdAntennaElevationBarGraph =(int16_t) round((lcdAntennaElevationBarGraph*10)/9); // * 100/90
    // Set smart color
    uint16_t antennaBargraphColor = COLOR_BLUE;
    if (((lcdSatelliteElevation - 1) <= lcdAntennaElevationBarGraph) && (lcdAntennaElevationBarGraph <= (lcdSatelliteElevation + 1)))
      antennaBargraphColor = COLOR_GREEN;
    TFTlcd.SetBarGraph(ATT_ELEVATION_ANTENNA_BAR_GRAPH_X,
                      ATT_ELEVATION_ANTENNA_BAR_GRAPH_Y,
                      ATT_ELEVATION_BAR_GRAPH_WIDTH,
                      ATT_ELEVATION_BAR_GRAPH_HEIGHT,
                      lcdAntennaElevationBarGraph,antennaBargraphColor);
    
    // Set satellite name
    strcpy((char *)strSatName, (char *) psSatelliteTLE[0]);    
    TFTlcd.SetLableValue(page_id_lcd,ATT_LABEL_SAT_NAME,strSatName);

    // Get GPS time and write variables
    TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_HH,(uint16_t)hour);
    TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_MM,(uint16_t)minute);
    TFTlcd.SetNumberValue(page_id_lcd,ATT_NUMBER_SS,(uint16_t)second);

    // Set GPS signal ON-OFF
    const uint16_t ATT_GPS_ON_X = 153;
    const uint16_t ATT_GPS_ON_Y = 388;
    const uint16_t ATT_GPS_WIDTH = 17;
    const uint16_t ATT_GPS_HEIGTH = 17;
    uint16_t gpsRectangleColor = (uGpsReceiving) ? COLOR_DARKGREEN : COLOR_GREY;
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

float getPositiveAngle(float angle)
{
  while (angle < 0)
  {
    angle += (float) 2*M_PI;
  }
  while (angle > 2*M_PI)
  {
    angle -= (float) 2*M_PI;
  }
  return angle;
}

