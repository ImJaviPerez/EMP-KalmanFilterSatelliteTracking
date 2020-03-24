// SatTrack.h
#ifndef SAT_TRACK_H
#define SAT_TRACK_H

#define THIS_PROJECT_HAS_ACCEL_GYRO
#define THIS_PROJECT_HAS_COMPASS


// Constants for smart delay sampling ====================
// Period of time to get info from any device
const unsigned long TIME_PERIOD_GPS_DT = 3000; // milliseconds
const unsigned long GPS_TIME_PERIOD = 50; // milliseconds
const unsigned long COMPASS_TIME_PERIOD = 35; // milliseconds
const unsigned long ANGLES_TIME_PERIOD = 30; // milliseconds
const unsigned long SAT_ATTITUDE_TIME_PERIOD = 40; // milliseconds
const unsigned long SAT_LCD_TIME_PERIOD = 200; // milliseconds
const unsigned long COMPASS_SAMPLING_RATE = 50; // milliseconds


// ----------------------------------------------------
// Forward declarations -------------------------------
//
// It is neccesary to declare serialEvent3() because it will be used at setup()
// serialEvent3() is automatically called at loop() when data is available
//// void serialEvent3();
void mySerialEvent3(); 

inline void showTraceInfo();

inline void getGPSInfo();
inline void getSatelliteAttitude();
inline void transformAccelGyroInfo2Angles();
inline void getAccGyroDeviceInfo();
inline void calculateRotationAngles();
inline void horizontalHeading();
// positiveAngle(angleValue): Returns a Positive Angle  between [0º, 360º]
#define positiveAngle(angleValue) ((angleValue)<(0)?(angleValue += 360):((angleValue)>(360)?(angleValue -= 360):(angleValue)))
/*
  if (angleValue < 0) 
    angleValue += 360;
  else if (angleValue > 360) 
    angleValue -= 360;
  
  return angleValue;
*/
// angle_pm180(angleValue): Returns an angle between [-180º, 180º]
#define angle_pm180(angleValue) ((angleValue)<(-180)?(angleValue += 360):((angleValue)>(180)?(angleValue -= 360):(angleValue)))


// LCD Forward declarations
void LcdIICInterrupt();
void ProcessMessage( PCTRL_MSG msg, uint16_t dataSize );
void UpdateUI();
void NotifyTouchButton(uint8_t page_id, uint8_t control_id, uint8_t  state,uint8_t type,uint8_t value);
void NotifyTouchCheckbox(uint8_t page_id, uint8_t control_id, uint8_t  state,uint8_t type,uint8_t value);
void NotifyTouchSlider(uint8_t page_id, uint8_t control_id, uint8_t  state,uint8_t type,uint8_t value);
void NotifyTouchEdit(uint8_t page_id, uint8_t control_id, uint8_t  state,uint8_t type,uint8_t value);
void NotifyGetEdit(PEDIT_MSG msg);
void NotifyGetTouchEdit(PEDIT_MSG msg);
void NotifyGetPage(uint8_t page_id,uint8_t status);
void NotifyGetCheckbox(uint8_t page_id, uint8_t control_id, uint8_t  state,uint8_t type,uint8_t value);
void NotifyGetSlider(uint8_t page_id, uint8_t control_id, uint8_t  state,uint8_t type,uint8_t value);


//// void ReadTLEFile()
// ----------------------------------------------------

#endif // SAT_TRACK_H