// SatTrack.h
#ifndef SAT_TRACK_H
#define SAT_TRACK_H

// ----------------------------------------------------
// Forward declarations -------------------------------
//
// It is neccesary to declare serialEvent3() because it will be used at setup()
// serialEvent3() is automatically called at loop() when data is available
//// void serialEvent3();
void mySerialEvent3(); 

inline void getGPSInfo();
inline void getSatelliteAttitude();
inline void getDeviceAngles();

//// void ReadTLEFile()
// ----------------------------------------------------

#endif // SAT_TRACK_H