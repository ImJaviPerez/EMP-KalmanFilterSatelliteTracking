/**
 * @brief This library is based on QMC5883L library (https://github.com/dthain/QMC5883L)
 * wroten by Douglas Thain (https://www3.nd.edu/~dthain/).
 * 
 * I have added 
 * - two constructors
 * - a destructor and 
 * - a new member variable m_xAngleOffset because QMC5883L 
 * hardware's device X axis can be placed rotated. It helps
 * selecting the offset rotated angle of device X axis.
 * - Change member function
 *      int m_readRaw( int16_t *x, int16_t *y, int16_t *z, int16_t *t );
 *    by
 *      int m_readRaw( int16_t *x, int16_t *y, int16_t *z);
 * because variable t is not used
 * - #define QMC5883L_ST_TRACE in order to trace this class
 * 
 */
#ifndef QMC5883L_H
#define QMC5883L_H

typedef enum 
{ 
    COMPASS_X_AXIS_FORWARD = 0, 
    COMPASS_X_AXIS_LEFT = 90,
    COMPASS_X_AXIS_BACKWARD = 180,
    COMPASS_X_AXIS_RIGHT = 270
} compassXposition;


//namespace SatTrack
//{
class QMC5883L_ST {
public:


  QMC5883L_ST();
  QMC5883L_ST(compassXposition compassXposition, float localDeclination, float localInclination, float sdX, float sdY, float sdZ);
  ~QMC5883L_ST();
  void init();
  void reset();
  int  ready();
  void reconfig();
  
  float readHeading();

  bool readRawRotated( int16_t &x, int16_t &y, int16_t &z);
  bool readRawNormalizedRotated(double &rnx, double &rny, double &rnz);


  bool readAngles(float &rollX,float &pitchY,float &yawZ);
  bool readRotatedAngles(float &rollX,float &pitchY,float &yawZ);  
  int16_t rawValueX();
  int16_t rawValueY();
  int16_t rawValueZ();
  float rollX();
  float pitchY();
  float yawZ();

  void resetCalibration();

  void setSamplingRate( int rate );
  void setRange( int range );
  void setOversampling( int ovl );

  compassXposition offsetAngle();

  void getVarianceAngle(float &varRollX, float &varPitchY, float &varYawZ);

  void getLocalAngles(float &localRollX, float &localPitchY, float &localYawZ);
  
private:
  // Compass raw values
  int16_t m_compassX, m_compassY, m_compassZ;

  // Compass raw maximum and minimum values
  int16_t xhigh, xlow;
  int16_t yhigh, ylow;
  int16_t zhigh, zlow;

  uint8_t addr;
  uint8_t mode;
  uint8_t rate;
  uint8_t range;
  uint8_t oversampling;

  float m_localDeclination;
  float m_localInclination;
  float m_localRollX, m_localPitchY, m_localYawZ;
  
  // standard deviation
  float m_sdCompassX, m_sdCompassY,  m_sdCompassZ;
  // variance
  float m_varAngleX, m_varAngleY,  m_varAngleZ;
  // QMC5883L hardware's device X axis can be placed rotated.
  // Select the offset rotated angle of device X axis
  compassXposition m_xAngleOffset;

  // magnetometer rotation angles
  float m_rollX, m_pitchY, m_yawZ;

  int m_readRaw( int16_t *x, int16_t *y, int16_t *z, bool scaled);
  bool m_readRawNormalized(double &nx, double &ny, double &nz, bool rotated);
  bool m_readAngles(float &rollX,float &pitchY,float &yawZ, bool rotated);
  void m_scaleValues(int16_t &x, int16_t &y, int16_t &z);
  //// void m_rotateRawValues(int16_t &rx, int16_t &ry, int16_t &rz);
  inline void m_rotateRawValues();

  // Calculates variance angles
  inline void m_calculateVarAngles();

};
//}
#endif
