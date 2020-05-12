#include <Wire.h>
#include <math.h>
#include "QMC5883L_ST.h"

// #define QMC5883L_ST_TRACE
#ifdef QMC5883L_ST_TRACE
  #include <Arduino.h>
#endif
#include <Arduino.h> // To use min()

//using namespace SatTrack;

/*
 * QMC5883L
 * http://wiki.epalsite.com/images/7/72/QMC5883L-Datasheet-1.0.pdf
 */

/* The default I2C address of this chip */
#define QMC5883L_ADDR 0x0D

/* Register numbers */
#define QMC5883L_X_LSB 0
#define QMC5883L_X_MSB 1
#define QMC5883L_Y_LSB 2
#define QMC5883L_Y_MSB 3
#define QMC5883L_Z_LSB 4
#define QMC5883L_Z_MSB 5
#define QMC5883L_STATUS 6
#define QMC5883L_TEMP_LSB 7
#define QMC5883L_TEMP_MSB 8
#define QMC5883L_CONFIG 9
#define QMC5883L_CONFIG2 10
#define QMC5883L_RESET 11
#define QMC5883L_RESERVED 12
#define QMC5883L_CHIP_ID 13

/* Bit values for the STATUS register */
#define QMC5883L_STATUS_DRDY 1
#define QMC5883L_STATUS_OVL 2
#define QMC5883L_STATUS_DOR 4

/* Oversampling values for the CONFIG register */
#define QMC5883L_CONFIG_OS512 0b00000000
#define QMC5883L_CONFIG_OS256 0b01000000
#define QMC5883L_CONFIG_OS128 0b10000000
#define QMC5883L_CONFIG_OS64  0b11000000

/* Range values for the CONFIG register */
#define QMC5883L_CONFIG_2GAUSS 0b00000000
#define QMC5883L_CONFIG_8GAUSS 0b00010000

/* Rate values for the CONFIG register */
#define QMC5883L_CONFIG_10HZ   0b00000000
#define QMC5883L_CONFIG_50HZ   0b00000100
#define QMC5883L_CONFIG_100HZ  0b00001000
#define QMC5883L_CONFIG_200HZ  0b00001100

/* Mode values for the CONFIG register */
#define QMC5883L_CONFIG_STANDBY 0b00000000
#define QMC5883L_CONFIG_CONT    0b00000001

/* Apparently M_PI isn't available in all environments. */
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

#define sdAngleAtanDiff1(y,x,sdY,sdX) (atan2(2.0*(y*sdX + x*sdY),(x*x + y*y - sdX*sdX - sdY*sdY)))
#define sdAngleAtanDiff(y,x,sdY,sdX) (atan2((y + sdY),(x - sdX)) - atan2((y - sdY),(x + sdX)))

/**
 * @brief Construct a new qmc5883l st::qmc5883l st object
 * Y axis points North direction
 * X axis points East direction
 * 
 */
QMC5883L_ST::QMC5883L_ST()
{ 
  // These constants values are between (-32768, 32767).
  // There are neither degrees nor radians
  const float m_LOCAL_DECLINATION = -0.25 * DEG_TO_RAD; //0º15' = 0.25
  const float m_LOCAL_INCLINATION = 58.4764 * DEG_TO_RAD; //58º28'35''
  const float m_SD_COMPASS_X = 11.369459546F;
  const float m_SD_COMPASS_Y = 12.118513032F;
  const float m_SD_COMPASS_Z = 13.846676811F;
  
  QMC5883L_ST(COMPASS_X_AXIS_FORWARD, m_LOCAL_DECLINATION, m_LOCAL_INCLINATION, m_SD_COMPASS_X, m_SD_COMPASS_Y, m_SD_COMPASS_Z);
}


/**
 * @brief Construct a new qmc5883l st::qmc5883l st object
 * Y axis points North direction
 * X axis points East direction
 * 
 * @param compassXposition 
 * @param localDeclination in radians
 * @param localInclination in radians
 * @param sdX 
 * @param sdY 
 * @param sdZ 
 */
QMC5883L_ST::QMC5883L_ST(compassXposition compassXposition, float localDeclination, float localInclination, float sdX, float sdY, float sdZ)
{
  m_xAngleOffset = compassXposition;

  m_localDeclination = localDeclination;
  m_localInclination = localInclination;
  //Bilbao: m_localRollX = -58.48º, m_localPitchY = -179.85º, m_localYawZ = 90.25
  m_localRollX = atan2(sin(m_localInclination), (cos(m_localInclination)*cos(m_localDeclination)));
  m_localPitchY = atan2((cos(m_localInclination)*sin(m_localDeclination)), sin(m_localInclination));
  m_localYawZ = M_PI_2 - m_localDeclination;

  m_sdCompassX = sdX;
  m_sdCompassY = sdY;
  m_sdCompassZ = sdZ;
}

QMC5883L_ST::~QMC5883L_ST()
{
}

static void write_register( int addr, int reg, int value )
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

static int read_register( int addr, int reg, int count )
{
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(addr,count);
  int n = Wire.available();
  #ifdef QMC5883L_ST_TRACE
    if(n!=count)
    {
      Serial.print("QMC5883L_ST::read_register().n = ");
      Serial.print(n);
      Serial.print(".count = ");
      Serial.println(count);
    }
  #endif
  if(n!=count) return 0;

  return n;
}

void QMC5883L_ST::reconfig()
{
  write_register(addr,QMC5883L_CONFIG,oversampling|range|rate|mode);  
}

void QMC5883L_ST::reset()
{
  write_register(addr,QMC5883L_RESET,0x01);
  reconfig();
}

void QMC5883L_ST::setOversampling( int x )
{
  switch(x) {
    case 512:
      oversampling = QMC5883L_CONFIG_OS512;
      break;
    case 256:
      oversampling = QMC5883L_CONFIG_OS256;
      break;
    case 128:
      oversampling = QMC5883L_CONFIG_OS128;
      break;
    case 64:
      oversampling = QMC5883L_CONFIG_OS64;
      break;
  } 
  reconfig();
}

void QMC5883L_ST::setRange( int x )
{
  switch(x) {
    case 2:
      range = QMC5883L_CONFIG_2GAUSS;
      break;
    case 8:
      range = QMC5883L_CONFIG_8GAUSS;
      break;
  }
  reconfig();
}

void QMC5883L_ST::setSamplingRate( int x )
{
  switch(x) {
    case 10:
      rate = QMC5883L_CONFIG_10HZ;
      break;
    case 50:
      rate = QMC5883L_CONFIG_50HZ;
      break;
    case 100:
      rate = QMC5883L_CONFIG_100HZ;
      break;
    case 200:
      rate = QMC5883L_CONFIG_200HZ;
      break;
  }
  reconfig();
}

void QMC5883L_ST::init() {
  /* This assumes the wire library has been initialized. */
  addr = QMC5883L_ADDR;
  // imjaviperez
  // oversampling = QMC5883L_CONFIG_OS512;
  oversampling = QMC5883L_CONFIG_OS128;
  // oversampling = QMC5883L_CONFIG_OS512;  
  //range = QMC5883L_CONFIG_2GAUSS;
  range = QMC5883L_CONFIG_8GAUSS;
  
  rate = QMC5883L_CONFIG_50HZ;
  mode = QMC5883L_CONFIG_CONT;
  reset();
}

int QMC5883L_ST::ready()
{
  if(!read_register(addr,QMC5883L_STATUS,1)) return 0;
  uint8_t status = Wire.read();
  #ifdef QMC5883L_ST_TRACE
      Serial.print("QMC5883L_ST::ready()");
      Serial.print(". status = ");
      Serial.print(status, BIN);
      // Serial.print(". QMC5883L_STATUS_DRDY = ");
      // Serial.print(QMC5883L_STATUS_DRDY, BIN);
      Serial.print(". ready() = ");
      Serial.println(status & QMC5883L_STATUS_DRDY, BIN);
  #endif

  return status & QMC5883L_STATUS_DRDY; 
}

/**
 * @brief 
 * 
 * @param x 
 * @param y 
 * @param z 
 * @return int 
 */
int QMC5883L_ST::m_readRaw( int16_t *x, int16_t *y, int16_t *z, bool scaled)
{
  #ifdef QMC5883L_ST_TRACE
    ////int resultReady = ;
    int auxCounter = 0;
    while (!ready())
    {
      Serial.print("QMC5883L_ST::m_readRaw(). Not ready ");
      Serial.print(++auxCounter);
    }
  #else
    while(!ready()) {}
  #endif

  if(!read_register(addr,QMC5883L_X_LSB,6)) return 0;

  *x = Wire.read() | (Wire.read()<<8);
  *y = Wire.read() | (Wire.read()<<8);
  *z = Wire.read() | (Wire.read()<<8);

  // Save new values
  m_compassX = *x;
  m_compassY = *y;
  m_compassZ = *z;

  // imjaviperez
  if (scaled)
  {
    m_scaleValues(*x,*y,*z);
  }
  return 1;
}

bool QMC5883L_ST::readRawRotated( int16_t &x, int16_t &y, int16_t &z)
{
  int valuesRead = m_readRaw(&x, &y, &z, false);
  m_rotateRawValues();
  x = m_compassX;
  y = m_compassY;
  z = m_compassZ;

  m_calculateVarAngles();

  return (valuesRead);
}


/**
 * @brief There could happen that:
 * abs(max(x)) != abs(min(x)) 
 * so it is necessary to reescale values in every axis 
 * to obtain accurate messures with each compass 
 * orientation.
 * 
 * This function is based on readHeading() by Douglas Thain.
 * 
 * @author imjaviperez
 * @param x 
 * @param y 
 * @param z 
 * @return int 1 if values have been scaled. 
 * @return int 0 if values have not been scaled. 
 */
void QMC5883L_ST::m_scaleValues(int16_t &x, int16_t &y, int16_t &z)
{
  /* Update the observed boundaries of the measurements */
  if(x<xlow) xlow = x;
  if(x>xhigh) xhigh = x;
  if(y<ylow) ylow = y;
  if(y>yhigh) yhigh = y;
  if(z<zlow) zlow = z;
  if(z>zhigh) zhigh = z;

  /* Bail out if not enough data is available. */
  if( xlow==xhigh || ylow==yhigh || zlow==zhigh ) return;

  /* Recenter the measurement by subtracting the average */
  x -= (xhigh+xlow)/2;
  y -= (yhigh+ylow)/2;
  z -= (zhigh+zlow)/2;

  /* Rescale the measurement to the range observed. */
  // The magnetometer X, Y and Z axis could do different
  // calibration range, so the maximum and minimum values
  // for each axis there could be different.
  // Here that range is reescaled
  const int16_t INT_16_MAX_VALUE = pow(2,15)-1; // (2^15)-1; // 32767;
  float fx = INT_16_MAX_VALUE*(float)x/(xhigh-xlow);
  float fy = INT_16_MAX_VALUE*(float)y/(yhigh-ylow);
  float fz = INT_16_MAX_VALUE*(float)z/(zhigh-zlow);

  x = (int16_t)fx;
  y = (int16_t)fy;
  z = (int16_t)fz;
  
  //// return 1;
}


/**
 * @brief It returns (rx,ry,rz) but there will be as if 
 * magnetomer is placed with X axis forward.
 * rx will be a virtual forward axis and
 * ry will be a virtual left axis.
 * rz is a real upward axis.
 * 
 * @author imjaviperez
 * @param rx 
 * @param ry 
 * @param rz 
 */
inline void QMC5883L_ST::m_rotateRawValues()
{
  // Rotate X and Y angles
  if (m_xAngleOffset == COMPASS_X_AXIS_FORWARD) // 0
  {
    // Do not do anything
  }
  else if (m_xAngleOffset == COMPASS_X_AXIS_LEFT) //90
  {
    int16_t tmpX = m_compassX;
    m_compassX = -m_compassY;
    m_compassY = tmpX;
  }
  else if (m_xAngleOffset == COMPASS_X_AXIS_BACKWARD) // 180
  {
    m_compassX = -m_compassX;
    m_compassY = -m_compassY;
  }
  else if (m_xAngleOffset == COMPASS_X_AXIS_RIGHT) // 270
  {
    int16_t tmpX = m_compassX;
    m_compassX = m_compassY;
    m_compassY = -tmpX;
  }

}



/**
  * @brief It returns rotated and no rotated normalized
  * magnetometer values
 * 
 * @author imjaviperez
 * @param nx 
 * @param ny 
 * @param nz 
 * @return true : if returned values are right.
 * @return false : false if returned values are wrong.
 */
bool QMC5883L_ST::m_readRawNormalized(double &nx, double &ny, double &nz, bool rotated)
{
  int16_t bx, by, bz;
  if (!m_readRaw(&bx,&by,&bz,true))
  {
    #ifdef QMC5883L_ST_TRACE
      Serial.println("m_readRawNormalized() ERROR m_readRaw");
    #endif
    return false;
  } 

  if (rotated)
  {
    m_rotateRawValues();
  }
  bx = m_compassX;
  by = m_compassY;
  bz = m_compassZ;

  m_calculateVarAngles();

  // Normalize magnetometer readings
  if((bx == 0) && (by ==0 ) && (bz == 0))
  {
    #ifdef QMC5883L_ST_TRACE
      Serial.println("readRawNormalized() ERROR values");
    #endif
    return false;
  } 
  // Normalize magnetometer readings
  double d_bx = bx;
  double d_by = by;
  double d_bz = bz;
  double bNorm = sqrt(d_bx*d_bx + d_by*d_by + d_bz*d_bz);

  nx = d_bx/bNorm;
  ny = d_by/bNorm;
  nz = d_bz/bNorm;

  return true;  
}


/**
 * @brief It returns magnetometer normalized values but this time
 * (rnx,rny,rnz) will be as if magnetomer is placed with X axis forward.
 * rnx will be a virtual normalized forward axis and
 * rny will be a virtual normalized left axis.
 * rnz is a real upward normalized axis.
 * 
 * @param rnx 
 * @param rny 
 * @param rnz 
 * @return true : if returned values are right.
 * @return false : false if returned values are wrong.
 */
bool QMC5883L_ST::readRawNormalizedRotated(double &rnx, double &rny, double &rnz)
{
  return m_readRawNormalized(rnx,rny,rnz,true);
}

void QMC5883L_ST::resetCalibration() {
  xhigh = yhigh = 0;
  xlow = ylow = 0;
  zhigh = zlow = 0;
}

/**
 * @brief 
 * 
 * @return float Compass North heading in radians
 */
float QMC5883L_ST::readHeading()
{
  int16_t x, y, z; //, t;

  if(!readRawRotated(x,y,z))  return 0;

  /* Update the observed boundaries of the measurements */
  if(x<xlow) xlow = x;
  if(x>xhigh) xhigh = x;
  if(y<ylow) ylow = y;
  if(y>yhigh) yhigh = y;

  /* Bail out if not enough data is available. */
  if( xlow==xhigh || ylow==yhigh ) return 0;

  /* Recenter the measurement by subtracting the average */
  x -= (xhigh+xlow)/2;
  y -= (yhigh+ylow)/2;

  /* Rescale the measurement to the range observed. */
  float fx = (float)x/(xhigh-xlow);
  float fy = (float)y/(yhigh-ylow);

  int heading = atan2(fy,fx);

  return heading;
}


/**
 * @brief Calculates variance angles roll X, pitch Y and yaw Z
 * in (radians)^2
 * 
 */
inline void QMC5883L_ST::m_calculateVarAngles()
{
  // Calculate standard deviation
  double sigmaCompassEast =  sdAngleAtanDiff(m_compassZ, m_compassY, m_sdCompassZ, m_sdCompassY);
  double sigmaCompassNorth = sdAngleAtanDiff(m_compassX, m_compassZ, m_sdCompassX, m_sdCompassZ);
  double sigmaCompassZ =     sdAngleAtanDiff(m_compassY, m_compassX, m_sdCompassY, m_sdCompassX);

  // SD angle is between [0, 2*PI]
  sigmaCompassEast = min(abs(sigmaCompassEast), 2*M_PI);
  sigmaCompassNorth = min(abs(sigmaCompassNorth), 2*M_PI);
  sigmaCompassZ = min(abs(sigmaCompassZ), 2*M_PI);

  // Calculate variance
  m_varAngleX = (float) sigmaCompassEast * sigmaCompassEast;
  m_varAngleY = (float) sigmaCompassNorth * sigmaCompassNorth;
  m_varAngleZ = (float) sigmaCompassZ * sigmaCompassZ;
}


/**
 * @brief This member function gets rotation angles around
 * X axis (rollX), Y axis (pitchY) and Z axis (yawZ) 
 * with rotated or non rotated reference system.
 * Rotated means that the reference system is as if magnetomer
 * is placed with X axis forward.
 * X will be a virtual forward axis and
 * Y will be a virtual left axis.
 * Z is a real upward axis.
 * 
 * Every returned angle is related to Geographical North
 * coordinates system (not from Magnetic North coord. syst.)
 * 
 * @param rollX 
 * @param pitchY 
 * @param yawZ 
 * @param rotated 
 * @return true 
 * @return false 
 */
bool QMC5883L_ST::m_readAngles(float &rollX,float &pitchY,float &yawZ, bool rotated)
{
  int16_t x, y, z;

  if(!m_readRaw(&x,&y,&z,false)) return false;

  if (rotated)
  {
    m_rotateRawValues();
  }
  // Calculate variance after reading and ratating angles
  m_calculateVarAngles();

  //// x = m_compassX;
  //// y = m_compassY;
  //// z = m_compassZ;

  // Save rotation values in private local members
  m_rollX  = atan2(m_compassZ,m_compassY);/// - m_localRollX;
  m_pitchY = atan2(m_compassX,m_compassZ);/// - m_localPitchY;
  m_yawZ   = atan2(m_compassY,m_compassX);/// - m_localYawZ;
  // Copy values to return them
  rollX = m_rollX;
  pitchY = m_pitchY;
  yawZ = m_yawZ;

  return true;
}

/**
 * @brief This member function gets rotation angles around
 * X axis (rollX), Y axis (pitchY) and Z axis (yawZ).
 * 
 * @author imjaviperez
 * @param rollX 
 * @param pitchY 
 * @param yawZ 
 * @return true : it has rigth rotation angles.
 * @return false : if can not read any value.
 */
bool QMC5883L_ST::readAngles(float &rollX,float &pitchY,float &yawZ)
{
  return m_readAngles(rollX,pitchY,yawZ, false);
}

/**
 * @brief Asks magnetometer for new values of rollX, pitchY and yawZ
 * 
 * @param rollX Magnetometer roll angle. This variable is modified by the function.
 * @param pitchY Magnetometer pitch angle. This variable is modified by the function.
 * @param yawZ Magnetometer yaw angle. This variable is modified by the function.
 * @return true when reading values
 * @return false if there is no new values
 */
bool QMC5883L_ST::readRotatedAngles(float &rollX,float &pitchY,float &yawZ)
{
  return m_readAngles(rollX,pitchY,yawZ, true);
}

/**
 * @brief Returns magnetometer X axis raw value.
 * This value have been rotated yet, 
 * i.e.: the returned value is as if magnetometer had X axis forward.
 * 
 * @return int16_t 
 */
int16_t QMC5883L_ST::rawValueX()
{
  return m_compassX;
}

/**
 * @brief Returns magnetometer Y axis raw value.
 * This value have been rotated yet, 
 * i.e.: the returned value is as if magnetometer 
 * had X axis forward and Y axis left.
 * 
 * @return int16_t 
 */
int16_t QMC5883L_ST::rawValueY()
{
  return m_compassY;
}

/**
 * @brief Returns magnetometer Z axis raw value.
 * This value have been rotated yet, 
 * i.e.: the returned value is as if magnetometer had 
 * had X axis forward, Y axis left and Z axis upward direction
 * 
 * @return int16_t 
 */
int16_t QMC5883L_ST::rawValueZ()
{
  return m_compassZ;
}

/**
 * @brief Returns magnetometer rollX angle not asking again
 *  to the device. It sends last saved value
 * 
 * @return float 
 */
float QMC5883L_ST::rollX()
{
  return m_rollX;
}


/**
 * @brief Returns magnetometer pitchY angle not asking again
 *  to the device. It sends last saved value
 * 
 * @return float 
 */
float QMC5883L_ST::pitchY()
{
  return m_pitchY;
}


/**
 * @brief Returns magnetometer yawZ angle not asking again
 *  to the device. It sends last saved value
 * 
 * @return float 
 */
float QMC5883L_ST::yawZ()
{
  return m_yawZ;
}

/**
 * @brief It returns the compass offset angle orientation
 * in degrees from 0º to 360º.
 * 
 * @return int 
 */
compassXposition QMC5883L_ST::offsetAngle()
{
  return m_xAngleOffset;
}


/**
 * @brief Returns compass variance angles.
 * It overwrites parameters.
 * 
 * @param varRollX : variance of roll angle (angle around X axis)
 * @param varPitchY : variance of pitch angle (angle around Y axis)
 * @param varYawZ : variance of roll yaw (angle around Z axis)
 */
void QMC5883L_ST::getVarianceAngle(float &varRollX, float &varPitchY, float &varYawZ)
{
  varRollX = m_varAngleX;
  varPitchY = m_varAngleY;
  varYawZ = m_varAngleZ;
}


void QMC5883L_ST::getLocalAngles(float &localRollX, float &localPitchY, float &localYawZ)
{
  localRollX = m_localRollX;
  localPitchY = m_localPitchY;
  localYawZ = m_localYawZ;
}