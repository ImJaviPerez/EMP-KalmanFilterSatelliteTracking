#include <Wire.h>
#include <math.h>
#include "QMC5883L_ST.h"

// #define QMC5883L_ST_TRACE
#ifdef QMC5883L_ST_TRACE
  #include <Arduino.h>
#endif

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

// Constructor
QMC5883L_ST::QMC5883L_ST()
{
  QMC5883L_ST(COMPASS_X_AXIS_FORWARD);
}
// Constructor
QMC5883L_ST::QMC5883L_ST(compassXposition compassXposition)
{
  m_xAngleOffset = compassXposition;
  // m_localDeclitation = localDeclination;
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

// Do not use variable t
// int QMC5883L_ST::readRaw( int16_t *x, int16_t *y, int16_t *z, int16_t *t )
/**
 * @brief 
 * 
 * @param x 
 * @param y 
 * @param z 
 * @return int 
 */
int QMC5883L_ST::readRaw( int16_t *x, int16_t *y, int16_t *z, bool scaled)
{
  #ifdef QMC5883L_ST_TRACE
    ////int resultReady = ;
    int auxCounter = 0;
    while (!ready())
    {
      Serial.print("QMC5883L_ST::readRaw(). Not ready ");
      Serial.print(++auxCounter);
    }
  #else
    while(!ready()) {}
  #endif

  if(!read_register(addr,QMC5883L_X_LSB,6)) return 0;

  *x = Wire.read() | (Wire.read()<<8);
  *y = Wire.read() | (Wire.read()<<8);
  *z = Wire.read() | (Wire.read()<<8);

  // imjaviperez
  if (scaled)
  {
    scaleValues(*x,*y,*z);
  }
  return 1;
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
void QMC5883L_ST::scaleValues(int16_t &x, int16_t &y, int16_t &z)
{
  /* Update the observed boundaries of the measurements */
  if(x<xlow) xlow = x;
  if(x>xhigh) xhigh = x;
  if(y<ylow) ylow = y;
  if(y>yhigh) yhigh = y;
  /*
  // To accelerate the adjustament process we can compare X and Y axes
  if (xlow < ylow)
  {
    ylow = xlow;
  }
  else
  {
    xlow = ylow;
  }

  if (xhigh > yhigh)
  {
    yhigh = xhigh;
  }
  else
  {
    xhigh = yhigh;
  }
  */

  if(z<zlow) zlow = z;
  if(z>zhigh) zhigh = z;

  /* Bail out if not enough data is available. */
  if( xlow==xhigh || ylow==yhigh || zlow==zhigh ) return 0;

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
 * @brief It returns magnetometer raw values but this time
 * (rx,ry,rz) will be as if magnetomer is placed with X axis forward.
 * rx will be a virtual forward axis and
 * ry will be a virtual left axis.
 * rz is a real upward axis.
 * 
 * @author imjaviperez
 * @param rx 
 * @param ry 
 * @param rz 
 * @return true if returned values are right.
 * @return false if returned values are wrong.
 */
/*
bool QMC5883L_ST::readRotated(int16_t &rx, int16_t &ry, int16_t &rz)
{
  if (!readRaw(&rx,&ry,&rz,true))
  {
    #ifdef QMC5883L_ST_TRACE
      Serial.println("readRotated() ERROR readRaw");
    #endif
    return false;
  } 
  
  // Rotate X and Y angles
  if (m_xAngleOffset == COMPASS_X_AXIS_FORWARD) // 0
  {
    // Do not do anything
  }
  else if (m_xAngleOffset == COMPASS_X_AXIS_LEFT) //90
  {
    int16_t tmpX = rx;
    rx = -ry;
    ry = tmpX;
  }
  else if (m_xAngleOffset == COMPASS_X_AXIS_BACKWARD) // 180
  {
    rx = -rx;
    ry = -ry;
  }
  else if (m_xAngleOffset == COMPASS_X_AXIS_RIGHT) // 270
  {
    int16_t tmpX = rx;
    rx = -ry;
    ry = tmpX;
  }

  return true;
}
*/

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
void QMC5883L_ST::rotateAngles(int16_t &rx, int16_t &ry, int16_t &rz)
{
  // Rotate X and Y angles
  if (m_xAngleOffset == COMPASS_X_AXIS_FORWARD) // 0
  {
    // Do not do anything
  }
  else if (m_xAngleOffset == COMPASS_X_AXIS_LEFT) //90
  {
    int16_t tmpX = rx;
    rx = -ry;
    ry = tmpX;
  }
  else if (m_xAngleOffset == COMPASS_X_AXIS_BACKWARD) // 180
  {
    rx = -rx;
    ry = -ry;
  }
  else if (m_xAngleOffset == COMPASS_X_AXIS_RIGHT) // 270
  {
    int16_t tmpX = rx;
    rx = -ry;
    ry = tmpX;
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
bool QMC5883L_ST::m_readNormalized(double &nx, double &ny, double &nz, bool rotated)
{
  int16_t bx, by, bz;
  if (!readRaw(&bx,&by,&bz,true))
  {
    #ifdef QMC5883L_ST_TRACE
      Serial.println("m_readNormalized() ERROR readRaw");
    #endif
    return false;
  } 

  if (rotated)
  {
    rotateAngles(bx,by,bz);
  }

  // Normalize magnetometer readings
  if((bx == 0) && (by ==0 ) && (bz == 0))
  {
    #ifdef QMC5883L_ST_TRACE
      Serial.println("readNormalized() ERROR values");
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
 * @brief It returns normalized magnetometer values
 * 
 * @author imjaviperez
 * @param nx 
 * @param ny 
 * @param nz 
 * @return true : if returned values are right.
 * @return false : false if returned values are wrong.
 */
bool QMC5883L_ST::readNormalized(double &nx, double &ny, double &nz)
{
  return m_readNormalized(nx,ny,nz,false);
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
bool QMC5883L_ST::readNormalizedRotated(double &rnx, double &rny, double &rnz)
{
  return m_readNormalized(rnx,rny,rnz,true);
}

void QMC5883L_ST::resetCalibration() {
  xhigh = yhigh = 0;
  xlow = ylow = 0;
  zhigh = zlow = 0;
}

int QMC5883L_ST::readHeading()
{
  int16_t x, y, z; //, t;

  //// if(!readRaw(&x,&y,&z,&t)) return 0;
  if(!readRaw(&x,&y,&z,true)) return 0;

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

  int heading = 180.0*atan2(fy,fx)/M_PI;
  // if(heading<=0) heading += 360;
  // 
  // Offset correction to show
  // heading -= m_xAngleOffset;
  // if(heading<=0) heading += 360;

  return heading;
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
 * @param rollX 
 * @param pitchY 
 * @param yawZ 
 * @param rotated 
 * @return true 
 * @return false 
 */
bool QMC5883L_ST::m_readAngles(int16_t &rollX,int16_t &pitchY,int16_t &yawZ, bool rotated)
{
  int16_t x, y, z;

  if(!readRaw(&x,&y,&z,true)) return false;

  if (rotated)
  {
    rotateAngles(x,y,z);
  }

  rollX = (180.0 * atan2(z,y) / M_PI);
  pitchY = (180.0 * atan2(x,z) / M_PI);
  yawZ = (180.0 * atan2(y,x) / M_PI);

  // if (rollX < 0) rollX += 360;
  // if (pitchY < 0) pitchY += 360;
  // if (yawZ < 0) yawZ += 360;

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
bool QMC5883L_ST::readRawAngles(int16_t &rollX,int16_t &pitchY,int16_t &yawZ)
{
  return m_readAngles(rollX,pitchY,yawZ, false);
}

bool QMC5883L_ST::readRotatedAngles(int16_t &rollX,int16_t &pitchY,int16_t &yawZ)
{
  return m_readAngles(rollX,pitchY,yawZ, true);
}

/**
 * @brief It returns the compass offset angle orientation
 * in degrees from 0º to 360º.
 * 
 * @return int 
 */
compassXposition QMC5883L_ST::offSetAngle()
{
  return m_xAngleOffset;
}