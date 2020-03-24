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
 *      int readRaw( int16_t *x, int16_t *y, int16_t *z, int16_t *t );
 *    by
 *      int readRaw( int16_t *x, int16_t *y, int16_t *z);
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
  QMC5883L_ST(compassXposition xAngleOffset);
  ~QMC5883L_ST();
  void init();
  void reset();
  int  ready();
  void reconfig();
  
  int readHeading();

  //int readRaw( int16_t *x, int16_t *y, int16_t *z, int16_t *t );
  int readRaw( int16_t *x, int16_t *y, int16_t *z, bool scaled);


  // FIX-ME : ELIMINA ESTA FUNCION
  /// bool readRotated(int16_t &x, int16_t &y, int16_t &z);


  bool readRawAngles(int16_t &rollX,int16_t &pitchY,int16_t &yawZ);
  bool readRotatedAngles(int16_t &rollX,int16_t &pitchY,int16_t &yawZ);  
  bool readNormalized(double &nx, double &ny, double &nz);
  bool readNormalizedRotated(double &rnx, double &rny, double &rnz);

  void resetCalibration();

  void setSamplingRate( int rate );
  void setRange( int range );
  void setOversampling( int ovl );

  compassXposition offSetAngle();
  
private:
  int16_t xhigh, xlow;
  int16_t yhigh, ylow;
  int16_t zhigh, zlow;
  uint8_t addr;
  uint8_t mode;
  uint8_t rate;
  uint8_t range;
  uint8_t oversampling;
  // float m_localDeclitation;

  // QMC5883L hardware's device X axis can be placed rotated.
  // Select the offset rotated angle of device X axis
  compassXposition m_xAngleOffset;

  bool m_readNormalized(double &nx, double &ny, double &nz, bool rotated);
  bool m_readAngles(int16_t &rollX,int16_t &pitchY,int16_t &yawZ, bool rotated);
  void scaleValues(int16_t &x, int16_t &y, int16_t &z);
  void rotateAngles(int16_t &rx, int16_t &ry, int16_t &rz);
};
//}
#endif
