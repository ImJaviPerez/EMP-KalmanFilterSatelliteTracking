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

//namespace SatTrack
//{
class QMC5883L_ST {
public:
  QMC5883L_ST();
  QMC5883L_ST(int xAngleOffset);
  ~QMC5883L_ST();
  void init();
  void reset();
  int  ready();
  void reconfig();
  
  int readHeading();
  //int readRaw( int16_t *x, int16_t *y, int16_t *z, int16_t *t );
  int readRaw( int16_t *x, int16_t *y, int16_t *z);

  void resetCalibration();

  void setSamplingRate( int rate );
  void setRange( int range );
  void setOversampling( int ovl );
  
private:
  int16_t xhigh, xlow;
  int16_t yhigh, ylow;
  uint8_t addr;
  uint8_t mode;
  uint8_t rate;
  uint8_t range;
  uint8_t oversampling;

  // QMC5883L hardware's device X axis can be placed rotated.
  // Select the offset rotated angle of device X axis
  int m_xAngleOffset;
};
//}
#endif
