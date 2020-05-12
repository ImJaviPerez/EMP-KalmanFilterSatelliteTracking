#ifndef Kalman_AGC_h
#define Kalman_AGC_h

#include <Arduino.h>

class Kalman_AGC {
public:
    typedef enum 
    { 
        ACCELER = 0, 
        GYROSPE = 1,
        COMPASS = 2
    } observationIndices;

    typedef enum 
    { 
        ANGLE = 0, 
        ANGLE_RATE = 1
    } stateIndices;

    Kalman_AGC();

    // Kalman filter: returns filtered angle
    //void filteredState(float newAccelerometerAngle, float newGyroRate, float newCompassAngle, float dt, float newH_diagonal[3], float (&filteredState)[2]);
    void filteredState(float newAccelerometerAngle, float newGyroRate, float newCompassAngle, float dt, float newH_diagonal[3], float &filteredState);

    // Initialize Kalman filter values
    void initialize(float H_diagonal[3], float Q_diagonal[2], float a1[2], float P1_diagonal[2]);

private:

    //// // State vector: x = (Angle, AngleRate)
    //// float m_x[2];
    //// // Observation vector: y = (accelerometer, gyroscope, compass)
    //// float m_y[3];

    // epsilon: observation error density
    // H_k: observation error density covariance matrix
    // H_k is (3x3) diagonal matrix but we will use only its diagonal elements
    float m_H[3];

    // eta: state error density
    // Q_k: state error density covariance matrix
    // Q_k is (2x2) diagonal matrix  but we will use only its diagonal elements
    // This matrix will be constat along the whole filtering process
    float m_Q[2];

    // a_1: state vector (x_1) initial mean value:
    // a_k is a (2x1) vector
    float m_a[2];

    // P_1: a_1 covariance matrix
    // P_k is (2x2) matrix 
    float m_P[2][2];
};
#endif // Kalman_AGC_h
