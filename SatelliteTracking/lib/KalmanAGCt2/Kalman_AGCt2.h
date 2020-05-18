#ifndef Kalman_AGCt2_h
#define Kalman_AGCt2_h

#include <Arduino.h>

class Kalman_AGCt2 {
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
        ANGULAR_VELOCITY = 1,
        ANGULAR_ACCELERATION = 2
    } stateIndices;

    Kalman_AGCt2();

    // Kalman filter: returns filtered angle
    //void filteredState(float newAccelerometerAngle, float newGyroRate, float newCompassAngle, float dt, float newH_diagonal[3], float (&filteredState)[2]);
    void filteredState(float newAccelerometerAngle, float newGyroRate, float newCompassAngle, float dt, float newH_diagonal[3], float &filteredState);

    // Initialize Kalman filter values
    void initialize(float H_diagonal[3], float Q_diagonal[3], float a1[3], float P1_diagonal[3]);

private:
    // epsilon: observation error density
    // H_k: observation error density covariance matrix
    // H_k is (3x3) diagonal matrix but we will use only its diagonal elements
    float m_H[3];

    // eta: state error density
    // Q_k: state error density covariance matrix
    // Q_k is (3x3) diagonal matrix  but we will use only its diagonal elements
    // This matrix will be constat along the whole filtering process
    float m_Q[3];

    // a_1: state vector (x_1) initial mean value:
    // a_k is a (1x3) vector
    float m_a[3];

    // P_1: a_1 covariance matrix
    // P_k is (3x3) matrix 
    float m_P[3][3];
};
#endif // Kalman_AGCt2_h
