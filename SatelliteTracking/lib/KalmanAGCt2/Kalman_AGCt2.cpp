#include "Kalman_AGCt2.h"

/**
 * @brief Construct a new Kalman_AGCt2::Kalman_AGCt2 object
 * 
 */
Kalman_AGCt2::Kalman_AGCt2()
{
}

/**
 * @brief Initialize Kalman filter.
 * 
 * @author @ImJaviPerez
 * 
 * @param H1_diagonal : observation error density covariance matrix (3x3)
 *  initial diagonal values:
 *      sigma^2_Accelerometer (rad)^2
 *      sigma^2_Gyroscope (rad/seconds)^2
 *      sigma^2_Compass (rad)^2
 * @param Q_diagonal : state error density covariance matrix (2x2).
 *   It is a constant matrix with initial diagonal values:
 *      sigma^2_Angle (rad)^2
 *      sigma^2_AngularVelocity (rad/seconds)^2
 *      sigma^2_AngularAcceleration (rad/seconds^2)^2
 * @param a1 : state vector (x_1) initial mean value (2x1)
 *  initial values:
 *      angle (rad)
 *      angularVelocity (rad/seconds)
 *      angularAcceleration (rad/seconds^2)
 * @param P1_diagonal a_1 covariance matrix (3x3)
 *  initial diagonal values:
 *      sigma^2_angle (rad)^2
 *      sigma^2_angularVelocity (rad/seconds)^2
 *      sigma^2_angularAcceleration (rad/seconds^2)^2
 */
void Kalman_AGCt2::initialize(float H1_diagonal[3], float Q_diagonal[3], float a1[3], float P1_diagonal[3])
{
    // Accelerometer initial variance
    m_H[ACCELER] = H1_diagonal[ACCELER];
    // Gyroscope initial variance
    m_H[GYROSPE] = H1_diagonal[GYROSPE];
    // Compass initial variance
    m_H[COMPASS] = H1_diagonal[COMPASS];

    // Angle variance
    m_Q[ANGLE] = Q_diagonal[ANGLE];
    // Angular velocity variance
    m_Q[ANGULAR_VELOCITY] = Q_diagonal[ANGULAR_VELOCITY];
    // Angular acceleration variance
    m_Q[ANGULAR_ACCELERATION] = Q_diagonal[ANGULAR_ACCELERATION];

    // Angle initial mean value
    m_a[ANGLE] = a1[ANGLE];
    // Angular velocity initial mean value
    m_a[ANGULAR_VELOCITY] = a1[ANGULAR_VELOCITY];
    // Angular acceleration initial mean value
    m_a[ANGULAR_ACCELERATION] = a1[ANGULAR_ACCELERATION];
    
    // Initial mean value angle variance
    m_P[0][0] = P1_diagonal[ANGLE];
    m_P[0][1] = 0.0F;
    m_P[0][2] = 0.0F;
    // Initial mean value angular velocity variance
    m_P[1][0] = 0.0F;
    m_P[1][1] = P1_diagonal[ANGULAR_VELOCITY];
    m_P[1][2] = 0.0F;
    // Initial mean value angular acceleration variance
    m_P[2][0] = 0.0F;
    m_P[2][1] = 0.0F;
    m_P[2][2] = P1_diagonal[ANGULAR_ACCELERATION];
}


/**
 * @brief Kalman filter: returns filtered state values: 
 *  angle (rad) and angle rate (rad/second)
 * 
 * @param newAccelerometerAngle : new accelerometer angle in rad
 * @param newGyroRate : new gyroscope rate in rad/second
 * @param newCompassAngle new compass angle in radians
 * @param dt : delta time (seconds)
 * @param newH_diagonal : new observation error density covariance matrix
 * @param stateVector : Returns a vector with 2 values:
 *  stateVector[ANGLE] : filtered angle (rad)
 *  stateVector[ANGULAR_VELOCITY] : filtered angular velocity (rad/second)
 *  stateVector[ANGULAR_ACCELERATION] : filtered angular acceleration (rad/second)
 */
void Kalman_AGCt2::filteredState(float newAccelerometerAngle, 
        float newGyroRate, 
        float newCompassAngle, 
        float dt, 
        float newH_diagonal[3], 
        //float  (&stateVector)[3])
        float  &stateVector)
{
    // Innovation ----------------------------------------
    // v = y - Z a
    // float vk[3];
    // vk[ACCELER] = newAccelerometerAngle - m_a[ANGLE];
    // vk[GYROSPE] = newGyroRate - m_a[ANGULAR_VELOCITY];
    // vk[COMPASS] = newCompassAngle - m_a[ANGLE];
    // ---------------------------------------------------
    // F_k: innovation variance matrix
    // F[0][0] = m_P[0][0] + newH_diagonal[ACCELER]
    // F[0][1] = m_P[0][1]
    // F[0][2] = m_P[0][0]
    // F[1][0] = m_P[1][0]
    // F[1][1] = m_P[1][1] + newH_diagonal[GYROSPE]
    // F[1][2] = m_P[1][0]
    // F[2][0] = m_P[0][0]
    // F[2][1] = m_P[0][1]
    // F[2][2] = m_P[0][0] + newH_diagonal[COMPASS]
    // Inverse of innovation variance matrix: F_k^{-1}
    // F = Z P Z' + H 
    float F_inv[3][3];
    // determinat of H
    float detF = 2*m_P[0][0]*m_P[0][1]*m_P[1][0] 
        - m_P[0][0]*m_P[0][0]*(m_P[1][1] + newH_diagonal[GYROSPE]) 
        - m_P[0][1]*m_P[1][0]*(m_P[0][0] + newH_diagonal[ACCELER]) 
        - m_P[0][1]*m_P[1][0]*(m_P[0][0] + newH_diagonal[COMPASS]) 
        + (m_P[0][0] + newH_diagonal[ACCELER])*(m_P[0][0] + newH_diagonal[COMPASS])*(m_P[1][1] + newH_diagonal[GYROSPE]);


    // F inverse elements
    F_inv[0][0]=( m_P[0][0]*m_P[1][1] 
        - m_P[0][1]*m_P[1][0] 
        + m_P[0][0]*newH_diagonal[GYROSPE] 
        + m_P[1][1]*newH_diagonal[COMPASS] 
        + newH_diagonal[COMPASS]*newH_diagonal[GYROSPE] )/detF;
    F_inv[0][1]=( -m_P[0][1]*newH_diagonal[COMPASS] )/detF;
    F_inv[0][2]=( m_P[0][1]*m_P[1][0] 
        - m_P[0][0]*m_P[1][1] 
        - m_P[0][0]*newH_diagonal[GYROSPE] )/detF;
    F_inv[1][0]=( -m_P[1][0]*newH_diagonal[COMPASS] )/detF;
    F_inv[1][1]=( m_P[0][0]*newH_diagonal[ACCELER] 
        + m_P[0][0]*newH_diagonal[COMPASS] 
        + newH_diagonal[ACCELER]*newH_diagonal[COMPASS] )/detF;
    F_inv[1][2]=( -m_P[1][0]*newH_diagonal[ACCELER] )/detF;
    F_inv[2][0]=( m_P[0][1]*m_P[1][0]
        - m_P[0][0]*m_P[1][1] 
        - m_P[0][0]*newH_diagonal[GYROSPE] )/detF;
    F_inv[2][1]=( -m_P[0][1]*newH_diagonal[ACCELER] )/detF;
    F_inv[2][2]=( m_P[0][0]*m_P[1][1] 
        - m_P[0][1]*m_P[1][0] 
        + m_P[0][0]*newH_diagonal[GYROSPE] 
        + m_P[1][1]*newH_diagonal[ACCELER] 
        + newH_diagonal[ACCELER]*newH_diagonal[GYROSPE] )/detF;


  // Gain matrix ---------------------------------------
    // K = T P Z' F_inv
    float K[3][3];
    K[0][0]= F_inv[0][0]*(dt*dt*m_P[2][0]/2 + dt*m_P[1][0] + m_P[0][0]) + F_inv[1][0]*(dt*dt*m_P[2][1]/2 + dt*m_P[1][1] + m_P[0][1]) + F_inv[2][0]*(dt*dt*m_P[2][0]/2 + dt*m_P[1][0] + m_P[0][0]);
    K[0][1]= F_inv[0][1]*(dt*dt*m_P[2][0]/2 + dt*m_P[1][0] + m_P[0][0]) + F_inv[1][1]*(dt*dt*m_P[2][1]/2 + dt*m_P[1][1] + m_P[0][1]) + F_inv[2][1]*(dt*dt*m_P[2][0]/2 + dt*m_P[1][0] + m_P[0][0]);
    K[0][2]= F_inv[0][2]*(dt*dt*m_P[2][0]/2 + dt*m_P[1][0] + m_P[0][0]) + F_inv[1][2]*(dt*dt*m_P[2][1]/2 + dt*m_P[1][1] + m_P[0][1]) + F_inv[2][2]*(dt*dt*m_P[2][0]/2 + dt*m_P[1][0] + m_P[0][0]);
    K[1][0]= F_inv[0][0]*(dt*m_P[2][0] + m_P[1][0]) + F_inv[1][0]*(dt*m_P[2][1] + m_P[1][1]) + F_inv[2][0]*(dt*m_P[2][0] + m_P[1][0]);
    K[1][1]= F_inv[0][1]*(dt*m_P[2][0] + m_P[1][0]) + F_inv[1][1]*(dt*m_P[2][1] + m_P[1][1]) + F_inv[2][1]*(dt*m_P[2][0] + m_P[1][0]);
    K[1][2]= F_inv[0][2]*(dt*m_P[2][0] + m_P[1][0]) + F_inv[1][2]*(dt*m_P[2][1] + m_P[1][1]) + F_inv[2][2]*(dt*m_P[2][0] + m_P[1][0]);
    K[2][0]= F_inv[0][0]*m_P[2][0] + F_inv[1][0]*m_P[2][1] + F_inv[2][0]*m_P[2][0];
    K[2][1]= F_inv[0][1]*m_P[2][0] + F_inv[1][1]*m_P[2][1] + F_inv[2][1]*m_P[2][0];
    K[2][2]= F_inv[0][2]*m_P[2][0] + F_inv[1][2]*m_P[2][1] + F_inv[2][2]*m_P[2][0];


    // a_k+1: x_k+1 mean value ---------------------------
    // Temporal matrix
    float m_a_kp1[3];
    m_a_kp1[ANGLE]= K[0][0]*(-m_a[ANGLE] + newAccelerometerAngle) 
        + K[0][1]*(-m_a[ANGULAR_VELOCITY] + newGyroRate) 
        + K[0][2]*(-m_a[ANGLE] + newCompassAngle) 
        + dt*dt*m_a[ANGULAR_ACCELERATION]/2 
        + dt*m_a[ANGULAR_VELOCITY] 
        + m_a[ANGLE];
    m_a_kp1[ANGULAR_VELOCITY]= K[1][0]*(-m_a[ANGLE] + newAccelerometerAngle) 
        + K[1][1]*(-m_a[ANGULAR_VELOCITY] + newGyroRate) 
        + K[1][2]*(-m_a[ANGLE] + newCompassAngle) 
        + dt*m_a[ANGULAR_ACCELERATION] 
        + m_a[ANGULAR_VELOCITY];
    m_a_kp1[ANGULAR_ACCELERATION]= K[2][0]*(-m_a[ANGLE] + newAccelerometerAngle) 
        + K[2][1]*(-m_a[ANGULAR_VELOCITY] + newGyroRate) 
        + K[2][2]*(-m_a[ANGLE] + newCompassAngle) 
        + m_a[ANGULAR_ACCELERATION];

    // Angle must be between {0,2*PI}
    while (m_a_kp1[ANGLE] > 2.0F*M_PI)
    {
        m_a_kp1[ANGLE] -= 2.0F*M_PI;
    }
    while (m_a_kp1[ANGLE] < 0)
    {
        m_a_kp1[ANGLE] += 2.0F*M_PI;
    }
    
    // Actualize values
    m_a[ANGLE] = m_a_kp1[ANGLE];
    m_a[ANGULAR_VELOCITY] = m_a_kp1[ANGULAR_VELOCITY];
    m_a[ANGULAR_ACCELERATION] = m_a_kp1[ANGULAR_ACCELERATION];


    // P_k+1: covariance matrix of x_k+1 -----------------
    // Temporal matrix
    float m_P_kp1[3][3];
    m_P_kp1[0][0]= dt*dt*(dt*dt*m_P[2][2]/2 + dt*m_P[1][2] + m_P[0][2])/2 
        + m_Q[ANGLE] 
        + (-K[0][1] + dt)*(dt*dt*m_P[2][1]/2 + dt*m_P[1][1] + m_P[0][1]) 
        + (-K[0][0] - K[0][2] + 1)*(dt*dt*m_P[2][0]/2 + dt*m_P[1][0] + m_P[0][0]);
    m_P_kp1[0][1]= dt*(dt*dt*m_P[2][2]/2 + dt*m_P[1][2] + m_P[0][2]) 
        + (1 - K[1][1])*(dt*dt*m_P[2][1]/2 + dt*m_P[1][1] + m_P[0][1]) 
        + (-K[1][0] - K[1][2])*(dt*dt*m_P[2][0]/2 + dt*m_P[1][0] + m_P[0][0]);
    m_P_kp1[0][2]= -K[2][1]*(dt*dt*m_P[2][1]/2 + dt*m_P[1][1] + m_P[0][1]) 
        + dt*dt*m_P[2][2]/2 
        + dt*m_P[1][2] 
        + m_P[0][2] 
        + (-K[2][0] - K[2][2])*(dt*dt*m_P[2][0]/2 + dt*m_P[1][0] + m_P[0][0]);
    m_P_kp1[1][0]= dt*dt*(dt*m_P[2][2] 
        + m_P[1][2])/2 
        + (-K[0][1] + dt)*(dt*m_P[2][1] + m_P[1][1]) 
        + (dt*m_P[2][0] + m_P[1][0])*(-K[0][0] - K[0][2] + 1);
    m_P_kp1[1][1]= dt*(dt*m_P[2][2] + m_P[1][2]) 
        + m_Q[ANGULAR_VELOCITY] 
        + (1 - K[1][1])*(dt*m_P[2][1] + m_P[1][1]) 
        + (-K[1][0] - K[1][2])*(dt*m_P[2][0] + m_P[1][0]);
    m_P_kp1[1][2]= -K[2][1]*(dt*m_P[2][1] + m_P[1][1]) 
        + dt*m_P[2][2] 
        + m_P[1][2] 
        + (-K[2][0] - K[2][2])*(dt*m_P[2][0] + m_P[1][0]);
    m_P_kp1[2][0]= dt*dt*m_P[2][2]/2 
        + m_P[2][0]*(-K[0][0] - K[0][2] + 1) 
        + m_P[2][1]*(-K[0][1] + dt);
    m_P_kp1[2][1]= dt*m_P[2][2] 
        + m_P[2][0]*(-K[1][0] - K[1][2]) 
        + m_P[2][1]*(1 - K[1][1]);
    m_P_kp1[2][2]= -K[2][1]*m_P[2][1] 
        + m_P[2][0]*(-K[2][0] - K[2][2]) 
        + m_P[2][2] + m_Q[ANGULAR_ACCELERATION];

    // Actualize values
    m_P[0][0] = m_P_kp1[0][0];
    m_P[0][1] = m_P_kp1[0][1];
    m_P[0][2] = m_P_kp1[0][2];
    m_P[1][0] = m_P_kp1[1][0];
    m_P[1][1] = m_P_kp1[1][1];
    m_P[1][2] = m_P_kp1[1][2];
    m_P[2][0] = m_P_kp1[2][0];
    m_P[2][1] = m_P_kp1[2][1];
    m_P[2][2] = m_P_kp1[2][2];

    // Return filtered values
    stateVector = m_a[ANGLE];
    //stateVector[ANGLE] = m_a[ANGLE];
    //stateVector[ANGULAR_VELOCITY] = m_a[ANGULAR_VELOCITY];
    //stateVector[ANGULAR_ACCELERATION] = m_a[ANGULAR_ACCELERATION];
}
