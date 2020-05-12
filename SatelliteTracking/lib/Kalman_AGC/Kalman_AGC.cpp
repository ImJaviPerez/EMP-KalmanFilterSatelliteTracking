#include "Kalman_AGC.h"

/**
 * @brief Construct a new Kalman_AGC::Kalman_AGC object
 * 
 */
Kalman_AGC::Kalman_AGC()
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
 *      sigma^2_AngleRate (rad/seconds)^2
 * @param a1 : state vector (x_1) initial mean value (2x1)
 *  initial values:
 *      angle (rad)
 *      AngleRate (rad/seconds)
 * @param P1_diagonal a_1 covariance matrix (2x2)
 *  initial diagonal values:
 *      angle (rad)^2
 *      AngleRate (rad/seconds)^2
 */
void Kalman_AGC::initialize(float H1_diagonal[3], float Q_diagonal[2], float a1[2], float P1_diagonal[2])
{
    // Accelerometer initial variance
    m_H[ACCELER] = H1_diagonal[ACCELER];
    // Gyroscope initial variance
    m_H[GYROSPE] = H1_diagonal[GYROSPE];
    // Compass initial variance
    m_H[COMPASS] = H1_diagonal[COMPASS];

    // Angle variance
    m_Q[ANGLE] = Q_diagonal[ANGLE];
    // Angle rate variance
    m_Q[ANGLE_RATE] = Q_diagonal[ANGLE_RATE];

    //  Angle initial mean value
    m_a[ANGLE] = a1[ANGLE];
    //  Angle rate initial mean value
    m_a[ANGLE_RATE] = a1[ANGLE_RATE];
    
    // Initial mean value angle variance
    m_P[0][0] = P1_diagonal[ANGLE];
    m_P[0][1] = 0.0F;
    // Initial mean value angle rate variance
    m_P[1][1] = P1_diagonal[ANGLE_RATE];
    m_P[1][0] = 0.0F;
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
 *  stateVector[ANGLE_RATE] : filtered angle rate (rad/second)
 */
void Kalman_AGC::filteredState(float newAccelerometerAngle, 
        float newGyroRate, 
        float newCompassAngle, 
        float dt, 
        float newH_diagonal[3], 
        //float  (&stateVector)[2])
        float  &stateVector)
{
    // Innovation ----------------------------------------
    // v = y - Z a
    // float vk[3];
    // vk[ACCELER] = newAccelerometerAngle - m_a[ANGLE];
    // vk[GYROSPE] = newGyroRate - m_a[ANGLE_RATE];
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
         -m_P[0][0]*m_P[0][0]*(m_P[1][1] + newH_diagonal[GYROSPE])
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
    float K[2][3];
    K[0][0]= F_inv[0][0]*(dt*m_P[1][0] + m_P[0][0]) + F_inv[1][0]*(dt*m_P[1][1] + m_P[0][1]) + F_inv[2][0]*(dt*m_P[1][0] + m_P[0][0]);
    K[0][1]= F_inv[0][1]*(dt*m_P[1][0] + m_P[0][0]) + F_inv[1][1]*(dt*m_P[1][1] + m_P[0][1]) + F_inv[2][1]*(dt*m_P[1][0] + m_P[0][0]);
    K[0][2]= F_inv[0][2]*(dt*m_P[1][0] + m_P[0][0]) + F_inv[1][2]*(dt*m_P[1][1] + m_P[0][1]) + F_inv[2][2]*(dt*m_P[1][0] + m_P[0][0]);
    K[1][0]= F_inv[0][0]*m_P[1][0] + F_inv[1][0]*m_P[1][1] + F_inv[2][0]*m_P[1][0];
    K[1][1]= F_inv[0][1]*m_P[1][0] + F_inv[1][1]*m_P[1][1] + F_inv[2][1]*m_P[1][0];
    K[1][2]= F_inv[0][2]*m_P[1][0] + F_inv[1][2]*m_P[1][1] + F_inv[2][2]*m_P[1][0];

    // a_k+1: x_k+1 mean value ---------------------------
    // Temporal matrix
    float m_a_kp1[2];
    m_a_kp1[ANGLE]= dt*m_a[ANGLE_RATE] 
        + K[0][0]*(newAccelerometerAngle - m_a[ANGLE]) 
        + K[0][1]*(newGyroRate - m_a[ANGLE_RATE]) 
        + K[0][2]*(newCompassAngle - m_a[ANGLE]) 
        + m_a[ANGLE];
    m_a_kp1[ANGLE_RATE]= K[1][0]*(newAccelerometerAngle - m_a[ANGLE]) 
        + K[1][1]*(newGyroRate - m_a[ANGLE_RATE]) 
        + K[1][2]*(newCompassAngle - m_a[ANGLE]) 
        + m_a[ANGLE_RATE];

    // Angles must be between {0,2*PI}
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
    m_a[ANGLE_RATE] = m_a_kp1[ANGLE_RATE];


    // P_k+1: covariance matrix of x_k+1 -----------------
    // Temporal matrix
    float m_P_kp1[2][2];
    m_P_kp1[0][0]= m_Q[ANGLE] + (dt - K[0][1])*(dt*m_P[1][1] + m_P[0][1]) + (dt*m_P[1][0] + m_P[0][0])*(-K[0][0] - K[0][2] + 1);
    m_P_kp1[0][1]= (1 - K[1][1])*(dt*m_P[1][1] + m_P[0][1]) + (-K[1][0] - K[1][2])*(dt*m_P[1][0] + m_P[0][0]);
    m_P_kp1[1][0]= m_P[1][0]*(-K[0][0] - K[0][2] + 1) + m_P[1][1]*(dt - K[0][1]);
    m_P_kp1[1][1]= m_P[1][0]*(-K[1][0] - K[1][2]) + m_P[1][1]*(1 - K[1][1]) + m_Q[ANGLE_RATE];

    // Actualize values
    m_P[0][0] = m_P_kp1[0][0];
    m_P[0][1] = m_P_kp1[0][1];
    m_P[1][0] = m_P_kp1[1][0];
    m_P[1][1] = m_P_kp1[1][1];

    // Return filtered values
    stateVector = m_a[ANGLE];
    //stateVector[ANGLE] = m_a[ANGLE];
    //stateVector[ANGLE_RATE] = m_a[ANGLE_RATE];
}
