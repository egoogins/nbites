#include "KalmanFilter.h"

// We're doing lots of NBMath so make life easier...
using namespace NBMath;

namespace man {
namespace balltrack {

KalmanFilter::KalmanFilter(bool stationary_,
                           KalmanFilterParams params_)
    :  m_params(params_),
       m_updated(false),
       m_last_update_time(0),
       m_delta_time(0.03f),
       m_rel_x_dest(0.f),
       m_rel_y_dest(0.f)
{
    m_score = 0.f;
    m_cur_entry = 0.f;
    m_correction_mag_buffer = new float[BUFF_LENGTH];
    m_stationary = stationary_;
    initialize();
}

KalmanFilter::~KalmanFilter() {
    delete m_correction_mag_buffer;
}

// Evaluate 2D PDF
float KalmanFilter::getProbAtMean() {
    float detOfCov = m_cov(0,0) * m_cov(1,1) - m_cov(0,1) * m_cov(1,0);
    return 1 / (2 * PI * std::sqrt(detOfCov));
}

void KalmanFilter::updateDeltaTime()
{
    // Get time since last update
    const long long int time = monotonic_micro_time(); //from common
    m_delta_time = static_cast<float>(time - m_last_update_time)/1000000.0f; // u_s to sec

    // Guard against a zero dt (maybe possible?)
    if (m_delta_time <= 0.0){
        m_delta_time = 0.0001f;
    }

    if (m_delta_time > 1)
        m_delta_time = .03f; // Guard against first frame issues

    m_last_update_time = time;
}

void KalmanFilter::update(messages::VisionBall visionBall,
                          messages::RobotLocation  motion)
{
    m_updated = true;

    //Get passed time
    updateDeltaTime();

    predict(motion);
    //Note: Only update if we have an observation...
    if(visionBall.on())
        updateWithObservation(visionBall);
}

void KalmanFilter::predict(messages::RobotLocation odometry)
{
    // Overload for offline data simulation
    predict(odometry, m_delta_time);
}

void KalmanFilter::predict(messages::RobotLocation odometry, float deltaT)
{
    float diffX = odometry.x() - m_last_odometry.x();
    m_last_odometry.set_x(odometry.x());
    float diffY = odometry.y() - m_last_odometry.y();
    m_last_odometry.set_y(odometry.y());
    float diffH = odometry.h() - m_last_odometry.h();
    m_last_odometry.set_h(odometry.h());

    float sinh, cosh;
    sincosf(odometry.h(), &sinh, &cosh);

    // change into the robot frame
    float dX = cosh*diffX + sinh*diffY;
    float dY = cosh*diffY - sinh*diffX;
    float dH = diffH;// * 2.4f;

    if( (std::fabs(dX) > 3.f) || (std::fabs(dY) > 3.f) ) {
        //Probably reset odometry somewhere so skip a frame
        dX = 0.f;
        dY = 0.f;
        dH = 0.f;
    }

    //Calculate A = rotation matrix * trajectory matrix
    //First calc rotation matrix
    float sinDeltaH, cosDeltaH;
    sincosf(dH,&sinDeltaH, &cosDeltaH);

    // Keep track of the deviation
    float rotationDeviation = dH * m_params.rotationDeviation;
    float sinRotDev, cosRotDev;
    sincosf(rotationDeviation, &sinRotDev, &cosRotDev);

    // Generate the rotation and rotationDeviationRotation matrices
    // We rotate counterclockwise for positive dH
    // So everything rotates clockwise around us
    ufmatrix4 rotation = boost::numeric::ublas::identity_matrix <float>(4);
    ufmatrix4 rotationDeviationRotation = boost::numeric::ublas::identity_matrix <float>(4);

    // nxm matrix is n rows, m columns
    rotation(0,0) = cosDeltaH;
    rotation(1,0) = -sinDeltaH;
    rotation(0,1) = sinDeltaH;
    rotation(1,1) = cosDeltaH;

    rotationDeviationRotation(0,0) = cosRotDev;
    rotationDeviationRotation(1,0) = -sinRotDev;
    rotationDeviationRotation(0,1) = sinRotDev;
    rotationDeviationRotation(1,1) = cosRotDev;

    if (m_stationary){
        rotation(2,2) = 0.f;
        rotation(3,3) = 0.f;

        rotationDeviationRotation(2,2) = 0.f;
        rotationDeviationRotation(3,3) = 0.f;
    }
    else { // estimate assumes ball is moving
        rotation(2,2) = cosDeltaH;
        rotation(3,2) = sinDeltaH;
        rotation(2,3) = -sinDeltaH;
        rotation(3,3) = cosDeltaH;

        rotationDeviationRotation(2,3) = cosRotDev;
        rotationDeviationRotation(3,2) = sinRotDev;
        rotationDeviationRotation(2,3) = -sinRotDev;
        rotationDeviationRotation(3,3) = cosRotDev;
    }

    // Calculate the trajectory
    ufmatrix4 trajectory = boost::numeric::ublas::identity_matrix <float>(4);
    if (!m_stationary) { //if estimate is moving, predict to where using velocity
        trajectory(0,2) = deltaT;
        trajectory(1,3) = deltaT;
    }

    // Calculate the translation from odometry
    // If we go left (positive y, everything else shifts down
    // NOTE: Using notation from Probabilistic Robotics, B = Identity
    //       so no need to compute it or calculate anything with it
    ufvector4 translation = NBMath::vector4D(-dX, -dY, 0.f, 0.f);
    // And deviation
    float xTransDev = dX * m_params.transXDeviation;
    xTransDev *= xTransDev;

    float yTransDev = dY * m_params.transYDeviation;
    yTransDev *= yTransDev;

    ufvector4 translationDeviation = NBMath::vector4D(xTransDev,
                                                      yTransDev,
                                                      0.f, 0.f);

    // Incorporate Friction
    // Have params.ballFriction in cm/sec^2
    // ballFriction * deltaT = impact from friction that frame in cm/sec (velocity)
    // in each direction, so need to add that impact if velocity is positive,
    //                       need to subtract that impact if velocity is negative

    // Incorporate friction if the ball is moving

    if (!m_stationary) // moving
    {
        float xVelFactor = (std::abs(m_x(2)) + m_params.ballFriction*deltaT)/std::abs(m_x(2));
        float yVelFactor = (std::abs(m_x(3)) + m_params.ballFriction*deltaT)/std::abs(m_x(3));

        if( xVelFactor < 0.001f)
            m_x(2) = .0001f;
        if( yVelFactor < 0.001f)
            m_x(3) = .0001f;

        // Determine if ball is still moving
        float velMagnitude = getSpeed();

        if(velMagnitude > 2.f) // basically still moving
        {
            // vel = vel * (absVel + decel)/absVel
            m_x(2) *= xVelFactor;
            m_x(3) *= yVelFactor;
        }
    }

    // Calculate the expected state
    ufmatrix4 A = prod(rotation,trajectory);
    ufmatrix4 ATranspose = trans(A);
    ufvector4 p = prod(A,m_x);
    m_x = p + translation;

    // Calculate the covariance Cov = A*Cov*ATranspose
    ufmatrix4 covTimesATranspose = prod(m_cov, ATranspose);
    m_cov = prod(A, covTimesATranspose);

    // Add noise: Process noise, rotation dev, translation dev
    ufvector4 noise;
    noise = boost::numeric::ublas::zero_vector<float>(4);

    // Add process noise
    noise(0) += m_params.processDeviationPosX;
    noise(1) += m_params.processDeviationPosY;

    if(!m_stationary){
        noise(2) += m_params.processDeviationVelX;
        noise(3) += m_params.processDeviationVelY;
    }

    // Add translation deviation noise
    noise += translationDeviation;

    // Add rotation deviation noise
    // We've already made the matrix using the deviation matrix
    // so lets pump it through
    // #CRUDE_APPROXIMATION @b_human
    ufvector4 noiseFromRot = prod(rotationDeviationRotation, m_x) - m_x;

    for (int i=0; i<4; i++)
        noise(i) += std::abs(noiseFromRot(i));

    // Add all this noise to the covariance
    for(int i=0; i<4; i++){
        m_cov(i,i) += noise(i);
    }

    // Housekeep
    m_filtered_dist = std::sqrt(m_x(0)*m_x(0) + m_x(1)*m_x(1));
    m_filtered_bear = NBMath::safe_atan2(m_x(1),m_x(0));
}

void KalmanFilter::updateWithObservation(messages::VisionBall visionBall)
{
    // Declare C and C transpose (ublas)
    // C takes state estimate to observation frame so
    // c = 1  0  0  0
    //     0  1  0  0
    ufmatrix c (2,4);
    for(unsigned i=0; i<c.size1(); i++){
        for(unsigned j=0; j<c.size2(); j++){
            if(i == j)
                c(i,j) = 1.f;
            else
                c(i,j) = 0.f;
        }
    }
    ufmatrix cTranspose(4,2);
    cTranspose = trans(c);

    // Calculate the gain
    // Calc c*cov*c^t
    ufmatrix cCovCTranspose(2,2);
    cCovCTranspose = prod(m_cov,cTranspose);
    cCovCTranspose = prod(c,cCovCTranspose);

    cCovCTranspose(0,0) += m_params.obsvRelXVariance;
    cCovCTranspose(1,1) += m_params.obsvRelYVariance;

    // gain = cov*c^t*(c*cov*c^t + var)^-1
    ufmatrix kalmanGain(2,2);

    kalmanGain = prod(cTranspose,NBMath::invert2by2(cCovCTranspose));
    kalmanGain = prod(m_cov,kalmanGain);

    ufvector posEstimates(2);
    posEstimates = prod(c, m_x);

    // x straight ahead, y to the right
    float sinB,cosB;
    sincosf(visionBall.bearing(),&sinB,&cosB);

    ufvector measurement(2);
    measurement(0) = visionBall.distance()*cosB;
    measurement(1) = visionBall.distance()*sinB;

    ufvector innovation(2);
    innovation = measurement - posEstimates;

    ufvector correction(4);
    correction = prod(kalmanGain,innovation);
    m_x += correction;

    //cov = cov - k*c*cov
    ufmatrix4 identity;
    identity = boost::numeric::ublas::identity_matrix <float>(4);
    ufmatrix4 modifyCov;
    modifyCov = identity - prod(kalmanGain,c);
    m_cov = prod(modifyCov,m_cov);

    // Housekeep
    m_filtered_dist = std::sqrt(m_x(0)*m_x(0) + m_x(1)*m_x(1));
    m_filtered_bear = NBMath::safe_atan2(m_x(1),m_x(0));

    float curErr = std::sqrt(correction(0)*correction(0) + correction(1)*correction(1));
    m_correction_mag_buffer[m_cur_entry] = curErr;

    m_score = 0.f;
    for(int i=0; i<BUFF_LENGTH; i++)
        m_score+= m_correction_mag_buffer[i];
    m_score = m_score/10; // avg correction over 10 frames

    //get magnitude of correction
    // std::cout << "Is moving filter " << isStationary() << std::endl;
    // std::cout << "Score: " << score << "  cur error " << curErr << std::endl;
}

void KalmanFilter::initialize()
{
    m_x = NBMath::vector4D(10.0f, 0.0f, 0.f, 0.f);
    m_cov = boost::numeric::ublas::identity_matrix <float>(4);
}

void KalmanFilter::initialize(ufvector4 x_,
                              ufmatrix4 cov_)
{
    // references, not pointers so will copy values
    m_x = x_;
    m_cov = cov_;
}


void KalmanFilter::predictBallDest()
{
    if(m_stationary)
    {
        m_rel_x_dest = m_x(0);
        m_rel_y_dest = m_x(1);
    }
    else // moving
    {
        float speed = getSpeed();

        //Calculate time until stop
        float timeToStop = std::abs(speed / m_params.ballFriction);

        //Calculate deceleration in each direction
        float decelX = (m_x(2)/speed) * m_params.ballFriction;
        float decelY = (m_x(3)/speed) * m_params.ballFriction;

        // Calculate end position
        m_rel_x_dest = m_x(0) + m_x(2)*timeToStop + .5f*decelX*timeToStop*timeToStop;
        m_rel_y_dest = m_x(1) + m_x(3)*timeToStop + .5f*decelY*timeToStop*timeToStop;

        //Calculate the time until intersects with robots y axis
        float timeToIntersect = NBMath::getLargestMagRoot(m_x(0),m_x(2),.5f*decelX);
        // Use quadratic :(
        m_rel_y_intersect_dest = m_x(1) + m_x(3)*timeToStop + .5f*decelY*timeToStop*timeToStop;
    }

}

} // balltrack
} // man
