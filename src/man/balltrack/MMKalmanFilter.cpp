#include "MMKalmanFilter.h"

namespace man {
namespace balltrack {

/**
 * @ Brief- Constructor of my 'Puppetmaster'
 *          Grab the params, gen a bunch of filters to avoid nulls
 *          Set frames w/o ball high so we re-init based on initial observations
 *          Clearle consecutive observation is false if we havent seen the ball...
 *          State est and vis history can stay zero,
 *          Same with bestFilter, stationary, and lastUpdateTime
 */
MMKalmanFilter::MMKalmanFilter(MMKalmanFilterParams params_)
{
    m_params = params_;

    m_frames_without_ball = m_params.framesTillReset;
    m_consecutive_observation = false;
    m_best_filter = 0;
    m_obsv_buffer = new CartesianObservation[m_params.bufferSize];
    m_cur_entry = 0;
    m_full_buffer = false;

    m_prev_state_est = boost::numeric::ublas::zero_vector<float> (4);
    m_prev_cov_est   = boost::numeric::ublas::identity_matrix <float>(4);
    m_state_est     = boost::numeric::ublas::zero_vector<float> (4);
    m_cov_est       = boost::numeric::ublas::identity_matrix <float>(4);

    initialize();
}

/**
 * @Brief - #nomememoryleaks
 *           (hopefully)
 */
MMKalmanFilter::~MMKalmanFilter()
{
    // EJ--fixme?
    //delete &filters;
}

/**
 * @Brief - Main interface, takes in an update with a visionBall and a motion message
 *          Should be called whenever new information, and simply dont pass it the same
 *               message twice!
 * @params - visionball is a NEW vision message
           - motion is a NEW motion message
 */
void MMKalmanFilter::update(messages::VisionBall    visionBall,
                            messages::RobotLocation odometry)
{
    predictFilters(odometry);

    // Update with sensor if we have made an observation
    // sometimes get weird ball observatoins so check for valid dist (indicator)
    if(visionBall.on() && (visionBall.distance() > 5) && (visionBall.distance() < 800)) // We see the ball
    {
        //Before we mess with anything, decide if we saw the ball twice in a row
        m_consecutive_observation = (m_frames_without_ball == 0) ? true : false;

        // Update our visual observation history
        m_last_vis_rel_x = m_vis_rel_x;
        m_last_vis_rel_y = m_vis_rel_y;
        //Calc relx and rely from vision
        float sinB,cosB;
        sincosf(visionBall.bearing(),&sinB,&cosB);
        m_vis_rel_x = visionBall.distance()*cosB;
        m_vis_rel_y = visionBall.distance()*sinB;

        // Update our observation buffer for re-initializing the moving filter
        m_cur_entry = (m_cur_entry+1)%m_params.bufferSize;
        m_obsv_buffer[m_cur_entry] = CartesianObservation(m_vis_rel_x, m_vis_rel_y);

        if (!m_full_buffer && m_cur_entry==0)
        { // If buffer wasnt full but is now
            m_full_buffer = true;
        }

        // If we havent seen the ball for a long time, re-init our filters
        if (m_frames_without_ball >= m_params.framesTillReset)
        {
            // Reset the filters
            initialize(m_vis_rel_x, m_vis_rel_y, m_params.initCovX, m_params.initCovY);

            // Reset relevant variables
            m_frames_without_ball = 0;
        }

        // #HACK for competition - If we get into a bad observation cycle then change it
        else if (m_filters.at((unsigned)1)->getSpeed() > 700.f && m_consecutive_observation){
            initialize(m_vis_rel_x, m_vis_rel_y, m_params.initCovX, m_params.initCovY);
        }

        /*else if (fullBuffer) {
            // Calc velocity through these frames
            // Will reset worst stationary and worst moving filters
            CartesianObservation vel = calcVelocityOfBuffer();

            float speedThroughFrames = calcSpeed(vel.relX, vel.relY);

            // Calc diff between observation and estimata
            float estDiff = calcSpeed(visRelX - filters.at((unsigned)0)->getRelXPosEst(),
                                      visRelY - filters.at((unsigned)0)->getRelYPosEst());

            // Much higher than our current estimate
            // if ((speedThroughFrames > filters.at((unsigned)1)->getSpeed() + 60.f)
            //     && (estDiff > params.badStationaryThresh))
            //if(estDiff > params.badStationaryThresh)

            // If moving velocity <10, give this a try
            if (std::abs(filters.at((unsigned)1)->getSpeed()) < 10.f
                && speedThroughFrames > 10.f)
            {
                //std::cout << "\nBall Kicked!" << std::endl;
                ufvector4 newMovingX = filters.at((unsigned)0)->getStateEst();
                newMovingX(2) = vel.relX;
                newMovingX(3) = vel.relY;
                ufmatrix4 newMovingCov = boost::numeric::ublas::identity_matrix <float>(4);
                newMovingCov(0,0) = 10.f;
                newMovingCov(1,1) = 10.f;
                newMovingCov(2,2) = 20.f;
                newMovingCov(3,3) = 20.f;

                filters.at((unsigned)1)->initialize(newMovingX, newMovingCov);
            }
        }*/

        // Now correct our filters with the vision observation
        updateWithVision(visionBall);

        updatePredictions();
    }
    else {
        m_consecutive_observation = false;

        // don't use/wipeout buffer
        m_full_buffer = false;
        m_cur_entry = 0;
    }

    /*
    float stat_det = filters.at((unsigned)0)->getDetOfCov();
    float mov_det = filters.at((unsigned)1)->getDetOfCov();
    float stat_prob = 1 / (2 * PI * std::sqrt(stat_det));
    float mov_prob = 1 / (2 * PI * std::sqrt(mov_det));

    bestFilter = (stat_prob > mov_prob) ? 0 : 1; */

    // Choose the filter whose covariance has the highest probability at their estimate
    float highestProbability = 0;
    KalmanFilter* worstStationaryFilter = 0;
    KalmanFilter* worstMovingFilter = 0;
    for (std::vector<KalmanFilter *>::iterator it = m_filters.begin(); it != m_filters.end(); it++) {
        // Determine best, and worst filters
        if ((*it)->getProbAtMean() > highestProbability) {
            m_best_filter = *it;
        }

        // If can calculate estimate for recent velocity, re-init worst filters
        if (m_full_buffer) {
            if ((*it)->isStationary()) {
                if (!worstStationaryFilter ||
                        worstStationaryFilter->getProbAtMean() > (*it)->getProbAtMean()) {
                    worstStationaryFilter = *it;
                }
            } else { // is moving
                if (!worstMovingFilter ||
                        worstMovingFilter->getProbAtMean() > (*it)->getProbAtMean()) {
                    worstMovingFilter = *it;
                }
            }
        }
    }

    // Now update our estimates before housekeeping
    m_prev_state_est = m_state_est;
    m_prev_cov_est   = m_cov_est;

    m_state_est = m_best_filter->getStateEst();
    m_cov_est   = m_best_filter->getCovEst();

    // Housekeep
    m_frames_without_ball = (visionBall.on()) ? (0) : (m_frames_without_ball+1);
    m_stationary = m_best_filter->isStationary();

    // Re init worst filters
    if (worstStationaryFilter && worstMovingFilter) {
        // High uncertainty to start
        ufmatrix4 poorCov = boost::numeric::ublas::zero_matrix<float>(4);
        poorCov(0,0) = 30.f;
        poorCov(1,1) = 30.f;
        poorCov(2,2) = 30.f;
        poorCov(3,3) = 30.f;

        ufvector4 stationaryX = NBMath::vector4D(m_vis_rel_x, m_vis_rel_y, 0, 0);
        CartesianObservation vel = calcVelocityOfBuffer();
        ufvector4 movingX = NBMath::vector4D(m_vis_rel_x, m_vis_rel_y, vel.relX, vel.relY);

        worstStationaryFilter->initialize(stationaryX, poorCov);
        worstMovingFilter->initialize(movingX, poorCov);
    }
}

/**
 * @brief - In charge of cycling through the filters, finding the worst stationary
 *           and replacing it with a new init filter. Also re-inits a new moving filter
 *            if we have had two consecutive observations and can calculate velocity
 */
 void MMKalmanFilter::cycleFilters()
 {
     //Find the two worst filters
     int worstStationary = -1;
     int worstMoving = -1;
     for(unsigned i=0; i<filters.size(); i++)
     {
         if (filters.at(i)->isStationary()){
             if(worstStationary<0)
                 worstStationary = (int)i;
             else if (filters.at(i)->getWeight() < filters.at((unsigned) worstStationary)->getWeight())
                 worstStationary = (int)i;
         }
         else
             if(worstMoving<0)
                 worstMoving = (int)i;
             else if (filters.at(i)->getWeight() < filters.at((unsigned) worstMoving)->getWeight())
                 worstMoving = (int)i;
     }

     // Re-init the worst stationary filter
     ufvector4 newX = boost::numeric::ublas::zero_vector<float>(4);
     newX(0) = visRelX;
     newX(1) = visRelY;
     ufmatrix4 newCov = boost::numeric::ublas::zero_matrix<float>(4);
     newCov(0,0) = .5f;//params.initCovX;
     newCov(1,1) = .5f;//params.initCovY;
     filters.at((unsigned) worstStationary)->initialize(newX, newCov);

     // Re-init the worst moving filter if we can calc a velocity
     if (consecutiveObservation){
         newX(2) = (visRelX - lastVisRelX) / deltaTime;
         newX(3) = (visRelY - lastVisRelY) / deltaTime;

         // HACK - magic number. need this in master asap though
         newCov(2,2) = 30.f;
         newCov(3,3) = 30.f;

         filters.at((unsigned) worstMoving)->initialize(newX, newCov);
     }
 }

/*
 * @brief - Normalize the filter weights, pretty standard
 */
 unsigned MMKalmanFilter::normalizeFilterWeights(){
     // Calc sum of the weights to normalize
     float totalWeights = 0;
     for(unsigned i=0; i<filters.size(); i++)
     {
         float curWeight = filters.at(i)->getWeight();
         // Don't want approx zero or =0 weights
         if (curWeight < .000001)
             filters.at(i)->setWeight(.000001f);
         totalWeights += (curWeight);
     }

     // Normalize and choose the best one
     unsigned tempBestFilter = 0;
     float bestWeight = filters.at(0)->getWeight();
     for(unsigned i=1; i<filters.size(); i++)
     {
         float curWeight = filters.at(i)->getWeight();
         if (curWeight > bestWeight)
         {
             tempBestFilter = i;
             bestWeight = curWeight;
         }
         filters.at(i)->setWeight(curWeight/totalWeights);
     }

     return tempBestFilter;
 }

/**
 * @brief - Initialize all the filters!
 * @params- given a relX and relY for the position mean
 *          also a covX and covY since we may want to init
 *          w/ diff certainties throughout the life
 * @choice  I chose to have the velocities randomly initialized since there are
            soooo many combos
 */
void MMKalmanFilter::initialize(float relX, float relY, float covX, float covY)
{
    // clear the filters
    m_filters.clear();

    // make a random generator for initilizing different filters
    boost::mt19937 rng;
    rng.seed(std::time(0));
    boost::uniform_real<float> posCovRange(-2.f, 2.f);
    boost::variate_generator<boost::mt19937&,
                             boost::uniform_real<float> > positionGen(rng, posCovRange);
    boost::uniform_real<float> randVelRange(-30.f, 30.f);
    boost::variate_generator<boost::mt19937&,
                             boost::uniform_real<float> > velocityGen(rng, randVelRange);

    //make stationary
    for (int i=0; i<m_params.numFilters/2; i++)
    {
        // Needs to be stationary, have given mean, and add noise
        //   to the covariance matrix
        KalmanFilter *stationaryFilter = new KalmanFilter(true);
        ufvector4 x = boost::numeric::ublas::zero_vector<float>(4);
        x(0) = relX;
        x(1) = relY;
        x(2) = 0.f;
        x(3) = 0.f;

        ufmatrix4 cov = boost::numeric::ublas::zero_matrix<float>(4);
        cov(0,0) = covX + positionGen();
        cov(1,1) = covY + positionGen();

        // init and push it back
        stationaryFilter->initialize(x, cov);
        m_filters.push_back(stationaryFilter);
    }

    // make moving
    for (int i=0; i<m_params.numFilters/2; i++)
    {
        // Needs to be moving, have given mean, and add noise
        //   to the covariance matrix
        KalmanFilter *movingFilter = new KalmanFilter(false);
        ufvector4 x = boost::numeric::ublas::zero_vector<float>(4);
        x(0)= relX;
        x(1)= relY;
        x(2) = velocityGen();
        x(3) = velocityGen();

        // Choose to assum obsv mean is perfect and just have noisy velocity
        ufmatrix4 cov = boost::numeric::ublas::zero_matrix<float>(4);
        cov(0,0) = covX;
        cov(1,1) = covY;
        cov(2,2) = 20.f;
        cov(3,3) = 20.f;

        movingFilter->initialize(x, cov);
        m_filters.push_back(movingFilter);
    }
}

// for offline testing, need to be able to specify the time which passed
void MMKalmanFilter::predictFilters(messages::RobotLocation odometry, float t)
{
    m_delta_time = t;
    for (std::vector<KalmanFilter *>::iterator it = m_filters.begin(); it != m_filters.end(); it++)
    {
        (*it)->predict(odometry, m_delta_time);
    }
}
/**
 * @brief - Predict each of the filters given info on where robot has moved
 *          Grab delta time from the system and then call the predict on each filter
 *
 */
void MMKalmanFilter::predictFilters(messages::RobotLocation odometry)
{
    // Update the time passed
    updateDeltaTime();

    // Update each filter
    for (std::vector<KalmanFilter *>::iterator it = m_filters.begin(); it != m_filters.end(); it++)
        (*it)->predict(odometry, m_delta_time);
}

/**
 * @brief - Correct each filter given an observation
 *          Pretty straightforward...
 */
void MMKalmanFilter::updateWithVision(messages::VisionBall visionBall)
{
    for (std::vector<KalmanFilter *>::iterator it = m_filters.begin(); it != m_filters.end(); it++)
    {
        (*it)->updateWithObservation(visionBall);
    }
}

/**
 * @brief - update the filters predictions for where the ball will stop moving
 */
void MMKalmanFilter::updatePredictions()
{
    for (std::vector<KalmanFilter *>::iterator it = m_filters.begin(); it != m_filters.end(); it++)
        (*it)->predictBallDest();
}

/**
 * @brief - Update the delta time from the system. Delta time is in Seconds
 *
 */
void MMKalmanFilter::updateDeltaTime()
{
    // Get time since last update
    const long long int time = monotonic_micro_time(); //from common
    m_delta_time = static_cast<float>(time - m_last_update_time)/
        1000000.0f; // u_s to sec

    // Guard against a zero dt (maybe possible?)
    if (m_delta_time <= 0.0){
        m_delta_time = 0.0001f;
    }
    if (m_delta_time > 1)
        m_delta_time = .03f; // Guard against first frame issues

    m_last_update_time = time;
}

CartesianObservation MMKalmanFilter::calcVelocityOfBuffer()
{
    CartesianObservation calcVel;

    float sumVelX = 0;
    float sumVelY = 0;
    for(int i=1; i<m_params.bufferSize; i++)
    {
        sumVelX += (m_obsv_buffer[i].relX - m_obsv_buffer[i-1].relX) / m_delta_time;
        sumVelY += (m_obsv_buffer[i].relY - m_obsv_buffer[i-1].relY) / m_delta_time;
    }

    calcVel.relX = sumVelX / (float)m_params.bufferSize;
    calcVel.relY = sumVelY / (float)m_params.bufferSize;

    // SANITY CHECKS
    // Major Concern with calculating from a large history is we don't want our
    // calculation to be watered down by observations when the ball was stationary
    // So check if there is a drastic inconsistency (off by 100 cm/s) in the speed
    bool consistent = true;
    float estSpeed = calcSpeed(calcVel.relX, calcVel.relY);

    // Also check that we're getting these values from relatively close balls
    //    ie within 300 centimeters
    float dist = calcSpeed(m_obsv_buffer[m_cur_entry].relX, m_obsv_buffer[m_cur_entry].relY);
    if (dist > 300.f)
        consistent = false;

    for(int i=1; i<m_params.bufferSize && consistent; i++)
    {
        //current speed
        float curSpeed = calcSpeed((m_obsv_buffer[i].relX - m_obsv_buffer[i-1].relX)/m_delta_time,
                                   (m_obsv_buffer[i].relY - m_obsv_buffer[i-1].relY)/m_delta_time);

        if (diff(curSpeed,estSpeed) > 100)
            consistent = false;
    }

    if (consistent)
        return calcVel;
    else //wasnt consistent so return no velocity
        return CartesianObservation(0.f,0.f);
}

float MMKalmanFilter::diff(float a, float b)
{
    return std::abs(std::abs(a) - std::abs(b));
}

float MMKalmanFilter::calcSpeed(float a, float b)
{
    return std::sqrt(a*a + b*b);
}

} // balltrack
} // man
