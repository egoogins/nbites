/**
 * Multimodel Kalman Filter
 *
 * The idea is to have 2 filters, one assuming the ball is stationary,
 * one assuming the ball is moving. When the moving filter thinks the velocity
 * is above a threshold then we switch to using that filter for our estimates.
 * Otherwise we use the stationary filter
 *
 * The initial concept was to pull a B-Human and compute 12 filters, constantly
 * re-initializing the worst filters, but the inability to choose the best filter
 * consistently due to (obviously) noisy estimates lead to currently having 2.
 * There is still some legacy code from that attempt in case it wants to be
 * RESURRECTED! (spelling?)
 *
 */
#pragma once

#include "KalmanFilter.h"

#include "NBMath.h"

#include "BallModel.pb.h"
#include "Motion.pb.h"
#include "RobotLocation.pb.h"

#include <boost/shared_ptr.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/math/constants/constants.hpp>

#include <vector>

using namespace NBMath;

namespace man {
namespace balltrack {

static const MMKalmanFilterParams DEFAULT_MM_PARAMS =
{
    12,                 // numFilters
    500,                // framesTillReset
    10.f,               // initCovX
    10.f,               // initCovY
    25.f,               // initCovVelX
    25.f,               // initCovVelY
    35.f,               // threshold for ball is moving!
    7,                  // buffer size
    30.f                // badStationaryThresh
};

class MMKalmanFilter
{

public:
    MMKalmanFilter(MMKalmanFilterParams params_ = DEFAULT_MM_PARAMS);
    ~MMKalmanFilter();

    void update(messages::VisionBall visionBall,
                messages::RobotLocation odometry);

    ufvector4 getStateEst() { return m_state_est; }
    float getRelXPosEst() { return m_state_est(0); }
    float getRelYPosEst() { return m_state_est(1); }
    float getRelXVelEst() { return m_state_est(2); }
    float getRelYVelEst() { return m_state_est(3); }

    ufmatrix4 getCovEst() { return m_cov_est; }
    float getCovXPosEst() { return m_cov_est(0,0); }
    float getCovYPosEst() { return m_cov_est(1,1); }
    float getCovXVelEst() { return m_cov_est(2,2); }
    float getCovYVelEst() { return m_cov_est(3,3); }

    float getFilteredDist() { return m_best_filter->getFilteredDist(); }
    float getFilteredBear() { return m_best_filter->getFilteredBear(); }

    float getSpeed() { return m_best_filter->getSpeed(); }
    float getRelXDest() { return m_best_filter->getRelXDest(); }
    float getRelYDest() { return m_best_filter->getRelYDest(); }
    float getRelYIntersectDest() { return m_best_filter->getRelYIntersectDest(); }

    bool isStationary() { m_best_filter->isStationary(); }

    void printEst() { std::cout << "Filter Estimate:\n\t"
                                << "'Stationary' is\t" << m_stationary << "\n\t"
                                << "X-Pos:\t" << m_state_est(0) << "Y-Pos:\t" << m_state_est(1)
                                << "\n\t"
                                << "x-Vel:\t" << m_state_est(2) << "y-Vel:\t" << m_state_est(3)
                                << "\n\t"
                                << "Uncertainty x:\t" << m_cov_est(0,0) << "\t,\t"
                                << "y:\t" << m_cov_est(1,1) << std::endl;}

    void initialize(float relX=50.f, float relY=50.f, float covX=50.f, float covY=50.f);

protected:
    void predictFilters(messages::RobotLocation odometry);
    void predictFilters(messages::RobotLocation odometry, float t);
    void updateWithVision(messages::VisionBall visionBall);

    void updatePredictions();

    void updateDeltaTime();

    CartesianObservation calcVelocityOfBuffer();
    float diff(float a, float b);
    float calcSpeed(float a, float b);

    MMKalmanFilterParams m_params;

    std::vector<KalmanFilter*> m_filters;

    int m_frames_without_ball;

    ufvector4 m_prev_state_est;
    ufmatrix4 m_prev_cov_est;

    ufvector4 m_state_est;
    ufmatrix4 m_cov_est;

    // Keep track of the last couple observations
    CartesianObservation* m_obsv_buffer;
    int m_cur_entry;
    bool m_full_buffer;

    float m_vis_rel_x;
    float m_vis_rel_y;

    KalmanFilter* m_best_filter;

    bool m_stationary;
    bool m_consecutive_observation;

    float m_last_vis_rel_x;
    float m_last_vis_rel_y;


    // Keep track of real time passing for calculations
    long long int m_last_update_time;
    float m_delta_time;
};

} // balltrack
} // man
