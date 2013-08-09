/**
 * @brief  Define a class responsible for updating particles based on an
 *         odometry measurement from the motion system
 *
 * @author EJ Googins <egoogins@bowdoin.edu>
 * @date   February 2013
 */

#pragma once

// Function Parameters
#include "RobotLocation.pb.h"
#include "Particle.h"

namespace man {
namespace localization {

class MotionSystem
{
public:
    MotionSystem(float xAndYNoise_, float hNoise_);
    ~MotionSystem();

    void update(ParticleSet& particles,
                const messages::RobotLocation& deltaMotionInfo,
                float error);

    void noiseShiftWithOdo(Particle* particle, float dX, float dY, float dH, float error);

    void setFallen(bool fallen);

    void resetNoise(float xyNoise_, float hNoise_);

    boost::mt19937 rng;
private:
    float xAndYNoise;
    float hNoise;

    bool robotFallen;

    messages::RobotLocation lastOdometry;
};
} // namespace localization
} // namespace man
