/*
 * @brief  The localization module class. Takes input from motion and vision for
 *             calculations, also an inPortal for resetting. Uses a LocSystem for
 *             processing data, to change a Localization System used, alter the
 *             allocation in the constructor. Anything specific to a type of LocSystem should
 *             be checked for with a dynamic cast
 *
 * @author EJ Googins <egoogins@gmail.com>
 * @date   February 2013
 */
#pragma once

/** Messages **/
#include "RoboGrams.h"
#include "VisionField.pb.h"
#include "RobotLocation.pb.h"
#include "GameState.pb.h"
#include "BallModel.pb.h"
#include "ParticleSwarm.pb.h" // specific to Particle Filter

/** Filter Headers **/
#include "LocSystem.h"

namespace man {
namespace localization{

/**
 * @class LocalizationModule
 */
class LocalizationModule : public portals::Module
{
public:
    LocalizationModule();
    ~LocalizationModule();

    /** In Portals **/
    portals::InPortal<messages::RobotLocation> motionInput;
    portals::InPortal<messages::VisionField>   visionInput;
    portals::InPortal<messages::RobotLocation> resetInput;
    portals::InPortal<messages::GameState>     gameStateInput;
    portals::InPortal<messages::FilteredBall>  ballInput;

    /** Out Portals **/
    portals::OutPortal<messages::RobotLocation> output;
    portals::OutPortal<messages::ParticleSwarm> particleOutput; // Specific to PF

protected:
    /**
     * @brief Update inputs, calculate new state of the filter
     */
    void run_();

    /**
     * @brief Updates the current robot pose estimate given
     *        the most recent motion control inputs and
     *        measurements taken.
     */
    void update();

private:
    // The localization system instance
    LocSystem* locSystem;

    // Record the last time the localization was reset
    long long lastReset;
};

} // namespace localization
} // namespace man
