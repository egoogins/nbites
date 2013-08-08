#include "LocalizationModule.h"

#include "Profiler.h"
#include "RoboCupGameControlData.h"
#include "ParticleFilter.h"
#include "DebugConfig.h"

namespace man {
namespace localization {

LocalizationModule::LocalizationModule()
    : portals::Module(),
      output(base()),
      particleOutput(base())
{
    locSystem = new ParticleFilter();
    // Chooose on the field looking up as a random initial
    locSystem->resetLocTo(110,658,-1.5);
}

LocalizationModule::~LocalizationModule()
{
    delete locSystem;
}

void LocalizationModule::update()
{
#ifndef OFFLINE
    // Modify based on control portal
    if (lastReset != resetInput.message().timestamp())
    {
        lastReset = resetInput.message().timestamp();
        locSystem->resetLocTo(resetInput.message().x(),
                                   resetInput.message().y(),
                                   resetInput.message().h());
    }
#endif

    curOdometry.set_x(motionInput.message().x());
    curOdometry.set_y(motionInput.message().y());
    curOdometry.set_h(motionInput.message().h());

#ifndef OFFLINE
    bool inSet = (STATE_SET == gameStateInput.message().state());
    // Update the Particle Filter with the new observations/odometry

    if (inSet)
        locSystem->update(curOdometry, visionInput.message(), ballInput.message());
    else
#endif
        locSystem->update(curOdometry, visionInput.message());

    // Update the locMessage and the swarm (if logging)
    portals::Message<messages::RobotLocation> locMessage(&locSystem->
                                                         getCurrentEstimate());
#if defined( LOG_LOCALIZATION) || defined(OFFLINE)
    portals::Message<messages::ParticleSwarm> swarmMessage(&locSystem->
                                                           getCurrentSwarm());
    particleOutput.setMessage(swarmMessage);
#endif

    output.setMessage(locMessage);
}

void LocalizationModule::run_()
{
    // Profiler
    PROF_ENTER(P_SELF_LOC);

    motionInput.latch();
    visionInput.latch();
#ifndef OFFLINE
    gameStateInput.latch();
    ballInput.latch();
    resetInput.latch();
#endif

    update();

    // Profiler
    PROF_EXIT(P_SELF_LOC);
}

} // namespace localization
} // namespace man
