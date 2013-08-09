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
    // NOTE: Behaviors resets localization appropriately, this is just a safety net
    locSystem->resetLocTo(110,658,-1.5);
}

LocalizationModule::~LocalizationModule()
{
    delete locSystem;
}

void LocalizationModule::update()
{
#ifndef OFFLINE
    // Update the estimate based on reset portal
    if (lastReset != resetInput.message().timestamp())
    {
        lastReset = resetInput.message().timestamp();
        locSystem->resetLocTo(resetInput.message().x(),
                                   resetInput.message().y(),
                                   resetInput.message().h());
    }

    // Determine if we're in set (should use ball observations)
    bool inSet = (STATE_SET == gameStateInput.message().state());

    // Update the Particle Filter with the new observations/odometry
    // Use ball information if !OFFLINE & in set
    // Don't use ball information if we're offline or not in set
    if (inSet)
        locSystem->update(motionInput.message(), visionInput.message(), ballInput.message());
    else
#endif // OFFLINE
        locSystem->update(motionInput.message(), visionInput.message());

    // Update the locMessage
    portals::Message<messages::RobotLocation> locMessage(&locSystem->getCurrentEstimate());

    output.setMessage(locMessage);
}

void LocalizationModule::run_()
{
    // Profiler
    PROF_ENTER(P_SELF_LOC);

    motionInput.latch();
    visionInput.latch();

    // Inputs that are only on the robot
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
