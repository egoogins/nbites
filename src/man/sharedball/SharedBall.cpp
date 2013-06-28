#include "SharedBall.h"

namespace man {
namespace context {

SharedBallModule::SharedBallModule() :
    portals::Module(),
    sharedBallOutput(base())
{
    x = CENTER_FIELD_X;
    y = CENTER_FIELD_Y;

    alphaGrowth = 0.f;
    growthRate = (1-ALPHA)/ (float)FRAMES_BEFORE_RESET;

    framesSinceUpdate = 0;
}

SharedBallModule::~SharedBallModule()
{
}

void SharedBallModule::run_() {
    updatesThisFrame = 0;
    sumFrameX = 0;
    sumFrameY = 0;

    for (int i=0; i<NUM_PLAYERS_PER_TEAM; i++) {
        worldModelIn[i].latch();
        if(i == 0) {
            incorporateGoalieWorldModel(worldModelIn[i].message());
            if(updatesThisFrame>0) // if goalie saw it, trust in the goalie
                break;
        }
        else
            incorporateWorldModel(worldModelIn[i].message());
    }

    if (updatesThisFrame > 0) {
        // Update estimate
        x = (ALPHA+alphaGrowth)*x +
            (1-(ALPHA+alphaGrowth))*(sumFrameX/(float)updatesThisFrame);
        y = (ALPHA+alphaGrowth)*y +
            (1-(ALPHA+alphaGrowth))*(sumFrameY/(float)updatesThisFrame);

        // housekeep
        framesSinceUpdate = 0;
        alphaGrowth = 0;
    }
    else {
        framesSinceUpdate++;
        if( alphaGrowth < ((1-ALPHA) - growthRate))
            alphaGrowth += growthRate;
    }

    portals::Message<messages::SharedBall> sharedBallMessage(0);
    sharedBallMessage.get()->set_x(x);
    sharedBallMessage.get()->set_y(y);
    sharedBallMessage.get()->set_age(framesSinceUpdate);
    sharedBallOutput.setMessage(sharedBallMessage);
}

void SharedBallModule::incorporateWorldModel(messages::WorldModel newModel) {
    if(newModel.ball_on() && (newModel.my_uncert() < 5.f)) {
        // heading + bearing
        float hb = TO_RAD*newModel.my_h() + TO_RAD*newModel.ball_bearing();
        float sinHB, cosHB;
        sincosf(hb, &sinHB, &cosHB);

        float globalX = newModel.my_x() + newModel.ball_dist()*cosHB;
        float globalY = newModel.my_y() + newModel.ball_dist()*sinHB;

        sumFrameX += (ALPHA+alphaGrowth)*globalX + (1-(ALPHA+alphaGrowth))*x;
        sumFrameY += (ALPHA+alphaGrowth)*globalY + (1-(ALPHA+alphaGrowth))*y;
        updatesThisFrame++;
    }
}

void SharedBallModule::incorporateGoalieWorldModel(messages::WorldModel newModel) {
    // Assume goalie in position (FIELD_WHITE_LEFT_SIDELINE,
    //                            CENTER_FIELD_Y,
    //                            HEADING_RIGHT
    if(newModel.ball_on()) {
        // heading + bearing
        float hb = TO_RAD*HEADING_RIGHT + TO_RAD*newModel.ball_bearing();
        float sinHB, cosHB;
        sincosf(hb, &sinHB, &cosHB);

        float globalX = FIELD_WHITE_LEFT_SIDELINE_X + newModel.ball_dist()*cosHB;
        float globalY = CENTER_FIELD_Y + newModel.ball_dist()*sinHB;

        sumFrameX += (ALPHA+alphaGrowth)*globalX + (1-(ALPHA+alphaGrowth))*x;
        sumFrameY += (ALPHA+alphaGrowth)*globalY + (1-(ALPHA+alphaGrowth))*y;
        updatesThisFrame++;
    }
}

} // namespace man
} // namespace context
