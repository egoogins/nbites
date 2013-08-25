/**
 * @brief Defines the required interface for a localization system. This
 *            polymorphism is most notable used in the LocalizationModule
 *            so that all which would be required to use a different LocSystem
 *            is a modification the the allocated system in it's constructor. Any
 *            functions important enough to be used in the LocalizationModule
 *            should be abstractly defined here.
 *
 * @author EJ Googins <egoogins@bowdoin.edu>
 * @date February 2013
 * @date August 2013 (refactor)
 */
#pragma once

/** Message Passing Architecture **/
#include "RoboGrams.h"

/** Function Parameters **/
#include "LocStructs.h"
#include "VisionField.pb.h"
#include "RobotLocation.pb.h"
#include "BallModel.pb.h"

namespace man {
namespace localization {

class LocSystem
{
public:
    /** Core Functions **/

    // Update the estimate with current world observations
    virtual void update(const messages::RobotLocation& motionInput,
                        const messages::VisionField& visionInput) = 0;
    virtual void update(const messages::RobotLocation& motionInput,
                        const messages::VisionField& visionInput,
                        const messages::FilteredBall& ballInput) = 0;

    // Should reset the estimate to a specific pose, also flipping the pose
    // if the x,y,h are negative
    virtual void resetLocTo(float x, float y, float h,
                            LocNormalParams params = LocNormalParams()) = 0;

    // Should reset the estimate to the most likely location on a given side
    virtual void resetLocToSide(bool defendingSide) = 0;

    /** Pose Estimatee Getters **/
    virtual const messages::RobotLocation& getCurrentEstimate() const = 0;
    virtual float getXEst() const = 0;
    virtual float getYEst() const = 0;
    virtual float getHEst() const = 0;
    virtual float getHEstDeg() const = 0;
    virtual bool onDefendingSide() const = 0;

    // Overload print operation //
    friend std::ostream& operator<< (std::ostream &o,
                                     const LocSystem &s)
    {
        return o << "Est: (" << s.getXEst() << ", " << s.getYEst() << ", "
                 << s.getHEst() << ")\n";
    }

};

} // namespace localization
} // namespace man
