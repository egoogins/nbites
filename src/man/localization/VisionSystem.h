/**
 * @brief  Define a class responsible for updating particle swarm based on
 *         visual observations
 *
 * @author EJ Googins <egoogins@bowdoin.edu>
 * @date   February 2013
 */

#pragma once

#include "BallModel.pb.h"
#include "VisionField.pb.h"
#include "Particle.h"
#include "LineSystem.h"
#include <list>

#include <boost/math/distributions.hpp>

namespace man {
namespace localization {

static const float MIN_LINE_LENGTH = 100.f;

/**
 * @class Vision System
 * @brief Responsible for updating particles based on Visual Observations
 *        from the vision system.
 */
class VisionSystem
{
public:
    VisionSystem();
    ~VisionSystem();

    // updates given particleset by reference, returns true if observations was nonempty
    bool update(ParticleSet& particles,
                const messages::VisionField& obsv);

    // overloaded to use ball observations
    bool update(ParticleSet& particles,
                const messages::VisionField& obsv,
                const messages::FilteredBall& ballObsv);

    float scoreFromVisDetect(const Particle& particle,
                             const messages::VisualDetection& obsv);

    float scoreFromBallObsv(const Particle& particle,
                            const messages::FilteredBall& obsv);

    static Line prepareVisualLine(const messages::RobotLocation& loc,
                                  const messages::VisualLine& visualLine,
                                  bool stdLineLength = false);

    void setUpdated(bool val);
    float getAvgError(){return avgError;};

    float getConfidenceError(const messages::RobotLocation& loc,
                             const messages::VisionField& obsv);

    std::list<ReconstructedLocation> getReconstructedLocations(){return reconstructedLocations;};

private:
    // Functions to calculate possible positions observation viewed from
    void addCornerReconstructionsToList(messages::VisualCorner corner);
    void addGoalPostReconstructionsToList(messages::VisualGoalPost leftPost,
                                          messages::VisualGoalPost rightPost);

    /** Calculate the error of a particle with respect to observations
     *
     * @param[in]  particle The particle with the hypothesized pose
     * @param[in]  obsv The vision field observations to be scored against
     * @param[out] curParticleError the particles accumulated arror in this frame
     * @param[out] numObsv The number of observations in the protocol buffer
     */
    void updateParticleWithField(Particle& particle,
                                 const messages::VisionField& obsv,
                                 float& curParticleError,
                                 int& numObsv);

    /** Calculate the error of a particle with respect to observations
     *
     * @param[in]  particle The particle with the hypothesized pose
     * @param[in]  obsv The vision field observations to be scored against
     * @param[in]  ballObsv The ball observations to be scored against
     * @param[out] curParticleError the particles accumulated arror in this frame
     * @param[out] numObsv The number of observations in the protocol buffer
     */
    void updateParticleWithFieldAndBall(Particle& particle,
                                                      const messages::VisionField& obsv,
                                                      const messages::FilteredBall& ballObsv,
                                                      float& curParticleError,
                                                      int& numObsv);

    void optimizeReconstructions();

private:
    // Random number generator to be used throughout the system
    boost::mt19937 rng;

    // Utilize line observations
    LineSystem* lineSystem;

    // Maintain a list of possible locations we could be based on corner observations
    std::list<ReconstructedLocation> reconstructedLocations;

    // Average error between predictions and observations from most recent frame
    float avgError;

    // Keep track of goal observations to indicate corner id confidence
    bool sawGoal;
};

} // namespace localization
} // namespace man
