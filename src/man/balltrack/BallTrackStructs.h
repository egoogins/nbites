#pragma once

#include "localization/LocStructs.h"

namespace man {
namespace balltrack {

using namespace localization;

struct KalmanFilterParams
{
    float transXDeviation;   //Deviation from odometry in x direction
    float transYDeviation;   //Deviation from odometry in y direction
    float rotationDeviation; //Deviation from odometry in rotation
    float obsvRelXVariance;  //Variance from observing a ball
    float obsvRelYVariance;  //Variance from observing a ball in y
    float processDeviationPosX;  //Deviation from difference between model & world
    float processDeviationPosY;
    float processDeviationVelX;
    float processDeviationVelY;
    float ballFriction;      // cm/sec^2
};

struct MMKalmanFilterParams
{
    int numFilters;          // Number of filters (even)
    int framesTillReset;   // Frames we can see a ball and not reset
    float initCovX;
    float initCovY;
    float initCovVelX;
    float initCovVelY;
    float movingThresh;
    int bufferSize;       // Size of buffer used to re-init moving filters
    float badStationaryThresh;
};

/*
 * @brief Struct to take observations from vision
 */
struct BallObservation
{
    BallObservation(float dist_, float bear_) : dist(dist_), bear(bear) {}
    BallObservation() {}
    float dist;
    float bear;
};

/*
 * @brief Struct to pass back Cartesian Ball Estimate
 */
struct CartesianBallEstimate
{
    CartesianBallEstimate(float relX_, float relY_) : relX(relX_), relY(relY_) {}
    CartesianBallEstimate() {}
    float relX;
    float relY;
};

/*
 * @brief Struct to use when storing visual history
 */
struct CartesianObservation
{
    CartesianObservation(float relX_, float relY_) : relX(relX_), relY(relY_) {}
    CartesianObservation() {}
    float relX;
    float relY;
};

struct KickDetector {
    bool full;
    int obsv;
    int BUFF_LENGTH;
    float ON_LINE_THRESH;
    float APPROACHING_THRESH;
    RingBuffer<Point> buff;

    // Note positive x in front, positive y to left
    // Values defining the line
    Line lineFromBuffer;
    float yIntercept;

    KickDetector() :
        buff(BUFF_LENGTH),
        BUFF_LENGTH(15),
        ON_LINE_THRESH(1000.f),
        APPROACHING_THRESH(2.f),
        full(false),
        obsv(0){
    }

    void resetFull() {
        obsv = 0;
        full = false;
    }

    bool checkForKick(float x, float y) {
        addObsv(x,y);
        if(areCollinear() )//&& moveThroughLine())
            return true;
        else
            return false;
    }

    void addObsv(float x, float y) {
        buff.add(Point(x,y));
        calculateLine();

        if(!full)
            obsv++;
        if (obsv>15)
            full = true;
    }

    // Update our line based on the newest buffer
    void calculateLine() {
        // Calculate average slope between
        float avgSlope = 0.f;
        for (int i=0; i<(BUFF_LENGTH-1); i++) {
            int pointA = (buff.curEntry - i)    % BUFF_LENGTH;
            int pointB = (buff.curEntry -(i+1)) % BUFF_LENGTH;
            Line temp(buff.buffer[pointA], buff.buffer[pointB]);
            avgSlope += temp.slope;
        }
        avgSlope = avgSlope/BUFF_LENGTH;

        // Create a line which goes through most recent obsv
        float onLineX = buff.get().x + 10.f;
        float onLineY = avgSlope*(10.f) + buff.get().y;
        Point onNewLine(onLineX, onLineY);
        Line newLine(buff.get(), onNewLine);
        lineFromBuffer = newLine;

        yIntercept = avgSlope*(-buff.get().x) + buff.get().y;
    }

    //@brief - determine if all points in buffer are on a line approx
    bool areCollinear() {
        // Loop through the buffer and return false if closest distance
        // to line is > threshold
        for(int i=0; i < BUFF_LENGTH; i++) {
            if (!lineFromBuffer.containsPoint(buff.buffer[i], ON_LINE_THRESH))
                return false;
        }

        // If got through the loop w/o returning then they're ~collinear
        return true;
    }

    // @brief - ensure the points move through the line, not jump back and forth (roughly)
    bool moveThroughLine() {
        // We only care if the ball is kicked to us
        // Ensure that the observations x is growing as we look farther back in time
        for(int i=0; i < (BUFF_LENGTH - 1); i++) {
            int closeX = ((buff.curEntry - i)    % BUFF_LENGTH);
            int   farX = ((buff.curEntry -(i+1)) % BUFF_LENGTH);

            if ((buff.buffer[farX].x - buff.buffer[closeX].x) > -APPROACHING_THRESH) {
                return false;
            }
        }

        return true;
    }

};


} // namespace balltrack
} // namespace man
