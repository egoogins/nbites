/**
 * @brief A class responsible for maintaing knowedge of lines on the field
 *        and comparing projectinos and observations
 *
 * @author EJ Googins <egoogins@bowdoin.edu>
 * @date   June 2013
 */

#pragma once

#include "FieldConstants.h"
#include "LocStructs.h"

namespace man {
namespace localization {

struct Point {
    float x,y;

    Point() {
        x = 0.f;
        y = 0.f;
    }

    Point(float x_, float y_)
    {
        x = x_;
        y = y_;
    }

    float distanceTo(Point p) {
        return std::sqrt((p.x-x)*(p.x-x) + (p.y-y)*(p.y-y));
    }
};

struct LineErrorMatch {
    float error;
    Point startMatch;
    Point endMatch;
};

struct Line {
    static const float ERROR = .1f;

    Point start, end;
    float slope;
    bool vert;

    Line(Point start_, Point end_) :
        start(start_),
        end(end_)
    {
        if(end.x == start.x) {
            vert = true;
            slope = 0.f;
        }
        else {
            slope = (end.y - start.y)/(end.x - start.x);
            vert = false;
        }
    }

    float length() {
        return std::sqrt((end.x - start.x)*(end.x - start.x) + (end.y - start.y)*(end.y - start.y));
    }

    Point midpoint() {
        Point midpoint = shiftDownLine(start, length()/2.f);
        if (!containsPoint(midpoint))
            midpoint =   shiftDownLine(start,-length()/2.f);
        if (!containsPoint(midpoint)) {
            std::cout << "Massive Precision Error in Line Struct" << std::endl;
            std::cout << "start " << start.x << " " << start.y << std::endl;
            std::cout << "end " << end.x << " " << end.y << std::endl;
            std::cout << "midpoint " << midpoint.x << " " << midpoint.y << std::endl;
        }
        return midpoint;
    }


    Point intersect(Line l) {
        // Determine if parallel
        float intX, intY;

        if (l.slope == slope)
            return Point(-1000000.f, -1000000.f); // return crazy value to reject intersect
         // is one vertical?
        else if(l.vert) {
            //intersect at x = l.start.x
            intX = l.start.x;
            intY = slope*(intX - start.x) + start.y;
        } // the other?
        else if(vert) {
            intX = start.x;
            intY = l.slope*(intX - l.start.x) + l.start.y;
        }
        else { // no special case, so just intersect them
            intX = ((slope*start.x)-(l.start.x*l.slope)+(l.start.y - start.y))/(slope-l.slope);
            intY = slope*(intX - start.x) + l.start.y;
        }

        //Sanity check on both lines
        Point intersection(intX, intY);
        if (l.containsPoint(intersection) && containsPoint(intersection))
            return intersection;
        else
            return Point(-1000000.f, -1000000.f);// return crazy value to reject intersect
    }

    bool containsPoint(Point p)
    {
        float off = std::fabs((start.y - p.y)*(end.x - p.x) - (end.y - p.y)*(start.x - p.x));
        if (vert) {
            // check on the line
            if(std::fabs(p.x - start.x) < ERROR)
                // check on the segment
                if((start.y <= p.y && p.y <= end.y) || (start.y >= p.y && p.y >= end.y))
                    return true;
            return false;
        }
       // check on the line
        else {
            if (std::fabs((start.y - p.y)*(end.x - p.x) - (end.y - p.y)*(start.x - p.x)) < ERROR) {
                // check on the segment
                if((start.x <= p.x && p.x <= end.x) || (start.x >= p.x && p.x >= end.x))
                    return true;
            }
            return false;
        }
    }

    /*
     * @brief - Calculate the point on the line segment closest to
     *          a given point in space
     */
    Point closestPointTo(Point p)
    {
        // l = start + t(end - start) for t = [0,1]
        // projection of p onto l given by:
        float n = NBMath::dotProduct(p.x-start.x,
                                     p.y - start.y,
                                     end.x - start.x,
                                     end.y - start.y);
        float d = NBMath::dotProduct(end.x - start.x,
                                     end.y - start.y,
                                     end.x - start.x,
                                     end.y - start.y);

        float t = n /d;
        if (t<0.f)
            return start;
        else if (t>1.f)
            return end;
        else {
            Point closest(start.x + t*(end.x - start.x), start.y + t*(end.y - start.y));
            return closest;
        }
    }

    /*
     * @brief - Project onto the line (not segment as seen in closestPoinTo)
     */
    Point project(Point p)
    {
        // l = start + t(end - start) for t = [0,1]
        // projection of p onto l given by:
        float n = NBMath::dotProduct(p.x-start.x,
                                     p.y - start.y,
                                     end.x - start.x,
                                     end.y - start.y);
        float d = NBMath::dotProduct(end.x - start.x,
                                     end.y - start.y,
                                     end.x - start.x,
                                     end.y - start.y);

        float t = n /d;
        Point closest(start.x + t*(end.x - start.x), start.y + t*(end.y - start.y));
        return closest;
    }

    LineErrorMatch getError(Line obsv, bool debug = false) {
        if (debug)
            std::cout << "\nGet Error" << std::endl;
        // New Strategy: project midpoint onto line and shift both ways equal to half length
        Point midpoint = obsv.midpoint();
        Point midpointProj = closestPointTo(midpoint);

        Point startProj = shiftDownLine(midpointProj,  obsv.length()/2); //start
        Point endProj = shiftDownLine(midpointProj, -obsv.length()/2); // end

        // check projections are close to the points they're 'projected' from
        if (startProj.distanceTo(obsv.end) < startProj.distanceTo(obsv.start)) {
            startProj = shiftDownLine(midpointProj, -obsv.length()/2);
            endProj   = shiftDownLine(midpointProj,  obsv.length()/2);
        }

        // if both arent on line, then return massive error (doesnt fit on line)
        if((!containsPoint(startProj)) && (!containsPoint(endProj))) {
            if (debug) {
                std::cout << "start proj " << startProj.x << " " << startProj.y << std::endl;
                std::cout << "end proj " << endProj.x << " " << endProj.y << std::endl;
                Line tooBig(startProj,endProj);
                std::cout << "Segment projected length " << tooBig.length() << std::endl;

                std::cout << "Doesnt fit on line" << std::endl;
            }

            LineErrorMatch shit;
            shit.error = 1000000.f;
            shit.startMatch = obsv.start;
            shit.endMatch   = obsv.end;
            return shit;
        }

        // if one isnt on line, shift the whole thing down
        if( !containsPoint(startProj) ) {
            // get distance off the line by
            float distOff = startProj.distanceTo(closestPointTo(startProj));

            // shift by that amount
            Point newStartProj = shiftDownLine(startProj, distOff);
            if (!containsPoint(newStartProj)) { // need to shift other way
                startProj = shiftDownLine(startProj, -distOff);
                endProj   = shiftDownLine(  endProj, -distOff);
            }
            else { // got it right
                startProj = shiftDownLine(startProj, distOff);
                endProj   = shiftDownLine(  endProj, distOff);
            }
        }
        else if( !containsPoint(endProj) ) {
            // get distance off the line by
            float distOff = endProj.distanceTo(closestPointTo(endProj));

            // shift by that amount
            Point newEndProj = shiftDownLine(endProj, distOff);
            if (!containsPoint(newEndProj)) { // need to shift other way
                startProj = shiftDownLine(startProj, -distOff);
                endProj   = shiftDownLine(  endProj, -distOff);
            }
            else { // got it right
                startProj = shiftDownLine(startProj, distOff);
                endProj   = shiftDownLine(  endProj, distOff);
            }
        }

        // if one is still not on line then return massive error
        if(!containsPoint(startProj) || !containsPoint(endProj)) {
            if (debug) {
                std::cout << "start proj " << startProj.x << " " << startProj.y << std::endl;
                std::cout << "end proj " << endProj.x << " " << endProj.y << std::endl;
                Line tooBig(startProj,endProj);
                std::cout << "Segment projected length " << tooBig.length() << std::endl;

                std::cout << "Doesnt fit on line 2" << std::endl;
            }

            LineErrorMatch shit;
            shit.error = 1000000.f;
            shit.startMatch = obsv.start;
            shit.endMatch   = obsv.end;
            return shit;
        }

        // matched segment onto this line, calculate area of given polygon
        // split into two trianges: obsvstart, start Proj, obsvend and
        //                          obsvEnd, endProj, startProj
        // UNLESS the lines intersect, in which case the triangles are:
        //                          obsvStart, startProj, intersect
        //                          obsvEnd, endProj, intersect

        //Ensure proper triangles
        // Whichever combination has the smallest projection distance we want to preserve
        float sToSP = obsv.start.distanceTo(startProj);
        float sToEP = obsv.start.distanceTo(endProj);
        float eToSP = obsv.end.distanceTo(startProj);
        float eToEP = obsv.end.distanceTo(endProj);

        // If s to end projection or end to start projection are min, switch the projections
        if (((sToEP < sToSP) && (sToEP < eToSP) && (sToEP < eToEP))
            || ((eToSP < sToSP) && (eToSP < sToEP) && (eToSP < eToEP))) {
            if(debug)
               std::cout << "SWITCH!" << std::endl;

            Point temp;
            temp.x = startProj.x;
            temp.y = startProj.y;

            startProj.x = endProj.x;
            startProj.y = endProj.y;

            endProj.x = temp.x;
            endProj.y = temp.y;
        }

        Line segmentMatched(startProj, endProj);

        Point intersect = segmentMatched.intersect(obsv);

        if (obsv.containsPoint(intersect) && containsPoint(intersect))
        {
            // obsv segment and matching segment intersect, so triangles are
            //                          obsvStart, startProj, intersect
            //                          obsvEnd, endProj, intersect

            float l1 = obsv.start.distanceTo(startProj);
            float l2 = startProj.distanceTo (intersect);
            float l3 = intersect.distanceTo (obsv.start);
            float area1 = NBMath::calcTriangleArea(l1, l2, l3);

            l1 = obsv.end.distanceTo (endProj);
            l2 = endProj.distanceTo  (intersect);
            l3 = intersect.distanceTo(obsv.end);
            float area2 = NBMath::calcTriangleArea(l1, l2, l3);

            // Return in units sqrt(cm), not cm^2 to scale it
            float error = std::sqrt(std::sqrt((area1 + area2)));
            //float error = (area1 + area2)/length();
            LineErrorMatch errorMatch;
            errorMatch.error = error;
            errorMatch.startMatch = startProj;
            errorMatch.endMatch   = endProj;

            if(debug) {
                std::cout << "Segment Matched " << segmentMatched.start.x << " " << segmentMatched.start.y << " "
                          << segmentMatched.end.x << " " << segmentMatched.end.y << std::endl;

                std::cout << "Obsv " << obsv.start.x << " " << obsv.start.y << " "
                          << obsv.end.x << " " << obsv.end.y << std::endl;
                std::cout << "Error " << errorMatch.error << std::endl;
            }

            return errorMatch;
        }
        else {
            // get side lengths of first triangle
            float l1 = obsv.start.distanceTo(startProj);
            float l2 = startProj.distanceTo (obsv.end);
            float l3 = obsv.end.distanceTo  (obsv.start);
            float area1 = NBMath::calcTriangleArea(l1, l2, l3);

            // get side lengths of second triangle
            l1 = obsv.end.distanceTo(endProj);
            l2 = endProj.distanceTo(startProj);
            l3 = startProj.distanceTo(obsv.end);
            float area2 = NBMath::calcTriangleArea(l1, l2, l3);

            // Return in units sqrt(cm), not cm^2
            float error = std::sqrt(std::sqrt((area1 + area2)));

            LineErrorMatch errorMatch;
            errorMatch.error = error;
            errorMatch.startMatch = startProj;
            errorMatch.endMatch   = endProj;

            if(debug) {
                std::cout << "Segment Matched " << segmentMatched.start.x << " " << segmentMatched.start.y << " "
                          << segmentMatched.end.x << " " << segmentMatched.end.y << std::endl;

                std::cout << "Obsv " << obsv.start.x << " " << obsv.start.y << " "
                          << obsv.end.x << " " << obsv.end.y << std::endl;
                std::cout << "Error " << errorMatch.error << std::endl;
            }

            return errorMatch;
        }
    }


    /*
     * @brief - Starting at given point, calculate point given dist along line from inital
     *          Not gaurenteed to be on line segment,
     *          -dist = left, +dist = right (assume slope is never vertical)
     * @assume - given point is on this line
     */
    Point shiftDownLine(Point initial, float dist) {
        // Shift in x is dist*cos((tan^-1(slope))
        // Calculated w/o trig w/ identities #wolfram
        if (vert) {
            Point vertShift(initial.x, initial.y - dist);
            return vertShift;
        }

        else {
            float dX = dist / std::sqrt((slope * slope) + 1.f);
            float shiftedX = initial.x + dX;
            float shiftedY = slope*(shiftedX - start.x) + start.y;
            Point shifted(shiftedX, shiftedY);
            return shifted;
        }
    }
};

typedef std::vector<Line> LineSet;
typedef LineSet::iterator LineIt;

class LineSystem {
public:
    LineSystem();
    virtual ~LineSystem();

    void addLine(float startX, float startY, float endX, float endY);

    float scoreObservation(Line globalObsv);
    LineErrorMatch scoreAndMatchObservation(Line globalObsv, bool debug = false);

private:
    LineSet lines;
};

} // namespace localization
} // namespace man
