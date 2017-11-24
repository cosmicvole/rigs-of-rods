#ifdef USE_ANGELSCRIPT

#include "CollisionAvoidance_GridQuad.h"
#include "OgreSubsystem.h"
#include "Beam.h"
#include "BeamFactory.h"
#include "BeamEngine.h"
#include "RoRFrameListener.h"
#include "VehicleAI.h"

using namespace Ogre;

CollisionAvoidance_GridQuad::CollisionAvoidance_GridQuad() : m_sim_controller(nullptr)
{
    frontLeft = Vector3::ZERO;
    frontRight = Vector3::ZERO;
    rearLeft = Vector3::ZERO;
    rearRight = Vector3::ZERO;
    pLeftQuad = nullptr;
    pRightQuad = nullptr;
    pFrontQuad = nullptr;
    pRearQuad = nullptr;
    waypointIDAtRear = -1;
    waypointIDInFront = -1;    
}

CollisionAvoidance_GridQuad::~CollisionAvoidance_GridQuad()
{
    
}

PredictedTruckPosition/*&*/ CollisionAvoidance_GridQuad::AddVehiclePosition(int truckNum, float time)
{
    for (int i=0; i<trucks.size(); i++)
    {
        PredictedTruckPosition ptp = trucks[i];
        if (ptp.truckNum == truckNum)
        {
            if (time < ptp.minTime)
            {
                ptp.minTime = time;
            }
            if (time > ptp.maxTime)
            {
                ptp.maxTime = time;
            }
            trucks[i] = ptp;
            return ptp;
        }
    }
    PredictedTruckPosition ptpNew;
    ptpNew.minTime = time;
    ptpNew.maxTime = time;
    ptpNew.truckNum = truckNum;
    ptpNew.pGridQuad = this;
    trucks.push_back(ptpNew);
    return trucks[trucks.size()-1];
}

PredictedTruckPosition/*&*/ CollisionAvoidance_GridQuad::AddVehiclePosition(int truckNum, float startTime, float endTime)
{
    for (int i=0; i<trucks.size(); i++)
    {
        PredictedTruckPosition ptp = trucks[i];
        if (ptp.truckNum == truckNum)
        {
            if (startTime < ptp.minTime)
            {
                ptp.minTime = startTime;
            }
            if (startTime > ptp.maxTime)
            {
                ptp.maxTime = startTime;
            }
            if (endTime < ptp.minTime)
            {
                ptp.minTime = endTime;
            }
            if (endTime > ptp.maxTime)
            {
                ptp.maxTime = endTime;
            }
            trucks[i] = ptp;
            return ptp;
        }
    }
    PredictedTruckPosition ptpNew;
    ptpNew.minTime = startTime;
    ptpNew.maxTime = endTime;
    ptpNew.truckNum = truckNum;
    ptpNew.pGridQuad = this;
    trucks.push_back(ptpNew);
    return trucks[trucks.size()-1];
}

void CollisionAvoidance_GridQuad::RemoveVehicle(int truckNum)
{
    for (int i=0; i<trucks.size(); i++)
    {
        if (trucks[i].truckNum == truckNum)
        {
            trucks.erase(trucks.begin() + i);
            //We rely on trucks only being in the vector once - cosmic vole May 14 2017
            return;
        }
    }
    return;
}

void CollisionAvoidance_GridQuad::Clear()
{
    trucks.clear();
}

void CollisionAvoidance_GridQuad::ClearOldPositions(float cutoffTime)
{
    for (int i=0; i<trucks.size(); i++)
    {
        PredictedTruckPosition ptp = trucks[i];
        if (ptp.maxTime < cutoffTime)
        {
            trucks.erase(trucks.begin() + i);
        }
    }
}

bool CollisionAvoidance_GridQuad::IsOccupied(float startTime, float endTime, int ignoreTruckNum)
{
    //TODO if we keep trucks sorted into startTime order, we can speed this search up by giving up once startTime > ptp.maxTime - cosmic vole May 14 2017
    for (int i=0; i<trucks.size(); i++)
    {
        PredictedTruckPosition ptp = trucks[i];
        if ((ignoreTruckNum < 0 || ptp.truckNum != ignoreTruckNum) && startTime <= ptp.maxTime && endTime >= ptp.minTime)
        {
            return true;
        }
    }
    return false;
}

bool CollisionAvoidance_GridQuad::IsLaneOccupied(float inFront, float behind, float startTime, float endTime, int ignoreTruckNum)
{
    //To debug this, what we need to do is log by lane number what each lane was last occupied with (copies of the old PredictedTruckPositions)
    //THEN when it's suddenly showing as NOT occupied, log where those trucks have gone now AND what's in the grid quads they were AND the ranges of quads checked by the lane
    //Could it be something to do with the lane number changing based on the starting quad??? Surely not
    static std::map<int, std::vector<PredictedTruckPosition>> debugLanesLastOccupants;
    int debugNumQuadsExamined = 0;
    float distInFront = 0.0f;
    float distBehind = 0.0f;
    int debugLaneNumber = this->GetLaneNumber();
    if (this->IsOccupied(startTime, endTime, ignoreTruckNum))
    {
        debugLanesLastOccupants[debugLaneNumber] = this->GetVehicles();
        return true;
    }

    debugNumQuadsExamined++;
    if (inFront > 0.0f)
    {
        CollisionAvoidance_GridQuad * pQuad = this->pFrontQuad;
        Vector3 lastRearLeft = (pQuad) ? pQuad->rearLeft : Vector3::ZERO;
        Vector3 lastRearRight = (pQuad) ? pQuad->rearRight : Vector3::ZERO;
        while (pQuad)
        {
            float quadLength = ((pQuad->rearLeft - lastRearLeft + pQuad->rearRight - lastRearRight) * 0.5f).length();
            distInFront += quadLength;
            if (distInFront > inFront)
            {
                //LOG("Dist in front is too far. Num quads examined: " + TOSTRING(debugNumQuadsExamined) + " Quad Length: " + TOSTRING(quadLength) + " rearLeft: " + TOSTRING(rearLeft) + " lastRearLeft: " + TOSTRING(lastRearLeft) + ".");
                break;
            }
            lastRearLeft = pQuad->rearLeft;
            lastRearRight = pQuad->rearRight;
            if (pQuad->IsOccupied(startTime, endTime, ignoreTruckNum))
            {
                debugLanesLastOccupants[debugLaneNumber] = pQuad->GetVehicles();
                return true;
            }
            debugNumQuadsExamined++;
            pQuad = pQuad->pFrontQuad;
        }
    }
    if (behind > 0.0f)
    {
        CollisionAvoidance_GridQuad * pQuad = this->pRearQuad;
        Vector3 lastRearLeft = (pQuad) ? pQuad->rearLeft : Vector3::ZERO;
        Vector3 lastRearRight = (pQuad) ? pQuad->rearRight : Vector3::ZERO;
        while (pQuad)
        {
            float quadLength = ((pQuad->rearLeft - lastRearLeft + pQuad->rearRight - lastRearRight) * 0.5f).length();
            distBehind += quadLength;
            if (distBehind > behind)
            {
                //LOG("Dist behind is too far. Num quads examined: " + TOSTRING(debugNumQuadsExamined) + " Quad Length: " + TOSTRING(quadLength) + " rearLeft: " + TOSTRING(rearLeft) + " lastRearLeft: " + TOSTRING(lastRearLeft) + ".");
                break;
            }
            lastRearLeft = pQuad->rearLeft;
            lastRearLeft = pQuad->rearRight;
            if (pQuad->IsOccupied(startTime, endTime, ignoreTruckNum))
            {
                debugLanesLastOccupants[debugLaneNumber] = pQuad->GetVehicles();
                return true;
            }
            debugNumQuadsExamined++;
            pQuad = pQuad->pRearQuad;
        }        
    }
    #if 0
    //Debug whether it was occupied before and what with:
    //if (debugLanesLastOccupants.size() > 0)
    //{
    //    LOG("Num quads examined: " + TOSTRING(debugNumQuadsExamined) + " Truck Num now: " + TOSTRING(ignoreTruckNum));
    //}
    std::vector<PredictedTruckPosition> lastOccupants = debugLanesLastOccupants[debugLaneNumber];
    //LOG("Last occupants count for lane 0 (cur lane is " + TOSTRING(debugLaneNumber) + ") = " + TOSTRING(debugLanesLastOccupants[0].size()));
    //LOG("Last occupants count for lane 1 (cur lane is " + TOSTRING(debugLaneNumber) + ") = " + TOSTRING(debugLanesLastOccupants[1].size()));
    //LOG("Last occupants count for lane 2 (cur lane is " + TOSTRING(debugLaneNumber) + ") = " + TOSTRING(debugLanesLastOccupants[2].size()));
    for (int i = 0; i < lastOccupants.size(); i++)
    {
        PredictedTruckPosition ptp = lastOccupants[i];
        int truckNum = ptp.truckNum;
        if (truckNum == ignoreTruckNum)
            continue;
        //OK, so where is that truck now?!
        if (endTime - ptp.maxTime > 4000.0f)
        {
            LOG("ptp.maxTime of " + TOSTRING(ptp.maxTime) + " is too old for endTime " + TOSTRING(endTime) + ".");
            //This is OLD
            continue;
        }
        
        Beam *truck = m_sim_controller->GetBeamFactory()->getTruck(truckNum);
        //TODO Get path and position and examine! Get current lane number for it!
        VehicleAI *truckAI = truck->getVehicleAI();
        if (!truckAI)
        {
            LOG("truckAI null in IsLaneOccupied() " + TOSTRING(truckNum));
            continue;
        }
        const std::vector<PredictedTruckPosition>& truckPath = truckAI->GetPredictedVehiclePath();
        if (truckPath.size() == 0)
        {
            LOG("Lane's last occupant's truck path is empty! " + TOSTRING(truckNum));
            continue;
        }
        float minDistInTimeRange = -1000000000.0f;
        float minTimeInDistRange = -1000000000.0f;
        std::map<int, bool> laneNums;
        for (int j = 0; j < truckPath.size(); j++)
        {
            PredictedTruckPosition ptp2 = truckPath[j];
            if (!ptp2.pGridQuad)
            {
                LOG("Grid Quad " + TOSTRING(j) + " is NULL for truck " + TOSTRING(truckNum));
                continue;
            }
            if (ptp2.pGridQuad == this)
            {
                LOG("THIS GridQuad is in truck " + TOSTRING(truckNum) + "'s path at time (" + TOSTRING(ptp2.minTime) + " - " + TOSTRING(ptp2.maxTime) + ") but lane seems to be empty.");
            }
            //Debug how far off this truck position is from the time and distance ranges we have been checking
            float dist = (ptp2.pGridQuad->GetLaneCenterRear() - this->GetLaneCenterRear()).length();
            if (ptp2.minTime <= endTime && ptp2.maxTime >= startTime)
            {
                if ((minDistInTimeRange == -1000000000.0f) || (fabsf(dist) < fabsf(minDistInTimeRange)))
                    minDistInTimeRange = dist;
            }
            else if ((dist >= 0.0f && dist <= distInFront) || (dist < 0.0f && dist >= -distBehind))
            {
                float timeOffset = 0.0f;
                if (ptp2.minTime > endTime)
                    timeOffset = ptp2.minTime - endTime;
                else
                    timeOffset = startTime - ptp2.maxTime;
                if ((abs(timeOffset) < abs(minTimeInDistRange)) || (minTimeInDistRange == -1000000000.0f))
                    minTimeInDistRange = timeOffset;
            }
            
            int laneNum = truckPath[j].pGridQuad->GetLaneNumber();
            laneNums[laneNum] = true;
        }
        std::map<int, bool>::iterator iter;
        LOG("Lane " + TOSTRING(debugLaneNumber) + "'s last occupant " + TOSTRING(truckNum) + " is now going over lanes: ");
        for (iter = laneNums.begin(); iter != laneNums.end(); ++iter)
        {
            if (iter->second)
                LOG("   " + TOSTRING(iter->first));
        }
        LOG("Min dist in time range: " + TOSTRING(minDistInTimeRange) + " min time in dist range: " + TOSTRING(minTimeInDistRange) + ".");
        LOG("But the lane seems to be empty!");
        
    }
    #endif
    debugLanesLastOccupants[debugLaneNumber].clear();
    //End debug
    
    return false;
}

bool CollisionAvoidance_GridQuad::LaneHasSlowerVehicleInFront(float inFront, float startTime, float endTime, float maxSpeed, bool ignoreTrucksUsingOtherLanes, int ignoreTruckNum)
{
    //static std::map<int, std::vector<PredictedTruckPosition>> debugLanesLastOccupants;
    //int debugNumQuadsExamined = 0;
    float distInFront = 0.0f;
    float distBehind = 0.0f;
    int laneNumber = this->GetLaneNumber();
    
    if (this->IsOccupied(startTime, endTime, ignoreTruckNum))
    {
        std::vector<PredictedTruckPosition> vehicles = this->GetVehicles();
        for (int i = 0; i < vehicles.size(); i++)
        {
            PredictedTruckPosition ptp = vehicles[i];
            if (ptp.minTime <= endTime && ptp.maxTime >= startTime && ptp.truckNum != ignoreTruckNum)
            {
                Beam *truck = m_sim_controller->GetBeamFactory()->getTruck(ptp.truckNum);
                if (truck && (maxSpeed < 0 || truck->getVelocity().length() <= maxSpeed))
                {
                    if (ignoreTrucksUsingOtherLanes)
                    {
                        VehicleAI *truckAI = truck->getVehicleAI();
                        if (truckAI)
                        {
                            if (truckAI->lane_change_end_wpt < truckAI->current_waypoint_id || truckAI->lane_change_start_wpt > truckAI->current_waypoint_id + 25)
                            {
                                //The truck is not holding this lane. It is following the racing line.
                                continue;
                            }
                            else
                            {
                                //The slower truck is holding, or moving into, this lane
                                if (truckAI->lane_change_to == laneNumber)
                                    return true;
                            }
                        }
                    }
                    else
                    {
                        return true;
                    }
                }
            }
        }
    }

    //debugNumQuadsExamined++;
    if (inFront > 0.0f)
    {
        CollisionAvoidance_GridQuad * pQuad = this->pFrontQuad;
        Vector3 lastRearLeft = (pQuad) ? pQuad->rearLeft : Vector3::ZERO;
        Vector3 lastRearRight = (pQuad) ? pQuad->rearRight : Vector3::ZERO;
        while (pQuad)
        {
            float quadLength = ((pQuad->rearLeft - lastRearLeft + pQuad->rearRight - lastRearRight) * 0.5f).length();
            distInFront += quadLength;
            if (distInFront > inFront)
            {
                //LOG("Dist in front is too far. Num quads examined: " + TOSTRING(debugNumQuadsExamined) + " Quad Length: " + TOSTRING(quadLength) + " rearLeft: " + TOSTRING(rearLeft) + " lastRearLeft: " + TOSTRING(lastRearLeft) + ".");
                break;
            }
            lastRearLeft = pQuad->rearLeft;
            lastRearRight = pQuad->rearRight;
            if (pQuad->IsOccupied(startTime, endTime, ignoreTruckNum))
            {
                //debugLanesLastOccupants[debugLaneNumber] = pQuad->GetVehicles();
                std::vector<PredictedTruckPosition> vehicles = this->GetVehicles();
                for (int i = 0; i < vehicles.size(); i++)
                {
                    PredictedTruckPosition ptp = vehicles[i];
                    if (ptp.minTime <= endTime && ptp.maxTime >= startTime && ptp.truckNum != ignoreTruckNum)
                    {
                        Beam *truck = m_sim_controller->GetBeamFactory()->getTruck(ptp.truckNum);
                        if (truck && (maxSpeed < 0 || truck->getVelocity().length() <= maxSpeed))
                        {
                            if (ignoreTrucksUsingOtherLanes)
                            {
                                VehicleAI *truckAI = truck->getVehicleAI();
                                if (truckAI)
                                {
                                    if (truckAI->lane_change_end_wpt < truckAI->current_waypoint_id || truckAI->lane_change_start_wpt > truckAI->current_waypoint_id + 25)
                                    {
                                        //The truck is not holding this lane. It is following the racing line.
                                        continue;
                                    }
                                    else
                                    {
                                        //The slower truck is holding, or moving into, this lane
                                        if (truckAI->lane_change_to == laneNumber)
                                            return true;
                                    }
                                }
                            }
                            else
                            {
                                return true;
                            }
                        }
                    }
                }

            }
            //debugNumQuadsExamined++;
            pQuad = pQuad->pFrontQuad;
        }
    }
    
    return false;
}


bool CollisionAvoidance_GridQuad::HasCollisionWith(int truckNum)
{
    int numTrucks = trucks.size();
    if (numTrucks <= 1)
    {
        return false;
    }
    
    float startTime = -1.0f;
    float endTime = -1.0f;
    for (int i=0; i<numTrucks; i++)
    {
        PredictedTruckPosition ptp = trucks[i];
        if (ptp.truckNum == truckNum)
        {
            startTime = ptp.minTime;
            endTime = ptp.maxTime;
            break;
        }
    }
    if (startTime < 0.0f && endTime < 0.0f)
    {
        return false;
    }
    
    for (int i=0; i<numTrucks; i++)
    {
        PredictedTruckPosition ptp = trucks[i];
        if (ptp.truckNum != truckNum && startTime <= ptp.maxTime && endTime >= ptp.minTime)
        {
            return true;
        }
    }
    return false;
}

bool CollisionAvoidance_GridQuad::HasCollisionWith(PredictedTruckPosition ptp)
{
    int numTrucks = trucks.size();
    if (numTrucks <= 1)
    {
        return false;
    }

    for (int i=0; i<numTrucks; i++)
    {
        PredictedTruckPosition ptp2 = trucks[i];
        if (ptp2.truckNum != ptp.truckNum && ptp.minTime <= ptp2.maxTime && ptp.maxTime >= ptp2.minTime)
        {
            return true;
        }
    }
    return false;
}

bool CollisionAvoidance_GridQuad::Contains(int truckNum)
{
    int numTrucks = trucks.size();
    for (int i=0; i<numTrucks; i++)
    {
        PredictedTruckPosition ptp = trucks[i];
        if (ptp.truckNum == truckNum)
        {
            return true;
        }
    }
    return false;
}


void CollisionAvoidance_GridQuad::GetCollisionsWith(PredictedTruckPosition ptp, std::vector<PredictedTruckPosition>& collidingTrucks)
{
    int numTrucks = trucks.size();
    if (numTrucks <= 1)
    {
        return;
    }

    for (int i=0; i<numTrucks; i++)
    {
        PredictedTruckPosition ptp2 = trucks[i];
        if (ptp2.truckNum != ptp.truckNum && ptp.minTime <= ptp2.maxTime && ptp.maxTime >= ptp2.minTime)
        {
            collidingTrucks.push_back(ptp2);
        }
    }
} 

const PredictedTruckPosition * CollisionAvoidance_GridQuad::GetSoonestCollision(PredictedTruckPosition ptp)
{
    int numTrucks = trucks.size();
    if (numTrucks <= 1)
    {
        return nullptr;
    }
    
    float minMinTime = -1.0f;
    float minMaxTime = -1.0f;
    PredictedTruckPosition * ptpMin = nullptr;

    for (int i=0; i<numTrucks; i++)
    {
        PredictedTruckPosition ptp2 = trucks[i];
        if (ptp2.truckNum != ptp.truckNum && ptp.minTime <= ptp2.maxTime && ptp.maxTime >= ptp2.minTime)
        {
            if (ptp2.minTime < minMinTime || minMinTime == -1.0f || (ptp2.minTime == minMinTime && (ptp2.maxTime < minMaxTime || minMaxTime == -1.0f)))
            {
                minMinTime = ptp2.minTime;
                minMaxTime = ptp2.maxTime;
                ptpMin = &ptp2;
            }
        }
    }
    return ptpMin;
}


std::vector<PredictedTruckPosition> const &CollisionAvoidance_GridQuad::GetVehicles() const
{
    return trucks;
}

void CollisionAvoidance_GridQuad::GetCoordinates(Ogre::Vector3& frontLeft, Ogre::Vector3& frontRight, Ogre::Vector3& rearRight, Ogre::Vector3& rearLeft)
{
    frontLeft = this->frontLeft;
    frontRight = this->frontRight;
    rearLeft = this->rearLeft;
    rearRight = this->rearRight;
}

void CollisionAvoidance_GridQuad::SetCoordinates(Ogre::Vector3 frontLeft, Ogre::Vector3 frontRight, Ogre::Vector3 rearRight, Ogre::Vector3 rearLeft)
{
    this->frontLeft = frontLeft;
    this->frontRight = frontRight;
    this->rearLeft = rearLeft;
    this->rearRight = rearRight;    
}

bool CollisionAvoidance_GridQuad::IntersectsQuad(Ogre::Vector3 frontLeft, Ogre::Vector3 frontRight, Ogre::Vector3 rearRight, Ogre::Vector3 rearLeft)
{
    return QuadsIntersect(frontLeft, frontRight, rearRight, rearLeft, this->frontLeft, this->frontRight, this->rearRight, this->rearLeft);
}

bool CollisionAvoidance_GridQuad::ContainsPoint(Ogre::Vector3 point)
{
    return PointIsInsideQuad(point, this->frontLeft, this->frontRight, this->rearRight, this->rearLeft);
}

bool CollisionAvoidance_GridQuad::IntersectsQuadCenteredOnWaypoints(std::map<int, Ogre::Vector3>& waypoints, Ogre::Vector3 frontLeft, Ogre::Vector3 frontRight, Ogre::Vector3 rearRight, Ogre::Vector3 rearLeft)
{
    Vector3 thisFrontLeft = this->frontLeft;
    Vector3 thisFrontRight = this->frontRight;
    Vector3 thisRearRight = this->rearRight;
    Vector3 thisRearLeft = this->rearLeft;
    Vector3 centerFront = GetLaneCenterFront();
    Vector3 centerRear = GetLaneCenterRear();
    Vector3 waypointFront = waypoints[waypointIDInFront];
    Vector3 waypointRear = waypoints[waypointIDAtRear];
    Vector3 roadLeftFront = FindRoadLeftEdgeFront();
    Vector3 roadRightFront = FindRoadRightEdgeFront();
    Vector3 roadLeftRear = FindRoadLeftEdgeRear();
    Vector3 roadRightRear = FindRoadRightEdgeRear();
    Vector3 roadCenterFront = (roadLeftFront + roadRightFront) * 0.5f;
    Vector3 roadCenterRear = (roadLeftRear + roadRightRear) * 0.5f;
    float roadWidthFront = (roadRightFront - roadLeftFront).length();
    float roadWidthRear = (roadRightRear - roadLeftRear).length();
    //Move the waypoints so they're offsets from the center of the road instead of the origin
    waypointFront -= roadCenterFront;
    waypointRear -= roadCenterRear;
    //Scale the waypoint positions into the lane width
    float scaleIntoLaneFront = (thisFrontRight - thisFrontLeft).length() / roadWidthFront;
    float scaleIntoLaneRear = (thisRearRight - thisRearLeft).length() / roadWidthRear;
    waypointFront *= scaleIntoLaneFront;
    waypointRear *= scaleIntoLaneRear;
    //Move the waypoints so they're offset from the lane center
    //waypointFront += centerFront;
    //waypointRear += centerRear;
    thisFrontLeft = thisFrontLeft /*- centerFront*/ + waypointFront;
    thisFrontRight = thisFrontRight /*- centerFront*/ + waypointFront;
    thisRearLeft = thisRearLeft /*- centerRear*/ + waypointRear;
    thisRearRight = thisRearRight /*- centerRear*/ + waypointRear;
    return QuadsIntersect(frontLeft, frontRight, rearRight, rearLeft, thisFrontLeft, thisFrontRight, thisRearRight, thisRearLeft);
}

void CollisionAvoidance_GridQuad::GetCoordinatesCenteredOnWaypoints(std::map<int, Ogre::Vector3>& waypoints, Ogre::Vector3& frontLeft, Ogre::Vector3& frontRight, Ogre::Vector3& rearRight, Ogre::Vector3& rearLeft)
{
    Vector3 thisFrontLeft = this->frontLeft;
    Vector3 thisFrontRight = this->frontRight;
    Vector3 thisRearRight = this->rearRight;
    Vector3 thisRearLeft = this->rearLeft;
    Vector3 centerFront = GetLaneCenterFront();
    Vector3 centerRear = GetLaneCenterRear();
    Vector3 waypointFront = waypoints[waypointIDInFront];
    Vector3 waypointRear = waypoints[waypointIDAtRear];
    Vector3 roadLeftFront = FindRoadLeftEdgeFront();
    Vector3 roadRightFront = FindRoadRightEdgeFront();
    Vector3 roadLeftRear = FindRoadLeftEdgeRear();
    Vector3 roadRightRear = FindRoadRightEdgeRear();
    Vector3 roadCenterFront = (roadLeftFront + roadRightFront) * 0.5f;
    Vector3 roadCenterRear = (roadLeftRear + roadRightRear) * 0.5f;
    float roadWidthFront = (roadRightFront - roadLeftFront).length();
    float roadWidthRear = (roadRightRear - roadLeftRear).length();
    //Move the waypoints so they're offsets from the center of the road instead of the origin
    waypointFront -= roadCenterFront;
    waypointRear -= roadCenterRear;
    //Scale the waypoint positions into the lane width
    float scaleIntoLaneFront = (thisFrontRight - thisFrontLeft).length() / roadWidthFront;
    float scaleIntoLaneRear = (thisRearRight - thisRearLeft).length() / roadWidthRear;
    waypointFront *= scaleIntoLaneFront;
    waypointRear *= scaleIntoLaneRear;
    //Move the waypoints so they're offset from the lane center
    //waypointFront += centerFront;
    //waypointRear += centerRear;
    thisFrontLeft = thisFrontLeft /*- centerFront*/ + waypointFront;
    thisFrontRight = thisFrontRight /*- centerFront*/ + waypointFront;
    thisRearLeft = thisRearLeft /*- centerRear*/ + waypointRear;
    thisRearRight = thisRearRight /*- centerRear*/ + waypointRear;
    frontLeft = thisFrontLeft;
    frontRight = thisFrontRight;
    rearLeft = thisRearLeft;
    rearRight = thisRearRight;
}

Vector3 CollisionAvoidance_GridQuad::FindRoadLeftEdgeRear()
{
    Vector3 edge = rearLeft;
    CollisionAvoidance_GridQuad *pLeft = pLeftQuad;
    while (pLeft)
    {
        edge = pLeft->rearLeft;
        pLeft = pLeft->pLeftQuad;
    }
    return edge;
}
    
Vector3 CollisionAvoidance_GridQuad::FindRoadRightEdgeRear()
{
    Vector3 edge = rearRight;
    CollisionAvoidance_GridQuad *pRight = pRightQuad;
    while (pRight)
    {
        edge = pRight->rearRight;
        pRight = pRight->pRightQuad;
    }
    return edge;    
}

Vector3 CollisionAvoidance_GridQuad::FindRoadLeftEdgeFront()
{
    Vector3 edge = frontLeft;
    CollisionAvoidance_GridQuad *pLeft = pLeftQuad;
    while (pLeft)
    {
        edge = pLeft->frontLeft;
        pLeft = pLeft->pLeftQuad;
    }
    return edge;
}
    
Vector3 CollisionAvoidance_GridQuad::FindRoadRightEdgeFront()
{
    Vector3 edge = frontRight;
    CollisionAvoidance_GridQuad *pRight = pRightQuad;
    while (pRight)
    {
        edge = pRight->frontRight;
        pRight = pRight->pRightQuad;
    }
    return edge;    
}

int CollisionAvoidance_GridQuad::GetLaneNumber()
{
    int laneNumber = 0;
    CollisionAvoidance_GridQuad *pLeft = pLeftQuad;
    while (pLeft)
    {
        pLeft = pLeft->pLeftQuad;
        laneNumber++;
    }
    return laneNumber;
}

#endif