/*
    This source file is part of Rigs of Rods

    Copyright 2005-2012 Pierre-Michel Ricordel
    Copyright 2007-2012 Thomas Fischer
    Copyright 2013-2016 Petr Ohlidal

    For more information, see http://www.rigsofrods.org/

    Rigs of Rods is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License version 3, as
    published by the Free Software Foundation.

    Rigs of Rods is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Rigs of Rods. If not, see <http://www.gnu.org/licenses/>.
*/

/// @file   CollisionAvoidance_GridQuad.h
/// @brief  Grid based collision avoidance AI
/// @author cosmic vole
/// @date   10/2017

#pragma once

//MIN() and MAX() added - cosmic vole February 28 2017
#define MIN(X,Y) ((X) <= (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) >= (Y) ? (X) : (Y))

//cosmic vole May 12 2017
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

#ifdef USE_ANGELSCRIPT

#include "RoRPrerequisites.h"
#include "scriptdictionary/scriptdictionary.h"
#include "Beam.h" //cosmic vole March 9 2017
 
//Helpers for collision avoidance calcs - cosmic vole March 7 2017
bool lineIntersectionXZ(Ogre::Vector3 start1, Ogre::Vector3 end1, Ogre::Vector3 start2, Ogre::Vector3 end2, bool& intersectionInBounds, bool& linesAreParallel, Ogre::Vector3& intersection);//cosmic vole February 28 2017
bool lineIntersectionXZ(Ogre::Vector3 start1, Ogre::Vector3 end1, bool allowOutOfBounds1, Ogre::Vector3 start2, Ogre::Vector3 end2, bool allowOutOfBounds2, bool& inBounds1, bool& inBounds2, bool& linesAreParallel, Ogre::Vector3& intersection);//cosmic vole August 7 2017
bool lineIntersectsQuad(Ogre::Vector3 lineStart, Ogre::Vector3 lineEnd, Ogre::Vector3 rectPoint1, Ogre::Vector3 rectPoint2, Ogre::Vector3 rectPoint3, Ogre::Vector3 rectPoint4);//cosmic vole May 20 2017
bool QuadsIntersect(Ogre::Vector3 rect1Point1, Ogre::Vector3 rect1Point2, Ogre::Vector3 rect1Point3, Ogre::Vector3 rect1Point4, Ogre::Vector3 rect2Point1, Ogre::Vector3 rect2Point2, Ogre::Vector3 rect2Point3, Ogre::Vector3 rect2Point4);//cosmic vole May 20 2017
bool PointIsInsideQuad(Ogre::Vector3 point, Ogre::Vector3 quadPoint1, Ogre::Vector3 quadPoint2, Ogre::Vector3 quadPoint3, Ogre::Vector3 quadPoint4);//cosmic vole June 18 2017
//float GetLaneOverlap(float& leftOverlap, float& rightOverlap, Ogre::Vector3 quadPoint1, Ogre::Vector3 quadPoint2, Ogre::Vector3 quadPoint3, Ogre::Vector3 quadPoint4, Ogre::Vector3 laneLeft, Ogre::Vector3 laneRight, Ogre::Vector3 laneCenter = Ogre::Vector3::ZERO, bool debug = false);
Ogre::Vector3 rotate(Ogre::Vector3& point, Ogre::Quaternion rotation, Ogre::Vector3& centre);
Ogre::Vector3 getClosestPointOnLine(Ogre::Vector3& A, Ogre::Vector3& B, Ogre::Vector3& P, bool clampToBounds);
//For debugging - adds a sphere to the scene and the supplied vector, used to debug collision avoidance - cosmic vole
void debugCollision(int& reuseNode, std::vector<Ogre::SceneNode*>& debugCol, Ogre::Vector3 position, std::string materialName);
Ogre::SceneNode* debugCollision(Ogre::SceneNode* node, Ogre::Vector3 position, std::string materialName);
void debugPoints(Ogre::Vector3 point1, Ogre::Vector3 point2, float minY);
struct truckDistanceSort;

class CollisionAvoidance_GridQuad;

//cosmic vole May 13 2017
struct PredictedTruckPosition
{
    int truckNum;
    float minTime;
    float maxTime;
    CollisionAvoidance_GridQuad* pGridQuad;
    //Position would be a range. We don't care as long as it's in the grid square. Ogre::Vector3 position; //TODO do we want a heading as well, or 4 truck corners
};

//cosmic vole May 13 2017
class CollisionAvoidance_GridQuad
{
public:
    CollisionAvoidance_GridQuad();
    ~CollisionAvoidance_GridQuad();
    PredictedTruckPosition/*&*/ AddVehiclePosition(int truckNum, float time);
    PredictedTruckPosition/*&*/ AddVehiclePosition(int truckNum, float startTime, float endTime);
    void RemoveVehicle(int truckNum);
    void Clear();
    void ClearOldPositions(float cutoffTime);
    bool IsOccupied(float startTime, float endTime, int ignoreTruckNum);
    bool IsLaneOccupied(float inFront, float behind, float startTime, float endTime, int ignoreTruckNum);
    bool LaneHasSlowerVehicleInFront(float inFront, float startTime, float endTime, float maxSpeed, bool ignoreTrucksUsingOtherLanes, int ignoreTruckNum);
    bool HasCollisionWith(int truckNum);
    bool HasCollisionWith(PredictedTruckPosition ptp);
    bool Contains(int truckNum);
    void GetCollisionsWith(PredictedTruckPosition ptp, std::vector<PredictedTruckPosition>& collidingTrucks);
    const PredictedTruckPosition* GetSoonestCollision(PredictedTruckPosition ptp);
    std::vector<PredictedTruckPosition> const &GetVehicles() const;
    void GetCoordinates(Ogre::Vector3& frontLeft, Ogre::Vector3& frontRight, Ogre::Vector3& rearRight, Ogre::Vector3& rearLeft);
    void SetCoordinates(Ogre::Vector3 frontLeft, Ogre::Vector3 frontRight, Ogre::Vector3 rearRight, Ogre::Vector3 rearLeft);
    bool IntersectsQuad(Ogre::Vector3 frontLeft, Ogre::Vector3 frontRight, Ogre::Vector3 rearRight, Ogre::Vector3 rearLeft);
    bool ContainsPoint(Ogre::Vector3 point);
    bool IntersectsQuadCenteredOnWaypoints(std::map<int, Ogre::Vector3>& waypoints, Ogre::Vector3 frontLeft, Ogre::Vector3 frontRight, Ogre::Vector3 rearRight, Ogre::Vector3 rearLeft);
    void GetCoordinatesCenteredOnWaypoints(std::map<int, Ogre::Vector3>& waypoints, Ogre::Vector3& frontLeft, Ogre::Vector3& frontRight, Ogre::Vector3& rearRight, Ogre::Vector3& rearLeft);
    Ogre::Vector3 FindRoadLeftEdgeRear();
    Ogre::Vector3 FindRoadRightEdgeRear();
    Ogre::Vector3 FindRoadLeftEdgeFront();
    Ogre::Vector3 FindRoadRightEdgeFront();
    int GetLaneNumber();
    inline Ogre::Vector3 GetLaneCenterRear()
    {
        return (rearLeft + rearRight) * 0.5f;
    }
    inline Ogre::Vector3 GetLaneCenterFront()
    {
        return (frontLeft + frontRight) * 0.5f;
    }
    inline void GetNeighbours(CollisionAvoidance_GridQuad*& pLeft, CollisionAvoidance_GridQuad*& pRight, CollisionAvoidance_GridQuad*& pFront, CollisionAvoidance_GridQuad*& pRear)
    {
        pLeft = pLeftQuad;
        pRight = pRightQuad;
        pFront = pFrontQuad;
        pRear = pRearQuad;
    }
    inline void SetNeighbourLeft(CollisionAvoidance_GridQuad* pLeft) { pLeftQuad = pLeft; }
    inline void SetNeighbourRight(CollisionAvoidance_GridQuad* pRight) { pRightQuad = pRight; }
    inline void SetNeighbourFront(CollisionAvoidance_GridQuad* pFront) { pFrontQuad = pFront; }
    inline void SetNeighbourRear(CollisionAvoidance_GridQuad* pRear) { pRearQuad = pRear; }
    /*inline*/ CollisionAvoidance_GridQuad* GetNeighbourLeft() { return pLeftQuad; }
    /*inline*/ CollisionAvoidance_GridQuad* GetNeighbourRight() { return pRightQuad; }
    /*inline*/ CollisionAvoidance_GridQuad* GetNeighbourFront() { return pFrontQuad; }
    /*inline*/ CollisionAvoidance_GridQuad* GetNeighbourRear() { return pRearQuad; }    
    inline void SetWaypointIDs(int rearWaypointID, int frontWaypointID)  { waypointIDAtRear = rearWaypointID; waypointIDInFront = frontWaypointID; }
    inline int GetWaypointIDRear() { return waypointIDAtRear; }
    inline int GetWaypointIDFront() { return waypointIDInFront; }
private:
    Ogre::Vector3 frontLeft;
    Ogre::Vector3 frontRight;
    Ogre::Vector3 rearRight;
    Ogre::Vector3 rearLeft;
    std::vector<PredictedTruckPosition> trucks;
    CollisionAvoidance_GridQuad* pLeftQuad;
    CollisionAvoidance_GridQuad* pRightQuad;
    CollisionAvoidance_GridQuad* pFrontQuad;
    CollisionAvoidance_GridQuad* pRearQuad;
    int waypointIDAtRear;
    int waypointIDInFront;
};

#endif //USE_ANGELSCRIPT