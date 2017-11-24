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

/// @file   VehicleAI.h
/// @brief  Simple waypoint AI
/// @author AnotherFoxGuy
/// @date   03/2016

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

//cosmic vole May 13 2017
class CollisionAvoidance_GridQuad;
struct PredictedTruckPosition;

/**
*	Enum with AI events
*/
enum Ai_events
{
    AI_HORN,
    AI_LIGHTSTOGGLE,
    AI_WAIT_SECONDS,
    AI_BEACONSTOGGLE
};

/**
*	Enum with AI values that can be set.
*/
enum Ai_values
{
    AI_SPEED,
    AI_POWER
};

class VehicleAI : public ZeroedMemoryAllocator
{
public:
    VehicleAI(Beam* b);
    ~VehicleAI();
    /**
     *  Activates/Deactivates the AI.
     *  @param [in] value Activate or deactivation the AI
     */
    void SetActive(bool value);
    /**
     *  Returns the status of the AI.
     *  @return True if the AI is driving
     */
    bool IsActive();

#ifdef USE_ANGELSCRIPT
    // we have to add this to be able to use the class as reference inside scripts
    void addRef()
    {
    };

    void release()
    {
    };
#endif
    /**
     *  Updates the AI.
     */
    void update(float dt, int doUpdate);
    /**
     *  Adds one waypoint.
     *
     *  @param [in] id The waypoint ID.
     *  @param [in] point The coordinates of the waypoint.
     */
    void AddWaypoint(Ogre::String& id, Ogre::Vector3& point);
    /**
     *  Adds a dictionary with waypoints.
     *  @param [in] d Dictionary with waypoints
     */
    void AddWaypoints(AngelScript::CScriptDictionary& d);
    /**
     *  Inserts one waypoint. cosmic vole March 3 2017
     *
     *  @param [in] id The waypoint ID.
     *  @param [in] point The coordinates of the waypoint.
     *  @param [in] position The position to insert the waypoint in the list.
     */
    void InsertWaypoint(Ogre::String& id, Ogre::Vector3& point, int position);    
    /**
     *  Adds a event
     *
     *  @param [in] id The waypoint ID.
     *  @param [in] ev The ID of the event.
     *
     *  @see Ai_events
     */     
    void AddEvent(Ogre::String& id, int& ev);
    /**
     *  Sets a value at a waypoint.
     *
     *  @param [in] id The waypoint ID.
     *  @param [in] value_id The ID of the value that will be set.
     *  @param [in] value The value itself.
     *
     *  @see Ai_values
     */
    void SetValueAtWaypoint(Ogre::String& id, int& value_id, float& value);
    /**
     *  Sets the current waypoint by ID. If it's not the first one, the collision avoidance algorithm can look backwards at a waypoint behind the vehicle.
     *  It also allows simplifies accurate initialization of a grid of cars where each starts at a slightly different point on the track. cosmic vole June 25 2017
     *
     *  @param [in] id The waypoint ID.
     *
     *  @see Ai_events
     */     
    void SetCurrentWaypoint(Ogre::String& id);
    /**
     *  Sets the number of waypoints that make up one lap of the race. This only needs to be used on circuits, for collision avoidance.
     *  If it is not set correctly, then collision avoidance may not work between two vehicles that are on different laps of the circuit.
     *  If you are not using a circuit, it does not need to be set and defaults to zero. cosmic vole June 25 2017
     * 
     *  @param [in] numWaypointsInLap The number of waypoints specified for one lap of the race.
     */     
    void SetNumWaypointsInLap(int numWaypointsInLap);
    /**
     * Finds the ID of the closest waypoint on the track that is just ahead of the specified location.
     * This can be used together with SetCurrentWaypoint() to correctly set the starting waypoints of race competitors on a starting grid. cosmic vole July 9 2017.
     * 
     * @param [in] location A position on the track for which to find the next waypoint.
     * @param [in] startWaypointID ID of the first waypoint to start searching from or -1 to start from the beginning.
     * @param [in] endWaypointID ID of the last waypoint to search or -1 to search all of them.
     */
     Ogre::String FindNextWaypointID(Ogre::Vector3& location, int startWaypointID, int endWaypointID);
    /**
     * Finds the index of the closest waypoint on the track that is just ahead of the specified location. cosmic vole July 9 2017.
     * 
     * @param [in] location A position on the track for which to find the next waypoint.
     * @param [in] startWaypointID ID of the first waypoint to start searching from or -1 to start from the beginning.
     * @param [in] endWaypointID ID of the last waypoint to search or -1 to search all of them.
     */
     int FindNextWaypointIndex(Ogre::Vector3 location, int startWaypointID, int endWaypointID);     
    /**
     *  Inserts one point defining the left edge of the road. These are optional for use by the AI. cosmic vole March 29 2017
     *
     *  @param [in] point The coordinates of the edge point.
     */
    void AddRoadEdgePointLeft(Ogre::Vector3& point);
    /**
     *  Inserts one point defining the right edge of the road. These are optional for use by the AI. cosmic vole March 29 2017
     *
     *  @param [in] point The coordinates of the edge point.
     */
    void AddRoadEdgePointRight(Ogre::Vector3& point);
    
    int GetClosestRoadEdgePointIndexLeft(Ogre::Vector3& location);
    
    int GetClosestRoadEdgePointIndexRight(Ogre::Vector3& location);
    
    /**
     *  Finds the closest points on the left and right hand side of the road to the supplied waypoint and / or location, using linear interpolation. cosmic vole March 29 2017
     *
     *  @param [in] waypointID A waypoint on the track to find edge points for. If "location" is specified, a point on the track in line with "location" just behind the waypoint will be used.
     *  @param [in] location Optionally the coordinates of a point anywhere on (or off!) the road to find the closest edge points from or Vector3::ZERO.
     *  @param [out] leftEdgePoint Returns the closest point to location on the left hand side of the road.
     *  @param [out] rightEdgePoint Returns the closest point to location on the right hand side of the road.
     *  @param [out] nextLeftIndex Returns the index of the next point along the left hand side of the road in road_edge_points_left or -1.
     *  @param [out] nextRightIndex Returns the index of the next point along the right hand side of the road in road_edge_points_right or -1.
     */
    void FindRoadEdgePoints(int waypointID, Ogre::Vector3 location, Ogre::Vector3& leftEdgePoint, Ogre::Vector3& rightEdgePoint, int& nextLeftIndex, int& nextRightIndex);
    /**
     *  Divides a section of track between the specified points into overtaking lanes for collision avoidance. cosmic vole April 20 2017
     *
     *  @param [in] start Position on the track to start the algorithm
     *  @param [in] end Position on the track to end the algorithm
     */
    //void CollisionAvoidance_GenerateLanes(Ogre::Vector3& start, Ogre::Vector3& end);
    /*static*/ void CollisionAvoidance_GenerateLanes(int startWaypoint, int endWaypoint, std::map<int, Ogre::Vector3>& waypoints, std::vector<Ogre::Vector3>& road_edge_points_left, std::vector<Ogre::Vector3>& road_edge_points_right);//Ogre::Vector3& start, Ogre::Vector3& end) //TODO instead of start and end vectors, use waypoints
    /*static*/ void CollisionAvoidance_AvoidCollisions(float currentTime/*, std::map<int, Ogre::Vector3>& waypoints, std::vector<Ogre::Vector3>& road_edge_points_left, std::vector<Ogre::Vector3>& road_edge_points_right*/);
    void CollisionAvoidance_FollowLane(VehicleAI *truckAI, int lane, int startWaypoint, int endWaypoint, bool smoothChange = true, bool resumeRacingLine = false, int steps = -1, int mergeFromLane = -1);//Used for debugging the lane algorithm. cosmic vole June 27 2017
    
    //cosmic vole May 12 2017
    void CollisionAvoidance_PredictVehiclePath(float currentTime/*Ogre::Vector3 mAgentAbsPosition, Ogre::Vector3 agentVelocity, Ogre::Vector3 mAgentHeading, Ogre::Vector3 agentDir*/);
    
    //cosmic vole July 1 2017
    void CollisionAvoidance_GetVehicleApproachAngles(Ogre::Vector3 agentPos, Ogre::Vector3 otherPos, Ogre::Vector3 agentDir, Ogre::Vector3 otherDir, Ogre::Degree& betweenDirs, Ogre::Degree& bearing, bool& isBehind, bool& isHeadOn);
    
    //cosmic vole May 31 2017
    /*static*/ void CollisionAvoidance_ClearVehiclePaths(Ogre::String raceID, bool pastOnly);
    
    //cosmic vole August 30 2017
    std::vector<CollisionAvoidance_GridQuad*>* CollisionAvoidance_GetGridQuads(Ogre::String raceID, int waypointID);
    
    //cosmic vole August 30 2017
    int CollisionAvoidance_GetLaneNumber(int waypoint, bool useAdjustedWaypoints);
    
    //cosmic vole July 3 2017
    void CollisionAvoidance_DebugVehiclePath(int truckNum = -1);
    
    //cosmic vole July 2 2017
    void CollisionAvoidance_ShowDebugMsg(bool isLaneOccupied, bool hasCollision, bool collisionWithTruckInFront, bool canAvoidTruck, int currentLane, int TargetLane, Ogre::UTFString& info, float dt = 0.0f);
    
    //cosmic vole June 9 2017
    float soonestCollisionTime();
    
    //cosmic vole July 6 2017
    void FollowLaneToggle(int laneNumber);
    
    //cosmic vole
    inline const std::vector<PredictedTruckPosition>& GetPredictedVehiclePath() { return predictedVehiclePath; }
    
    //cosmic vole August 23 2017
    inline void SetRaceID(int raceID) { this->raceID = raceID; }
    inline int GetRaceID() { return raceID; }
    int GetNumberOfWaypoints() { return waypoints.size(); }
    //cosmic vole September 4 2017
    inline void AllowRollCorrection(bool willAllow) { allowRollCorrection = willAllow; }
    
	//cosmic vole November 23 2017
	inline void SetSimController(RoRFrameListener* sim) { m_sim_controller = sim; }
	
private:
    /**
     *   Updates the AI waypoint.
     */
    void updateWaypoint();
    
    float GetPIDControlFactor(float errorAcrossTrack, float dt);

    bool is_waiting;//!< 
    float wait_time;//!<(seconds) The amount of time the AI has to wait on this waypoint.
    int task_after_waiting;//!< The task to do after it has waited.

    //cosmic vole added auto reset code for stuck vehicles October 9 2016
    bool is_stuck;
    float stuck_time;
    Ogre::Vector3 stuck_position;
    float stuck_cancel_distance;
    float stuck_reset_delay;
    //cosmic vole added optional driver models for active AI cars
    Character* character;

    float maxspeed;// = 50;//!<(KM/H) The max speed the AI is allowed to drive.
    Beam* beam;//!< The verhicle the AI is driving.
    bool is_enabled;// = false;//!< True if the AI is driving.
    Ogre::Vector3 current_waypoint;//!< The coordinates of the waypoint that the AI is driving to.
    int current_waypoint_id;// = 0;//!< The curent waypoint ID.
    std::map<int, Ogre::Vector3> waypoints;//!< Map with all waypoints.
    std::map<Ogre::String, int> waypoint_ids;//!< Map with all waypoint IDs.
    std::map<int, Ogre::String> waypoint_names;//!< Map with all waypoint names.
    std::map<int, int> waypoint_events;//!< Map with all waypoint events.
    std::map<int, float> waypoint_speed;//!< Map with all waypoint speeds.
    std::map<int, float> waypoint_power;//!< Map with all waypoint engine power.
    std::map<int, float> waypoint_wait_time;//!< Map with all waypoint wait times.
    std::vector<Ogre::Vector3> road_edge_points_left;//!< Map with all points on left edge of the road (optional used for race overtakes). cosmic vole March 16 2017
    std::vector<Ogre::Vector3> road_edge_points_right;//!< Map with all points on right edge of the road (optional used for race overtakes). cosmic vole March 16 2017
    std::vector<Ogre::Vector3> hard_edge_points_left;//!< Map with all points on a wall or other boundary that mustn't be crossed on left of the road (optional, used for racing). cosmic vole March 16 2017
    std::vector<Ogre::Vector3> hard_edge_points_right;//!< Map with all points on a wall or other boundary that mustn't be crossed on right of the road (optional, used for racing). cosmic vole March 16 2017
    std::map<int, Ogre::Vector3> pit_lane_waypoints;//!< Map with all pit lane waypoints (optional, used for races). cosmic vole March 16 2017
    std::map<Ogre::String, int> pit_lane_waypoint_ids;//!< Map with all pit lane waypoint IDs (optional, used for races). cosmic vole March 16 2017
    std::map<int, Ogre::String> pit_lane_waypoint_names;//!< Map with all pit lane waypoint names (optional, used for races). cosmic vole March 16 2017
    std::map<int, Ogre::Vector3> adjusted_waypoints;//!< Map with waypoints that have been moved for this vehicle to avoid collisions. cosmic vole May 31 2017
    std::map<int, float> adjusted_waypoint_speed;//!< Map with waypoint speeds that have been altered for this vehicle to avoid collisions. cosmic vole July 2 2017
    static std::map<int, std::vector<Ogre::Vector3>> raceLanePoints;//!< Map of lists of coordinates that divide each race track into lanes for overtaking and collision avoidance. The key is the raceID. cosmic vole May 18 2017
    static std::map<Ogre::String, int> raceWaypointToLanePointIndex;//!< Maps raceID + "|||" + waypoint name to an index into that race's entry in raceLanePoints. cosmic vole May 18 2017 
    static std::map<Ogre::String, std::vector<CollisionAvoidance_GridQuad*>> raceGridQuads;//!<  Maps raceID + "|||" + waypoint name to a list of CollisionAvoidance_GridQuads at that waypoint. cosmic vole May 21 2017
    std::vector<PredictedTruckPosition> predictedVehiclePath;//!< List of positions (with times) on the race track grid that this vehicle is predicted to drive over. cosmic vole May 31 2017 /*std::vector<CollisionAvoidance_GridQuad&> predictedVehiclePath;*/
    std::map<int, Ogre::Vector3> vehiclePositionsAtWaypoints;//!< For debugging, records the exact position of the vehicle as it passes each waypoint. cosmic vole August 15 2017
    int raceID;//!< ID of current race as used in races.as, or -1. This is needed to ensure the racing lanes are only used for vehicles in the same race. cosmic vole May 18 2017
    bool is_pitting;//!< This will be true if the vehicle is diverting into the pit lane (optional, used for races). cosmic vole March 16 2017

    int free_waypoints;// = 0;//!< The amount of waypoints.
    int num_waypoints_in_lap;//!< If the vehicle is driving on a circuit, this has the number of waypoints in each lap, or 0. Used in collision avoidance. cosmic vole June 25 2017
    float acc_power;// = 0.8;//!< The engine power.
    Ogre::String collision_waypoint_id;//!< ID of the next waypoint to re-enable collision avoidance - cosmic vole March 4 2017
    float collision_avoid_time;//!< Time since collision avoidance was started - cosmic vole March 8 2017
    float time_since_last_collision_avoid;//!< Time since we last steered to avoid a collision, used to smooth out straightening - cosmic vole April 8 2017
    Ogre::Vector3 collision_avoid_target;//!< How far left or right on the track to aim - cosmic vole March 8 2017
    float steering_time;//!< Time until next steering hydro command can be issued. Used to slow down steering - cosmic vole March 8 2017
    float steering_delay;//!< Time in seconds to wait between steering hydro commands. Used to slow down steering - cosmic vole March 8 2017
    float last_steering_yaw;//!< Last steering direction - cosmic vole March 11 2017
    float last_skid_hydrodirstate;//!< Steering direction of last skid - cosmic vole March 11 2017
    bool is_counter_steering;//!< Used in skid control - cosmic vole March 11 2017
    bool direction_changed;//!< Used in skid control - cosmic vole March 11 2017
    float skid_time;//!< Time since last skid - cosmic vole March 11 2017
    float turn_time;//!< Time spent interpolating current turn - cosmic vole March 11 2017
    float accel_time;//!< Used to estimate the vehicle's current acceleration for collision avoidance - cosmic vole April 7 2017
    float last_health;//!< Used to track the vehicle's health to detect accidents, currently for debugging - cosmic vole August 12 2017
    float last_crash_time;//!< Last time a vehicle crash was detected, currently used for debugging - cosmic vole August 12 2017
    float stability_time;//!< Used to track the vehicle's stability as it follows the waypoints - cosmic vole August 14 2017
    float stability_plus_minus_transitions;//!< Number of times steering has gone from big negative to big postive value or vice versa. - cosmic vole August 14 2017
    float stability_max_positive_steer;//!< Maximum positive raw steering force. - cosmic vole August 14 2017
    float stability_max_negative_steer;//!< Maximum negative raw steering force. - cosmic vole August 14 2017
    float stability_last_positive_steer;//!< Last big positive raw steering force. - cosmic vole August 14 2017
    float stability_last_negative_steer;//!< Last big negative raw steering force. - cosmic vole August 14 2017
    float stability_last_steer;//!< Last big raw steering force. - cosmic vole August 14 2017
    float stability_max_speed;//!< Max speed, used to track the vehicle's stability as it follows the waypoints - cosmic vole August 14 2017
    bool stability_lost_control;//!< Set true if the vehicle is estimated to be losing control based on the above statistics. - cosmic vole August 14 2017
    int stability_skip_to_waypoint;//!< Skips waypoints up to this ID while the vehicle recovers control - cosmic vole August 14 2017
    Ogre::Vector3 accel_velocity;//!< Used to estimate the vehicle's current acceleration for collision avoidance - cosmic vole April 7 2017
    Ogre::Vector3 average_accel;//!< Average acceleration. Used for collision avoidance - cosmic vole April 7 2017
    float soonest_collision_time;//!< Estimated time of soonest predicted collision - cosmic vole June 9 2017
    int lane_change_start_wpt;
    int lane_change_mid_wpt;
    int lane_change_end_wpt;
    int lane_change_from;
    int lane_change_to;
    int lane_force_follow;//!< Lane follow keyboard controls override the AI and force it follow the specified lane - cosmic vole July 6 2017
    float timeStuckBehindVehicle;//!< How long the truck has been stuck behind the vehicle in front - cosmic vole July 2 2017
    float aggression;//!< From 0 - 100 how aggressive the AI is in collision avoidance / overtaking maneuvers - cosmic vole August 18 2017
    bool allowRollCorrection;//!< If true, the AI will steer into a roll. Only suitable for flat surfaces - cosmic vole September 4 2017

    
    //PID control variables - cosmic vole April 11 2017
    float lastErrorAcrossTrack;
    float sumErrorsAcrossTrack;
    float PID_P;
    float PID_I;
    float PID_D;
    Ogre::Vector3 lastWaypointNavigated; //Last waypoint that was actually navigated towards. At speed waypoints are skipped and this takes that into account.
    RoRFrameListener *m_sim_controller; //Needed for new way to access the BeamFactory. cosmic vole November 23 2017
	
	friend class CollisionAvoidance_GridQuad; //Needed so that class can check lane following logic of the trucks. cosmic vole August 13 2017
};

//Helpers for collision avoidance calcs - cosmic vole March 7 2017
bool lineIntersectionXZ(Ogre::Vector3 start1, Ogre::Vector3 end1, Ogre::Vector3 start2, Ogre::Vector3 end2, bool& intersectionInBounds, bool& linesAreParallel, Ogre::Vector3& intersection);//cosmic vole February 28 2017
bool lineIntersectionXZ(Ogre::Vector3 start1, Ogre::Vector3 end1, bool allowOutOfBounds1, Ogre::Vector3 start2, Ogre::Vector3 end2, bool allowOutOfBounds2, bool& inBounds1, bool& inBounds2, bool& linesAreParallel, Ogre::Vector3& intersection);//cosmic vole August 7 2017
bool lineIntersectsQuad(Ogre::Vector3 lineStart, Ogre::Vector3 lineEnd, Ogre::Vector3 rectPoint1, Ogre::Vector3 rectPoint2, Ogre::Vector3 rectPoint3, Ogre::Vector3 rectPoint4);//cosmic vole May 20 2017
bool QuadsIntersect(Ogre::Vector3 rect1Point1, Ogre::Vector3 rect1Point2, Ogre::Vector3 rect1Point3, Ogre::Vector3 rect1Point4, Ogre::Vector3 rect2Point1, Ogre::Vector3 rect2Point2, Ogre::Vector3 rect2Point3, Ogre::Vector3 rect2Point4);//cosmic vole May 20 2017
bool PointIsInsideQuad(Ogre::Vector3 point, Ogre::Vector3 quadPoint1, Ogre::Vector3 quadPoint2, Ogre::Vector3 quadPoint3, Ogre::Vector3 quadPoint4);//cosmic vole June 18 2017
float GetLaneOverlap(float& leftOverlap, float& rightOverlap, Ogre::Vector3 quadPoint1, Ogre::Vector3 quadPoint2, Ogre::Vector3 quadPoint3, Ogre::Vector3 quadPoint4, Ogre::Vector3 laneLeft, Ogre::Vector3 laneRight, Ogre::Vector3 laneCenter = Ogre::Vector3::ZERO, bool debug = false);
Ogre::Vector3 rotate(Ogre::Vector3& point, Ogre::Quaternion rotation, Ogre::Vector3& centre);
Ogre::Vector3 getClosestPointOnLine(Ogre::Vector3& A, Ogre::Vector3& B, Ogre::Vector3& P, bool clampToBounds);
//For debugging - adds a sphere to the scene and the supplied vector, used to debug collision avoidance - cosmic vole
void debugCollision(int& reuseNode, std::vector<Ogre::SceneNode*>& debugCol, Ogre::Vector3 position, std::string materialName);
Ogre::SceneNode* debugCollision(Ogre::SceneNode* node, Ogre::Vector3 position, std::string materialName);
void debugPoints(Ogre::Vector3 point1, Ogre::Vector3 point2, float minY);
struct truckDistanceSort;

#endif // USE_ANGELSCRIPT
