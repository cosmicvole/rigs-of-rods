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

#ifdef USE_ANGELSCRIPT

#include "RoRPrerequisites.h"
#include "scriptdictionary/scriptdictionary.h"
#include "Beam.h" //cosmic vole March 9 2017

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
    /**
     *  Finds the closest points on the left and right hand side of the road to the supplied location. cosmic vole March 29 2017
     *
     *  @param [in] location The coordinates of a point anywhere on (or off!) the road to find the closest edge points from.
     *  @param [out] leftEdgePoint Returns the closest point to location on the left hand side of the road.
     *  @param [out] rightEdgePoint Returns the closest point to location on the right hand side of the road.
     */
    void FindRoadEdgePoints(Ogre::Vector3& location, Ogre::Vector3& leftEdgePoint, Ogre::Vector3& rightEdgePoint);
    
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
    bool is_pitting;//!< This will be true if the vehicle is diverting into the pit lane (optional, used for races). cosmic vole March 16 2017

    int free_waypoints;// = 0;//!< The amount of waypoints.
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
    Ogre::Vector3 accel_velocity;//!< Used to estimate the vehicle's current acceleration for collision avoidance - cosmic vole April 7 2017
    Ogre::Vector3 average_accel;//!< Average acceleration. Used for collision avoidance - cosmic vole April 7 2017
    
    //PID control variables - cosmic vole April 11 2017
    float lastErrorAcrossTrack;
    float sumErrorsAcrossTrack;
    float PID_P;
    float PID_I;
    float PID_D;
    Ogre::Vector3 lastWaypointNavigated; //Last waypoint that was actually navigated towards. At speed waypoints are skipped and this takes that into account.
};

//Helpers for collision avoidance calcs - cosmic vole March 7 2017
bool lineIntersectionXZ(Ogre::Vector3 start1, Ogre::Vector3 end1, Ogre::Vector3 start2, Ogre::Vector3 end2, bool& intersectionInBounds, bool& linesAreParallel, Ogre::Vector3& intersection);//cosmic vole 28 February 2017
Ogre::Vector3 rotate(Ogre::Vector3& point, Ogre::Quaternion rotation, Ogre::Vector3& centre);
Ogre::Vector3 getClosetPointOnLine(Ogre::Vector3& A, Ogre::Vector3& B, Ogre::Vector3& P, bool clampToBounds);
//For debugging - adds a sphere to the scene and the supplied vector, used to debug collision avoidance - cosmic vole
void debugCollision(int& reuseNode, std::vector<Ogre::SceneNode*>& debugCol, Ogre::Vector3 position, std::string materialName);
struct truckDistanceSort;




#endif // USE_ANGELSCRIPT
