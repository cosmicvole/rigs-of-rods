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

#ifdef USE_ANGELSCRIPT
#include "ChampionshipManager.h"
#include "CollisionAvoidance_GridQuad.h"
#include "VehicleAI.h"
#include "Application.h" //To flash debug message - cosmic vole July 2 2017
#include "GUIManager.h" //To flash debug message - cosmic vole July 2 2017
#include "Beam.h"
#include "BeamFactory.h"
#include "BeamEngine.h"
#include "GUI_GameConsole.h"
//cosmic vole - OgreSubsystem() needed for timer used in collision avoidance May 31 2017
#include "OgreSubsystem.h"
#include "RoRFrameListener.h"
#include "MainMenu.h"
#include "Network.h"
//cosmic vole added AI driver model November 21 2016
#include "CharacterFactory.h"
//#include "PlayerColours.h"

#include "scriptdictionary/scriptdictionary.h"

using namespace Ogre;

VehicleAI::VehicleAI(Beam* b) : m_sim_controller(nullptr)
{
    //Initialization moved into constructor to avoid Visual Studio error 'only static const integral data members can be initialized within a class'
    maxspeed = 50;//!<(KM/H) The max speed the AI is allowed to drive.
    is_enabled = false;//!< True if the AI is driving.
    current_waypoint_id = 0;//!< The curent waypoint ID.
    free_waypoints = 0;//!< The amount of waypoints.
    acc_power = 0.8;//!< The engine power.

    //cosmic vole added auto reset code for stuck vehicles October 9 2016
    stuck_time = 0.0;
    stuck_position = b->getPosition();
    stuck_cancel_distance = 2.8;//2.6
    stuck_reset_delay = 8.0;//11.0;//Time in seconds until a stuck vehicle resets. Was 12
    aggression = 30.0f; //AI aggression. The higher the value, the less space and time it will give other cars. cosmic vole August 21 2017.
    collision_waypoint_id = "";
    collision_avoid_time = 0.0f;
    time_since_last_collision_avoid = 0.0f;
    steering_delay = 0.05f;//Time in seconds to wait between subsequent steering commands. 0.1 works well except when recovering from slides! cosmic vole March 8 2017
    steering_time = 0.0f;
    is_counter_steering = false;
    last_steering_yaw = 0.0f;
    last_skid_hydrodirstate = 0.0f;
    skid_time = 0.0f;
    direction_changed = false;
    turn_time = 0.0f;
    accel_time = 0.0f;
    last_health = 100.0f;
    last_crash_time = -1.0f;
    accel_velocity = b->getVelocity();
    average_accel = Vector3::ZERO;
    //cosmic vole added AI stability calculations August 14 2017
    stability_time = 0.0f;
    stability_max_speed = 0.0f;
    stability_plus_minus_transitions = 0;
    stability_last_negative_steer = 0.0f;
    stability_last_positive_steer = 0.0f;
    stability_max_negative_steer = 0.0f;
    stability_max_positive_steer = 0.0f;
    stability_last_steer = 0.0f;
    stability_skip_to_waypoint = 0;
    //cosmic vole added AI driver model November 21 2016
    character = nullptr;
    raceID = -1;//ID of current race - cosmic vole June 27 2017
    //Lane change variables, used for collision avoidance - cosmic vole June 30 2017
    lane_change_start_wpt = -1;
    lane_change_mid_wpt = -1;
    lane_change_end_wpt = -1;
    lane_change_from = -1;
    lane_change_to = -1;
    lane_force_follow = -1;
    allowRollCorrection = true;
    //PID control pararmeters - cosmic vole April 11 2017
    PID_P = 0.075f;//73//75f;//0.065f;//2.0f;//7.5f;//3.9f;//0.001 - it steers, VERY smoothly but VERY slowly. 1.5f;//13.0f;//2.5f;//0.08f;//0.02f;//0.23f; //0.5 0 0 gives huge zig zags - actually 4.0 is better, 20.0 oscillates intermittenly a lot (same as 1500! maxed out), 15 a bit!, 0.01 0 0 it hardly steers. 0.0001 it goes straight ahead pretty much.
    PID_I = 0.015f;//0.015f;//0.3f;//0.001f;//.007f .003f;//0.001f;//0.16f;
    PID_D = 0.035f;//0.03f;//0.0005f;//0.0005f 001f;//0.03f;
    //using 3.9 0.008 0.0005, the P value is definitely too small as it gets round the track as the vehicle goes straight for some distance and only then does it turn in.
    //7.0f 0.008 0.0005 is getting there. It just steers a bit harshly which causes the car to bounce / oversteer.
    //9.0f 0.008 0.0005 starting to oscillate noticably.
    //9.5f 0.009 0.0005 seems to sometimes have a damped oscillation.
    //9.95 0.02 0.0005 it winds up sawing the wheel so much it barely moves forward - same thing with 9.95 0.015 0.0005 - same thing with 9.95 0.06 0.0005
    //9.4 0.0095 0.0005 still settles into lots of fast oscillation, sometimes very slightly damped
    //8.0 0.003 0.0005 still settles into that fast oscillation (maybe marginally slower) HANG ON is it trying to collision avoid? At one point it went backwards!!! There are no cars around! NOPE apparently not
    //8.0 0.001 0.0005 still settles into that fast oscillation
    //3.0 0.8 0.00005 is actually the best result so far. Starts with a very big oscillation that slowly settles down.
    //3.0 0.3 0.0005 has big oscillations later on especially as the car bounces and gets damaged - FIXED waypoint code at this point.
    //disabled beam->hydrospeedcoupling at this point, so steering control is now analog and added a speed limit on steering changes with PID
    //0.2 0 0 definitely too small P
    //0.2-0.7 0 0 it rolls over on the second steering adjustment
    //1.5 0 0 Still rolls over at the start. Settles into a wide amplitude, slow oscillation after the first couple of turns. It's like when the error is big, it's steering too much, but when the error is small it's steering too little.
    //Slowed max PID steering speed to 1.0. Now at 1.8 0 0, it steers in a wide amplitude slow slalom constantly from the start.
    //0.4 0 0 and reduced error logarithmically and it still seems to be turning the car way too much
    //All with error reduced as error = log(1.0 + abs(error)) where error is sideways dist from track center:
    //0.02 0 0 DEFINITELY too little steering input, it runs almost 100% straight, into the walls
    //0.03 0 0 v smooth but still too little - only just misses the wall and then goes so wide on turn it ends in gravel, but not a million miles off!
    //0.04 0 0 misses the wall a bit more - we can start to see where the lack of steering input is starting to make it trace a very big very slow zig zag
    //0.05 0 0 misses the wall a bit more, possibly still a bit slow. The zig zig eventually makes the car roll over.
    //0.06 0 0 similar behaviour to the above.
    //0.08 0 0 still seems maybe too little. Crashes into the wall on the left.
        
    //New test using steeringForceRaw as our PID error value:
    //0.08 0.0 0.0 Smooth up to first bend then doesn't seem to turn in quite enough, then seems to over-correct
    //0.10 0 0 actually seemed to steer slightly too much for the bend - elsewhere nice and light / smooth. Steering is too slow to recover from a slide.
    //0.09 0 0 as above
    //0.085 0 0 almost perfect round the whole track except turns too tight on that very first right hander but corrects successfully after going on grass
    //0.0825 0 0 still turns too tight on that right hander I think, then ends up on the gravel on left hand side by screwing up the correction
    //0.07 0 0 still turns slightly too tightly on that first right hander and very slightly too much on the other bends as well
    //0.06 0 0 turns marginally too tightly on the first right hander, then a bit too tightly on the second part of the same bend which winds it on the grass into the right hand wall!
    //0.05 0 0 Vehicle 1 was almost right on the first part of the right hander (fractionally too tight), then when heading for a wall didn't correct sharply enough - but we don't have wall avoidance code! It should be driving slower and turning quicker if way off course like that!
    //0.065 0.01 0.01 Seems a little bit unstable at times
    //0.065 0.01 0.02 Almost perfect - gave us a 2:14.01 on the first lap but crashed at that first right hander on the second lap!
    //0.065 0.015 0.03 Crashed near the pit lane entrance. With these settings recovery from skids seems slow as well. maxsteerspeed currently = 9.0 / (10.0 + fabs(beam->WheelSpeed / 2.0));
    //0.07 0.015 0.03 maxsteerspeed = 11.0 / (10.0 + fabs(beam->WheelSpeed / 2.0)); Good! did a 2:14.94 on first lap. Crashed on first right hander again as it steered hard right into the wall when overhanging grass. Possible roll avoidance manoeuvre?
    //Same settings as above (roll correction code weakened but may be a coincidence): 2:13.22 first lap. 2:16.25 second lap. On 3rd lap it did graze the wall on that first right hander after a lockup - maybe lower approach speed?
    //0.075 0.015 0.03 maxsteerspeed = 11.0 *** Used for MANY MONTHS up to August 2017. Mostly good but crashed on bend just before pit lane entrance. Game was lagging a lot. TODO Also check "off course" logic.
    
    lastErrorAcrossTrack = 0.0f;
    sumErrorsAcrossTrack = 0.0f;
    lastWaypointNavigated = Vector3::ZERO;

    beam = b;
}

VehicleAI::~VehicleAI()
{
    std::map<Ogre::String, std::vector<CollisionAvoidance_GridQuad*>>::iterator iter;
    for (iter = raceGridQuads.begin(); iter!=raceGridQuads.end(); iter++)
    {
        std::vector<CollisionAvoidance_GridQuad*> gridQuads = iter->second;
        for (int i=0; i<gridQuads.size(); i++)
        {
            delete gridQuads[i];
        }
        gridQuads.clear();
    }
	if (character != nullptr)
	{
		delete(character);
	}
}

//Define the static vectors for lane based collision avoidance - cosmic vole May 26 2017
std::map<int, std::vector<Ogre::Vector3>> VehicleAI::raceLanePoints;//!< Map of lists of coordinates that divide each race track into lanes for overtaking and collision avoidance. The key is the raceID. cosmic vole May 18 2017
std::map<Ogre::String, int> VehicleAI::raceWaypointToLanePointIndex;//!< Maps raceID + "|||" + waypoint name to an index into that race's entry in raceLanePoints. cosmic vole May 18 2017 
std::map<Ogre::String, std::vector<CollisionAvoidance_GridQuad*>> VehicleAI::raceGridQuads;

void VehicleAI::SetActive(bool value)
{
    is_enabled = value;
    //cosmic vole added AI Character driver model November 21 2016
    if (value && BSETTING("ShowAIDrivers", true) && character == nullptr)
    {
        //int aiColour = /*PlayerColours::getSingleton().getRandomColourNum();*/Ogre::ColourValue(frand(), frand(), frand(), 1.0f);
        int aiColor = RoR::Networking::GetRandomColorNum();
		character = new Character(beam->trucknum, aiColor);//CharacterFactory::getSingleton().createAIInstance(beam->trucknum, (int)aiColour);
        character->setBeamCoupling(true, beam);
    }
    else if (!value && character != nullptr)
    {
        delete character;//CharacterFactory::getSingleton().removeAIInstance(beam->trucknum);
    }
}

bool VehicleAI::IsActive()
{
    return is_enabled;
}

void VehicleAI::AddWaypoint(String& id, Vector3& point)
{
    if (current_waypoint == Vector3::ZERO)
        current_waypoint = point;

    free_waypoints++;
    waypoints.emplace(free_waypoints, point);
    waypoint_ids.emplace(id, free_waypoints);
    waypoint_names.emplace(free_waypoints, id);
    // Adjusted waypoints are the same as the original ones until adjustment takes place - cosmic vole June 3 2017
    adjusted_waypoints.emplace(free_waypoints, point);
    if (free_waypoints > 1) //Can't do this yet as we likely haven't got a raceID and haven't necessarily got the edge points yet.
        CollisionAvoidance_GenerateLanes(free_waypoints-1, free_waypoints, waypoints, road_edge_points_left, road_edge_points_right);
}

void VehicleAI::AddWaypoints(AngelScript::CScriptDictionary& d)
{
    for (auto item : d)
    {
        Ogre::Vector3* point;
        item.GetValue(point, item.GetTypeId());
        Ogre::String key(item.GetKey());
        this->AddWaypoint(key, (*point));
    }
}

//cosmic vole March 3 2017
void VehicleAI::InsertWaypoint(String& id, Vector3& point, int position)
{
   
    free_waypoints++;
    if (position > free_waypoints)
        position = free_waypoints;
    if (position < 0)
        position = 0;
    
    //As position is actually a key in the maps, we need to renumber all the waypoints
    //that come after the one we've just inserted, although we don't rename them. cosmic vole March 7 2017
    for (int i=free_waypoints; i>position; i--)
    {
        int h = i-1;
        Vector3 pointToMove = waypoints[h];
        waypoints[i] = pointToMove;
        String idToMove = waypoint_names[h];
        float speedToMove = waypoint_speed[h];
        float adjSpeedToMove = adjusted_waypoint_speed[h];
        float powerToMove = waypoint_power[h];
        int eventToMove = waypoint_events[h];
        waypoint_ids[idToMove] = i;
        waypoint_names[i] = idToMove;
        if (speedToMove)
            waypoint_speed[i] = speedToMove;
        if (adjSpeedToMove)
            adjusted_waypoint_speed[i] = speedToMove;
        if (powerToMove)
            waypoint_power[i] = powerToMove;
        if (eventToMove)
            waypoint_events[i] = eventToMove;
        pointToMove = adjusted_waypoints[h];
        adjusted_waypoints[i] = pointToMove;
    }
    waypoints[position] = point;
    waypoint_ids[id] = position;
    waypoint_names[position] = id;
    adjusted_waypoints[position] = point;
    
    if (current_waypoint == Vector3::ZERO)
    {
        current_waypoint = point;
        current_waypoint_id = position;
    }
    else if (current_waypoint_id > position && current_waypoint_id < free_waypoints)
    {
        //The waypoint the vehicle was on just got moved
        current_waypoint_id++;
        current_waypoint = waypoints[current_waypoint_id];
    }
    else if (current_waypoint_id == position)
    {
        //The vehicle now has the new waypoint to go to before its existing waypoint
        current_waypoint = point;
    }
}

void VehicleAI::AddEvent(String& id, int& ev)
{
    int waypointid = waypoint_ids[id];
    if (waypointid)
        waypoint_events.emplace(waypointid, ev);
}

void VehicleAI::SetValueAtWaypoint(String& id, int& value_id, float& value)
{
    int waypointid = waypoint_ids[id];
    if (waypointid)
    {
        switch (value_id)
        {
        case AI_SPEED:
            waypoint_speed.emplace(waypointid, value);
            adjusted_waypoint_speed.emplace(waypointid, value);
            break;
        case AI_POWER:
            waypoint_power.emplace(waypointid, value);
            break;
        default:
            break;
        }
    }
}

//cosmic vole June 25 2017
void VehicleAI::SetCurrentWaypoint(Ogre::String& id)
{
    int waypointid = waypoint_ids[id];
    current_waypoint_id = waypointid;
    if (current_waypoint_id < /*0*/1)
        current_waypoint_id = /*0*/1;
    if (current_waypoint_id >/*=*/ free_waypoints)
        current_waypoint_id = free_waypoints;// - 1;
    if (current_waypoint_id < adjusted_waypoints.size())
        current_waypoint = adjusted_waypoints[current_waypoint_id];
    else if (current_waypoint_id < waypoints.size())
        current_waypoint = waypoints[current_waypoint_id];
    else
        current_waypoint = Vector3::ZERO;
}

//cosmic vole June 25 2017
void VehicleAI::SetNumWaypointsInLap(int numWaypointsInLap)
{
   num_waypoints_in_lap = numWaypointsInLap; 
}

//cosmic vole July 9 2017
int VehicleAI::FindNextWaypointIndex(Ogre::Vector3 location, int startWaypointID, int endWaypointID)
{
    if ((waypoints.size() < 1))// || (startWaypointID > waypoints.size()))
    {
        LOG("FindNextWaypointIndex() called when there are NO WAYPOINTS! trucknum: "+TOSTRING(beam->trucknum)+" free_waypoints: "+TOSTRING(free_waypoints)+" startWaypointID: "+TOSTRING(startWaypointID)+" waypoints.size(): "+TOSTRING(waypoints.size())+" location: "+TOSTRING(location)+".");
        return 0;
    }
    if (startWaypointID <= 0 || startWaypointID >= waypoints.size())
        startWaypointID = 1;
    if (endWaypointID <= 0 || endWaypointID > waypoints.size())
        endWaypointID = waypoints.size();
    location.y = 0.0f;
    Vector3 waypoint = waypoints[startWaypointID];
    Vector3 lastWaypoint = Vector3::ZERO;
    float minDist = -1.0f;
    int closestWptIndex = -1;
    for (int i = startWaypointID + 1; i <= endWaypointID; i++)
    {
        int l = i - 1;
        lastWaypoint = waypoint;
        lastWaypoint.y = 0.0f;
        
        waypoint = waypoints[i];
        waypoint.y = 0.0f;
        float dist = lastWaypoint.distance(location);
        if (dist < minDist || minDist < 0.0f)
        {
            //This is the closest waypoint we've found so far.
            //So, are we in front of or behind it on the track?
            //TODO We also need a function that tests whether "location" and "waypoint" are on the same bit of track or not. It can use the surrounding waypoints and edge points provided they've been defined.
            Vector3 trackDirection = waypoint - lastWaypoint;//??? Track direction isn't REALLY between the waypoints. That's the racing line.
            trackDirection.y = 0.0f;
            if (trackDirection == Vector3::ZERO)
                continue;
            trackDirection.normalise();
            Vector3 toLocation = location - lastWaypoint;
            toLocation.y = 0.0f;
            float distAlongTrackToLocation = toLocation.dotProduct(trackDirection);
            bool lastWaypointIsPastLocation = (distAlongTrackToLocation < 0.0f);
            if (lastWaypointIsPastLocation)
            {
                //We've found our closest "next waypoint" so far
                closestWptIndex = l;
                minDist = dist;
            }
        }
    }
    if (minDist < 0.0f)
    {
        LOG("For truck "+TOSTRING(beam->trucknum)+" location "+TOSTRING(location)+" closest waypoint was not found!");
        return startWaypointID;
    }
    else
    {
        LOG("For truck "+TOSTRING(beam->trucknum)+" location "+TOSTRING(location)+" closest waypoint is "+TOSTRING(closestWptIndex)+": "+TOSTRING(waypoints[closestWptIndex])+" Dist: "+TOSTRING(minDist)+".");
        return closestWptIndex;
    }
}

//cosmic vole July 9 2017
Ogre::String VehicleAI::FindNextWaypointID(Ogre::Vector3& location, int startWaypointID, int endWaypointID)
{
    return waypoint_names[FindNextWaypointIndex(location, startWaypointID, endWaypointID)];
}

//cosmic vole March 16 2017
void VehicleAI::AddRoadEdgePointLeft(Vector3& point)
{
    //if (current_left_edge_point == Vector3::ZERO)
    //    current_left_edge_point = point;

    road_edge_points_left.push_back(point);
    //free_edge_points_left++;
    //waypoi.emplace(free_waypoints, point);
    //waypoint_ids.emplace(id, free_waypoints);
    //waypoint_names.emplace(free_waypoints, id);
}

//cosmic vole March 16 2017
void VehicleAI::AddRoadEdgePointRight(Vector3& point)
{
    road_edge_points_right.push_back(point);
}

//cosmic vole April 21 2017
int VehicleAI::GetClosestRoadEdgePointIndexLeft(Vector3& location)
{
    
}

//cosmic vole April 21 2017
int VehicleAI::GetClosestRoadEdgePointIndexRight(Vector3& location)
{
    
}

//cosmic vole March 16 2017
void VehicleAI::FindRoadEdgePoints(int waypointID, Vector3 location, Vector3& leftEdgePoint, Vector3& rightEdgePoint, int& nextLeftIndex, int& nextRightIndex)
{

    nextLeftIndex = -1;
    nextRightIndex = -1;
    float minDist = -1.0f;
    //int minDistIndex = -1;
    //int start = 0;
    //int end = road_edge_points_left.size()-1;
    //if (end < 0 || road_edge_points_right.size() < 1)
    //{
    //    leftEdgePoint = rightEdgePoint = Vector3(Vector3::ZERO);
    //    return;
    //}
    
    //If a location but no waypoint was provided, we just look for the closest waypoint on the track. Note that this may cross track boundaries.
    if (waypointID < 0)
    {
        if (location == Vector3::ZERO)
        {
            leftEdgePoint = rightEdgePoint = Vector3(Vector3::ZERO);
            return;
        }
        for (int i=/*0*/1; i<=free_waypoints/*<waypoints.size()*/; i++)
        {
            float dist = location.distance(waypoints[i]);
            if (dist < minDist || minDist < 0.0f)
            {
                minDist = dist;
                waypointID = i;
            }
        }       
    }
    else if (waypointID >/*=*/ free_waypoints)
    {//This was causing MAJOR problems - it somehow broke ALL the GridQuads when I erroneously clamped to free_waypoints-1. Maybe safest to compare against waypoints.size().... June 29th 2017
        waypointID = free_waypoints /*- 1*/;
    }
    else if (waypointID == 0)
    {
        //They are NOT zero indexed!
        waypointID = 1;
    }
    
    if (location == Vector3::ZERO)
    {
        location = waypoints[waypointID];
    }
    
    if (road_edge_points_left.empty() || road_edge_points_right.empty())
    {
        leftEdgePoint = rightEdgePoint = Vector3(Vector3::ZERO);
        return;
    }
    
    //TODO FIX THIS! At the moment this assumes the closest left and right edge points to each other are at the same index. On a circuit they definitely won't be
    //because there'll be a lot more points round the outside of the track than inside!
    //Simulate driving around the track using the waypoints and test which edge points we have passed as we go.
    //Although this is a slow way to find the one we want, it should avoid problems with unusual tracks like figure of 8 etc. cosmic vole June 22 2017
    
    int leftIndex = 0;
    int rightIndex = 0;
    Vector3 lastWaypoint = Vector3::ZERO;
    Vector3 waypoint = Vector3::ZERO;
    leftEdgePoint = road_edge_points_left[0];
    rightEdgePoint = road_edge_points_right[0];
    Vector3 lastLeftEdgePoint = leftEdgePoint;
    Vector3 lastRightEdgePoint = rightEdgePoint;
    
    bool passedLocation = false;
    bool passedLastLeft = false;
    bool passedLastRight = false;
    bool passedNextLeft = false;
    bool passedNextRight = false;
    
    float distAlongTrackToLeft = 0.0f;
    float distAlongTrackToRight = 0.0f;
    float distAcrossTrackToLeft = 0.0f;
    float distAcrossTrackToRight = 0.0f;
    float distAlongTrackToLocation = 0.0f;
    float distAcrossTrackToLocation = 0.0f;

    float lastDistAlongTrackToLeft = 0.0f;
    float lastDistAlongTrackToRight = 0.0f;
    float lastDistAcrossTrackToLeft = 0.0f;
    float lastDistAcrossTrackToRight = 0.0f;
    float lastDistAlongTrackToLocation = 0.0f;
    float lastDistAcrossTrackToLocation = 0.0f;

    for (int i = 1/*0 They are NOT zero indexed!*/; i <= waypointID; i++) //<free_waypoints; i++)
    {
        lastWaypoint = waypoint;
        waypoint = waypoints[i];
        
        //TODO why are we saving these? don't we just need the bools and coords? Actually no because we want to see where the sign changes as well probably.
        lastDistAlongTrackToLeft = distAlongTrackToLeft;
        lastDistAlongTrackToRight = distAlongTrackToRight;
        lastDistAcrossTrackToLeft = distAcrossTrackToLeft;
        lastDistAcrossTrackToRight = distAcrossTrackToRight;
        lastDistAlongTrackToLocation = distAlongTrackToLocation;
        lastDistAcrossTrackToLocation = distAcrossTrackToLocation;

        if (lastWaypoint == Vector3::ZERO)
        {
            continue;
        }
        Vector3 trackDirection = Vector3::ZERO;
        Vector3 trackCentre = Vector3::ZERO;
        trackDirection = waypoint - lastWaypoint;//??? Track direction isn't REALLY between the waypoints. That's the racing line.
        //trackCentre = getClosestPointOnLine(lastWaypoint, waypoint, mAgentPosition, false);
        if (trackDirection != Vector3::ZERO)
        {
            trackDirection.y = 0.0f;
            trackDirection.normalise();
            Vector3 acrossTrack = Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_Y) * trackDirection;
            //trackDirection.normalise();
            acrossTrack.y = 0.0f;
            acrossTrack.normalise();
            //Vector3 toTarget = current_waypoint - mAgentPosition;
            //float distAlongTrack = toTarget.dotProduct(trackDirection);
            //float distAcrossTrack = toTarget.dotProduct(acrossTrack);
            Vector3 toNextLeft = leftEdgePoint - waypoint;//WAS lastWaypoint
            Vector3 toNextRight = rightEdgePoint - waypoint;
            //If these distances are negative values, it means waypoint is past the left or right points. If they're positive, they are further ahead on the track than waypoint.
            //We keep looping along the track edges until we find the first points on the left and right that waypoint has not passed
            passedNextLeft = false;
            do
            {//BUG !!!! In the unusual situation that the waypoint is past the location, we want to stop when we find the first edge points past *location* not *waypoint*
            //!!!       whereas normally it IS past waypoint that we need
                if (passedNextLeft)
                {
                    leftIndex++;
                    if (leftIndex >= road_edge_points_left.size())
                    {
                        leftIndex--;
                        break;
                    }
                    lastLeftEdgePoint = leftEdgePoint;
                    leftEdgePoint = road_edge_points_left[leftIndex];
                    if (leftEdgePoint == lastLeftEdgePoint)
                        LOG("road_edge_points_left duplicate edge point. lastLeftEdgePoint: " + TOSTRING(lastLeftEdgePoint) + " leftEdgePoint: " + TOSTRING(leftEdgePoint) + " index: " + TOSTRING(leftIndex) + ".");
                    toNextLeft = leftEdgePoint - waypoint;
                }
                float distAlongTrackToLeft = toNextLeft.dotProduct(trackDirection);
                float distAcrossTrackToLeft = toNextLeft.dotProduct(acrossTrack);
           
                passedNextLeft = (distAlongTrackToLeft <= 0.0f);
            }
            while (passedNextLeft);
            
            passedNextRight = false;
            do
            {
                if (passedNextRight)
                {
                    rightIndex++;
                    if (rightIndex >= road_edge_points_right.size())
                    {
                        rightIndex--;
                        break;
                    }
                    lastRightEdgePoint = rightEdgePoint;
                    rightEdgePoint = road_edge_points_right[rightIndex];
                    if (rightEdgePoint == lastRightEdgePoint)
                        LOG("road_edge_points_left duplicate edge point. lastRightEdgePoint: " + TOSTRING(lastRightEdgePoint) + " rightEdgePoint: " + TOSTRING(rightEdgePoint) + " index: " + TOSTRING(rightIndex) + ".");
                    toNextRight = rightEdgePoint - waypoint;                    
                }
                float distAlongTrackToRight = toNextRight.dotProduct(trackDirection);
                float distAcrossTrackToRight = toNextRight.dotProduct(acrossTrack);
            
                passedNextRight = (distAlongTrackToRight <= 0.0f);
            }
            while (passedNextRight);

            
            Vector3 toLocation = location - waypoint;//lastWaypoint;//TODO we only need to do these from waypoint. Can copy lastWaypoint value next iteration each time.
            float distAlongTrackToLocation = toLocation.dotProduct(trackDirection);
            float distAcrossTrackToLocation = toLocation.dotProduct(acrossTrack);
            
            //passedNextLeft = (distAlongTrackToLeft <= 0.0f);
            //passedNextRight = (distAlongTrackToRight <= 0.0f);
            
            //We HAVE TO find the pair of waypoints where lastWaypoint is not past location (or is ZERO) and waypoint IS past or level with location
            //Then between those we find the four edge points where the same is true.
            //TODO If the location is completely outside all track bounds, the second part of this condition may never be satisfied
            //If ALL the waypoints and edge points are ahead of the location (e.g. the location is behind the start of a drag strip), we can just return the first edge points or Vector3::ZERO
            //****************************************************************************************************************
            //**** BUG This is ONLY valid if location is actually within the *current* track bounds!!! Say, for example it's on a series of hairpins, if we start at the start / finish line, we could get a false positive!
            //** AND to see if it's in track bounds we need to check our candidate edge points
            //** At the moment "location" is always a waypoint in the calling code. We could also just allow the nearest waypoint OR even any points on track closer than the distance to the second nearest waypoint to location.
            //** BUT closest waypoint will give false positives if we're on track but the opposite side to the waypoint
            passedLocation = (distAlongTrackToLocation <= 0.0f && lastDistAlongTrackToLocation > 0.0f); //abs(distAlongTrackToLocation) < 20.0f && abs(distAcrossTrackToLocation) < 20.0f);
            
            //bool reachedLocation = (abs(distAlongTrackToLocation) < 6.5f && abs(distAcrossTrackToLocation) < 10.0f);
            //It's possible that the last waypoint hasn't reached location but the next has passed it too far to detect in which case the vector will change sign - OR the distance along track will be greater than distance between the waypoints?
            //It's possible when we've incremented the waypoint, that we passed several edge points....... so we need to keep looping through them and testing against location
                        
            //bool reachedNextLeft = (abs(distAlongTrackToLeft) < 6.5f && abs(distAcrossTrackToLeft) < 10.0f);
            //bool reachedNextRight = (abs(distAlongTrackToRight) < 6.5f && abs(distAcrossTrackToRight) < 10.0f);
            
            
        }
    }
    
    
    
    
    //If we got here, we've found the closest four edge points on the track and we'll use linear interpolation along the track sides to get the results
        /*
        int i = minDistIndex;
        int j;
        if (i == end)
        {
            //wrap around. If the track doesn't actually do this, that set of coordinates will just get ignored
            j = start;
        }
        else
        {
            j = i + 1;
        }
        */
        Vector3/*&*/ curLeftStart = lastLeftEdgePoint;//road_edge_points_left[i];
        Vector3/*&*/ curRightStart = lastRightEdgePoint;//road_edge_points_right[i];
        Vector3/*&*/ curLeftEnd = leftEdgePoint;//road_edge_points_left[j];
        Vector3/*&*/ curRightEnd = rightEdgePoint;//road_edge_points_right[j];
        
        //We find our closest track edge points by finding the closest point on the track center line to the supplied location
        //and then returning the closest points on the left and right side of the track to that point, where the left and right
        //side of the track are just straight lines between the edge points we have already found.
        //At some point this could be improved by fitting curves to the edge points instead of straight lines, but that's probably
        //OTT I would think. cosmic vole March 29 2017
        Vector3 midpointStart = curLeftStart + ((curRightStart - curLeftStart) * 0.5f);
        Vector3 midpointEnd = curLeftEnd + ((curRightEnd - curLeftEnd) * 0.5f);
        Vector3 closestTrackCenter = getClosestPointOnLine(midpointStart, midpointEnd, location, true);
        
        leftEdgePoint = getClosestPointOnLine(curLeftStart, curLeftEnd, closestTrackCenter, true);
        rightEdgePoint = getClosestPointOnLine(curRightStart, curRightEnd, closestTrackCenter, true);        
    
    //TODO the logic for these may need looking at
    leftIndex++;
    rightIndex++;
    if (leftIndex >= road_edge_points_right.size())
        leftIndex--;
    if (rightIndex >= road_edge_points_right.size())
        rightIndex--;
    nextLeftIndex = leftIndex;
    nextRightIndex = rightIndex;
}

float VehicleAI::soonestCollisionTime()
{
    return soonest_collision_time;
}

//cosmic vole July 6 2017
void VehicleAI::FollowLaneToggle(int laneNumber)
{
    if (VehicleAI::raceGridQuads.size() == 0)
    {
        //No lanes defined.
        #ifdef USE_MYGUI
            RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_INFO, RoR::Console::CONSOLE_SYSTEM_NOTICE, UTFString("No lanes defined to follow!"), "warning.png", 3000);
            RoR::App::GetGuiManager()->PushNotification("Follow Lane:", "No lanes defined to follow!");
        #endif // USE_MYGUI
        LOG("No lanes defined to follow!");
        return;
    }
    else if (waypoints.size() == 0)
    {
        //This truck has no waypoints assigned.
        #ifdef USE_MYGUI
            RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_INFO, RoR::Console::CONSOLE_SYSTEM_NOTICE, UTFString("Cannot follow lane. Truck has no waypoints!"), "warning.png", 3000);
            RoR::App::GetGuiManager()->PushNotification("Follow Lane:", "Truck has no waypoints assigned!");
        #endif // USE_MYGUI
        LOG("Follow Lane: Truck has no waypoints assigned!");
        return;
    }
    if (!is_enabled)
        SetActive(true);
    int oldLaneFollow = lane_force_follow;
    if (laneNumber == lane_force_follow)
    {
        //Toggle off lane following
        lane_force_follow = -1;
        #ifdef USE_MYGUI
            RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_INFO, RoR::Console::CONSOLE_SYSTEM_NOTICE, UTFString("Lane following deactivated."), "information.png", 3000);
            RoR::App::GetGuiManager()->PushNotification("Follow Lane:", "Lane following deactivated.");
        #endif // USE_MYGUI
        LOG("Lane following deactivated.");
    }
    else
    {
        lane_force_follow = laneNumber;
    }
    int lastWaypointID = free_waypoints;
    if (lane_force_follow >= 0)
    {
        if (lane_force_follow != lane_change_to || lane_change_start_wpt > current_waypoint_id)
        {
            CollisionAvoidance_FollowLane(this, lane_force_follow, current_waypoint_id, lastWaypointID, true, false);
            lane_change_from = lane_force_follow;
            lane_change_to = lane_force_follow;
            lane_change_start_wpt = current_waypoint_id;
            lane_change_mid_wpt = current_waypoint_id + 1;
            lane_change_end_wpt = lastWaypointID;
        }
        else if (lane_change_end_wpt < lastWaypointID)
        {
            //Continue to follow the selected lane but no smooth transition is needed, as one was already scheduled
            CollisionAvoidance_FollowLane(this, lane_force_follow, lane_change_end_wpt, lastWaypointID, false, false);
            lane_change_from = lane_force_follow;
            lane_change_to = lane_force_follow;
            lane_change_end_wpt = lastWaypointID;            
        }
        #ifdef USE_MYGUI
            RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_INFO, RoR::Console::CONSOLE_SYSTEM_NOTICE, UTFString("Lane following activated."), "information.png", 3000);
            RoR::App::GetGuiManager()->PushNotification("Follow Lane:", "Lane following activated.");
        #endif // USE_MYGUI
        LOG("Lane following activated for lane " + TOSTRING(lane_force_follow));

    }
    else //Disable lane following (resume original waypoints)
    {
        CollisionAvoidance_FollowLane(this, -1, current_waypoint_id, current_waypoint_id + 14, true, true);
        CollisionAvoidance_FollowLane(this, -1, current_waypoint_id + 15, lastWaypointID, false, true);
        lane_change_from = oldLaneFollow;
        lane_change_to = -1;
        lane_change_start_wpt = current_waypoint_id;
        lane_change_mid_wpt = current_waypoint_id + 1;
        lane_change_end_wpt = current_waypoint_id  + 14;
    }
}

//cosmic vole April 20 2017
void VehicleAI::CollisionAvoidance_GenerateLanes(int startWaypoint, int endWaypoint, std::map<int, Ogre::Vector3>& waypoints, /*int startLeftEdgePoint, int startRightEdgePoint,*/ std::vector<Vector3>& road_edge_points_left, std::vector<Vector3>& road_edge_points_right /*, bool wrapAround = true*/)//Ogre::Vector3& start, Ogre::Vector3& end) //TODO instead of start and end vectors, use waypoints
{
    float minLaneWidth = 2.5f;//3.0f;
    //float gridRectLength = 3.0f;
    int numLanes = 3;//4;//3;
    //The current number of lanes on the track can vary based on track width - cosmic vole May 10 2017
    int curNumLanes = 3;//4//3;
    /* Wrong: They are indexed from 1! cosmic vole July 5 2017:
    if (startWaypoint < 0)
        startWaypoint = 0;
    if (endWaypoint < 0)
        endWaypoint = waypoints.size() - 1; */
    if (startWaypoint < 1)
        startWaypoint = 1;
    if (endWaypoint < 0)
        endWaypoint = waypoints.size();
    //if (startLeftEdgePoint < 0)
    //    startLeftEdgePoint = 0;
    //if (startRightEdgePoint < 0)
    //    startRightEdgePoint = 0;
    std::vector<Vector3>& lanePoints = raceLanePoints[raceID];
    lanePoints.clear();
    Vector3 leftEdgePointStart;
    Vector3 rightEdgePointStart;
    Vector3 leftEdgePointEnd;
    Vector3 rightEdgePointEnd;
    int nextIndexLeft;
    int nextIndexRight;
    Vector3 start = waypoints[startWaypoint];
    Vector3 end = waypoints[endWaypoint];
    Vector3 wpt = start;
    int i = startWaypoint - 1;
    
    if (waypoints.empty() || road_edge_points_left.empty() || road_edge_points_right.empty())
    {
        return;
    }
    
    FindRoadEdgePoints(endWaypoint, end, leftEdgePointEnd, rightEdgePointEnd, nextIndexLeft, nextIndexRight);
    FindRoadEdgePoints(startWaypoint - 1, start, leftEdgePointStart, rightEdgePointStart, nextIndexLeft, nextIndexRight);
    //LOG("Gen Lanes. FindRdEdgPts() wpt("+TOSTRING(start.x)+","+TOSTRING(start.z)+") left("+TOSTRING(leftEdgePointStart.x)+","+TOSTRING(leftEdgePointStart.z)+") right("+TOSTRING(rightEdgePointStart.x)+","+TOSTRING(rightEdgePointStart.z)+")");
    Vector3 leftRoadEdge = leftEdgePointStart;
    Vector3 rightRoadEdge = rightEdgePointStart;
    
    //if (wrapAround)
    {
        //Use leftEdgePointEnd and rightEdgePointEnd to be our zeroth road edge points. These are behind the vehicle's starting position because the track wraps around.
        //*** BUG Although strictly speaking most of the cars on the grid will be way behind the end waypoint at the start of the race! Problem!
        //Maybe the best way to get around that is give each of the cars a different starting waypoint.????
        //For that to work properly we'd have to differentiate between track waypoint index and vehicle waypoint index (ID)
        //How about, for all of them write an extra "zeroth" lap and have a SetFirstWaypointID() Angelscript method that's a point near the end of that "zeroth" lap. IRL that would have been the formation lap.
        
        
    }
    
    //LOG(Ogre::String("Generate Lanes:\n"));
    
    //*** BUG Think this is wrong because these should be at the NEXT WAYPOINT not just next edge points!
    //Vector3 nextLeftRoadEdge = road_edge_points_left[nextIndexLeft];//??? road_edge_points_left.size()];
    //Vector3 nextRightRoadEdge = road_edge_points_right[nextIndexRight];
    int lanePointsIndex = 0;
    int lastLanePointsIndex;
    int numNewQuads = 0;
    int lastNumNewQuads = 0;
    int firstNumNewQuads = -1;
    std::vector<CollisionAvoidance_GridQuad*> *pWaypointQuads = nullptr;
    std::vector<CollisionAvoidance_GridQuad*> *pLastWaypointQuads = nullptr;
    std::vector<CollisionAvoidance_GridQuad*> *pFirstWaypointQuads = nullptr;
    while (1)
    {
        lastLanePointsIndex = lanePointsIndex;
        lanePointsIndex = lanePoints.size();
        Vector3 acrossRoad = leftRoadEdge - rightRoadEdge;
        Vector3 acrossRoadDir = acrossRoad;
        acrossRoadDir.normalise();
        float roadWidth = acrossRoad.length();
        int maxNumLanes = (int)(roadWidth / minLaneWidth);
        if (maxNumLanes < numLanes && maxNumLanes > 0)
        {
            curNumLanes = maxNumLanes; //numLanes = maxNumLanes;
            //Go back to start and try again!
            //leftRoadEdge = leftEdgePointStart;
            //rightRoadEdge = rightEdgePointStart;
            //lanePoints.clear();
        }
        else
        {
            curNumLanes = numLanes;
        }
        float laneWidth = roadWidth / (float)curNumLanes;//numLanes;
        Vector3 laneRight = rightRoadEdge;
        lanePoints.push_back(laneRight);
        for (int j=0; j<numLanes-1; j++)
        {
            Vector3 laneLeft = laneRight + (laneWidth * acrossRoadDir);
            float debugLaneWidth = (laneRight - laneLeft).length();
            if (debugLaneWidth > 200.0f || debugLaneWidth > roadWidth)
            {
                LOG("AARGH! waypoint:("+TOSTRING(wpt)+") lane: "+TOSTRING(j)+" laneWidth:("+TOSTRING(laneWidth)+") roadLeft:("+TOSTRING(leftRoadEdge)+") roadRight:("+TOSTRING(rightRoadEdge)+") laneLeft:("+TOSTRING(laneLeft)+") laneRight:("+TOSTRING(laneRight)+") i:"+TOSTRING(i));
                return;
            }            
            if (j >= curNumLanes-1)
            {
                //At the moment we use ZERO vectors to indicate this part of the track has
                //a reduced number of lanes. A cleaner solution might be some kind of lanes class
                //that holds a lane count and the lane points itself instead of just using Vector3s. - cosmic vole May 10th 2017
                lanePoints.push_back(Vector3::ZERO);
                //lanePoints.push_back(Vector3::ZERO);
            }
            else
            {
                lanePoints.push_back(laneLeft);
            }
            laneRight = laneLeft;
        }

        if (curNumLanes < numLanes)
        {
            lanePoints.push_back(Vector3::ZERO);
            //lanePoints.push_back(Vector3::ZERO);
        }
        else
        {
            Vector3 laneLeft = leftRoadEdge;
            //lanePoints.push_back(laneRight);
            lanePoints.push_back(laneLeft);
        }
        //Store a lookup from the race ID and waypoint name to get these lane points
        Ogre::String raceWaypointKey = TOSTRING(raceID) + "|||" + waypoint_names[i];
        raceWaypointToLanePointIndex[raceWaypointKey] = lanePointsIndex;
        if (i >= startWaypoint)//!! BUG This may be causing an GenerateLanes to be run over and over as it suggests no GridQuad will ever be stored for waypoint 0
        {
            //TODO Generate collision avoidance grid quads for last pair of waypoints as well
            //TODO we really need a standard way to make a waypoint behind the starting position of each vehicle to generate the first grid squares
            //CollisionAvoidance_GridQuad quad;
            std::vector<CollisionAvoidance_GridQuad*>& waypointQuads = raceGridQuads[raceWaypointKey];
            int numExistingQuads = waypointQuads.size();
            pLastWaypointQuads = pWaypointQuads;
            pWaypointQuads = &waypointQuads;
            if (!pFirstWaypointQuads)
                pFirstWaypointQuads = pWaypointQuads;
            lastNumNewQuads = numNewQuads;
            numNewQuads = 0;
            for (int j=0; j<numLanes/*-1*/; j++)
            {
                if (lanePointsIndex+j+1 >= lanePoints.size())
                    break;
                if (lastLanePointsIndex+j+1 >= lanePoints.size())
                    break;
                Vector3 laneRightRear = lanePoints[lastLanePointsIndex+j];
                Vector3 laneLeftRear = lanePoints[lastLanePointsIndex+j+1];
                Vector3 laneRightFront = lanePoints[lanePointsIndex+j];
                Vector3 laneLeftFront = lanePoints[lanePointsIndex+j+1];
                if (laneLeftRear == Vector3::ZERO ||
                laneRightRear == Vector3::ZERO ||
                laneLeftFront == Vector3::ZERO ||
                laneRightFront == Vector3::ZERO)
                    continue;// The track is too narrow for this lane here
                CollisionAvoidance_GridQuad *pQuad;
                if (j < numExistingQuads)
                {
                    //This quad has already been constructed on a previous invocation of this function!
                    pQuad = waypointQuads[j];
                    if (pQuad == nullptr)
                    {
                        LOG("A null grid quad was found!");
                        return;
                    }
                }
                else
                {
                    pQuad = new CollisionAvoidance_GridQuad();
					pQuad->SetSimController(ChampionshipManager::getSingleton().GetSimController());
                    waypointQuads.push_back(pQuad);
                }
                pQuad->SetCoordinates(laneLeftFront, laneRightFront, laneRightRear, laneLeftRear);
                if ((laneRightRear - laneLeftRear).length() > 200.0f)
                {
                    //LOG("AARGHx2! waypoint:("+TOSTRING(wpt)+") lane: "+TOSTRING(j)+" laneWidth:("+TOSTRING((laneRightRear - laneLeftRear).length())+") roadLeft:("+TOSTRING(leftRoadEdge)+") roadRight:("+TOSTRING(rightRoadEdge)+") laneLeftRear:("+TOSTRING(laneLeftRear)+") laneRightRear:("+TOSTRING(laneRightRear)+") i:"+TOSTRING(i));
                }
                //LOG(Ogre::String("wpt ") + TOSTRING(i) + " (" + TOSTRING(laneLeftFront.x) + ", " + TOSTRING(laneLeftFront.z) + ") (" + TOSTRING(laneRightFront.x) + ", " + TOSTRING(laneRightFront.z) + ") (" + TOSTRING(laneLeftRear.x) + ", " + TOSTRING(laneLeftRear.z) + ") (" + TOSTRING(laneRightRear.x) + ", " + TOSTRING(laneRightRear.z) + "),");
                
                numNewQuads++;
                pQuad->SetWaypointIDs(i-1, i);
                //static std::map<Ogre::String, std::vector<CollisionAvoidance_GridQuad>> raceGridQuads;
            }
            if (firstNumNewQuads < 0 && numNewQuads > 0)
            {
                firstNumNewQuads = numNewQuads;
            }
            
            int numQuads = waypointQuads.size();
            for (int j = 1; j < numNewQuads; j++)
            {
                CollisionAvoidance_GridQuad *curQuad = waypointQuads[numQuads-j];
                CollisionAvoidance_GridQuad *prevQuad = waypointQuads[numQuads-j-1];
                curQuad->SetNeighbourRight(prevQuad);
                prevQuad->SetNeighbourLeft(curQuad);
            }
            
            if (numNewQuads > 0 && lastNumNewQuads > 0 && pLastWaypointQuads && pLastWaypointQuads != pWaypointQuads)
            {
                int lastNumQuads = pLastWaypointQuads->size();
                for (int j = 1; j <= numNewQuads || j <= lastNumNewQuads; j++)
                {
                    if (j <= numNewQuads)
                    {
                        CollisionAvoidance_GridQuad *frontQuad = waypointQuads[numQuads-j];
                        if (j <= lastNumNewQuads)
                        {
                            CollisionAvoidance_GridQuad *rearQuad = (*pLastWaypointQuads)[lastNumQuads-j];//waypointQuads[numQuads-numNewQuads-j];
                            if (frontQuad != rearQuad)
                            {
                                frontQuad->SetNeighbourRear(rearQuad);
                                rearQuad->SetNeighbourFront(frontQuad);
                            }
                            else
                            {
                                LOG("frontQuad==rearQuad! Wpt:" + TOSTRING(i) + " j: " + TOSTRING(j));
                            }
                        }
                        else
                        {
                            //Different number of lanes front to rear, so merge the lanes at each right edge (the rear quad should actually just be at index 0).
                            CollisionAvoidance_GridQuad *rearQuad = (*pLastWaypointQuads)[lastNumQuads-lastNumNewQuads];//waypointQuads[numQuads-numNewQuads-lastNumNewQuads];
                            if (frontQuad != rearQuad)
                            {
                                frontQuad->SetNeighbourRear(rearQuad);
                                rearQuad->SetNeighbourFront(frontQuad);
                            }
                            else
                            {
                                LOG("frontQuad==rearQuad! Wpt:" + TOSTRING(i) + " j: " + TOSTRING(j));
                            }                        
                        }
                    }
                    else if (j <= lastNumNewQuads)
                    {
                        //Different number of lanes front to rear, so merge the lanes at each right edge
                        CollisionAvoidance_GridQuad *frontQuad = waypointQuads[numQuads-numNewQuads];
                        CollisionAvoidance_GridQuad *rearQuad = (*pLastWaypointQuads)[lastNumQuads-j];//waypointQuads[numQuads-numNewQuads-j];
                        if (frontQuad != rearQuad)
                        {
                            frontQuad->SetNeighbourRear(rearQuad);
                            rearQuad->SetNeighbourFront(frontQuad);
                        }
                        else
                        {
                            LOG("frontQuad==rearQuad! Wpt:" + TOSTRING(i) + " j: " + TOSTRING(j));
                        }
                    }
                }
            }
            
        }
        //Now we need to consult waypoints to see what direction the road moves next!
        i++;
        if (i <= endWaypoint)//waypoints.size())
        {
            wpt = waypoints[i];
            Vector3 lastLeftRoadEdge = leftRoadEdge;
            Vector3 lastRightRoadEdge = rightRoadEdge;
            FindRoadEdgePoints(i, wpt, leftRoadEdge, rightRoadEdge, nextIndexLeft, nextIndexRight);
            //LOG("Gen Lanes. FindRdEdgPts() wpt("+TOSTRING(wpt.x)+","+TOSTRING(wpt.z)+") left("+TOSTRING(leftRoadEdge.x)+","+TOSTRING(leftRoadEdge.z)+") right("+TOSTRING(rightRoadEdge.x)+","+TOSTRING(rightRoadEdge.z)+")");
            if ((lastLeftRoadEdge == leftRoadEdge) && (lastRightRoadEdge == rightRoadEdge) && (i > startWaypoint))
            {
                //This shouldn't ever happen, but it IS happening, once for every waypoint!
                LOG("Gen Lanes. FindRdEdgPts() Road edges were same as last values! i="+TOSTRING(i)+" wpt("+TOSTRING(wpt.x)+","+TOSTRING(wpt.z)+") left("+TOSTRING(leftRoadEdge.x)+","+TOSTRING(leftRoadEdge.z)+") right("+TOSTRING(rightRoadEdge.x)+","+TOSTRING(rightRoadEdge.z)+")");
                int iprev = i-1;
                if (iprev > 0)
                {
                    Vector3 prevWpt = waypoints[iprev];
                    Vector3 prevLeftRoadEdge, prevRightRoadEdge;
                    int prevNextIndexLeft, prevNextIndexRight;
                    FindRoadEdgePoints(iprev, prevWpt, prevLeftRoadEdge, prevRightRoadEdge, prevNextIndexLeft, prevNextIndexRight);
                    LOG("prevWpt: " + TOSTRING(prevWpt) + " prevLeftRoadEdge: " + TOSTRING(prevLeftRoadEdge) + " lastLeftRoadEdge: " + TOSTRING(lastLeftRoadEdge) + " prevRightRoadEdge: " + TOSTRING(prevRightRoadEdge) + " lastRightRoadEdge: " + TOSTRING(lastRightRoadEdge) + "." );
                }
            }
        }
        else
        {
            break;
        }
    }
    
    //Wrap around if it's a circuit - TODO make configurable! cosmic vole June 30 2017
    if (numNewQuads > 0 && firstNumNewQuads > 0 && pFirstWaypointQuads && pWaypointQuads && pFirstWaypointQuads != pWaypointQuads)
    {
        int numQuads = pWaypointQuads->size();
        int firstNumQuads = pFirstWaypointQuads->size();
        for (int j = 1; j <= numNewQuads || j <= firstNumNewQuads; j++)
        {
            if (j <= numNewQuads)
            {
                CollisionAvoidance_GridQuad *rearQuad = (*pWaypointQuads)[numQuads-j];
                if (j <= firstNumNewQuads)
                {
                    CollisionAvoidance_GridQuad *frontQuad = (*pFirstWaypointQuads)[firstNumQuads-j];//waypointQuads[numQuads-numNewQuads-j];
                    if (frontQuad != rearQuad)
                    {
                        frontQuad->SetNeighbourRear(rearQuad);
                        rearQuad->SetNeighbourFront(frontQuad);
                    }
                    else
                    {
                        LOG("frontQuad==rearQuad! Wpt:" + TOSTRING(i) + " j: " + TOSTRING(j));
                    }
                }
                else
                {
                    //Different number of lanes front to rear, so merge the lanes at each right edge (the rear quad should actually just be at index 0).
                    CollisionAvoidance_GridQuad *frontQuad = (*pFirstWaypointQuads)[firstNumQuads-firstNumNewQuads];//waypointQuads[numQuads-numNewQuads-lastNumNewQuads];
                    if (frontQuad != rearQuad)
                    {
                        frontQuad->SetNeighbourRear(rearQuad);
                        rearQuad->SetNeighbourFront(frontQuad);
                    }
                    else
                    {
                        LOG("frontQuad==rearQuad! Wpt:" + TOSTRING(i) + " j: " + TOSTRING(j));
                    }                        
                }
            }
            else if (j <= firstNumNewQuads)
            {
                //Different number of lanes front to rear, so merge the lanes at each right edge
                CollisionAvoidance_GridQuad *rearQuad = (*pWaypointQuads)[numQuads-numNewQuads];
                CollisionAvoidance_GridQuad *frontQuad = (*pFirstWaypointQuads)[firstNumQuads-j];//waypointQuads[numQuads-numNewQuads-j];
                if (frontQuad != rearQuad)
                {
                    frontQuad->SetNeighbourRear(rearQuad);
                    rearQuad->SetNeighbourFront(frontQuad);
                }
                else
                {
                    LOG("frontQuad==rearQuad! Wpt:" + TOSTRING(i) + " j: " + TOSTRING(j));
                }
            }
        }
    }

    //Debug - check the pointers
        std::map<Ogre::String, std::vector<CollisionAvoidance_GridQuad*>>::iterator iter;
        int numQuads = 0;
        int numNoFronts = 0;
        int numNoRears = 0;
        for (iter = raceGridQuads.begin(); iter != raceGridQuads.end(); ++iter)
        {
            Ogre::String raceWaypointKey = iter->first;
            std::vector<CollisionAvoidance_GridQuad*>& gridQuads = iter->second;
            std::vector<CollisionAvoidance_GridQuad*>::iterator iter2;
            for (iter2 = gridQuads.begin(); iter2 != gridQuads.end(); ++iter2)
            {
                CollisionAvoidance_GridQuad *gridQuad = *iter2;
                numQuads++;
                if (!gridQuad->GetNeighbourFront())
                    numNoFronts++;
                if (!gridQuad->GetNeighbourRear())
                    numNoRears++;
            }
        }
        LOG(TOSTRING(numQuads) + " grid quads were created of which " + TOSTRING(numNoFronts) + " have no pFrontQuad and " + TOSTRING(numNoRears) + " have no pRearQuad.");
    //End Debug check the pointers
}

//Makes the car try to follow the specified lane instead of using the whole track. Used for debugging. cosmic vole June 27 2017
void VehicleAI::CollisionAvoidance_FollowLane(VehicleAI *truckAI, int lane, int startWaypoint, int endWaypoint, bool smoothChange, bool resumeRacingLine, int steps, int mergeFromLane)
{
    if (waypoints.empty() || truckAI->waypoints.empty() || road_edge_points_left.empty() || road_edge_points_right.empty())
        return;
    if (startWaypoint < 1 || startWaypoint > waypoints.size())
        startWaypoint = 1;
    if (endWaypoint < 1 || endWaypoint > waypoints.size())
        endWaypoint = waypoints.size();
    int numWaypoints = endWaypoint - startWaypoint + 1;
    /*
    static int start = -1;
    if (start == -1 && current_waypoint_id >= 1)
    {
        start = current_waypoint_id;
    }
    if (start < 1)
        start = 1;
    */
    //Number of waypoints to smoothly merge into the lane
    if (steps <= 0)
    {
        steps = 7;//6;//4 is too litte on the fast bits. 5 needs more testing on the fast straight but otherwise I think OK. 6 and 9 seem to work everywhere just about (on F1 long with waypoints every 10m);//25; 25 worked beautifully (didn't upset vehicle) but very slowly//20;//40
        if (resumeRacingLine)
            steps = 20;//15;//16;
    }
    if (!smoothChange)
    {
        steps = 0;
    }
    else if (steps > numWaypoints)
    {
        steps = numWaypoints;
    }
    
    float debugMaxAdjustment = 0.0f;
    float debugMinRoadWidth = 10000.0f;
    float debugMaxLaneWidth = 0.0f;
    bool debugIsCurrentTruck = truckAI->beam && m_sim_controller->GetBeamFactory()->getCurrentTruck() && truckAI->beam->trucknum == m_sim_controller->GetBeamFactory()->getCurrentTruck()->trucknum;
        
    for (int i=startWaypoint; i<=endWaypoint; i++)
    {
        int nextAgentWaypointID = i;
/*        
        int nextWaypointOnFirstLap;
        int lastWaypointOnFirstLap;
        if (truckAI->num_waypoints_in_lap > 0 && truckAI->num_waypoints_in_lap <= truckAI->free_waypoints)
        {
            nextWaypointOnFirstLap = ((nextAgentWaypointID - 1) % truckAI->num_waypoints_in_lap) + 1;
            lastWaypointOnFirstLap = nextWaypointOnFirstLap - 1;
            //Wrap around - this will work even if it's a single lap race on a circuit. It just won't work on a drag strip, so set num_waypoints_in_lap to 0 for that. cosmic vole June 25 2017
            if (lastWaypointOnFirstLap < 1)
                lastWaypointOnFirstLap += truckAI->num_waypoints_in_lap;
        }
        else
        {
            nextWaypointOnFirstLap = nextAgentWaypointID;
            lastWaypointOnFirstLap = nextAgentWaypointID - 1;
        }
        
        Ogre::String raceWaypointKey("");
        //TODO !!!! We can speed this up if we only do this dictionary lookup each time the waypoint changes! cosmic vole June 25 2017
        if (lastWaypointOnFirstLap >= 0)//nextAgentWaypointID > 0)//BUG / Problem waypoint 0 is not supported - see GenerateLanes() - cosmic vole June 25 2017 = 0)
        {
            Ogre::String raceIDStr = TOSTRING(truckAI->raceID);
            if (raceIDStr.compare("") == 0) //TODO should never happen - raceID is an int! -1 when no race! and it doesn't get initialized anywhere yet!
            {
                //adjusted_waypoints[i] = Vector3::ZERO;
                //return;
            }
            Ogre::String nextAgentWaypointName = truckAI->waypoint_names[nextWaypointOnFirstLap];//nextAgentWaypointID];
            raceWaypointKey = raceIDStr + "|||" + nextAgentWaypointName;
            //endLaneIndex = raceWaypointToLanePointIndex[raceIDStr + "|||" + nextAgentWaypointName];
            //if (nextAgentWaypointID >= 1)
            //{
            //    Ogre::String lastAgentWaypointName = waypoint_names[lastWaypointOnFirstLap];//nextAgentWaypointID-1];
            //    startLaneIndex = raceWaypointToLanePointIndex[raceIDStr + "|||" + lastAgentWaypointName];//!< Maps raceID + "|||" + waypoint name to an index into that race's entry in raceLanePoints. cosmic vole May 18 2017
            //}        
        }
        else
        {
            if (debugIsCurrentTruck)
                LOG("ERROR in FollowLane() Waypoint: " + TOSTRING(i) + " skipped because lastWaypointOnFirstLap = " + TOSTRING(lastWaypointOnFirstLap) + ".");
            continue;//return;
        }
        
        std::map<Ogre::String, std::vector<CollisionAvoidance_GridQuad*>>::iterator iter;
        iter = raceGridQuads.find(raceWaypointKey);
        //OLD - Lazy load the race track's collision avoidance grid of lanes if it hasn't already been generated - cosmic vole June 24 2017 NOTE on some tracks this may cause lag!
        if (iter == raceGridQuads.end())
        {
            // **** TODO This is wrong, needs to be static / held in a Race object. cosmic vole June 27 2017
            CollisionAvoidance_GenerateLanes(-1, -1, truckAI->waypoints, truckAI->road_edge_points_left, truckAI->road_edge_points_right);
            iter = raceGridQuads.find(raceWaypointKey);
            if (iter == raceGridQuads.end())
            {
               // LOG("Ehh key " + raceWaypointKey + " still not found! Waypoint index: " + TOSTRING(i));
                if (debugIsCurrentTruck)
                    LOG("ERROR in FollowLane() Waypoint: " + TOSTRING(i) + " skipped because grid quads not found for: '" + raceWaypointKey + "'.");
                continue;
            }
        }
        std::vector<CollisionAvoidance_GridQuad*>& gridQuads = iter->second;//raceGridQuads[raceWaypointKey];
*/
        //All the above commented out code should be handled here now:
        std::vector<CollisionAvoidance_GridQuad*>* pGridQuads = CollisionAvoidance_GetGridQuads(TOSTRING(truckAI->raceID), nextAgentWaypointID);
        
        if (!pGridQuads || pGridQuads->size() < 1)
        {
            if (debugIsCurrentTruck)
                LOG("ERROR in FollowLane() Waypoint: " + TOSTRING(i) + " skipped because no grid quads found for that waypoint.");
            continue;
        }
        //CollisionAvoidance_GridQuad& gridQuad = *gridQuads[0];//Accessing anything inside this object seems to seg fault!
        //LOG(TOSTRING(gridQuad.GetWaypointIDRear()));
        CollisionAvoidance_GridQuad *pLeft = (*pGridQuads)[0];//&gridQuad;
        if (!pLeft)
        {
            if (debugIsCurrentTruck)
                LOG("ERROR in FollowLane() Waypoint: " + TOSTRING(i) + " skipped because gridQuads[0] is null.");
            continue;
        }
        while ((pLeft != nullptr) && (pLeft->GetNeighbourLeft() != nullptr))
        {
            pLeft = pLeft->GetNeighbourLeft();
        }
        CollisionAvoidance_GridQuad *pQuad = pLeft;
        int curLane;
        for (curLane = 0; curLane < lane; curLane++)
        {
            if (pQuad->GetNeighbourRight())
            {
                pQuad = pQuad->GetNeighbourRight();
            }
            else
            {
                break;
            }
        }
        
        if (curLane != lane && lane >= 0)
        {
            if (debugIsCurrentTruck)
                LOG("ERROR in FollowLane() for Waypoint: " + TOSTRING(i) + " Lane: '" + TOSTRING(lane) + "' not found. Max lane found: " + TOSTRING(curLane) + ".");            
        }
        
        CollisionAvoidance_GridQuad *pQuadMergeFrom = nullptr;
        Vector3 mergeFromLaneCenter = Vector3::ZERO;
        float mergeFromLaneWidth = 0.0f;
        if (mergeFromLane >= 0)
        {
            pQuadMergeFrom = pLeft;
            for (int curLane = 0; curLane < mergeFromLane; curLane++)
            {
                if (pQuadMergeFrom->GetNeighbourRight())
                {
                    pQuadMergeFrom = pQuadMergeFrom->GetNeighbourRight();
                }
                else
                {
                    break;
                }
            }
            Vector3 quadFrontLeft, quadFrontRight, quadRearLeft, quadRearRight;
            pQuadMergeFrom->GetCoordinates(quadFrontLeft, quadFrontRight, quadRearRight, quadRearLeft);
            Vector3 acrossLaneRear = quadRearRight - quadRearLeft;
            mergeFromLaneWidth = acrossLaneRear.length();
            mergeFromLaneCenter = (quadFrontLeft + quadFrontRight) * 0.5f;

        }
        
        Vector3 waypoint = truckAI->waypoints[i];
        Vector3 adjusted_waypoint = truckAI->adjusted_waypoints[i];
        Vector3 mergeFromLanePoint = Vector3::ZERO;
        

        Vector3 quadFrontLeft, quadFrontRight, quadRearLeft, quadRearRight;
        pQuad->GetCoordinates(quadFrontLeft, quadFrontRight, quadRearRight, quadRearLeft);
        //This code isn't working correctly - it's ending up the same as the road width. Is it a bug in Ogre or undefined behavior?! cosmic vole June 29 2017
        //float laneWidthRear = Vector3(quadRearRight - quadRearLeft).length();
        //float laneWidthFront = Vector3(quadFrontRight - quadFrontLeft).length();
        
        //float minLaneWidth = laneWidthRear;
        //if (laneWidthFront < minLaneWidth)
        //{
        //    minLaneWidth = laneWidthFront;
        //}
        
        Vector3 acrossLaneRear = quadRearRight - quadRearLeft;
        float laneWidth = acrossLaneRear.length();
        if (laneWidth > debugMaxLaneWidth)
            debugMaxLaneWidth = laneWidth;
        
        Vector3 laneCenter = (quadFrontLeft + quadFrontRight) * 0.5f;/*(quadRearLeft + quadRearRight) * 0.5f; */
        Vector3 roadCenter, roadLeft, roadRight, roadLeftFront, roadRightFront, roadLeftRear, roadRightRear;
        //roadLeft = pQuad->FindRoadLeftEdgeRear();
        //roadRight = pQuad->FindRoadRightEdgeRear();
        roadLeft = pQuad->FindRoadLeftEdgeFront();
        roadRight = pQuad->FindRoadRightEdgeFront();
        //if (roadLeftRear == Vector3::ZERO || roadRightRear == Vector3::ZERO)
        //{
        //    roadRight 
        //}
        if (roadLeft == Vector3::ZERO || roadRight == Vector3::ZERO)
        {
            if (debugIsCurrentTruck)
                LOG("ERROR in FollowLane() Waypoint: " + TOSTRING(i) + " skipped because (roadLeft == Vector3::ZERO || roadRight == Vector3::ZERO).");
            continue;
        }
        if (quadRearLeft == Vector3::ZERO || quadRearRight == Vector3::ZERO)
        {
            if (debugIsCurrentTruck)
                LOG("ERROR in FollowLane() Waypoint: " + TOSTRING(i) + " skipped because (quadRearLeft == Vector3::ZERO || quadRearRight == Vector3::ZERO).");
            continue;
        }
        if (quadFrontLeft == Vector3::ZERO || quadFrontRight == Vector3::ZERO)
        {
            if (debugIsCurrentTruck)
                LOG("ERROR in FollowLane() Waypoint: " + TOSTRING(i) + " skipped because (quadFrontLeft == Vector3::ZERO || quadFrontRight == Vector3::ZERO).");            
            continue;
        }
        roadCenter = (roadLeft + roadRight) * 0.5f;
        Vector3 acrossRoad = roadRight - roadLeft;
        float roadWidth = acrossRoad.length();
        if (roadWidth < debugMinRoadWidth)
            debugMinRoadWidth = roadWidth;

        if (mergeFromLane >= 0)
        {
            //Don't merge from the existing adjusted waypoint, merge from the center of the supplied lane number instead
            mergeFromLanePoint = truckAI->waypoints[i];
            //Move the waypoint so it's an offset from the center of the road instead of the origin
            mergeFromLanePoint -= roadCenter;     
            float scaleIntoLane = mergeFromLaneWidth / roadWidth;
            mergeFromLanePoint *= scaleIntoLane;
            //Move the waypoint so it's an offset from the center of mergeFromLane
            mergeFromLanePoint += mergeFromLaneCenter;            
        }

        Vector3 curWaypointTarget = waypoint;
        if (lane < 0)
        {
            curWaypointTarget = adjusted_waypoint;
        }
        else
        {
            //Move the waypoint so it's an offset from the center of the road instead of the origin
            curWaypointTarget -= roadCenter;
            
            //See if the waypoint is outside the road edges! This can happen e.g. on bend apexes and could give
            //rise to false positives in the lane collision detection code.
            float acrossRoadToTarget = curWaypointTarget.dotProduct(acrossRoad.normalisedCopy());
            if (acrossRoadToTarget > roadWidth * 0.5f)
            {
                curWaypointTarget = roadRight - roadCenter;
            }
            else if (acrossRoadToTarget < -roadWidth * 0.5f)
            {
                curWaypointTarget = roadLeft - roadCenter;
            }
            
            
            
            //Scale the waypoint down to fit in the chosen lane rather than the whole road
            //We have to scale in the left/right direction only! Actually it should be OK.
            //We can simplify the scale to a translation towards the center of the road because the points are already on the axis we are scaling along.
            //No, should be OK
            //This is VERY weird - minLaneWidth is coming out as equal to road width!!!
            float scaleIntoLane = /*0.75f * */laneWidth / roadWidth;
            //LOG("scale into lane: " + TOSTRING(scaleIntoLane) + " lane width: " + TOSTRING(laneWidth) + " road width: " + TOSTRING(roadWidth));
            //LOG("Lane right: ("+TOSTRING(quadFrontRight)+") Lane left: ("+TOSTRING(quadFrontLeft)+") Road left: ("+TOSTRING(roadLeft)+")"+" Road right: ("+TOSTRING(roadRight)+")");
            curWaypointTarget *= scaleIntoLane;//  /= 3.0f;//*= (minLaneWidth / roadWidth) * 0.75f;//TODO it should just be /= (float)curNumLanes YEAH -> divide by 3 works, so something else is wrong (lane width or road width calcs)
            //Move the waypoint so it's an offset from the center of the chosen lane
            curWaypointTarget += laneCenter;
            //curWaypointTarget = roadCenter;
            //curWaypointTarget = laneCenter;
        }
        //We progressively move the waypoints into the chosen lane
        Vector3 adjustment;// = curWaypointTarget - adjusted_waypoint;
        if (mergeFromLane >= 0)
        {
            if (resumeRacingLine)
            {
                //Special case, we're merging out of mergeFromLane back to the racing line at waypoint. The "lane" parameter is actually ignored here.
                adjustment = mergeFromLanePoint - waypoint;
            }
            else
            {
                //Special case, we're merging out of mergeFromLane into lane.
                adjustment = curWaypointTarget - mergeFromLanePoint;
            }
        }
        else
        {
            if (resumeRacingLine)
            {
                //We're resuming the racing line at waypoint, from lane.
                adjustment = curWaypointTarget - waypoint;
            }
            else
            {
                //We're merging into the new lane at curWaypointTarget from the old adjusted waypoint, or vice versa if resuming racing line.
                adjustment = curWaypointTarget - adjusted_waypoint;
            }
        }
        
        float adjustmentLength = adjustment.length();
        if (adjustmentLength > debugMaxAdjustment)
            debugMaxAdjustment = adjustmentLength;
        float progress = 1.0f;
        int d = (resumeRacingLine)? (steps + startWaypoint - i) : (i - startWaypoint);
        if (d < steps && d >= 0)
        {
            progress = ((float)d) / ((float)steps);
        }
        else if (d < 0)
        {
            progress = 0.0f;
        }
     
        if (progress != 1.0f)
        {
            //What's happening is target coords are WAY off, kind of out of sync with, waypoint's coords. Which is odd when we plot road centre vs waypoint, they're not off.
            //Could it be because of the static value and the fact this is being run across multiple trucks???? Yep that was it!!!
            //Merging it towards roadCenter works perfectly now. But lane 0 veers progressively off to the left for some reason.
            //laneCenter of lane 0 works well too. Car made it round the track pretty well - time estimated about 02:20 - 02:30 (outside of track) - couldn't get full time as it hit the other cars on the grid!
            //LOG("waypoint:("+TOSTRING(waypoint.x)+","+TOSTRING(waypoint.z)+") target:("+TOSTRING(curWaypointTarget.x)+","+TOSTRING(curWaypointTarget.z)+") interp:("+TOSTRING(Vector3(waypoint+Vector3(progress*adjustment)).x)+","+TOSTRING(Vector3(waypoint+Vector3(progress*adjustment)).z)+") progress:("+TOSTRING(progress)+")");
        }
        
        if (resumeRacingLine)
        {
            waypoint = waypoint + Vector3(progress * adjustment);
        }
        else
        {
            if (mergeFromLane >= 0)
            {
                waypoint = mergeFromLanePoint + Vector3(progress * adjustment);
            }
            else
            {
                waypoint = /*waypoint*/adjusted_waypoint + Vector3(progress * adjustment);
            }
        }
        
        if (( i == 23 || i == 24 || waypoint.distance(roadRight) > roadWidth * 1.2f || waypoint.distance(roadLeft) > roadWidth * 1.2f) && debugIsCurrentTruck)
        {
            if (i == 23 || i == 24)
            {
                LOG("Here's that troublesome bend!:");
            }
            LOG("Possible Error in FollowLane: waypoint:("+TOSTRING(waypoint)+") old: ("+TOSTRING(waypoints[i])+") progress: "+TOSTRING(progress)+" curWaypointTarget: "+TOSTRING(curWaypointTarget)+" lane: "+TOSTRING(lane)+" laneCenter:("+TOSTRING(laneCenter)+") laneWidth:("+TOSTRING(laneWidth)+") roadLeft:("+TOSTRING(roadLeft)+") roadRight:("+TOSTRING(roadRight)+") quadRearLeft:("+TOSTRING(quadRearLeft)+") quadRearRight:("+TOSTRING(quadRearRight)+") i:"+TOSTRING(i));
            
            if (truckAI->waypoints.size() != waypoints.size() || truckAI->waypoints[i] != waypoint)
            {
                //BUG This is happening because we're not assigning a raceID yet so trucks waiting on a different race grid get collision detected for the wrong race!
                //LOG("Ehhh truck " + TOSTRING(truckAI->beam->trucknum) + " has different waypoints to " + TOSTRING(beam->trucknum) + " " + TOSTRING(truckAI->waypoints[i]) + " vs " + TOSTRING(waypoints[i]) + ". i is " + TOSTRING(i) + ".");
            }
        }
        truckAI->adjusted_waypoints[i] = waypoint;//curWaypointTarget;//roadCenter;//roadLeft;//curWaypointTarget;//waypoint;
        if (i == truckAI->current_waypoint_id)
        {
            truckAI->current_waypoint = waypoint;
        }
    }
    
    if (debugIsCurrentTruck)
        LOG("FollowLane() Truck " + TOSTRING(truckAI->beam->trucknum) + " Max adjustment: " + TOSTRING(debugMaxAdjustment) + " Steps: " + TOSTRING(steps) + " Resume Racing Line?: " + TOSTRING(resumeRacingLine) + " Lane: " + TOSTRING(lane) + " Merge from: " + TOSTRING(mergeFromLane) + " NumWaypts: " + TOSTRING(numWaypoints) + " StartWaypt: " + TOSTRING(startWaypoint) + " EndWaypt: " + TOSTRING(endWaypoint) + " Min Road Width: " + TOSTRING(debugMinRoadWidth) + " Max Lane Width: " + TOSTRING(debugMaxLaneWidth) + " Sum Errors: " + TOSTRING(sumErrorsAcrossTrack) + ".");
}

//cosmic vole June 1 2017
void VehicleAI::CollisionAvoidance_AvoidCollisions(float currentTime/*, std::vector<Ogre::Vector3>& waypoints, std::vector<Ogre::Vector3>& road_edge_points_left, std::vector<Ogre::Vector3>& road_edge_points_right*/)
{
    if (waypoints.empty())
        return;
    static int debugLaneNum = -1;
    int maxIterations = 3;
    float minDistForSwerve = 20.0f;//50.0f;//12.0f;//15.0f;//13.0f;//12.0f;//14.0f;//16.0f;//18.0f;//20.0f;//TODO estimate this from predicted average speed
    float timeSafetyMargin = 0.4f;//0.5f;//0.65f;//0.7f;//0.6f;//0.5f;
    Beam** trucks = m_sim_controller->GetBeamFactory()->getTrucks();
    int numTrucks = m_sim_controller->GetBeamFactory()->getTruckCount();
    
    CollisionAvoidance_ClearVehiclePaths(Ogre::StringUtil::BLANK, false);
    
    //We're now prediciting all the vehicle paths inside this method cosmic vole June 21 2017
    for (int t = 0; t < numTrucks; t++)
    {
        if (!trucks[t])
            continue;
        if (trucks[t]->vehicle_ai)// && (trucks[t]->vehicle_ai->IsActive()))
        {
            //BUG?! July 2 2017 At the particular time this is called, unless this is the last truck in the list,
            //Not all of the other trucks locations will have been written to that trucks PredictedVehiclePositions in its path, as they are copies!
            //It shouldn't matter though because of the way we examine the path, further below.
            
            //TODO July 3 2017 At the very start of the race, we should predict the default path based on the vehicles staying in lane
            //rather than following the waypoints because the proximity code will make lane following activate on the next frame, so it's more accurate
            //and will stop them targeting lanes to avoid collisions that won't really happen and even veering off course.
            trucks[t]->vehicle_ai->CollisionAvoidance_PredictVehiclePath(currentTime);//cosmic vole June 20 2017
        }
    }
    
    CollisionAvoidance_DebugVehiclePath();
    
    //TODO June 16 2017 - Need to sort or maybe just randomize the truck list to make this fairer. At the moment the first trucks are always given the first
    //chance to pull out and overtake which can then block the track for the later trucks in the list. Especially at the start of a race.
    //Sorting them by how soon a collision occurs may not be fair or optimal either. We really want trucks to take evasive action in plenty of time.
    for (int i = 0; i < numTrucks; i++)
    {
        if (!trucks[i] || !trucks[i]->vehicle_ai)
            continue;
        Beam& truck = *trucks[i];
        VehicleAI& truckAI = *truck.getVehicleAI();
        std::vector<PredictedTruckPosition>& truckPath = truckAI.predictedVehiclePath;
        
        /*
        if (truckPath.size() > 10000)
        {
            //We've not seen this so far...
            LOG("truckPath size: " + TOSTRING(truckPath.size()));
        }
        */
        UTFString debugInfo = L"";
        
        bool noEvasiveAction = true;
        bool noCollisions = true;
        Beam *agentTruck = &truck;//truckAI.beam;
        bool isCurrentTruck = agentTruck && m_sim_controller->GetBeamFactory()->getCurrentTruck() && agentTruck->trucknum == m_sim_controller->GetBeamFactory()->getCurrentTruck()->trucknum;
        Vector3 agentPos = agentTruck->getPosition();
        Vector3 agentVelocity = agentTruck->getVelocity();
        Vector3 agentPos2D = agentPos;
        agentPos2D.y = 0.0f;
        Vector3 agentFrontLeft, agentFrontRight, agentRearLeft, agentRearRight;
        agentTruck->getCorners2D(agentFrontLeft, agentFrontRight, agentRearLeft, agentRearRight, 0.5f);//0.7f);//1.0f);
        float leftOverlap, rightOverlap;
        float maxOverlap = 0.0f;
        
        int currentLane = 0;
        int targetLane = 0;
        CollisionAvoidance_GridQuad *firstQuad = nullptr;
        
        if (truckPath.size() > 0)
        {
            firstQuad = truckPath[0].pGridQuad;
            if (firstQuad)
            {
                //The truck may be straddling two lanes. Check the distance to the center of each adjacent lane to pick the best current lane.
                CollisionAvoidance_GridQuad *pLeft, *pRight;
                Vector3 frontLeft, frontRight, rearLeft, rearRight;
                pLeft = firstQuad->GetNeighbourLeft();
                pRight = firstQuad->GetNeighbourRight();

                Vector3 laneCenterRear = firstQuad->GetLaneCenterRear();
                Vector3 laneCenterFront = firstQuad->GetLaneCenterFront();
                laneCenterRear.y = laneCenterFront.y = 0.0f;
                //V. important that the clamping is false here, otherwise we get the wrong lanes being identified due to differeing forward / backwards offsets. Scaling of the beam corners in PredictVehiclePath seemed to cause problems also. cosmic vole August 8 2017.
                Vector3 laneCenter = getClosestPointOnLine(laneCenterRear, laneCenterFront, agentPos2D, false);
                Vector3 firstCenter = laneCenter;
                Vector3 newCenter = laneCenter;
                firstQuad->GetCoordinates(frontLeft, frontRight, rearLeft, rearRight);
                Vector3 firstLeft = rearLeft;
                Vector3 firstRight = rearRight;
                Vector3 firstFrontLeft = frontLeft;
                Vector3 firstFrontRight = frontRight;

                if (pLeft || pRight)
                {
                    /* For debugging, these have been moved outside the if block. For performance, they can be moved back.
                    Vector3 laneCenterRear = firstQuad->GetLaneCenterRear();
                    Vector3 laneCenterFront = firstQuad->GetLaneCenterFront();
                    laneCenterRear.y = laneCenterFront.y = 0.0f;
                    Vector3 laneCenter = getClosestPointOnLine(laneCenterRear, laneCenterFront, agentPos2D, true);
                    Vector3 firstCenter = laneCenter;
                    firstQuad->GetCoordinates(frontLeft, frontRight, rearLeft, rearRight);
                    Vector3 firstLeft = rearLeft;
                    Vector3 firstRight = rearRight;                    
                    */
                    float laneDist = agentPos2D.distance(laneCenter);
                    float maxOverlap = GetLaneOverlap(leftOverlap, rightOverlap, agentFrontLeft, agentFrontRight, agentRearRight, agentRearLeft, rearLeft, rearRight, laneCenter);
                    
                    if (pLeft)
                    {
                        laneCenterRear = pLeft->GetLaneCenterRear();
                        laneCenterFront = pLeft->GetLaneCenterFront();
                        laneCenterRear.y = laneCenterFront.y = 0.0f;
                        laneCenter = getClosestPointOnLine(laneCenterRear, laneCenterFront, agentPos2D, false);//true);
                        float laneLeftDist = agentPos2D.distance(laneCenter);
                        pLeft->GetCoordinates(frontLeft, frontRight, rearLeft, rearRight);
                        float overlap = GetLaneOverlap(leftOverlap, rightOverlap, agentFrontLeft, agentFrontRight, agentRearRight, agentRearLeft, rearLeft, rearRight, laneCenter);
                        if (laneLeftDist < laneDist)
                        {
                            firstQuad = pLeft;
                            laneDist = laneLeftDist;
                            newCenter = laneCenter;
                            if (overlap > maxOverlap)
                            {
                                maxOverlap = overlap;
                            }
                            else
                            {
                                //if (isCurrentTruck)
                                //    LOG("Vehicle's center's closer to left lane but overlap is less!: "+TOSTRING(overlap)+" Max overlap: "+TOSTRING(maxOverlap) + " Vehicle center: " + TOSTRING(agentPos2D) + " 1st lane center: "+TOSTRING(firstCenter)+" left lane center: "+TOSTRING(laneCenter)+
                                //    "Vehicle corners: ("+TOSTRING(agentFrontLeft)+") ("+TOSTRING(agentFrontRight)+") ("+TOSTRING(agentRearLeft)+") ("+TOSTRING(agentRearRight)+") . 1st lane left: " + TOSTRING(firstLeft)  + " right: " + TOSTRING(firstRight) + " Left lane left: " + TOSTRING(rearLeft) + " right: " + TOSTRING(rearRight) + ".");
                            }
                        }
                        else if (overlap > maxOverlap)
                        {
                            //if (agentTruck && BeamFactory::getSingleton().getCurrentTruck() && agentTruck->trucknum == BeamFactory::getSingleton().getCurrentTruck()->trucknum)
                            //    LOG("Vehicle's center's further from left lane but overlap is more!: "+TOSTRING(overlap)+" Max overlap: "+TOSTRING(maxOverlap) + " Vehicle center: " + TOSTRING(agentPos2D) + " 1st lane center: "+TOSTRING(firstCenter)+" left lane center: "+TOSTRING(laneCenter)+
                            //    "Vehicle corners: ("+TOSTRING(agentFrontLeft)+") ("+TOSTRING(agentFrontRight)+") ("+TOSTRING(agentRearLeft)+") ("+TOSTRING(agentRearRight)+") . 1st lane left: " + TOSTRING(firstLeft)  + " right: " + TOSTRING(firstRight) + " Left lane left: " + TOSTRING(rearLeft) + " right: " + TOSTRING(rearRight) + ".");
                            maxOverlap = overlap;
                            //firstQuad = pLeft;
                            //laneDist = laneLeftDist;
                        }
                    }
                    if (pRight)
                    {
                        laneCenterRear = pRight->GetLaneCenterRear();
                        laneCenterFront = pRight->GetLaneCenterFront();
                        laneCenterRear.y = laneCenterFront.y = 0.0f;
                        laneCenter = getClosestPointOnLine(laneCenterRear, laneCenterFront, agentPos2D, false);//true);
                        float laneRightDist = agentPos2D.distance(laneCenter);
                        pRight->GetCoordinates(frontLeft, frontRight, rearLeft, rearRight);
                        float overlap = GetLaneOverlap(leftOverlap, rightOverlap, agentFrontLeft, agentFrontRight, agentRearRight, agentRearLeft, rearLeft, rearRight, laneCenter);
                        if (laneRightDist < laneDist)
                        {
                            firstQuad = pRight;
                            laneDist = laneRightDist;
                            newCenter = laneCenter;
                            if (overlap > maxOverlap)
                            {
                                maxOverlap = overlap;
                            }
                            else
                            {
                                //if (agentTruck && BeamFactory::getSingleton().getCurrentTruck() && agentTruck->trucknum == BeamFactory::getSingleton().getCurrentTruck()->trucknum)
                                //    LOG("Vehicle's center's closer to right lane but overlap is less!: "+TOSTRING(overlap)+" Max overlap: "+TOSTRING(maxOverlap) + " Vehicle center: " + TOSTRING(agentPos2D) + " 1st lane center: "+TOSTRING(firstCenter)+" right lane center: "+TOSTRING(laneCenter)+
                                //    "Vehicle corners: ("+TOSTRING(agentFrontLeft)+") ("+TOSTRING(agentFrontRight)+") ("+TOSTRING(agentRearLeft)+") ("+TOSTRING(agentRearRight)+") . 1st lane left: " + TOSTRING(firstLeft)  + " right: " + TOSTRING(firstRight) + " Right lane left: " + TOSTRING(rearLeft) + " right: " + TOSTRING(rearRight) + ".");
                            }                            
                        }
                        else if (overlap > maxOverlap)
                        {
                            maxOverlap = overlap;
                            //firstQuad = pRight;
                            //laneDist = laneRightDist;
                        }                        
                    }
                }
                currentLane = firstQuad->GetLaneNumber();
                targetLane = currentLane;
                if (isCurrentTruck && (debugLaneNum != currentLane))
                {
                    firstQuad->GetCoordinates(frontLeft, frontRight, rearLeft, rearRight);
                    LOG("Lane changed from " + TOSTRING(debugLaneNum) + " to " + TOSTRING(currentLane) + ". Vehicle center: " + TOSTRING(agentPos2D) + " 1st lane center: "+TOSTRING(firstCenter)+" New lane center: "+TOSTRING(newCenter)+
                                    "Vehicle corners: ("+TOSTRING(agentFrontLeft)+") ("+TOSTRING(agentFrontRight)+") ("+TOSTRING(agentRearLeft)+") ("+TOSTRING(agentRearRight)+") . 1st lane rearLeft: " + TOSTRING(firstLeft)  + " rearRight: " + TOSTRING(firstRight) + " frontLeft: " + TOSTRING(firstFrontLeft) + " frontRight: " + TOSTRING(firstFrontRight) + " New lane rearLeft: " + TOSTRING(rearLeft) + " rearRight: " + TOSTRING(rearRight) + " frontLeft: " + TOSTRING(frontLeft) + " frontRight: " + TOSTRING(frontRight) + ".");
                    debugLaneNum = currentLane;
                }
            }
            else
            {
                if (isCurrentTruck)
                    debugInfo = L"Truck path first quad is NULL!";
                LOG("Truck path first quad is NULL!");
            }
        }
        else
        {
            if (isCurrentTruck)
                debugInfo = L"Truck path is empty!";
            if (truckAI.lane_change_to >= 0)
            {
                //TODO get the lane number of the last adjusted waypoint
                currentLane = truckAI.lane_change_to;
                targetLane = currentLane;
            }
        }
        
       
        //CollisionAvoidance_ShowDebugMsg(bool isLaneOccupied, bool hasCollision, bool collisionWithTruckInFront, bool canAvoidTruck, int currentLane, int TargetLane, Ogre::String& info)
        bool hasCollisionWithTruckInFront = false;
        bool hasToBrakeForOtherTruck = false;
        Beam *otherTruck = nullptr;
        bool canAvoidTruck = false;
        bool alreadyCommittedToManeuver = false;
        bool isCurLaneOccupied;
        if (firstQuad)
        {
            //Is the next 100 metres occupied in the next 4.8 seconds (which is the max time lookahead in PredictVehiclePath)? cosmic vole July 2 2017
            //BUG OK This is more often than not (but not constantly BIG BUG - WHY?!) returning true when we'd expect it to BUT 
            //noCollisions doesn't seem to be getting set false. gridQuad->HasCollisionWith(/*ptp*/beam->trucknum)) returns true a lot giving red markers though, so what gives? July 2 2017
            
            isCurLaneOccupied = firstQuad->IsLaneOccupied(100.0f, 20.0f, currentTime, currentTime + /*4.8f*/10.0f, agentTruck->trucknum);
        }
        else
        {
            isCurLaneOccupied = false;
        }
        
        for (int j=0; j<truckPath.size(); j++)
        {
            //TODO This really depends on how far apart the waypoints are positioned on the track - cosmic vole June 1 2017 AND on the vehicle speed and road curvature - July 1 2017
            int minWaypointsForSwerve = 7;//6;//2;//1;//2;//3;
            int maxWaypointsForSwerve = 8;//7;//4;
            //If the vehicle's moving less than about 8 mph, reduce the waypoints needed to steer around and obstacle
            //if (agentVelocity.length() < 11.0f)
            //    minWaypointsForSwerve = 6;//5;
            if (agentVelocity.length() < 4.0f)//3.5f)
                minWaypointsForSwerve = 5;//4;//3;
            if (agentVelocity.length() < 2.5f)
            {
                float wayptspeed = adjusted_waypoint_speed[truckAI.current_waypoint_id];
                if (wayptspeed < /*11.0f*/12.0f && wayptspeed > 0.0f)
                {
                    minWaypointsForSwerve = 1;//2;
                }
                else
                {
                    minWaypointsForSwerve = 3;//4;//This seems to keep turning out as 2 in FollowLane()!; 2 causes crashes because the truck accelerates. A speed limit would need setting.
                }
            }
            //Note that the time needed to turn to avoid an opponent can be much longer than the (lap) time gap between the opponents,
            //as long as they both keep moving in the same direction
            float minTimeForSwerve = 0.3f;//0.4f;//0.5f;//0.75f;            
            
            PredictedTruckPosition/*&*/ ptp = truckPath[j];
            //Expand the predicted collision time window a bit to err on the side of caution
            //ptp.minTime -= timeSafetyMargin;
            //ptp.maxTime += timeSafetyMargin + 0.2f;
            if (!ptp.pGridQuad)
            {
                continue;
            }
            CollisionAvoidance_GridQuad& quad = *ptp.pGridQuad;
            std::vector<PredictedTruckPosition> quadCollisions;
            //See which truck(s) we would collide with. Are we currently behind them?
            //If we try to overtake, we need to set some kind of flag in that truck beingOvertakenOnRight = true (or left), or a list beingOvertakenByTrucks
            //to avoid both trucks trying to evade the collision at the same time.
            quad.GetCollisionsWith(ptp, quadCollisions);
            //bool hasCollisionWithPtp = quadCollisions.size() > 0;//quad.HasCollisionWith(ptp);
            //Will *this* truck take evasive action for this collision. cosmic vole July 1 2017
            bool shouldAvoidCollision = false;
            if (quadCollisions.size() > 0)
                noCollisions = false;
                
            if (truckAI.lane_change_start_wpt <= truckAI.current_waypoint_id && truckAI.lane_change_end_wpt >= truckAI.current_waypoint_id && /*lane_change_from != lane_change_to &&*/
                (truckAI.lane_change_to == targetLane || (truckAI.lane_change_to != truckAI.lane_change_from && truckAI.lane_change_to >= 0)))
                alreadyCommittedToManeuver = true;
            
            for (std::vector<PredictedTruckPosition>::iterator iter = quadCollisions.begin(); iter != quadCollisions.end(); ++iter)
            {
                PredictedTruckPosition ptp2 = *iter;
                otherTruck = m_sim_controller->GetBeamFactory()->getTruck(ptp2.truckNum);
                //Check the *current* positions and approach angles of the two trucks. Who is behind who? cosmic vole July 1 2017
                Vector3 otherPos = otherTruck->getPosition();
                Vector3 otherVelocity = otherTruck->getVelocity();
                Vector3 agentDir = agentTruck->getDirection();
                Vector3 otherDir = otherTruck->getDirection();
                Degree betweenDirs, bearingOfOther;
                bool otherIsBehind, isHeadOn;
                CollisionAvoidance_GetVehicleApproachAngles(agentPos, otherPos, agentDir, otherDir, betweenDirs, bearingOfOther, otherIsBehind, isHeadOn);
                //If one of the two trucks is almost immobile, the other one takes responsibility for evasion
                if (!otherIsBehind || (otherVelocity.length() < 1.5f && agentVelocity.length() >= 1.5f))//otherVelocity.length() < 2.0f)
                {
                    if (!otherIsBehind)
                    {
                        hasCollisionWithTruckInFront = true;
                    }
                    shouldAvoidCollision = true;
                    //TODO !!! We don't just want the first truck in the list that's in front. We want the *soonest* collision's truck that satisfies those criteria.
                    //TODO !!! Also, theoretically, the same grid quad could appear more than once in truckPath, so if we decide we don't have to act without looking at the rest of the path, it may be erroneous.
                    break;
                }
                else
                {
                    otherTruck = nullptr;
                }
            }

            //bool stayInLane = true;//Lane debugging
            if (shouldAvoidCollision) //hasCollisionWithPtp)// || stayInLane)
            {
                noEvasiveAction = false;
                //Is this far enough ahead to choose a different path? - BUG/TODO - we need to know this is the soonest collision (for THIS truck)- build a list of collisions in start time order
                //BUT regardless of whether it's the soonest collision, we still need to make sure every step of our adjusted path doesn't introduce a new collision before we accept it! Have to be able to undo any adjustments! - cosmic vole June 1 2017
                //TODO!!! Is this truck behind the other truck? If not, unless this truck is moving and the other barely is, we skip collision avoidance for this truck! If a truck is reversing, we need to reverse the direction check!
                float timeToQuad = ptp.minTime - currentTime;
                int trucksNextWaypoint = truckAI.current_waypoint_id;
                                
                int wayptIDRear = quad.GetWaypointIDRear();
                //float wayptSpeedRear = truckAI.waypoint_speed[wayptIDRear];
                //int wayptIDFront = quad.GetWaypointIDFront();
                int swerveWpts = minWaypointsForSwerve;
                bool isHazardous = false;
                for (int i = wayptIDRear; (i < wayptIDRear + swerveWpts) && (i <= truckAI.free_waypoints); i++)
                {
                    float wayptSpeed = truckAI.waypoint_speed[i];
                    //if ((wayptSpeedRear > 0.0f && wayptSpeedRear < 18.0f) || (wayptSpeedFront > 0.0f && wayptSpeedFront < 18.0f))
                    if (wayptSpeed > 0.0f && wayptSpeed < 75.0f) //20.0f)
                    {
                        isHazardous = true;
                        break;
                    }
                }
                
                if (isHazardous)
                {
                    //It's a bend or hazardous section of track
                    minWaypointsForSwerve += 4; //++; //*= 3;
                    maxWaypointsForSwerve += 4; //++;
                    minTimeForSwerve *= 1.5f;                    
                }
                
                int wayptSpace;
                //This will be the zero indexed lap number. All collision detection quads use waypoints on the first lap,
                //so they need remapping to the truck's current lap when adjusting the truck's waypoints. cosmic vole August 9 2017
                int truckLapNum = 0;
                if (truckAI.num_waypoints_in_lap > 0)
                {
                    truckLapNum = truckAI.current_waypoint_id / truckAI.num_waypoints_in_lap;
                    wayptSpace = (((wayptIDRear - 1) % truckAI.num_waypoints_in_lap) + 1) - (((trucksNextWaypoint - 1) % truckAI.num_waypoints_in_lap) + 1);
                    if (wayptSpace < -10.0f)
                    {
                        //LOG("ERROR Negative wayptSpace: " + TOSTRING(wayptSpace) + " Num waypts in lap: " + TOSTRING(truckAI.num_waypoints_in_lap) + " Truck's next waypt: " + TOSTRING(trucksNextWaypoint) + " wayptIDRear: " + TOSTRING(wayptIDRear) + ".");
                        //This can legitimately happen if both trucks are on the same lap, but the one up ahead on the track has only just started the lap
                        //whereas the other way is getting close to the start / finish line, just about to lap it, e.g.:
                        //"ERROR Negative wayptSpace: -310 Num waypts in lap: 320 Truck's next waypt: 316 wayptIDRear: 6."
                    }
                    while (wayptSpace < 0 && wayptSpace < 20.0f-truckAI.num_waypoints_in_lap)
                    {
                        wayptSpace += truckAI.num_waypoints_in_lap;
                    }
                }
                else
                {
                    wayptSpace = wayptIDRear - trucksNextWaypoint;
                    if (wayptSpace < -10.0f)
                    {
                        LOG("ERROR Negative wayptSpace: " + TOSTRING(wayptSpace) + " Num waypts in lap: " + TOSTRING(truckAI.num_waypoints_in_lap) + " Truck's next waypt: " + TOSTRING(trucksNextWaypoint) + " wayptIDRear: " + TOSTRING(wayptIDRear) + ".");
                    }
                }
                if (wayptSpace >= minWaypointsForSwerve && timeToQuad >= minTimeForSwerve)
                {
                    //We think we've got enough time and waypoints to adjust course.
                    //First we choose an avoidance direction by checking the quad's left and right neighbours
                    //TODO/WARNING - At the collision quad, the vehicles may have spanned two or more quads and collisions may have occurred in those also
                    //WHICH MEANS that removing the truck from this quad won't necessarily deal with all its collisions
                    CollisionAvoidance_GridQuad* pQuadUnoccupied = nullptr;
                    //When checking neighbouring quads, we need to expand the time window, as it will take a slightly different amount of time to turn into it
                    float minTime = ptp.minTime - minTimeForSwerve;//- timeSafetyMargin;
                    if (minTime < currentTime)
                    {
                        minTime = currentTime;
                    }
                    float maxTime = ptp.maxTime + timeSafetyMargin;
                    int quadLane = quad.GetLaneNumber();
                    bool lookLeft = true;
                    bool lookRight = true;
                    /*
                    if (stayInLane && !hasCollisionWithPtp)
                    {
                        //Just follow this lane
                        lookLeft = lookRight = false;
                        pQuadUnoccupied = &quad;
                    }
                    */
                    bool rightNext = true; //direction we will look in next - currently this is kind of redundant (it alternates anyway) but it could be used to change which direction we look first - cosmic vole June 1 2017
                    int offsetLeft = 0;
                    int offsetRight = 0;
                    CollisionAvoidance_GridQuad* pQuadRight = &quad;
                    CollisionAvoidance_GridQuad* pQuadLeft = &quad;
                    
                    while (lookLeft || lookRight)
                    {
                        //TODO ideally favour whichever side needs the least heading adjustment, or failing that, aim for the inside line of the next bend - cosmic vole June 1 2017
                        
                        CollisionAvoidance_GridQuad* pLeft = pQuadLeft->GetNeighbourLeft();
                        CollisionAvoidance_GridQuad* pRight = pQuadRight->GetNeighbourRight();
                        if (rightNext || !lookLeft)
                        {
                            rightNext = false;
                            if (!pRight)
                            {
                                lookRight = false;
                            }
                            else
                            {
                                offsetRight++;
                                if (!pRight->IsLaneOccupied(minDistForSwerve, minDistForSwerve, /*currentTime*/minTime, maxTime+minTimeForSwerve, truck.trucknum))//pRight->IsOccupied(minTime, maxTime, truck.trucknum))
                                {
                                    pQuadUnoccupied = pRight;
                                    offsetLeft = 0;
                                    break;
                                }
                                //!TODO June 16 2017 if the lane IS occupied near quad, we won't be able to move any further right unless there's at least a gap in this lane further back
                                //So we need a method FindLaneGap(float minGapLength, float maxDistanceFromQuad, <someTimeArgs>..., int ignoreTruckNum);
                                //Obviously the above gap would need to be min length = minDistForSwerve and it needs to be between the truck's current position/waypoint and the predicted collision position.
                                //If there's no such gap, we need to set lookRight to false.
                                //TODO June 16 2017 In future if we make the AI more sophisticated, it might be worth discounting lanes with stationary (upside down or wrecked) vehicles close ahead, if they aren't about to reset.
                                
                                pQuadRight = pRight;
                            }                        
                        }

                        if (!rightNext || !lookRight)
                        {
                            rightNext = true;
                            if (!pLeft)
                            {
                                lookLeft = false;
                            }
                            else
                            {
                                offsetLeft++;
                                if (!pLeft->IsLaneOccupied(minDistForSwerve, minDistForSwerve, /*currentTime*/minTime, maxTime+minTimeForSwerve, truck.trucknum))//pLeft->IsOccupied(minTime, maxTime, truck.trucknum))
                                {
                                    pQuadUnoccupied = pLeft;
                                    offsetRight = 0;
                                    break;
                                }
                                pQuadLeft = pLeft;
                                rightNext = true;
                            }
                        
                        }
                    }
                    
                    //Did we find a clear space to one side of the predicted collision?
                    if (pQuadUnoccupied != nullptr)
                    {
                        //What direction was it?
                        bool onRight = offsetRight > 0;
                        targetLane = quadLane + offsetRight - offsetLeft;
                        //We need more waypoints to cross two lanes
                        if (abs(targetLane - currentLane) > 1)//(offsetLeft + offsetRight > 1)
                            minWaypointsForSwerve *= 1.5f;//++; //= 7;//4;//3;
                        Vector3 frontLeft, frontRight, rearLeft, rearRight;
                        pQuadUnoccupied->GetCoordinates(frontLeft, frontRight, rearLeft, rearRight);
                        //We need to find the track aligned distance from the nearest waypoint to the chosen quad.
                        Vector3 rearCenter = (rearLeft + rearRight) * 0.5f;
                        int collisionWaypointID = pQuadUnoccupied->GetWaypointIDRear();
                        //We have to remap the collisionWaypintID onto the truck's current lap
                        if (collisionWaypointID <= truckAI.num_waypoints_in_lap && truckLapNum > 0)
                        {
                            collisionWaypointID += truckLapNum * truckAI.num_waypoints_in_lap;
                        }
                        Vector3 waypoint = truckAI.adjusted_waypoints[collisionWaypointID];//waypoints[collisionWaypointID];
                        //BUG!! This will make them go off track if waypoint is after a bend! Each waypoint adjustment needs to be relative to its respective target lane center, not the final one!
                        Vector3 adjustment = rearCenter - waypoint;
                        //Now adjust the waypoints, using linear interpolation to smooth the adjustment over a distance
                        //!!BUG June 16 2017 We don't want the maneuver to finish exactly at collisionWaypointID! Need an extra safety margin!
                        //!!AND it may need to be moved back further due to other vehicles in any lanes we have to cross!
                        int startWaypointID = trucksNextWaypoint;
                        /*
                        int maxStartWaypointID = collisionWaypointID - minWaypointsForSwerve;
                        if (startWaypointID < 0)
                            startWaypointID = 0;
                        if (startWaypointID < truckAI.current_waypoint_id)
                        {
                            startWaypointID = truckAI.current_waypoint_id;
                        }
                        */
                        int endWaypointID = collisionWaypointID + minWaypointsForSwerve;// * 2;
                        if (endWaypointID < truckAI.lane_change_end_wpt)
                            endWaypointID = truckAI.lane_change_end_wpt;
                        if (endWaypointID >= truckAI.waypoints.size())
                        {
                            endWaypointID = truckAI.waypoints.size();// - 1;
                        }
                        //The truck may already be on an adjusted path. We need to smoothly apply our new adjustment to its adjusted_waypoints.
                        int steps = collisionWaypointID - startWaypointID + 1;
                        if (steps > maxWaypointsForSwerve)
                        {
                            steps = maxWaypointsForSwerve;
                            //startWaypointID = collisionWaypointID - steps - 1;
                        }
                        
                        //if (/*lane_change_from != lane_change_to &&*/ lane_change_to != targetLane && lane_change_mid_wpt >= startWaypointID && abs(collisionWaypointID - lane_change_mid_wpt) < 5)
                        if (truckAI.lane_force_follow > 0 || (truckAI.lane_change_start_wpt <= truckAI.current_waypoint_id && truckAI.lane_change_end_wpt >= truckAI.current_waypoint_id && /*lane_change_from != lane_change_to &&*/
                            (truckAI.lane_change_to == targetLane || (truckAI.lane_change_to != truckAI.lane_change_from && truckAI.lane_change_to >= 0))))// && lane_change_mid_wpt >= startWaypointID && abs(collisionWaypointID - lane_change_mid_wpt) < 5)
                        {
                            //LOG("Already committed to previous maneuver."); TODO brake / decelerate to match speed of other vehicle? adjust maneuver in a stable way?
                            if (collisionWaypointID > startWaypointID)
                            {
                                // BUG TODO FIX THIS!!!!!
                                #if 0
                                truckAI.adjusted_waypoint_speed[/*collisionWaypointID-1*/startWaypointID] = truckAI.waypoint_speed[startWaypointID] * 0.90f;//0.80f;
                                if (startWaypointID+1 <= truckAI.waypoints.size())
                                    truckAI.adjusted_waypoint_speed[startWaypointID+1] = truckAI.waypoint_speed[startWaypointID+1] * 0.90f;
                                #endif
                            }
                            //debugInfo = L"Already committed to previous maneuver.";
                            break;
                        }
                        if (steps > 0)
                        {
                            if (steps < minWaypointsForSwerve)
                            {
                                //We keep getting steps == 8 here. cosmic vole August 14 2017
                                LOG("ERROR for truck " + TOSTRING(truckAI.beam->trucknum) + " Steps is only " + TOSTRING(steps) + " resuming racing line. Increasing.");
                                steps = minWaypointsForSwerve;
                            }
                            CollisionAvoidance_GridQuad *pQuad = pQuadUnoccupied;
                            int i = collisionWaypointID;
                            truckAI.lane_change_start_wpt = startWaypointID;
                            truckAI.lane_change_mid_wpt = startWaypointID + steps;//collisionWaypointID;
                            truckAI.lane_change_from = currentLane;//lane_change_to;
                            truckAI.lane_change_to = targetLane;//calc from offset left / right from whatever current lane is
                            truckAI.lane_change_end_wpt = endWaypointID;
                            if (truckAI.lane_change_mid_wpt >= truckAI.free_waypoints)
                                truckAI.lane_change_mid_wpt = truckAI.free_waypoints - 1;
                            
                            canAvoidTruck = true;
                            if (isCurrentTruck)
                                debugInfo = L"Initiating lane change to avoid truck.";
                            LOG("Initiating lane change for truck " + TOSTRING(truckAI.beam->trucknum) + " from lane: " + TOSTRING(currentLane) + " to lane: " + TOSTRING(targetLane) + " Start waypoint: " + TOSTRING(startWaypointID) + " End waypoint: " + TOSTRING(endWaypointID) + " Current waypoint: " + TOSTRING(truckAI.current_waypoint_id) + " Coll. waypt: " + TOSTRING(collisionWaypointID) + ".");
                         
                            CollisionAvoidance_FollowLane(&truckAI, targetLane, startWaypointID, truckAI.lane_change_mid_wpt/*collisionWaypointID*/, true, false, steps);//, currentLane);
                            CollisionAvoidance_FollowLane(&truckAI, targetLane, /*collisionWaypointID*/truckAI.lane_change_mid_wpt + 1, truckAI.free_waypoints/*endWaypointID*/, false, false);
                            
                            //Lane changing is all handled by FollowLane() now TODO Make FollowLane() able to make speed / power adjustments too for merges! Or just code separate curvature dependent AI for that.
                            
                            /*
                            for (int i=startWaypointID; i<=collisionWaypointID; i++)
                            {
                                Vector3 curWaypoint = truckAI->adjusted_waypoints[i];
                                float progress = (float)i / (float)steps;
                                //Adjustment needs to be relative to current waypoint.
                                adjustment = rearCenter - curWaypoint;
                                //TODO Also we were going to taper the waypoints into the correct lane by scaling and offset rather than just moving them all more and more towards the lane centre!
                                //!!Also there needs to be a safety margin at lane boundaries so half the vehicle doesn't overhang an adjacent lane!
                                //TODO Also we need to gradually tweak the waypoints AFTER the predicted collision to ease the vehicle back onto its original course
                                curWaypoint += progress * adjustment;
                                truckAI->adjusted_waypoints[i] = curWaypoint;
                            }
                            */
                            //TODO now we have to recalculate this truck's predicted path, as well as (if we accept the new path) any other trucks that could
                            //be affected by it
                            //We can assume it won't affect grid quads that aren't between the grid quads traced by the original path and the adjusted one,
                            //but the path may have crossed multiple lanes and if the track widens and narrows, we really need to regenerate the path.
                            //As we erase the old path, we need to build a list of the affected trucks and see if we have already run collision avoidance for them
                            //this frame. If we have, we need to do it again.
                            
                            break;
                        }// end if (steps > 0)
                        
                    }//end if (pQuadUnoccupied != nullptr)
                    else
                    {
                        if (isCurrentTruck)
                            debugInfo = L"No unoccupied lanes!"; // No reachable ones, anyway
                        //TODO brake!!!!!
                        
                        break;//Break the loop through predicted truck positions for this truck
                    }
                }
                else
                {
                    //wayptIDRear - trucksNextWaypoint >= minWaypointsForSwerve && timeToQuad >= minTimeForSwerve
                    /*
                    int wayptSpace;
                    if (num_waypoints_in_lap > 0)
                    {
                        wayptSpace = (wayptIDRear % num_waypoints_in_lap) - (trucksNextWaypoint % num_waypoints_in_lap);
                    }
                    else
                    {
                        wayptSpace = wayptIDRear - trucksNextWaypoint;
                    }
                    */
                    bool notEnoughSpace = wayptSpace < minWaypointsForSwerve;
                    bool notEnoughTime = timeToQuad < minTimeForSwerve;
                    if (notEnoughSpace)
                    {
                        if (notEnoughTime)
                        {
                            if (isCurrentTruck)
                                debugInfo = L"Not enough space or time ! Space: " + TOSTRING(wayptSpace) + L" waypts Time: " + TOSTRING(timeToQuad) + L" secs.";
                        }
                        else
                        {
                            if (isCurrentTruck)
                                debugInfo = L"Not enough space to avoid! Space: " + TOSTRING(wayptSpace) + L" waypts Time: " + TOSTRING(timeToQuad) + L" secs.";
                        }
                    }
                    else
                    {
                        if (isCurrentTruck)
                            debugInfo = L"Not enough  time to avoid! Space: " + TOSTRING(wayptSpace) + L" waypts Time: " + TOSTRING(timeToQuad) + L" secs.";
                    }
                    //We've predicted a collision with a truck we're responsible for avoiding but there's not enough space / time to overtake it
                    if (otherTruck)
                    {
                        //NOTE As we're currently relying on using multiple waypoints to overtake the other vehicle, we are going to need a BIG
                        //gap (unless we interpolate the waypoints so they're closer together - but at high speeds that won't work well).
                        //At the moment the waypoints and GridQuads are all 10 metres apart. What's tending to happen is the agent vehicle is driving up
                        //very close to the vehicle in front. Once it gets close to being in the same GridQuad, collisions will constantly be predicted
                        //that are too near to avoid. If we only brake until it's a bit further away, what will probably happen is in the next few frames
                        //the vehicles will just get too close again.
                        //Maybe we should plan the overtake here in the same step as braking - create the adjusted waypoints further up the track, assuming
                        //that the current collision will be delayed due to the braking.
                        //If we do that then we probably do need a way to make those adjusted waypoints have a condition attached (i.e. if no predicted collision up to first 2 or something).
                        //We need a way to allow further adjustment to previously adjusted waypoints, without the same adjustment getting applied 100s of times
                        //and checking that they don't get too tightly curved. Checking against current agent position as well. Is it possible to interp 3 ways between
                        //current agent position, original waypoint (that gives track direction) and target?
                        
                        //Increasing min gap to 25.0 means the braking easily avoids the collision BUT then collisions just stop being predicted
                        //(and currently IsLaneOccupied doesn't even pick up the car (WHY?).
                        //This means a better plan for overtakes is not to wait only until we have a predicted collision.
                        //Just change lane if current lane's occupied and an adjacent isn't, if we're lapping faster than the other vehicle.
                        
                        //At the moment the code is only looking at the gap to the other truck's current position. While this does matter (needs to be more than 1 GridQuad for an overtake)
                        //what really needs to be big enough for an overtake is the distance *to the predicted collision* which will normally be somewhere a bit further away!
                        
                        //TODO Is this the correct truck??? We have to be absolutely certain it is in front of us, otherwise both trucks will keep slowing down to a crawl!
                        //const PredictedTruckPosition *ptp2 = quad.GetSoonestCollision(ptp);
                        //Beam *otherTruck = BeamFactory::getSingleton().getTruck(ptp2->truckNum);
                        //Check the *current* positions and approach angles of the two trucks. Who is behind who? cosmic vole July 1 2017
                        Vector3 otherPosition = otherTruck->getPosition();
                        Vector3 otherVelocity = otherTruck->getVelocity();
                        Vector3 toOther = otherPosition - agentPos;
                        //TODO need to use dot products here. Need otherVelocity towards agent and agentVelocity towards other.
                        Vector3 relativeVelocity = otherVelocity - agentVelocity;
                        float closingSpeed = -(relativeVelocity.dotProduct(toOther.normalisedCopy()));
                        float distToOther = toOther.length();
                        LOG("Approaching vehicle. agentPos: " + TOSTRING(agentPos) + " otherPos: " + TOSTRING(otherPosition) + " closing speed:" + TOSTRING(closingSpeed) + " agent. velcoc.: " + TOSTRING(agentVelocity) + " other veloc.: " + TOSTRING(otherVelocity) + ".");
                        //TODO the following needs to be increased for long vehicles!
                        bool tooClose = false;
                        bool tooFast = false;
                        
                        if (aggression >= 30.0f)
                        {
                            tooClose = distToOther < 8.0f && hasCollisionWithTruckInFront;
                            //TODO the vehicles may be stuck together - need detection of this and then only randomly try braking / accelerating / momentary wiggling of steering if so. 
                            //Basically if we're stuck hitting this block of code many times in succession (count avg num vehicle to vehicle collisions
                            //also with stuckWithTruckNum), give up and accelerate every so often.
                            //5 m/s is 11 mph. 11 m/s is 25 mph. 18 m/s is 40 mph.
                            tooFast = (closingSpeed > 6.0f && distToOther < 6.0f) || (closingSpeed > 9.0f && distToOther < /*10.0f*/20.0f) || (closingSpeed > 25.0f && (distToOther < 25.0f || hasCollisionWithTruckInFront));//20.0f;//10.0f                                                                                    
                        }
                        else if (aggression >= 20.0f)
                        {
                            tooClose = distToOther < 10.0f && hasCollisionWithTruckInFront;
                            //TODO the vehicles may be stuck together - need detection of this and then only randomly try braking / accelerating / momentary wiggling of steering if so. 
                            //Basically if we're stuck hitting this block of code many times in succession (count avg num vehicle to vehicle collisions
                            //also with stuckWithTruckNum), give up and accelerate every so often.
                            //5 m/s is 11 mph. 11 m/s is 25 mph. 18 m/s is 40 mph.
                            tooFast = (closingSpeed > 4.0f && distToOther < 6.0f) || (closingSpeed > 7.0f && distToOther < /*10.0f*/20.0f) || (closingSpeed > 20.0f && (distToOther < 25.0f || hasCollisionWithTruckInFront));//20.0f;//10.0f                                                        
                        }
                        else if (aggression >= 10.0f)
                        {
                            tooClose = distToOther < 11.0f && hasCollisionWithTruckInFront;
                            //TODO the vehicles may be stuck together - need detection of this and then only randomly try braking / accelerating / momentary wiggling of steering if so. 
                            //Basically if we're stuck hitting this block of code many times in succession (count avg num vehicle to vehicle collisions
                            //also with stuckWithTruckNum), give up and accelerate every so often.
                            //5 m/s is 11 mph. 11 m/s is 25 mph. 18 m/s is 40 mph.
                            tooFast = (closingSpeed > 2.5f && distToOther < 6.0f) || (closingSpeed > 5.0f && distToOther < /*10.0f*/20.0f) || (closingSpeed > 15.0f && (distToOther < 25.0f || hasCollisionWithTruckInFront));//20.0f;//10.0f                            
                        }
                        else
                        {
                            tooClose = distToOther < /*4.0f*//*18.0f*//*16.0f*/15.0f && hasCollisionWithTruckInFront;
                            //TODO the vehicles may be stuck together - need detection of this and then only randomly try braking / accelerating / momentary wiggling of steering if so. 
                            //Basically if we're stuck hitting this block of code many times in succession (count avg num vehicle to vehicle collisions
                            //also with stuckWithTruckNum), give up and accelerate every so often.
                            //5 m/s is 11 mph. 11 m/s is 25 mph. 18 m/s is 40 mph.
                            tooFast = (closingSpeed > 1.5f && distToOther < 6.0f) || (closingSpeed > /*12.0f*//*10.0f*//*9.0f*//*5.0f*/2.5f && distToOther < /*10.0f*/20.0f) || (closingSpeed > 10.0f && (distToOther < 25.0f || hasCollisionWithTruckInFront));//20.0f;//10.0f
                        }
                        
                        if (tooClose || tooFast)
                        {
                            if (closingSpeed < 0.0f)
                                closingSpeed = 0.0f;
                            float approach = Radian(atan2(otherVelocity.z - agentVelocity.z, otherVelocity.x - agentVelocity.x)).valueDegrees();
                            if (approach < 0.0f)
                                approach = 360.0f + approach;
                            bool isSameDirection = !(approach > 90.0f && approach < 270.0f);
                            float agentSpeed = agentVelocity.length();
                            //TODO not sure this is right, need the speed adjustment in the agent's direction only
                            float targetSpeed = (agentSpeed - (closingSpeed * 0.8f) - /*1.0f*//*3.0f*//*3.5f*/2.0f) * 3.6f; // * 3.6 to convert to kph
                            if (distToOther < /*3.0f*//*5.0f*/5.5f && hasCollisionWithTruckInFront)
                            {
                                if (closingSpeed < 2.0f)//1.0f)
                                {
                                    if (targetSpeed < /*50.0f*/30.0f && (otherVelocity.length() * 3.6f) < /*50.0f*/30.0f)//12.0f)
                                    {
                                        //This condition is intended to stop two trucks slowing each other up more and more - e.g. if they're stuck together
                                        targetSpeed = /*50.0f*/30.0f;//12.0f; MAJOR BUG The waypoint speeds are in kph! We think our velocities are in metres per second! This has probably screwed PredictVehiclePath and other code too!
                                    }
                                    else if (aggression < 10.0f && targetSpeed < 15.0f)
                                    {
                                        targetSpeed = 15.0f;
                                    }
                                    else if (aggression >= 10.0f && targetSpeed < 23.0f)
                                    {
                                        targetSpeed = 23.0f;
                                    }
                                }
                                else if (aggression < 10.0f && targetSpeed < 4.0f)//2.0f)
                                {
                                    targetSpeed = 4.0f;//2.0f;
                                }
                                else if (aggression >= 10.0f && targetSpeed < 11.0f)
                                {
                                    targetSpeed = 11.0f;
                                }
                                
                            }
                            else if (aggression < 10.0f && targetSpeed < 5.0f)//4.0f)//3.5f)
                            {
                                targetSpeed = 5.0f;//4.0f;//3.5f;
                            }
                            else if (aggression >= 10.0f && aggression < 20.0f && targetSpeed < 12.0f)
                            {
                                targetSpeed = 12.0f;
                            }
                            else if (aggression >= 20.0f && aggression < 30.0f && targetSpeed < 18.0f)
                            {
                                targetSpeed = 18.0f;
                            }
                            else if (aggression >= 30.0f && targetSpeed < 25.0f)
                            {
                                targetSpeed = 25.0f;
                            }
                            
                            if (!hasCollisionWithTruckInFront)//60.0f)
                            {
                                if (aggression < 10.0f && targetSpeed < 65.0f)
                                {
                                    targetSpeed = 65.0f;//60.0f;
                                }
                                else if (aggression >= 10.0f && aggression < 20.0f && targetSpeed < 73.0f)
                                {
                                    targetSpeed = 73.0f;
                                }
                                else if (aggression >= 20.0f && aggression < 30.0f && targetSpeed < 81.0f)
                                {
                                    targetSpeed = 81.0f;
                                }
                                else if (aggression >= 30.0f && targetSpeed < 90.0f)
                                {
                                    targetSpeed = 90.0f;
                                }
                            }
                            
                            if (targetSpeed < truckAI.adjusted_waypoint_speed[truckAI.current_waypoint_id])
                            {
                                hasToBrakeForOtherTruck = true;
                                //!!! TODO What we need to do is log the time we set this adjusted waypoint speed and reset it to normal after that timeout
                                //!!! otherwise we can get to a point where the vehicle slows to a crawl, waiting to reach the next waypoint, even though the
                                //!!! vehicle it was trying to avoid is long gone. cosmic vole August 8 2017
                                if (isCurrentTruck)
                                    debugInfo = debugInfo.append(Ogre::UTFString(" Braking...."));
                                truckAI.adjusted_waypoint_speed[truckAI.current_waypoint_id] = targetSpeed;
                            }
                            
                            //Make sure we still stay in lane. We don't want the waypoints to try to resume the racing line while we're on a car's rear bumper.
                            int endWaypointID = truckAI.current_waypoint_id + 16;
                            //This check is REALLY important otherwise old lane change waypoints
                            //can make the vehicle swerve wildly and crash! cosmic vole August 14 2017
                            /*
                            if (endWaypointID < truckAI.lane_change_end_wpt)
                                endWaypointID = truckAI.lane_change_end_wpt;
                            if (endWaypointID > truckAI.free_waypoints)
                                endWaypointID = truckAI.free_waypoints;
                            if (truckAI.current_waypoint_id <= truckAI.lane_change_end_wpt && truckAI.lane_change_to >= 0)
                            {
                                targetLane = truckAI.lane_change_to;
                            }
                            */
                            if (agentPos.distance(truckAI.current_waypoint) < 6.5f)//6.0f)
                            {
                                int waypoint = truckAI.current_waypoint_id + 1;
                                if (waypoint <= truckAI.free_waypoints && targetSpeed < truckAI.adjusted_waypoint_speed[waypoint])
                                {
                                    hasToBrakeForOtherTruck = true;
                                    if (isCurrentTruck)
                                        debugInfo = debugInfo.append(Ogre::UTFString(" Braking...."));
                                    truckAI.adjusted_waypoint_speed[waypoint] = targetSpeed;                                    
                                }
                                /*
                                 * Don't add new stay in lane logic. The distance to the truck should already be enforcing some.
                                //CollisionAvoidance_FollowLane(&truckAI, currentLane, waypoint, waypoint + 2, false, false);
                                //CollisionAvoidance_FollowLane(&truckAI, currentLane, waypoint + 3, waypoint + 15, true, true);
                                CollisionAvoidance_FollowLane(&truckAI, currentLane, waypoint - 1, endWaypointID, true, false);//false, false);
                                */
                            }
                            else
                            {
                                /*
                                 * Don't add new stay in lane logic. The distance to the truck should already be enforcing some.
                                //CollisionAvoidance_FollowLane(&truckAI, currentLane, truckAI.current_waypoint_id, truckAI.current_waypoint_id + 3, false, false);
                                //CollisionAvoidance_FollowLane(&truckAI, currentLane, truckAI.current_waypoint_id + 4, truckAI.current_waypoint_id + 16, true, true);
                                CollisionAvoidance_FollowLane(&truckAI, currentLane, truckAI.current_waypoint_id, endWaypointID, true, false);//false, false);
                                 */
                            }
                            /*
                             * Don't add new stay in lane logic. The distance to the truck should already be enforcing some.
                            //Adjust all the waypoints to this lane for now, so there's no sudden transition if we run out
                            CollisionAvoidance_FollowLane(&truckAI, currentLane, endWaypointID, truckAI.free_waypoints, false, false);
                             */
                            //TODO Need a state variable to say it's staying in lane really. It can move out as soon as it needs to avoid a collision.
                            /*
                            truckAI.lane_change_from = currentLane;
                            truckAI.lane_change_to = currentLane;
                            truckAI.lane_change_start_wpt = truckAI.current_waypoint_id;
                            truckAI.lane_change_mid_wpt = truckAI.current_waypoint_id + 1;
                            truckAI.lane_change_end_wpt = endWaypointID;
                            targetLane = currentLane;
                            */
                        }
                        else
                        {
                            //If we're no longer too close to the other vehicle or we've slowed down enough, don't just stand there!
                            //TODO have a timeout and / or test actual agent velocity against the target speed.
                            if (!hasToBrakeForOtherTruck && ((agentVelocity.length() * 3.6f) <= truckAI.adjusted_waypoint_speed[truckAI.current_waypoint_id]) && truckAI.adjusted_waypoint_speed[truckAI.current_waypoint_id] < truckAI.waypoint_speed[truckAI.current_waypoint_id] * 0.90f)
                            {
                                truckAI.maxspeed = truckAI.adjusted_waypoint_speed[truckAI.current_waypoint_id] = truckAI.waypoint_speed[truckAI.current_waypoint_id];
                                if (truckAI.current_waypoint_id < truckAI.free_waypoints)
                                    truckAI.adjusted_waypoint_speed[truckAI.current_waypoint_id + 1] = truckAI.waypoint_speed[truckAI.current_waypoint_id + 1];
                            }
                        }
                    }
                    break;//Break the loop through predicted truck positions for this truck
                }
            }//end if (shouldAvoidCollision)

        }// end for loop on truckPath
        
        if (/*noCollisions*//*!hasCollisionWithTruckInFront &&*/ !hasToBrakeForOtherTruck && ((agentVelocity.length() * 3.6f) <= truckAI.adjusted_waypoint_speed[truckAI.current_waypoint_id]) && ((!alreadyCommittedToManeuver || truckAI.lane_change_to == currentLane) || (truckAI.adjusted_waypoint_speed[truckAI.current_waypoint_id] < truckAI.waypoint_speed[truckAI.current_waypoint_id] * 0.90f)))
        {
            truckAI.maxspeed = truckAI.adjusted_waypoint_speed[truckAI.current_waypoint_id] = truckAI.waypoint_speed[truckAI.current_waypoint_id];
            if (truckAI.current_waypoint_id < truckAI.free_waypoints)
                truckAI.adjusted_waypoint_speed[truckAI.current_waypoint_id + 1] = truckAI.waypoint_speed[truckAI.current_waypoint_id + 1];
        }
        else
        {
            static float msgTimeout = 8.0f;
            if (isCurrentTruck)
            {
                msgTimeout -= 1.0f;
                if (msgTimeout <= 0.0f)
                {
                    msgTimeout = 8.0f;
                    if (!hasToBrakeForOtherTruck)
                    {
                        if (truckAI.adjusted_waypoint_speed[truckAI.current_waypoint_id] > 0.0 && (agentVelocity.length() * 3.6f) > truckAI.adjusted_waypoint_speed[truckAI.current_waypoint_id])
                        {
                            LOG("Collision avoidance speed limit still in place because truck hasn't slowed yet to: " + TOSTRING(truckAI.adjusted_waypoint_speed[truckAI.current_waypoint_id]) + ".");
                        }
                        else if (truckAI.adjusted_waypoint_speed[truckAI.current_waypoint_id] > 0.0)
                        {
                            LOG("Collision avoidance speed limit still in place because maneuver underway. Limit is: " + TOSTRING(truckAI.adjusted_waypoint_speed[truckAI.current_waypoint_id]) + ".");
                        }
                    }
                    else
                    {
                        if (truckAI.adjusted_waypoint_speed[truckAI.current_waypoint_id] > 0.0)
                        {
                            if (hasCollisionWithTruckInFront)
                            {
                                LOG("Collision avoidance speed limit still in place because we're still braking for another truck IN FRONT. Limit is: " + TOSTRING(truckAI.adjusted_waypoint_speed[truckAI.current_waypoint_id]) + ".");
                            }
                            else
                            {
                                LOG("Collision avoidance speed limit still in place because we're still braking for another truck BEHIND. Limit is: " + TOSTRING(truckAI.adjusted_waypoint_speed[truckAI.current_waypoint_id]) + ".");
                            }
                        }
                    }
                }
            }
        }
        
        if (truckAI.lane_force_follow < 0 && /*noEvasiveAction &&*/ (/*truckAI.lane_change_to == truckAI.lane_change_from || truckAI.lane_change_to < 0 ||*/ truckAI.lane_change_end_wpt < truckAI.current_waypoint_id))
        {
            //Stay in the current lane if this truck is near to the other trucks. Otherwise it will follow the racing lane. cosmic vole July 1 2017
            bool hasNearbyTruck = false;
            for (int t = 0; t < numTrucks; t++)
            {
                Beam *otherTruck = trucks[t];
                float distToOther = otherTruck->getPosition().distance(agentPos);
                if (otherTruck && (otherTruck->trucknum != agentTruck->trucknum) && (distToOther < 60.0f));//90.0f)//150.0f)//300.0f)
                {   
                    //Ignore the player's vehicle at present as it just slows the AI competitors down too much - cosmic vole August 19 2017
                    //TODO we should probably also resume racing line if there are no trucks in front and no faster trucks very near behind
                    if (otherTruck->vehicle_ai)
                    {
                        //New - the other truck currently has to be in front or on the same waypoint, or very close and moving faster
                        if ((agentTruck->vehicle_ai && otherTruck->vehicle_ai->current_waypoint_id >/*=*/ agentTruck->vehicle_ai->current_waypoint_id) ||
                            (distToOther < 15.0f && otherTruck->getVelocity().length() > agentTruck->getVelocity().length() + 1.0f))
                        {
                            hasNearbyTruck = true;
                            break;
                        }
                    }
                }
            }
            if (hasNearbyTruck)
            {
                if (noCollisions && firstQuad)
                {
                    //TODO Even though no collision is predicted see if there's a slower car ahead in the current lane that we can overtake and whether the
                    //path is clear to the side to do so.
                    //TODO wait a random amount of time before running this again
                    //isCurLaneOccupied = firstQuad->IsLaneOccupied(100.0f, 20.0f, currentTime, currentTime + /*4.8f*/10.0f, agentTruck->trucknum);
                    if (firstQuad->LaneHasSlowerVehicleInFront(50.0f, currentTime, currentTime + 2.0f, agentTruck->getVelocity().length() - 1.0f, true, agentTruck->trucknum))
                    {
                        int newTargetLane = -1;
                        //TODO Favor inside line at next bend if possible
                        CollisionAvoidance_GridQuad *pLeft = firstQuad->GetNeighbourLeft();
                        CollisionAvoidance_GridQuad *pRight = firstQuad->GetNeighbourRight();
                        if (pLeft && !pLeft->IsLaneOccupied(65.0f, 20.0f, currentTime, currentTime + 6.0f, agentTruck->trucknum))
                        {
                            newTargetLane = pLeft->GetLaneNumber();
                        }
                        else if (pRight && !pRight->IsLaneOccupied(65.0f, 20.0f, currentTime, currentTime + 6.0f, agentTruck->trucknum))
                        {
                            newTargetLane = pRight->GetLaneNumber();
                        }
                        if (newTargetLane >= 0 && newTargetLane != currentLane && (newTargetLane != truckAI.lane_change_to || truckAI.current_waypoint_id > truckAI.lane_change_end_wpt))
                        {
                            int startWaypointID = truckAI.current_waypoint_id;
                            int steps = 12;//9;//maxWaypointsForSwerve;
                            int endWaypointID = startWaypointID + steps * 2;
                            if (endWaypointID < truckAI.lane_change_end_wpt)
                                endWaypointID = truckAI.lane_change_end_wpt;
                            if (endWaypointID > truckAI.free_waypoints)
                                endWaypointID = truckAI.free_waypoints;
                            truckAI.lane_change_start_wpt = startWaypointID;
                            truckAI.lane_change_mid_wpt = startWaypointID + steps;
                            truckAI.lane_change_from = currentLane;
                            truckAI.lane_change_to = newTargetLane;
                            truckAI.lane_change_end_wpt = endWaypointID;
                            if (truckAI.lane_change_mid_wpt <= truckAI.free_waypoints)
                                truckAI.lane_change_mid_wpt = truckAI.free_waypoints - 1;
                            
                            canAvoidTruck = true;
                            if (isCurrentTruck)
                                debugInfo = L"Initiating lane change (slower truck ahead).";
                            LOG("Initiating lane change (slower truck ahead) for truck " + TOSTRING(truckAI.beam->trucknum) + " from lane: " + TOSTRING(currentLane) + " to lane: " + TOSTRING(newTargetLane) + " Start waypoint: " + TOSTRING(startWaypointID) + " End waypoint: " + TOSTRING(endWaypointID) + " Current waypoint: " + TOSTRING(truckAI.current_waypoint_id) + ".");
                         
                            CollisionAvoidance_FollowLane(&truckAI, newTargetLane, startWaypointID, truckAI.lane_change_mid_wpt, true, false, steps); //currentLane);
                            CollisionAvoidance_FollowLane(&truckAI, newTargetLane, truckAI.lane_change_mid_wpt + 1, /*endWaypointID*/truckAI.free_waypoints, false, false);
                            targetLane = newTargetLane;
                        }
                    }
                }
                else
                {
                    if (isCurrentTruck && debugInfo == "")
                        debugInfo = UTFString("Staying in lane....");
    #if 0                                
                    //See if we're already merging into the lane. We don't want to keep reapplying the adjustment every frame or the turn will get increasingly severe. cosmic vole July 6 2017
                    bool alreadyMerging = (/*truckAI.lane_change_from == truckAI.lane_change_to &&*/ truckAI.lane_change_to >= 0 && truckAI.lane_change_start_wpt <= truckAI.current_waypoint_id && truckAI.lane_change_end_wpt >= truckAI.current_waypoint_id);
                    if (alreadyMerging /*&& abs(truckAI.lane_change_to - currentLane) < 2*/)
                    {
                        //Don't drift into an adjacent lane if we were already trying to stay in lane
                        //TODO possibly use the vehicle's position in the current lane as a starting point to do a gentle adjusment?
                        targetLane = truckAI.lane_change_to;
                    }
                    else
                    {
                        //if (isCurrentTruck)
                        //{
                        //    LOG("Wasn't already merging. Target lane: " + TOSTRING(currentLane) + " Previous lane_change_from: " + TOSTRING(truckAI.lane_change_from) + " to: " + TOSTRING(truckAI.lane_change_to) + " Curr. waypt: " + TOSTRING(truckAI.current_waypoint_id) + ".");
                        //}
                        targetLane = currentLane;
                    }
                    
                    if (alreadyMerging)
                    {
                        //We were already moving to stay in lane. Just extend it from the current waypoint.
                        int newEndWaypoint = (agentPos.distance(truckAI.current_waypoint) < 6.5f) ? truckAI.current_waypoint_id + 16 : truckAI.current_waypoint_id + 15;
                        //if (newEndWaypoint > truckAI.lane_change_end_wpt)
                        //{
                            //CollisionAvoidance_FollowLane(&truckAI, targetLane, truckAI.lane_change_end_wpt, newEndWaypoint, true, false, -1, currentLane); //false, false);
                            CollisionAvoidance_FollowLane(&truckAI, targetLane, truckAI.current_waypoint_id, newEndWaypoint, true, false, /*-1*/15, currentLane); //false, false);
                            truckAI.lane_change_end_wpt = newEndWaypoint;//BUG - This was causing erratic crashing because it could be less than the previous end waypoint
                            truckAI.lane_change_from = currentLane;
                            truckAI.lane_change_to = targetLane;
                            truckAI.lane_change_start_wpt = truckAI.current_waypoint_id;
                            truckAI.lane_change_mid_wpt = truckAI.current_waypoint_id + 7;
                        //}
                    }
                    else
                    {
                        if (agentPos.distance(truckAI.current_waypoint) < 6.5f)//6.0f)
                        {
                            int waypoint = truckAI.current_waypoint_id + 1;
                            //CollisionAvoidance_FollowLane(&truckAI, currentLane, waypoint, waypoint + 2, false, false);
                            //CollisionAvoidance_FollowLane(&truckAI, currentLane, waypoint + 3, waypoint + /*13*/15, true, true);
                            CollisionAvoidance_FollowLane(&truckAI, targetLane, truckAI.current_waypoint_id/*waypoint*/, waypoint + /*13*/15, true, false, 15);//, currentLane);
                        }
                        else
                        {
                            //CollisionAvoidance_FollowLane(&truckAI, currentLane, truckAI.current_waypoint_id, truckAI.current_waypoint_id + 3, false, false);
                            //CollisionAvoidance_FollowLane(&truckAI, currentLane, truckAI.current_waypoint_id + 4, truckAI.current_waypoint_id + /*14*/16, true, true);
                            CollisionAvoidance_FollowLane(&truckAI, targetLane, truckAI.current_waypoint_id, truckAI.current_waypoint_id + /*14*/15, true, false, 15);//, currentLane);
                        }
                        //TODO Need a state variable to say it's staying in lane really. It can move out as soon as it needs to avoid a collision.
                        truckAI.lane_change_from = targetLane;
                        truckAI.lane_change_to = targetLane;
                        truckAI.lane_change_start_wpt = truckAI.current_waypoint_id;
                        truckAI.lane_change_mid_wpt = truckAI.current_waypoint_id + 1;
                        truckAI.lane_change_end_wpt = truckAI.current_waypoint_id + 15;
                    }
    #endif

                    int endWaypointID = truckAI.current_waypoint_id + 15;
                    //This check is REALLY important otherwise old lane change waypoints
                    //can make the vehicle swerve wildly and crash! cosmic vole August 14 2017
                    if (endWaypointID < truckAI.lane_change_end_wpt)
                        endWaypointID = truckAI.lane_change_end_wpt;
                    if (endWaypointID > truckAI.free_waypoints)
                        endWaypointID = truckAI.free_waypoints;
                    if (truckAI.lane_change_to >= 0 && truckAI.lane_change_end_wpt >= truckAI.current_waypoint_id)
                    {
                        CollisionAvoidance_FollowLane(&truckAI, truckAI.lane_change_to, truckAI.current_waypoint_id, endWaypointID, false, false, 15);//, currentLane);
                        truckAI.lane_change_from = truckAI.lane_change_to;
                        targetLane = truckAI.lane_change_to;
                    }
                    else
                    {
                        CollisionAvoidance_FollowLane(&truckAI, currentLane, truckAI.current_waypoint_id, endWaypointID, true, false, 15);//, currentLane);
                        truckAI.lane_change_from = -1;
                        truckAI.lane_change_to = currentLane;
                        targetLane = currentLane;
                    }
                    //Adjust all the waypoints to this lane for now, so there's no sudden transition if we run out
                    CollisionAvoidance_FollowLane(&truckAI, currentLane, endWaypointID, truckAI.free_waypoints, false, false);
                    
                    truckAI.lane_change_start_wpt = truckAI.current_waypoint_id;
                    truckAI.lane_change_mid_wpt = truckAI.current_waypoint_id + 15;//1;
                    truckAI.lane_change_end_wpt = endWaypointID;
                }
            }
            else if (truckAI.lane_change_to >= 0)
            {
                if (isCurrentTruck && (debugInfo == "" || debugInfo == L""))
                    debugInfo = UTFString("Resuming racing line....");
                    
                //See if we're already merging into a lane.
                bool alreadyMerging = (truckAI.lane_change_from == truckAI.lane_change_to && truckAI.lane_change_to >= 0 && truckAI.lane_change_start_wpt <= truckAI.current_waypoint_id && truckAI.lane_change_end_wpt >= truckAI.current_waypoint_id);
                if (alreadyMerging /*&& abs(truckAI.lane_change_to - currentLane) < 2*/)
                {
                    //Don't drift into an adjacent lane if we were already trying to stay in lane
                    //TODO possibly use the vehicle's position in the current lane as a starting point to do a gentle adjusment?
                    targetLane = truckAI.lane_change_to;
                }
                else
                {
                    targetLane = currentLane;
                }
                int endWaypointID = truckAI.current_waypoint_id + 16;//22;/*15*//*10*/
                //This check is REALLY important otherwise old lane change waypoints
                //can make the vehicle swerve wildly and crash! cosmic vole August 14 2017
                if (endWaypointID < truckAI.lane_change_end_wpt)
                    endWaypointID = truckAI.lane_change_end_wpt;
                if (endWaypointID > truckAI.free_waypoints)
                    endWaypointID = truckAI.free_waypoints;
                //The truck can gradually resume the racing line
                CollisionAvoidance_FollowLane(&truckAI, /*targetLane*/-1, truckAI.current_waypoint_id, endWaypointID, true, true, 16);//22);
                //Make sure there are no rogue adjusted waypoints after this that could cause a crash
                CollisionAvoidance_FollowLane(&truckAI, -1, endWaypointID, truckAI.free_waypoints, false, true);
                //TODO Need a state variable to say it's resuming the racing line really. It can move out as soon as it needs to avoid a collision.
                truckAI.lane_change_from = targetLane;//currentLane;
                truckAI.lane_change_to = -1;//means racing line
                truckAI.lane_change_start_wpt = truckAI.current_waypoint_id;
                truckAI.lane_change_mid_wpt = truckAI.current_waypoint_id + 16;//22;//5;
                truckAI.lane_change_end_wpt = endWaypointID;
                targetLane = -1;
            }
        }
        
        if (agentTruck && m_sim_controller->GetBeamFactory()->getCurrentTruck() && agentTruck->trucknum == m_sim_controller->GetBeamFactory()->getCurrentTruck()->trucknum)
        {
            CollisionAvoidance_ShowDebugMsg(isCurLaneOccupied, !noCollisions, hasCollisionWithTruckInFront, canAvoidTruck, currentLane, targetLane, debugInfo);
        }
    }
}

//cosmic vole May 12 2017
void VehicleAI::CollisionAvoidance_PredictVehiclePath(float currentTime/*Vector3 mAgentAbsPosition, Vector3 agentVelocity, Vector3 mAgentHeading, Vector3 agentDir*/)
{
    //!TODO We assume before this is called that any existing PredictedTruckPositions have already been cleared from the VehicleAI and the race grid by the caller
    if (waypoints.empty())
        return;

    Vector3 mAgentAbsPosition = this->beam->getPosition();
    Vector3 agentVelocity = this->beam->getVelocity();
    Vector3 agentDir = this->beam->getDirection();
    agentDir.normalise();
    
    Vector3 agentTargetVector = current_waypoint - beam->getPosition();
    Vector3 agentTargetHeading = agentTargetVector;
    agentTargetHeading.normalise();
    float agentSpeedToTarget = agentVelocity.dotProduct(agentTargetHeading);
    //Make sure direction is not lost in the velocities
    if (agentSpeedToTarget < 0.001f)
        agentSpeedToTarget = 0.001f;
    Vector3 agentVelocityToTarget = agentTargetHeading * agentSpeedToTarget;
//    Vector3 agentDir = beam->getDirection();
//    agentDir.normalise();
    //Get agent current rotation - the same as beam->getHeadingDirectionAngle() except this handles where the truck doesn't have the usual camera nodes - cosmic vole
    float agentRotation = atan2(agentDir.dotProduct(Vector3::UNIT_X), (agentDir).dotProduct(-Vector3::UNIT_Z));
    //Turn it into a quaternion for current agent orientation
    Quaternion agentOrientation = Quaternion(Radian(agentRotation), Vector3::NEGATIVE_UNIT_Y);
    agentOrientation.normalise();//TODO does Ogre do this for us???
    //Get the desired agent rotation to face the target - cosmic vole
    float agentTargetRotation = atan2(agentTargetHeading.dotProduct(Vector3::UNIT_X), agentTargetHeading.dotProduct(-Vector3::UNIT_Z));
    //Turn it into a quaternion for desired agent orientation to target
    Quaternion agentTargetOrientation = Quaternion(Radian(agentTargetRotation), Vector3::NEGATIVE_UNIT_Y);
    agentTargetOrientation.normalise();
    
    
    Vector3 agentRect0, agentRect1, agentRect2, agentRect3;
    Vector3 agentFrontLeft, agentFrontRight, agentRearLeft, agentRearRight;
    beam->getCorners2D(agentFrontLeft, agentFrontRight, agentRearLeft, agentRearRight, 0.5f);//0.7f //1.0f);///*1.0f*/0.25f);//1.05f);//1.25f);//1.30f);
    //1.3f works quite well and 1.45f even better on start grid (1.65 is about the limit) BUT - think there's a bug in the scaling code

    //Brute force - just check the bounding rectangles of the vehicles at numerous time steps and calc how far to move.

    //Now we rotate the rectangles so the agent's one is axis aligned:
    Vector3 dirv = agentFrontRight - agentFrontLeft;
    dirv.normalise();
    Quaternion quatAxisAlign = dirv.getRotationTo(Vector3::UNIT_X);
    Vector3 newUp = quatAxisAlign * Vector3::UNIT_Y;
    newUp.normalise();
    Quaternion quatRestoreUp = newUp.getRotationTo(Vector3::UNIT_Y);
    quatAxisAlign = quatRestoreUp * quatAxisAlign;
    quatAxisAlign.normalise();
    Matrix4 matAxisAlign;// = Matrix4::IDENTITY;
    Matrix4 matWorldAlign;
    //Ogre like OpenGL works right to left for matrix multiplication
    matAxisAlign = /*Matrix4::getTrans(mAgentAbsPosition) * */ Matrix4(quatAxisAlign) * Matrix4::getTrans(-mAgentAbsPosition);
    matWorldAlign = matAxisAlign.inverse();
    Quaternion quatAlignToWorld = quatAxisAlign.Inverse();
    quatAlignToWorld.normalise();
    
    //Need to subtract centre of rotation first because quatAxisAlign is a rotation. That's why we made it into a matrix.
    agentRect0 = matAxisAlign * agentFrontLeft;//rotate(agentFrontLeft, quatAxisAlign, mAgentAbsPosition);//quatAxisAlign * agentFrontLeft;
    agentRect1 = matAxisAlign * agentFrontRight;//rotate(agentFrontRight, quatAxisAlign, mAgentAbsPosition);//quatAxisAlign * agentFrontRight;
    agentRect2 = matAxisAlign * agentRearLeft;//rotate(agentRearLeft, quatAxisAlign, mAgentAbsPosition);//quatAxisAlign * agentRearLeft;
    agentRect3 = matAxisAlign * agentRearRight;//rotate(agentRearRight, quatAxisAlign, mAgentAbsPosition);//quatAxisAlign * agentRearRight;
    
    Vector3 agentVelocAlign = quatAxisAlign * agentVelocity;
    
    Vector3 agentPosAlign = matAxisAlign * mAgentAbsPosition;//mAgentAbsPosition;//Agent position is the centre of rotation so doesn't change! quatAxisAlign * mAgentAbsPosition;
    
    //Calculate average current position of the vehicle, aligned to the agent's vehicle's axis
    Vector3 agentAvgPosition = (agentRect0 + agentRect1 + agentRect2 + agentRect3) * 0.25f;

    //This is where in the previous code we checked the approach angle between the agent and another vehicle. Checked if the opponent is moving head on towards the agent

    //Variables used in our collision avoidance simulation
    bool collisionOccurs = false;
    float timeLookAhead = 4.0f;//6.0f;//was 4.8f for ages until August 2017;//3.8f;//3.0f;//7.0f 5.0f;//3 second time lookahead. At 200 kph = 56 m/s, vehicle will travel 168 metres! At 60 kph = 17 m/s, 50 metres
#if 0  //Disabled until we can get the collisions to actually be detected! cosmic vole July 2 2017   
    //Increase time to look ahead if vehicle is moving faster - cosmic vole April 7 2017
    if (agentVelocity.length() >= 30.0f)
        timeLookAhead = 4.8f;//3.8f;
    else if (agentVelocity.length() >= 24.0f)
        timeLookAhead = 3.9f;//2.9f;//3.0f;
    else if (agentVelocity.length() >= 20.0f)
        timeLookAhead = 2.8f;//1.8f;//2.0f;
    else if (agentVelocity.length() >= 15.0f)
        timeLookAhead = 2.5f;//1.5f;//1.4f 1.6f;
    else
        timeLookAhead = 2.4f;//1.4f;//1.3f;//1.2f
#endif    
    Vector3 lastNewWaypointPos = Vector3::ZERO;
    Vector3 lastNewWaypointPosAlign = Vector3::ZERO;
    Vector3 nextAgentWaypoint = adjusted_waypoints[current_waypoint_id];//current_waypoint;
    Vector3 nextAgentWaypointAlign = matAxisAlign * current_waypoint;//rotate(current_waypoint, quatAxisAlign, mAgentAbsPosition);//quatAxisAlign * current_waypoint;
    int nextAgentWaypointID = current_waypoint_id;
    if (nextAgentWaypointID < 1)
        nextAgentWaypointID = 1;//This is important as the waypoints count from 1. Otherwise it will keep trying to load the collision avoidance grid every frame!
    int firstNewWaypointIndex = -1;
    float moveNeeded = 0.0f;
    //TODO accel estimate needs to be more sophisticated and include rpm, topping out and braking as well. Acceleration from a standing start is probably lower too.
    //TODO more to the point, if the other vehicle is slowed down due to another one ahead of it, it WILL NOT accelerate
    float agentMaxSpeed = (current_waypoint_id > 1) ? MAX(maxspeed, adjusted_waypoint_speed[current_waypoint_id-1]) : maxspeed;
    float estAgentAccel = (agentSpeedToTarget * 3.6f < agentMaxSpeed) ? (average_accel.length() * 0.8f/*0.9f * 0.5f*/) : 0.0f;//1.4f : 0.0f;//1.8f 2.2f 3.5f 2.2f 2.7f : 1.0f;

    if (agentVelocity.length() < 1.0f)//0.01f)
        estAgentAccel = 0.1f;
    //If the agent isn't pointing towards the target, we will use Quaternion.Slerp() to gradually turn it as we put down collision avoidance waypoints
    float agentRotNeeded = fabsf(agentRotation - agentTargetRotation) / (2.0f*M_PI);
    if (agentRotNeeded > 1.0f)
        agentRotNeeded = 1.0f;
    //This is the estimated turn speed where 1.0 would be a full 360 degree rotation in one second! 2.0 would be two rotations in one second.
    float estAgentTurnSpeed = (agentRotNeeded < 0.001f) ? 0.0f : (6.0f * agentRotNeeded);//was 3.0f for ages until July 2017 - worked v well but tiny bit slow was 2.0f 0.5 so a 360 degree rotation would be done over 2 seconds - cosmic vole
    float agentRotProgress = 0.0f;
    float agentTurnStartTime = 0.0f;
    float agentTotalDist = 0.0f;

    Vector3 agentPosWorld_ = mAgentAbsPosition;
    Vector3 agentVelocWorld_ = agentVelocity;
    Vector3 agentPosAlign_ = agentPosAlign;
    Vector3 agentDir_ = agentDir;
    Quaternion agentOrientation_ = agentOrientation;
    Vector3 agentDirAlign_ = quatAxisAlign * agentDir_;
    agentDirAlign_.normalise();
    Vector3 agentRect0_w = agentFrontLeft;
    Vector3 agentRect1_w = agentFrontRight;
    Vector3 agentRect2_w = agentRearLeft;
    Vector3 agentRect3_w = agentRearRight;
    
    bool frontCollision = false;
    bool rearCollision = false;
    bool leftCollision = false;
    bool rightCollision = false;
    
    static int reuseNode = -1;
    //if (t==0)
    //{
    //    reuseNode = 0;
    //}
    
    Ogre::String debugVehiclePath = "Vehicle Positions from waypoint " + TOSTRING(current_waypoint_id) + ":\n";
    Ogre::String debugFalsePositives = "";
    Ogre::String debugLane = "";
    CollisionAvoidance_GridQuad *debugLastFalsePosQuadLane0 = nullptr, *debugLastFalsePosQuadLane1 = nullptr, *debugLastFalsePosQuadLane2 = nullptr;
    
    //TODO can probably reduce time step if relative speed of the two vehicles is low
    //TODO we probably don't want to be doing this every single frame!!! Especially once it's already been run for another vehicle.
    //!!! MASSIVE BUG with this at the moment - after a lap or so, all vehicles seem to reset onto the same point! Superimposed on one another exactly! and it's not 0,0,0 in world space!
    //I'm thinking collision avoidance waypoints shouldn't be used for resets, for starters
    int step = -1;
    float timeStep = 0.02f;//0.05f;//0.01 probably best, was our long term value 0.01f;//0.0025 0.005 0.01f;
    float startTime = currentTime;//RoR::App::GetOgreSubsystem()->GetTimer()->getMilliseconds();
    bool waypointChanged = true;
    for (float time = /*0.01f*/0.0f; time < timeLookAhead; time+=timeStep)
    {
        step++;
        Vector3 agentRect0_, agentRect1_, agentRect2_, agentRect3_;
        //If the vehicles' initial velocities weren't aligned to their waypoint, we have to gradually rotate them
        //at a simulated turn speed
        bool rotated = false;
        //#if 0
        if (estAgentTurnSpeed != 0.0f /*&& agentRotProgress <= 1.0f*/)
        {
            rotated = true;
            agentRotProgress = estAgentTurnSpeed * (time-agentTurnStartTime);
            if (agentRotProgress > 1.0f)
                agentRotProgress = 1.0f;
            //Because we are not turning to face the target instantaneously, the target vector actually changes every iteration, so needs recalculating
            agentTargetVector = nextAgentWaypoint - agentPosWorld_;
            agentTargetHeading = agentTargetVector;
            agentTargetHeading.normalise();
            agentTargetRotation = atan2(agentTargetHeading.dotProduct(Vector3::UNIT_X), agentTargetHeading.dotProduct(-Vector3::UNIT_Z));
            //Turn it into a quaternion for desired agent orientation to target
            agentTargetOrientation = Quaternion(Radian(agentTargetRotation), Vector3::NEGATIVE_UNIT_Y);
            agentTargetOrientation.normalise();
            Quaternion newAgentOrientation = agentTargetOrientation;//Quaternion::Slerp(agentRotProgress, agentOrientation_, agentTargetOrientation, true);
            //q2 = r*q1 and q2*q1' = r
            //newAgentOrientation = newAgentRotation * agentOrientation
            //=> newAgentOrientation * agentOrientation.Inverse() == newAgentRotation
            //agentOrienation is the rotation of agentDir around the -ve y axis.
            Quaternion newAgentRotation = newAgentOrientation * agentOrientation.Inverse();
            newAgentRotation.normalise();
            agentDir_ = newAgentRotation * agentDir;
            agentDir_.normalise();
            //We will apply rotation to the vehicle, then update the velocity, then re-axis-align everything
            //Translate starting coordinates to origin, then rotate, then rotate to current position
            agentRect0_w = agentFrontLeft - mAgentAbsPosition;//BUG !!! After the first waypoint these will likely be wrong!
            agentRect1_w = agentFrontRight - mAgentAbsPosition;
            agentRect2_w = agentRearLeft - mAgentAbsPosition;
            agentRect3_w = agentRearRight - mAgentAbsPosition;

            agentRect0_w = newAgentRotation * agentRect0_w;
            agentRect1_w = newAgentRotation * agentRect1_w;
            agentRect2_w = newAgentRotation * agentRect2_w;
            agentRect3_w = newAgentRotation * agentRect3_w;
            
            agentRect0_w += agentPosWorld_;
            agentRect1_w += agentPosWorld_;
            agentRect2_w += agentPosWorld_;
            agentRect3_w += agentPosWorld_;
            
            //Now we rotate the velocity by the same amount.
            //Obviously this isn't accurate physically - we basically discard the components of the velocity
            //that were in the wrong direction - but if the turn is done slowly enough, it should be good enough
            //for our collision avoidance purposes (what we don't handle currently is vehicles spinning out) - cosmic vole March 4 2017
            float newWorldSpeed = agentVelocWorld_.dotProduct(agentDir_);
            if (newWorldSpeed < 0.001f)
                newWorldSpeed = 0.001f;
            agentVelocWorld_ = agentDir_ * newWorldSpeed;
        }
        
        //#endif
        
        //we should be doing this every step as the center of rotation moves
        //if (rotated)
        {
            //(Re)align everything to the agent rectangle's local axis
            //We rotate both the rectangles so the agent's one is axis aligned:
            dirv = agentRect1_w - agentRect0_w;//agentFrontRight - agentFrontLeft;
            dirv.normalise();
            quatAxisAlign = dirv.getRotationTo(Vector3::UNIT_X);
            newUp = quatAxisAlign * Vector3::UNIT_Y;
            newUp.normalise();
            quatRestoreUp = newUp.getRotationTo(Vector3::UNIT_Y);
            quatAxisAlign = quatRestoreUp * quatAxisAlign;
            quatAxisAlign.normalise();
            quatAlignToWorld = quatAxisAlign.Inverse();
            quatAlignToWorld.normalise();
            
        }
        
        //Accelerate the objects
        Vector3 prevAgentVelocAlign = agentVelocAlign;
        Vector3 prevAgentVelocWorld = agentVelocWorld_;
        Vector3 agentDist; 
        
        /*
        if (agentVelocAlign.length() < maxspeed)
        {
            agentDist = prevAgentVelocAlign * timeStep + (0.5f * estAgentAccel * timeStep * timeStep) * agentDirAlign_;
            agentVelocAlign += timeStep * estAgentAccel * agentDirAlign_; 
        }
        else
        {
            agentDist = prevAgentVelocAlign * timeStep;
        }
        */
        if (time > 0.0f)
        {
            if (agentVelocWorld_.length() * 3.6f < agentMaxSpeed)//maxspeed)
            {
                agentDist = prevAgentVelocWorld * timeStep + (0.5f * estAgentAccel * timeStep * timeStep) * agentDir_;
                agentVelocWorld_ += timeStep * estAgentAccel * agentDir_; 
            }
            else
            {
                agentDist = prevAgentVelocWorld * timeStep;
            }
        }
        else
        {
            agentDist = Vector3::ZERO;
        }
        
        
        agentVelocAlign = quatAxisAlign * agentVelocWorld_;

        //Update object positions - BUG / TODO - these incremental changes may be what is distorting the shape???
        agentRect0_w += agentDist;
        agentRect1_w += agentDist;
        agentRect2_w += agentDist;
        agentRect3_w += agentDist;
        agentPosWorld_ += agentDist;
        
        matAxisAlign = /*Matrix4::getTrans(mAgentAbsPosition) * */ Matrix4(quatAxisAlign) * Matrix4::getTrans(-agentPosWorld_);
        matWorldAlign = matAxisAlign.inverse();
        
        agentPosAlign_ = matAxisAlign * agentPosWorld_;//quatAlignToWorld * agentPosAlign_;
        //!! TODO check this! Is the new centre of rotation valid to align to world? What about aligning the velocities above?
        //otherPosWorld_ = rotate(otherPosAlign_, quatAlignToWorld, agentPosWorld_);//quatAlignToWorld * otherPosAlign_;
        agentTotalDist += agentDist.length();

        agentRect0_ = matAxisAlign * agentRect0_w; //rotate(agentRect0_w, quatAxisAlign, agentPosWorld_);//quatAxisAlign * agentRect0_w;
        agentRect1_ = matAxisAlign * agentRect1_w; //rotate(agentRect1_w, quatAxisAlign, agentPosWorld_);//quatAxisAlign * agentRect1_w;
        agentRect2_ = matAxisAlign * agentRect2_w;//rotate(agentRect2_w, quatAxisAlign, agentPosWorld_);//quatAxisAlign * agentRect2_w;
        agentRect3_ = matAxisAlign * agentRect3_w;//rotate(agentRect3_w, quatAxisAlign, agentPosWorld_);//quatAxisAlign * agentRect3_w;
        
        //agentVelocAlign = quatAxisAlign * agentVelocWorld_;
        //agentPosAlign_ = agentPosWorld_;//quatAxisAlign * agentPosWorld_;
        agentDirAlign_ = quatAxisAlign * agentDir_;
        agentDirAlign_.normalise();
        nextAgentWaypointAlign = matAxisAlign * nextAgentWaypoint; //rotate(nextAgentWaypoint, quatAxisAlign, agentPosWorld_);//quatAxisAlign * nextAgentWaypoint;
                                    
        //agentVelocWorld_ = quatAlignToWorld * agentVelocAlign;
                                    
        //See if we've reached any of the existing waypoints yet !!!! BUG change this measurement to be the same as the one used by the actual waypoint code - cosmic vole June 25 2017
        float agentDistToNextWPsq = agentPosWorld_.squaredDistance(nextAgentWaypoint);//agentPosAlign_.squaredDistance(nextAgentWaypointAlign);
        if (agentDistToNextWPsq < 25.0f)
        {
            //Update direction vectors to next waypoint
            waypointChanged = true;
            nextAgentWaypointID++;
            if (nextAgentWaypointID <= free_waypoints)
            {
                nextAgentWaypoint = /*waypoints*/adjusted_waypoints[nextAgentWaypointID];//TODO may need to renumber waypoints if we insert one, or use names
                nextAgentWaypointAlign = matAxisAlign * nextAgentWaypoint; //rotate(nextAgentWaypoint, quatAxisAlign, agentPosWorld_);//quatAxisAlign * nextAgentWaypoint;
                agentMaxSpeed = (nextAgentWaypointID > 1) ? adjusted_waypoint_speed[nextAgentWaypointID-1] : agentMaxSpeed;
                
                agentTargetVector = nextAgentWaypoint - agentPosWorld_;
                agentTargetHeading = agentTargetVector;
                agentTargetHeading.normalise();
                agentSpeedToTarget = agentVelocWorld_.dotProduct(agentTargetHeading);
                //Make sure direction is not lost in the velocities
                if (agentSpeedToTarget < 0.001f)
                    agentSpeedToTarget = 0.001f;
                agentVelocityToTarget = agentTargetHeading * agentSpeedToTarget;
                
                //Get agent current rotation - the same as beam->getHeadingDirectionAngle() except this handles where the truck doesn't have the usual camera nodes - cosmic vole
                float aNewAgentRotation = atan2(agentDir_.dotProduct(Vector3::UNIT_X), (agentDir_).dotProduct(-Vector3::UNIT_Z));
                //Turn it into a quaternion for current agent orientation
                agentOrientation_ = Quaternion(Radian(aNewAgentRotation), Vector3::NEGATIVE_UNIT_Y);
                agentOrientation_.normalise();
                 
                //Get the desired agent rotation to face the target - cosmic vole
                agentTargetRotation = atan2(agentTargetHeading.dotProduct(Vector3::UNIT_X), agentTargetHeading.dotProduct(-Vector3::UNIT_Z));
                //Turn it into a quaternion for desired agent orientation to target
                agentTargetOrientation = Quaternion(Radian(agentTargetRotation), Vector3::NEGATIVE_UNIT_Y);
                agentTargetOrientation.normalise();
                
                //TODO look up new maxspeed
                estAgentAccel = (agentSpeedToTarget * 3.6f < agentMaxSpeed) ? (average_accel.length() * 0.8f/*0.9f * 0.5f*/) : 0.0f;//1.4f : 0.0f;
                if (agentVelocWorld_.length() < 0.1f)//0.01f)
                    estAgentAccel = 0.1f;
                //If the agent isn't pointing towards the target, we will use Quaternion.Slerp() to gradually turn it as we put down collision avoidance waypoints
                agentRotNeeded = fabsf(aNewAgentRotation - agentTargetRotation) / (2.0f*M_PI);
                //This is the estimated turn speed where 1.0 would be a full 360 degree rotation in one second!
                estAgentTurnSpeed = (agentRotNeeded < 0.001f) ? 0.0f : (6.0f * agentRotNeeded);//Was 3.5 for ages until July 2017 0.5 so a 360 degree rotation would be done over 2 seconds - cosmic vole
                agentRotProgress = 0.0f;
                agentTurnStartTime = time;
                //TODO update flag for last waypoint
 
            }
        }
        else
        {
            waypointChanged = false;
        }

        //Test which lane grid squares the vehicle is over and mark off the time and information - cosmic vole May 19 2017
        //At the moment we look them up based on the closest waypoints but a quad tree could also be developed - cosmic vole May 19 2017
        //int startLaneIndex = 0;
        //int endLaneIndex = -1;
        Ogre::String raceWaypointKey;
        //See if we're on a circuit with multiple laps. If we are, we need to translate the current waypoint onto the equivalent on the first lap
        //so that we can collision detect between vehicles that are on different laps
        int nextWaypointOnFirstLap;
        int lastWaypointOnFirstLap;
        if (num_waypoints_in_lap > 0 && num_waypoints_in_lap <= free_waypoints)
        {
            nextWaypointOnFirstLap = ((nextAgentWaypointID - 1) % num_waypoints_in_lap) + 1;
            lastWaypointOnFirstLap = nextWaypointOnFirstLap - 1;
            //Wrap around - this will work even if it's a single lap race on a circuit. It just won't work on a drag strip, so set num_waypoints_in_lap to 0 for that. cosmic vole June 25 2017
            if (lastWaypointOnFirstLap <= 0)
                lastWaypointOnFirstLap += num_waypoints_in_lap;
        }
        else
        {
            nextWaypointOnFirstLap = nextAgentWaypointID;
            lastWaypointOnFirstLap = nextAgentWaypointID - 1;
        }
        
        //Keep track of the predicted vehicle positions at each waypoint, for debugging only
        if (waypointChanged || time == 0.0f)
        {
            debugVehiclePath += TOSTRING(agentRect0_w.x) + ", " + TOSTRING(agentRect0_w.z) + ",  " +
                                TOSTRING(agentRect1_w.x) + ", " + TOSTRING(agentRect1_w.z) + ",  " +
                                TOSTRING(agentRect3_w.x) + ", " + TOSTRING(agentRect3_w.z) + ",  " +
                                TOSTRING(agentRect2_w.x) + ", " + TOSTRING(agentRect2_w.z) + ",  \n";
        }
        
        //BIG BUG! Not many quads seem to intersect. Possibly need a loop to check previous waypoint quads as well as current (or previous, current and next?)
        //TODO ! We can speed this up if we only do this dictionary lookup each time the waypoint changes! cosmic vole June 25 2017
        Ogre::String raceIDStr = TOSTRING(raceID);
        if (lastWaypointOnFirstLap >= 0)//nextAgentWaypointID > 0)//BUG / Problem waypoint 0 is not supported - see GenerateLanes() - cosmic vole June 25 2017 = 0)
        {
            if (raceIDStr.compare("") == 0)//TODO raceID is an int, -1 if no race!
            {
                return;
            }
            Ogre::String nextAgentWaypointName = waypoint_names[nextWaypointOnFirstLap];//nextAgentWaypointID];
            raceWaypointKey = raceIDStr + "|||" + nextAgentWaypointName;
            //endLaneIndex = raceWaypointToLanePointIndex[raceIDStr + "|||" + nextAgentWaypointName];
            //if (nextAgentWaypointID >= 1)
            //{
            //    Ogre::String lastAgentWaypointName = waypoint_names[lastWaypointOnFirstLap];//nextAgentWaypointID-1];
            //    startLaneIndex = raceWaypointToLanePointIndex[raceIDStr + "|||" + lastAgentWaypointName];//!< Maps raceID + "|||" + waypoint name to an index into that race's entry in raceLanePoints. cosmic vole May 18 2017
            //}        
        }
        else
        {
            return;
        }
        
        std::map<Ogre::String, std::vector<CollisionAvoidance_GridQuad*>>::iterator iter;
        iter = raceGridQuads.find(raceWaypointKey);
        //OLD - Lazy load the race track's collision avoidance grid of lanes if it hasn't already been generated - cosmic vole June 24 2017 NOTE on some tracks this may cause lag!
        if (iter == raceGridQuads.end())
        {
            //NO - this wasn't working. It must have been running too early before all the waypoints and edge points had been defined!
            //It now runs in AddWaypoint() BUT that still requires each edge point to be added first AND it slows everything down
            //I think the FindRoadEdgePoint methods are possibly very slow. Worth caching the results for each waypoint.
            //It's probably worth either adding an angelscript function to set up collision avoidance that does this, or doing it as part of the race setup. cosmic vole June 27 2017
            //** OK WTF? If I have just this, only the first few debug points show. If I have this AND the call in AddWaypoint(), they all show and code is slow.
            //If I have the call in AddWaypoint() but not this, NONE of them show! WHY?! Two waypoints needed for each quad? ALSO raceID is needed!
            //Now even if we return from this when raceID is empty, we still only get the first few showing!
            CollisionAvoidance_GenerateLanes(-1, -1, waypoints, road_edge_points_left, road_edge_points_right);
            iter = raceGridQuads.find(raceWaypointKey);
            //return;
        }
        std::vector<CollisionAvoidance_GridQuad*>& nextGridQuads = iter->second;//raceGridQuads[raceWaypointKey];
        
        //Important - We actually have to check the quads for the previous waypoint as well, because the truck may not have left those yet even though
        //it got close enough to trigger the waypoint change!
        Ogre::String lastAgentWaypointName = waypoint_names[lastWaypointOnFirstLap];
        raceWaypointKey = raceIDStr + "|||" + lastAgentWaypointName;
        iter = raceGridQuads.find(raceWaypointKey);
        int numLastGridQuads = 0;
        std::vector<CollisionAvoidance_GridQuad*>& lastGridQuads = iter->second;
        if (iter != raceGridQuads.end())
            numLastGridQuads = lastGridQuads.size();
        
        for (int i = 0; i < numLastGridQuads + nextGridQuads.size(); i++)
        {
            CollisionAvoidance_GridQuad *gridQuad;
            if (i < numLastGridQuads)
            {
                gridQuad = lastGridQuads[i];
            }
            else
            {
                gridQuad = nextGridQuads[i - numLastGridQuads];
            }
            if (!gridQuad)
                continue;
            //TODO If it intersects multiple adjacent quads it might be worth just recording the minimum distance from the quad centre achieved by the vehicle, to aid lane selection when overtaking. 
            bool intersectsQuad = false;
            if (gridQuad->IntersectsQuadCenteredOnWaypoints(waypoints, agentRect0_w, agentRect1_w, agentRect3_w, agentRect2_w))
            {   //We could use time since race start OR just possibly App::GetOgreSubsystem()->GetTimer()->getMilliseconds(); although that timer can be reset
                PredictedTruckPosition ptp = gridQuad->AddVehiclePosition(beam->trucknum, startTime + time - timeStep, startTime + time);
                //Debug possible false positives
                bool falsePositive = false;
                if (beam)
                {
                    bool hasCollision = gridQuad->HasCollisionWith(ptp);
                    int quadLane = gridQuad->GetLaneNumber();
                    if (current_waypoint_id <= lane_change_end_wpt && current_waypoint_id >= lane_change_start_wpt &&
                        ((lane_change_from > quadLane && lane_change_to > quadLane) || (lane_change_from < quadLane &&
                        lane_change_to < quadLane && lane_change_from >= 0 && lane_change_to >= 0)))
                    {
                        Vector3 quadFrontLeft, quadFrontRight, quadRearRight, quadRearLeft;
                        gridQuad->GetCoordinates(quadFrontLeft, quadFrontRight, quadRearRight, quadRearLeft);
                        Vector3 quadFrontLeftWp, quadFrontRightWp, quadRearLeftWp, quadRearRightWp;
                        gridQuad->GetCoordinatesCenteredOnWaypoints(waypoints, quadFrontLeftWp, quadFrontRightWp, quadRearLeftWp, quadRearRightWp);
                        //if (hasCollision)
                        {
                            Vector3 lastAgentWaypoint = Vector3::ZERO;
                            int lastAgentWaypointID = nextAgentWaypointID;
                            if (nextAgentWaypointID > 1)
                            {
                                lastAgentWaypointID = nextAgentWaypointID - 1;
                                lastAgentWaypoint = adjusted_waypoints[lastAgentWaypointID];
                            }
                            int lastAgentWaypointLane = CollisionAvoidance_GetLaneNumber(lastAgentWaypointID, true);
                            int nextAgentWaypointLane = CollisionAvoidance_GetLaneNumber(nextAgentWaypointID, true);
                            Ogre::String msg = "Possible false positive collision for truck: " + TOSTRING(beam->trucknum) + " at time: " + TOSTRING(time) + " Quad lane: " + TOSTRING(quadLane) +
                            " Vehicle lane from: " + TOSTRING(lane_change_from) + " to: " + TOSTRING(lane_change_to) + " Vehicle corners: " +
                            TOSTRING(agentFrontLeft) + " " + TOSTRING(agentFrontRight) + " " + TOSTRING(agentRearRight) + " " + TOSTRING(agentRearLeft) +
                            " Projected vehicle corners: " + TOSTRING(agentRect0_w) + " " + TOSTRING(agentRect1_w) + " " + TOSTRING(agentRect3_w) + " " + TOSTRING(agentRect2_w) +
                            " Quad corners: " + TOSTRING(quadFrontLeft) + " " + TOSTRING(quadFrontRight) + " " + TOSTRING(quadRearRight) + " " +
                            TOSTRING(quadRearLeft) + 
                            " Quad corners round wpt: " + TOSTRING(quadFrontLeftWp) + " " + TOSTRING(quadFrontRightWp) + " " + TOSTRING(quadRearRightWp) + " " +
                            TOSTRING(quadRearLeftWp) + " Projected vehicle pos: " + TOSTRING(agentPosWorld_) + " Target wpt: " + TOSTRING(nextAgentWaypoint) +
                            " Last (adjusted) wpt: " + TOSTRING(lastAgentWaypoint) +
                            " Quad rear wpt: " + TOSTRING(waypoints[gridQuad->GetWaypointIDRear()]) + " Adjusted: " + TOSTRING(adjusted_waypoints[gridQuad->GetWaypointIDRear()]) +
                            " Quad front wpt: " + TOSTRING(waypoints[gridQuad->GetWaypointIDFront()]) + " Adjusted: " + TOSTRING(adjusted_waypoints[gridQuad->GetWaypointIDFront()]) +
                            " Last truck wpt lane: " + TOSTRING(lastAgentWaypointLane) + " Next truck wpt lane: " + TOSTRING(nextAgentWaypointLane) + ".";
                            //if (hasCollision)
                            //    LOG(msg);
                            //** TODO see what lane the truck's previous and current adjusted waypoints are in.
                            //** If the truck was originally resuming or leaving the racing line, note that lane_change_from may not be -1
                            //** as it gets set to currentLane in some of the code and some of the waypoints may drift out of the lane.
                            //** We can try and change the code to force the truck not to drift out of the lane but this may cause the trucks to crash...
                            debugFalsePositives += msg + "\n";
                            if (quadLane == 0)
                                debugLastFalsePosQuadLane0 = gridQuad;
                            else if (quadLane == 1)
                                debugLastFalsePosQuadLane1 = gridQuad;
                            else if (quadLane == 2)
                                debugLastFalsePosQuadLane2 = gridQuad;

                        }
                        if (!stability_lost_control && fabsf(stability_max_negative_steer) < 7.0f && fabsf(stability_max_positive_steer) < 7.0f)
                        {
                            falsePositive = true;
                        }
                    }
                }
                //TODO I suspect this is overflowing memory! Need to merge with ptps that are already there for the same grid square!
                if (!falsePositive)
                {
                    predictedVehiclePath.push_back(ptp);
                    intersectsQuad = true;
                }
            }//<--- added this as TEMP debug because the debug rects weren't appearing!s    
        //#if 0                
                //Debug projected positions of the vehicles - cosmic vole March 10 2017
                Beam *curTruck = m_sim_controller->GetBeamFactory()->getCurrentTruck();
                static std::vector<SceneNode*> debugCol = std::vector<SceneNode*>();
                //TODO these debug rects never seem to be showing up now! And it seg faults if we run this too much!
                if (curTruck && waypointChanged && (curTruck == beam /*|| gridQuad.HasCollisionWith(ptp)*/) /*&& (step % 6 == 0)*/)//(curTruck->trucknum == beam->trucknum))
                {
                    static MaterialPtr debugMaterialAgent;
                    static MaterialPtr debugMaterialOther;
                    static MaterialPtr debugMaterialCollision;
                    static bool madeMaterials = false;
                    SceneNode *nodeAgent, *nodeOther;
                    Entity *entityAgent, *entityOther;
                    //static int reuseNode = -1;

                    int count = debugCol.size();
                    if (count>500)//20000//3000)//12000)
                    {
                        if (reuseNode < 0)
                        {
                            reuseNode = 0;
                        }
                        else if (reuseNode >= count-8)
                        {
                            reuseNode = 0;
                        }
                        //int toDel = count - 20000;
                        /* Argh this deletion stuff currently seg faults - WHY?! cosmic vole
                        for (int k=0;k<toDel; k++)
                        {
                            SceneNode * node = debugCol.at(k);//debugCol[k];
                            //gEnv->sceneManager->destroyMovableObject(node->getAttachedObject(0)); Seg faults!
                            node->detachObject((unsigned short)0);
                            //gEnv->sceneManager->destroySceneNode(node);
                            //debugCol.erase(debugCol.begin());
                        }
                        */
                        
                    }
                    else
                    {
                        reuseNode = -1;
                    }
                    
                    //Hide all the existing markers first
                    if (i == 0 && time <= 0.01f)
                        for (int j = 0; j < debugCol.size(); j++)
                    {
                        SceneNode *node = debugCol.at(j);
                        Entity *entity = (Entity *)node->getAttachedObject(0);
                        node->setPosition(Vector3::ZERO);
                    }
                    
                    if (!madeMaterials)
                    {
                        madeMaterials = true;
                        /*
                        debugMaterialAgent = MaterialManager::getSingleton().create("debugMaterialAgent", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
                        debugMaterialAgent->setSelfIllumination(0.3f,0.0f,0.6f);
                        debugMaterialAgent->setAmbient(0.3f,0.0f,0.6f);
                        debugMaterialAgent->setDiffuse(0.3f,0.0f,0.6f,0.1f);
                        */
                        debugMaterialOther = MaterialManager::getSingleton().create("debugMaterialOther", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
                        debugMaterialOther->setSelfIllumination(0.0f,0.6f,0.1f);
                        debugMaterialOther->setAmbient(0.0f,0.6f,0.1f);
                        debugMaterialOther->setDiffuse(0.0f,0.6f,0.1f,0.9f);
                        /*
                        debugMaterialCollision = MaterialManager::getSingleton().create("debugMaterialCollision", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
                        debugMaterialCollision->setSelfIllumination(0.7f,0.0f,0.0f);
                        debugMaterialCollision->setAmbient(0.7f,0.0f,0.0f);
                        debugMaterialCollision->setDiffuse(0.7f,0.0f,0.0f,0.9f);
                        */
                    }
                    Vector3 debugRect0 = agentRect0_w;
                    Vector3 debugRect1 = agentRect1_w;
                    Vector3 debugRect2 = agentRect2_w;
                    Vector3 debugRect3 = agentRect3_w;
                    
                    //BUG something's wrong - agentRect0 and 1 seem to trace out the LHS of the vehicle only! July 1 2017
                    debugCollision(reuseNode, debugCol, debugRect0, std::string("debugMaterialOther"));
                    debugCollision(reuseNode, debugCol, debugRect1, std::string("debugMaterialOther"));
                    //debugCollision(reuseNode, debugCol, debugRect2, std::string("debugMaterialOther"));
                    //debugCollision(reuseNode, debugCol, debugRect3, std::string("debugMaterialOther"));
                    
                    /*
                    gridQuad->GetCoordinates(debugRect0, debugRect1, debugRect2, debugRect3);
                    debugRect0.y = debugRect1.y = debugRect2.y = debugRect3.y = mAgentAbsPosition.y + 0.2f;

                    //TODO ! These are never showing up anymore! They show eventually when we use the agentRect coords, suggesting the gridQuad coords are erroneous
                    //BUG - Note this will only show collisions with vehicles earlier in the trucks array since the paths are cleared each frame!
                    if (gridQuad->HasCollisionWith(beam->trucknum))
                    {
                        debugCollision(reuseNode, debugCol, debugRect0, std::string("debugMaterialCollision"));
                        debugCollision(reuseNode, debugCol, debugRect1, std::string("debugMaterialCollision"));
                        debugCollision(reuseNode, debugCol, debugRect2, std::string("debugMaterialCollision"));
                        debugCollision(reuseNode, debugCol, debugRect3, std::string("debugMaterialCollision"));

                    }
                    else if (!intersectsQuad)
                    {
                        //debugCollision(reuseNode, debugCol, debugRect0, std::string("debugMaterialOther"));
                        //debugCollision(reuseNode, debugCol, debugRect1, std::string("debugMaterialOther"));
                        //debugCollision(reuseNode, debugCol, debugRect2, std::string("debugMaterialOther"));
                        //debugCollision(reuseNode, debugCol, debugRect3, std::string("debugMaterialOther"));                        
                    }
                    else
                    {
                        debugCollision(reuseNode, debugCol, debugRect0, std::string("debugMaterialAgent"));
                        debugCollision(reuseNode, debugCol, debugRect1, std::string("debugMaterialAgent"));
                        debugCollision(reuseNode, debugCol, debugRect2, std::string("debugMaterialAgent"));
                        debugCollision(reuseNode, debugCol, debugRect3, std::string("debugMaterialAgent"));
                    }
                    */

                   
                }
                //End Debug projected positions
        //#endif                                    
                
                
                
            //}
        }
        
        #if 0
/*
        std::vector<Ogre::Vector3>& lanePoints = raceLanePoints[raceID];
        
        for (int i=startLaneIndex; i<endLaneIndex;)
        {
            Ogre::Vector3& lane1LeftRear = lanePoints[i];
            i++;
            Ogre::Vector3& lane2LeftRear = lanePoints[i];
            i++;
            Ogre::Vector3& lane3LeftRear = lanePoints[i];
            i++;
            Ogre::Vector3& lane3RightRear = lanePoints[i];
            i++;
            Ogre::Vector3& lane1LeftFront = lanePoints[i];
            i++;
            Ogre::Vector3& lane2LeftFront = lanePoints[i];
            i++;
            Ogre::Vector3& lane3LeftFront = lanePoints[i];
            i++;
            Ogre::Vector3& lane3RightFront = lanePoints[i];
            i++;
            bool inLane1 = QuadsIntersect(lane1LeftRear, lane1LeftFront, lane2LeftFront, lane2LeftRear, agentRect0_w, agentRect1_w, agentRect2_w, agentRect3_w);
            bool inLane2 = QuadsIntersect(lane2LeftRear, lane2LeftFront, lane3LeftFront, lane3LeftRear, agentRect0_w, agentRect1_w, agentRect2_w, agentRect3_w);
            bool inLane3 = QuadsIntersect(lane3LeftRear, lane3LeftFront, lane3RightFront, lane3RightRear, agentRect0_w, agentRect1_w, agentRect2_w, agentRect3_w);
            PredictedTruckPosition ptp;
            ptp.truckNum = beam->trucknum;
        }
*/        
        
        
        
        //Now test both rectangles for intersection
        //TODO this can probably be sped up with the separating axis algorithm if we don't need the actual intersection points anymore
        bool gotNewIntersection, linesAreParallel;
        bool intersectsFF, intersectsFL, intersectsFR, intersectsFB;
        bool intersectsLF, intersectsLL, intersectsLR, intersectsLB;
        bool intersectsRF, intersectsRL, intersectsRR, intersectsRB;
        bool intersectsBF, intersectsBL, intersectsBR, intersectsBB;
        Vector3 intersectionFF, intersectionFL, intersectionFR, intersectionFB;
        Vector3 intersectionLF, intersectionLL, intersectionLR, intersectionLB;
        Vector3 intersectionRF, intersectionRL, intersectionRR, intersectionRB;
        Vector3 intersectionBF, intersectionBL, intersectionBR, intersectionBB;
       
       /*
        //Test agentRect0->1 against otherRect0->1 (it's the fronts of the two trucks)
        //Note we ignore return value as we only care if the intersection is within the bounds of the two lines
        lineIntersectionXZ(agentRect0_, agentRect1_, otherRect0_, otherRect1_, intersectsFF, linesAreParallel, intersectionFF);
        //Test agentRect0->1 against otherRect0->2 (front of agent against LHS of other rect)
        lineIntersectionXZ(agentRect0_, agentRect1_, otherRect0_, otherRect2_, intersectsFL, linesAreParallel, intersectionFL);
        //Test agentRect0->1 against otherRect1->3 (front of agent against RHS of other rect)
        lineIntersectionXZ(agentRect0_, agentRect1_, otherRect1_, otherRect3_, intersectsFR, linesAreParallel, intersectionFR);
        //Test agentRect0->1 against otherRect2->3 (front of agent against rear of other rect)
        lineIntersectionXZ(agentRect0_, agentRect1_, otherRect2_, otherRect3_, intersectsFB, linesAreParallel, intersectionFB);

        
        //Test agentRect0->2 against otherRect0->1 (LHS of agent rect against front of other truck)
        lineIntersectionXZ(agentRect0_, agentRect2_, otherRect0_, otherRect1_, intersectsLF, linesAreParallel, intersectionLF);
        //Test agentRect0->2 against otherRect0->2 (LHS of agent against LHS of other rect)
        lineIntersectionXZ(agentRect0_, agentRect2_, otherRect0_, otherRect2_, intersectsLL, linesAreParallel, intersectionLL);
        //Test agentRect0->2 against otherRect1->3 (LHS of agent against RHS of other rect)
        lineIntersectionXZ(agentRect0_, agentRect2_, otherRect1_, otherRect3_, intersectsLR, linesAreParallel, intersectionLR);
        //Test agentRect0->2 against otherRect2->3 (LHS of agent against rear of other rect)
        lineIntersectionXZ(agentRect0_, agentRect2_, otherRect2_, otherRect3_, intersectsLB, linesAreParallel, intersectionLB);
        
        
        //Test agentRect1->3 against otherRect0->1 (RHS of agent rect against front of other truck)
        lineIntersectionXZ(agentRect1_, agentRect3_, otherRect0_, otherRect1_, intersectsRF, linesAreParallel, intersectionRF);
        //Test agentRect1->3 against otherRect0->2 (RHS of agent against LHS of other rect)
        lineIntersectionXZ(agentRect1_, agentRect3_, otherRect0_, otherRect2_, intersectsRL, linesAreParallel, intersectionRL);
        //Test agentRect1->3 against otherRect1->3 (RHS of agent against RHS of other rect)
        lineIntersectionXZ(agentRect1_, agentRect3_, otherRect1_, otherRect3_, intersectsRR, linesAreParallel, intersectionRR);
        //Test agentRect1->3 against otherRect2->3 (RHS of agent against rear of other rect)
        lineIntersectionXZ(agentRect1_, agentRect3_, otherRect2_, otherRect3_, intersectsRB, linesAreParallel, intersectionRB);
        
        
        //Test agentRect2->3 against otherRect0->1 (far end of agent rect against front of other truck)
        lineIntersectionXZ(agentRect2_, agentRect3_, otherRect0_, otherRect1_, intersectsBF, linesAreParallel, intersectionBF);
        //Test agentRect2->3 against otherRect0->2 (far end of agent against LHS of other rect)
        lineIntersectionXZ(agentRect2_, agentRect3_, otherRect0_, otherRect2_, intersectsBL, linesAreParallel, intersectionBL);
        //Test agentRect2->3 against otherRect1->3 (far end of agent against RHS of other rect)
        lineIntersectionXZ(agentRect2_, agentRect3_, otherRect1_, otherRect3_, intersectsBR, linesAreParallel, intersectionBR);
        //Test agentRect2->3 against otherRect2->3 (far end of agent against far end of other rect)
        lineIntersectionXZ(agentRect2_, agentRect3_, otherRect2_, otherRect3_, intersectsBB, linesAreParallel, intersectionBB);

 */
        gotNewIntersection = intersectsFF || intersectsFL || intersectsFR || intersectsFB ||
        intersectsLF || intersectsLL || intersectsLR || intersectsLB ||
        intersectsRF || intersectsRL || intersectsRR || intersectsRB ||
        intersectsBF || intersectsBL || intersectsBR || intersectsBB;
        
        if (!moveNeeded)
        {
            frontCollision = leftCollision = rightCollision = rearCollision = false;
        }
        
        if (gotNewIntersection)
        {
            frontCollision = frontCollision || intersectsFF || intersectsFL || intersectsFR || intersectsFB;
            rearCollision = rearCollision || intersectsBF || intersectsBB || intersectsBR || intersectsBL;
            leftCollision = leftCollision || intersectsLF || intersectsLB || intersectsLR || intersectsLL;
            rightCollision = rightCollision || intersectsRF || intersectsRB || intersectsRR || intersectsRL;
        }

        //TODO now check for special case where all four corners of one rectangle are fully enclosed by the other
        
        //TODO calculate how much to move from overlap
        //TODO Also we don't want to turn sharply if the vehicle is already in a bend or swerving
        //TODO Lining up for an overtake before a bend is OK, but not IN the bend
        float amountToMove = 3.0f;//1.4f;//0.8f;//0.4f;//1.2;
        bool moveIsNeeded = moveNeeded != 0.0f;
        bool isSoonestCollision = false;
        
        if ((gotNewIntersection || moveIsNeeded))// && numWaypointsAdded < 100000)
        {
            if (gotNewIntersection)
            {
                collisionOccurs = true;
                /*
                if (timeToCollision < 0.0f || time < timeToCollision)
                {
                    isSoonestCollision = true;
                    timeToCollision = time;
                }
                else
                {
                    //Not interested in collisions AFTER the soonest one we've found
                    break;
                    //TODO One thing worth bearing in mind though, is sometimes the soonest predicted collision
                    //is for a vehicle quite far away, even though there's another vehicle extremely close to
                    //the agent. I've seen it turn to avoid a far off collision and bash into the side of a
                    //nearby vehicle as a result! Checks for immediate vicinity could influence which way we turn
                    //or whether it's better to take no action yet (if collision many seconds away) or just slow down

                }
                
                 */ 
//                if (curTruck && (curTruck == beam) && (step % 6 == 0))
//                    debugCollision(reuseNode, debugCol, agentPosWorld_, std::string("debugMaterialCollision"));
               
                //TODO for debugging draw a sphere at the point(s) of intersection
                //AND draw coloured lines from each vehicle's current position to the projected positions
                //then leave them there for a few seconds or until canAdjustWaypoint is true
                
                //TODO at higher speeds we need a lot more space and also a bit more time to turn, or the vehicle will roll
                //TODO This is even more of an issue if approaching a bend
                if ((agentTotalDist < 1.5f || time < 0.9f) || (agentVelocWorld_.length() > 20.0f && (agentTotalDist < 15.0f || time < 3.0f)) || (agentVelocWorld_.length() > 24.0f && (agentTotalDist < 47.0f || time < 3.6f)))// 20 m/s is about 45 mph. 1.5 and 1.4   2.6 0.8agentDist.length() < 5.0 || time < 1.5f)
                {
                    if (time < 2.8f && steering_delay > 0.25f)
                    {
                        //quicken the steering a bit if necessary for a late swerve
                        steering_delay = 0.25f;
                    }

                    //If a collision is less than 3 metres or less than a second away,
                    //don't try to swerve, just brake and roll off power - BUG this just makes them all stop at the start of the race! We also want to match forward speed of vehicle in front normally
                    if (frontCollision && agentVelocWorld_.length() > 5.0f)//7.0f)//9.0f)//agentVelocity.length() > 10.0f)
                    {
                        //TODO if it's a front collision and the other vehicle isn't moving and is already directly in front, we need to reverse away
                        //if (canAdjustWaypoint)
                        //{
                        //    maxspeed = agentVelocWorld_.length() * 0.6f;//agentVelocity.length() * 0.9f;//0.8f;//0.6f;
                        //    if (acc_power > 0.8f) acc_power = 0.8f;//acc_power = 0.75f;//0.6f;
                        //}
                        //We really need to use the other vehicle's speed in the direction the agent is moving
                        /*
                        if (maxspeed > otherSpeedInAgentDir)// && otherVelocWorld_.length() > 8.0f)//otherVelocity.length() && otherVelocity.length() > 8.0f)//10.0f)
                        {
                            if (otherSpeedInAgentDir > 8.0f)//(otherVelocWorld_.length() > 8.0f)
                            {
                                maxspeed = otherSpeedInAgentDir * 0.6f;//otherVelocity.length() * 0.7f;//0.96f;
                                acc_power = 0.2f;
                            }
                            else
                            {
                                maxspeed = otherSpeedInAgentDir * 0.95f;//otherVelocWorld_.length();
                                if (maxspeed < 2.0f)
                                {
                                    maxspeed = 2.0f;
                                }
                                if (acc_power > 0.35f) acc_power = 0.35f;
                            }
                        }
                         */
                    }
                    if (agentTotalDist < 0.1f || time < 0.2f)//0.3f)//0.6f || time < 0.7f)
                    {
                        //!!!!??***** TODO We keep getting false positives here on the v first iteration of the loop
                        //e.g. time == 0.01f, agentTotalDist 0.1, otherTotalDist 0.01, agentVelocity 10.92
                        //Maybe when we try to move the rectangles into position after rotation it overlaps them???
                        
                        
                        //Definitely no point in adding waypoints; vehicle is too close
                        //TODO actually what can happen is it's a side to side collision, in which case it could be avoided by steering the opposite way
                        
                        /*
                        char debugCol[512];
                        
                        snprintf(debugCol, sizeof(debugCol), "Early Collision. Truck:%d. agentTotalDist:(%f)\nFront collision?(%d)\ntime:(%f) otherTotalDist(%f)\nagentVelocity:(%f)\nagentRectInit:(%f,%f)(%f,%f)\notherRectInit:(%f,%f)(%f,%f)\nagentRectCur:(%f,%f)(%f,%f)\notherRectCur:(%f,%f)(%f,%f)\nagentRectAlign:(%f,%f)(%f,%f)\notherRectAlign:(%f,%f)(%f,%f)\nagentPos Init:(%f,%f) Cur:(%f,%f)\notherPos Init(%f,%f) Cur:(%f,%f)\n\n",
                            beam->trucknum,
                            agentTotalDist,
                            frontCollision,
                            time,
                            otherTotalDist,
                            agentVelocity.length(),
                            agentFrontLeft.x,
                            agentFrontLeft.z,
                            agentRearRight.x,
                            agentRearRight.z,
                            otherFrontLeft.x,
                            otherFrontLeft.z,
                            otherRearRight.x,
                            otherRearRight.z,
                            agentRect0_w.x,
                            agentRect0_w.z,
                            agentRect3_w.x,
                            agentRect3_w.z,
                            otherRect0_w.x,
                            otherRect0_w.z,
                            otherRect3_w.x,
                            otherRect3_w.z,
                            //(long)(&agentRect0_w),
                            //(long)(&agentRect3_w),
                            //(long)(&otherRect0_w),
                            //(long)(&otherRect3_w),
                            agentRect0_.x,
                            agentRect0_.z,
                            agentRect3_.x,
                            agentRect3_.z,
                            otherRect0_.x,
                            otherRect0_.z,
                            otherRect3_.x,
                            otherRect3_.z,                                                    
                            mAgentAbsPosition.x,
                            mAgentAbsPosition.z,
                            agentPosWorld_.x,
                            agentPosWorld_.z,
                            otherPosition.x,
                            otherPosition.z,
                            otherPosWorld_.x,
                            otherPosWorld_.z
                        );
                        */
                        /*
                        if (maxspeed > otherSpeedInAgentDir)// && otherSpeedInAgentDir > 8.0f)//otherVelocWorld_.length() && otherVelocWorld_.length() > 8.0f)//otherVelocity.length() && otherVelocity.length() > 8.0f)//10.0f)
                        {
                            //Brake even harder
                            maxspeed = /*otherVelocWorld_.length()//otherSpeedInAgentDir * 0.4f;
                            acc_power = 0.1f;
                        }
                        */
                        ///*if (canAdjustWaypoint)*/ RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, 
                        //debugCol, "note.png");
                        if (frontCollision)
                            break;
                    }

                }
                //TODO should possibly have a sanity check on relative closing speed here and above
                if (!frontCollision || (agentTotalDist > /*1.5f*/2.0f && time > 2.0f))//1.4f))//(agentTotalDist > 2.0f && time > 1.5f))
                {
                    //reduce power to swerve - TODO needs to accelerate again when the vehicle straightens out - this would be better handled in the steering logic
                    //acc_power = 0.7f;
                    maxspeed = (current_waypoint_id >= 2) ? waypoint_speed[current_waypoint_id-1] : 0.0f;
                    if (!maxspeed)
                        maxspeed = 200.0f;
                    acc_power = (current_waypoint_id >= 2) ? waypoint_power[current_waypoint_id-1] : 1.0f;
                    if (!acc_power)
                        acc_power = 1.0f;
                    //acc_power *= 0.85f;
                }
                
                //If we already planned collision avoidance on a previous cycle, go no further than this as we won't add any new waypoints
                //until the existing avoidance waypoint has been reached
                //if (!canAdjustWaypoint)
                //{
                //    //break; TODO re-enable this if we start using waypoint adjustment again. At the moment I think it's just stopping collision avoidance from working sometimes!
                //}
            }
            if (moveIsNeeded)
            {
                if (fabsf(amountToMove) != fabsf(moveNeeded))
                {
                    amountToMove = moveNeeded;
                }
                else if (moveIsNeeded < 0.0f)
                {
                    amountToMove = -3.0f;//1.4f;//-0.8f;//-0.4f;//-1.2;
                }
            }
            
            //Calculate average current positions of the two vehicles - TODO we actually need the position that's used against waypoints instead
            //Vector3 agentAvgPosition = (agentRect0_ + agentRect1_ + agentRect2_ + agentRect3_) * 0.25f;
            //Vector3 otherAvgPosition = (otherRect0_ + otherRect1_ + otherRect2_ + otherRect3_) * 0.25f;
            
            Vector3 newWaypointPosAlign = agentPosAlign_;//agentAvgPosition;
            //TODO check against track limits, other vehicles, walls to choose side to pass
            //!!! TODO also all this code is assuming the agent's velocity is already perfectly aligned with its current_waypoint
            //!!! If it isn't, these waypoint adjustments will likely drive it off course!
            //TODO !!!!!!!!*** More to the point, we can't move THROUGH the vehicle if it's already overlapping by this point!
            if (moveIsNeeded || (leftCollision && !rightCollision) || (!rightCollision && nextAgentWaypointAlign.x > agentPosAlign_.x))//agentAvgPosition.x)
            {
                //Move a positive x amount, or in the direction saved in moveNeeded
                //newWaypointPosAlign.x += amountToMove;
            }
            else
            {
                //Move a negative x amount (amountToMove was always positive because moveIsNeeded is false)
                //newWaypointPosAlign.x -= amountToMove;
                amountToMove = -amountToMove;
            }
            //TODO we're checking time_since_last_collision_avoid to make sure, when collision_avoid_time is very low,
            //corrective steering doesn't keep being immediately reapplied, causing a severe swerve - because a low collision_avoid_time
            //is supposed to REDUCE the amount of swerve. However, we should probably be checking against the time collision avoidance
            //steering last STARTED, not when it ended OR split the delays into collision_avoid_time and collision_avoid_steer_time
            if (collision_avoid_time <= 0.03f && time_since_last_collision_avoid >= 0.2f)//0.4f)//collision_avoid_time <= 0.1f)//0.2f)
            {
                if (collision_avoid_time > 0.0f && (/*collision_avoid_target.x == 0.0f ||*/ (collision_avoid_target.x < 0.0f && amountToMove > 0.0f) || 
                (collision_avoid_target.x > 0.0f && amountToMove < 0.0f)))
                {
                    //To try and keep things smooth, just stop moving the steering if avoidance direction has changed prematurely
                    collision_avoid_target = Vector3::ZERO;
                }
                else
                {
                    //TODO we possibly ideally want track aligned positions here - especially if it's some distance ahead.
                    collision_avoid_target = agentPosAlign;//Vector3(amountToMove, 0.0f, 0.0f);//TODO worth knowing how far down the track to stop evasive action?
                    collision_avoid_target.x += amountToMove;
                    collision_avoid_target = matWorldAlign * collision_avoid_target;
                    //Best thing to do rather than timing how long we steer for may be to actually make collision_avoid_target be in world coordiantes
                    //At some distance up the road, like a pseudo waypoint
                    //so the car keeps steering until its moved over by amountToMove, then stays there until its passed that point
                }
                //scale how long we steer for by how far down the road the collision is!
                if (time >= 0.4f)//0.5f)
                {
                    collision_avoid_time = /*0.2f*//*0.10f*//*0.09f*//*0.06f*//*0.05f*//*0.04f*/0.3f / time;
                    if (collision_avoid_time > 0.9f)//0.6f)
                        collision_avoid_time = 0.9f;//0.6f;
                    if (agentVelocity.length() > 20.0f)
                    {
                        //Don't steer as hard if we're going over 45 mph
                        collision_avoid_time *= 0.75f;//0.8f;
                    }
                    if (collision_avoid_time > time)//0.6f)
                        collision_avoid_time = time;//0.6f;
                }
                else
                {
                    collision_avoid_time = 0.2f;//0.12f;//0.15f;//0.2f;//Time to steer away in seconds
                }
                if (collision_avoid_target != Vector3::ZERO && agentVelocity.length() > 15.0f)
                {
                    acc_power = (current_waypoint_id >= 2) ? waypoint_power[current_waypoint_id-1] : 1.0f;
                    if (!acc_power)
                        acc_power = 1.0f;
                        acc_power *= 0.7f;
                }
                break;
                //newWaypointPosAlign.x += amountToMove;
            }
            
        }
        #endif
    }
    
    //Now debug any lane false positives:
    if (debugFalsePositives.length() > 0)
    {
        LOG(debugFalsePositives);
        LOG(debugVehiclePath);
        for (int l = 0; l <= 2; l++)
        {
            CollisionAvoidance_GridQuad *pLaneQuad = (l == 0)? debugLastFalsePosQuadLane0 :
                                                     (l == 1)? debugLastFalsePosQuadLane1 :
                                                     debugLastFalsePosQuadLane2;
            if (!pLaneQuad)
                continue;
            LOG(Ogre::String("Coordinates for Lane ") + TOSTRING(l) + ":");
            Vector3 frontLeft, frontRight, rearRight, rearLeft;
            pLaneQuad->GetCoordinatesCenteredOnWaypoints(waypoints, frontLeft, frontRight, rearRight, rearLeft);
            LOG(TOSTRING(frontLeft.x) + "," + TOSTRING(frontLeft.z) + ", " + TOSTRING(frontRight.x) + "," + TOSTRING(frontRight.z));
            LOG(TOSTRING(rearLeft.x) + "," + TOSTRING(rearLeft.z) + ", " + TOSTRING(rearRight.x) + "," + TOSTRING(rearRight.z));
            for (int i = 0; i < 10; i++)
            {
                pLaneQuad = pLaneQuad->GetNeighbourRear();
                if (!pLaneQuad)
                    break;
                    
                pLaneQuad->GetCoordinatesCenteredOnWaypoints(waypoints, frontLeft, frontRight, rearRight, rearLeft);
                LOG(TOSTRING(rearLeft.x) + "," + TOSTRING(rearLeft.z) + ", " + TOSTRING(rearRight.x) + "," + TOSTRING(rearRight.z));
            }
        }
        
    }
    

    #if 0
    if (collisionOccurs) //TODO where does this get set false???? don't want next truck overriding max speed
    {
        //best to break the loop
        //TODO NOPE instead of breaking here, we need to check ALL possible collisions and avoid the one
        //that occurs earliest in TIME
        //break;
    }
    else
    {
        maxspeed = waypoint_speed[current_waypoint_id];
        if (!maxspeed)
            maxspeed = 200.0f;
        acc_power = waypoint_power[current_waypoint_id];
        if (!acc_power)
            acc_power = 1.0f;
    }
    #endif
    
    //*** END NEW CODE - cosmic vole March 3 2017
    
    
    
}

void VehicleAI::CollisionAvoidance_GetVehicleApproachAngles(Ogre::Vector3 agentPos, Ogre::Vector3 otherPos, Ogre::Vector3 agentDir, Ogre::Vector3 otherDir, Ogre::Degree& betweenDirs, Ogre::Degree& bearing, bool& isBehind, bool& isHeadOn)
{
    //First check if the 2 vehicles are pointing (or moving if velocities passed in) in the same direction 
    //!! TODO we also need to consider that the vehicle could be reversing! Not handled yet. Or sliding forwards upside down!
    //This finds the angle measured anticlockwise from mAgentHeading to otherTargetHeading
    float approach = Radian(atan2(otherDir.z - agentDir.z, otherDir.x - agentDir.x)).valueDegrees();
    if (approach < 0.0f)
        approach = 360.0f + approach;
    betweenDirs = Degree(approach);
    //An angle of 180 degrees be either when the vehicles are back to back or head on <--- --->  or ---> <---; we have to check the bearing of the other vehicle to tell
    bool isSameDirection = !(approach > 90.0f && approach < 270.0f);//(otherAvgFront).squaredDistance((agentRect0 + agentRect1) * 0.5f) > (otherAvgBack).squaredDistance((agentRect0 + agentRect1) * 0.5f);
    bool isFromRight = approach > 0.0f && approach < 180.0f;//other vehicle is pointing towards the LHS of the agent's path
    bool isFromLeft = approach > 180.0f && approach < 360.0f;//other vehicle is pointing towards the RHS of the agent's path

    float distBetween = agentPos.distance(otherPos);
    Vector3 otherFromAgent = otherPos - agentPos;
    otherFromAgent.normalise();

    //Now the bearing of other from agent. Find what direction the other vehicle is situated from the agent vehicle's point of view
    float agentDirAngle = Radian(atan2(agentDir.dotProduct(Vector3::UNIT_X), (agentDir).dotProduct(Vector3::UNIT_Z))).valueDegrees();
    float angleToOther = Radian(atan2(otherFromAgent.dotProduct(Vector3::UNIT_X), (otherFromAgent).dotProduct(Vector3::UNIT_Z))).valueDegrees();
    if (agentDirAngle < 0.0f)
        agentDirAngle = 360.0f + agentDirAngle;
    if (angleToOther < 0.0f)
        angleToOther = 360.0f + angleToOther;
    float angle = angleToOther - agentDirAngle;
    if (angle < 0.0f)
        angle = 360.0f + angle;
    bearing = Degree(angle);
    
    if (distBetween < 2.5f)//3.0f)//2.5f)//2.0f)
    {
        //LOG("agentDirAngle: " + TOSTRING(agentDirAngle) + " angleToOther: " + TOSTRING(angleToOther) + "\n" +
    //"angle: " + TOSTRING(angle) + " agentDir: (" + TOSTRING(agentDir.x) + ", " + TOSTRING(agentDir.z) + ") otherFromAgent: (" + TOSTRING(otherFromAgent.x) + ", " + TOSTRING(otherFromAgent.z) + ").\n");
        if (fabsf(angle) > 30.0f && fabsf(angle) < 150.0f)//40.0f && fabsf(angle) < 130.0f)
        {
            //TODO I'm not yet sure which is left vs right (are angles clockwise or anti???)
            bool hasCarOnRight = true;
            //LOG("Has car on right.\n");
        }
        else if (fabsf(angle) > 210.0f && fabsf(angle) < 330.0f)//220.0f && fabsf(angle) < 320.0f)
        {
            bool hasCarOnLeft = true;
            //LOG("Has car on left.\n");
        }
    }
    if (distBetween < 4.0f && (fabsf(angle) <= 30.0f || fabsf(angle) >= 330.0f))
    {
        //LOG("Has car in front.\n");
        //if (otherVelocity.length() < 5.0f)//2.0f)
        {
            bool hasStationaryObstacleInFront = true;
        }
    }
    
    if (fabsf(angle) > 90.0f && fabsf(angle) < 270.0f)
    {
        //The other vehicle is positioned behind the agent vehicle
        isBehind = true;
        isHeadOn = false;
        //if (distBetween < 10.0f) LOG("Has car behind.\n");
        //Other truck is behind us NOTE - we still need to re-examine whether both trucks are pointing in the same direction
        /*
        if (beam == BeamFactory::getSingleton().getCurrentTruck()) RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, 
            //"otherPos is behind agentPos. agentPosAlign: (" + TOSTRING(agentPosAlign.x) + "," + TOSTRING(agentPosAlign.z) +
            //") otherPosAlign: (" + TOSTRING(otherPosAlign.x) + "," + TOSTRING(otherPosAlign.z) + ")\n" +
            "otherPos is behind agentPos. angle: " + TOSTRING(angle) +
            " agentPosWrld: (" + TOSTRING(mAgentAbsPosition.x) + "," + TOSTRING(mAgentAbsPosition.z) +
            ")\n otherPosWrld: (" + TOSTRING(otherPosition.x) + "," + TOSTRING(otherPosition.z) + ") " +
            "agenttruckNum: " + TOSTRING(beam->trucknum) + " othertrucknum: " + TOSTRING(otherTruck->trucknum) + "\n", "note.png");
             */
             
             //TODO only continue if we aren't in the process of overtaking this vehicle!
                //continue;                                
    }
    else
    {
        //The other vehicle is positioned in front of the agent vehicle
        isHeadOn = !isSameDirection;
        isBehind = false;
        /*
        if (beam == BeamFactory::getSingleton().getCurrentTruck()) RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, 
            //"otherPos is behind agentPos. agentPosAlign: (" + TOSTRING(agentPosAlign.x) + "," + TOSTRING(agentPosAlign.z) +
            //") otherPosAlign: (" + TOSTRING(otherPosAlign.x) + "," + TOSTRING(otherPosAlign.z) + ")\n" +
            "otherPos is ahead of agentPos. angle: " + TOSTRING(angle) +
            " agentPosWrld: (" + TOSTRING(mAgentAbsPosition.x) + "," + TOSTRING(mAgentAbsPosition.z) +
            ")\n otherPosWrld: (" + TOSTRING(otherPosition.x) + "," + TOSTRING(otherPosition.z) + ") " +
            "agenttruckNum: " + TOSTRING(beam->trucknum) + " othertrucknum: " + TOSTRING(otherTruck->trucknum) + "\n", "note.png");
        */
    }
    //In the case of a head-on - if they just both move left, we should be all good. Otherwise we need some deterministic randomness or comms.

}

//cosmic vole May 31 2017
void VehicleAI::CollisionAvoidance_ClearVehiclePaths(Ogre::String raceID = Ogre::StringUtil::BLANK, bool pastOnly = false)
{
    //TODO May 31 2017 - the waypoints and waypoint names for a given raceID need to be statically accessible in a Race object really
    if (Ogre::StringUtil::match(raceID, Ogre::StringUtil::BLANK))
    {
        //If no raceID is specified, we will clear the collision avoidance grids for ALL races - cosmic vole June 21 2017
        std::map<Ogre::String, std::vector<CollisionAvoidance_GridQuad*>>::iterator iter;
        
        for (iter = raceGridQuads.begin(); iter != raceGridQuads.end(); ++iter)
        {
            std::vector<CollisionAvoidance_GridQuad*>& gridQuads = iter->second;
            for (int j=0; j<gridQuads.size(); j++)
            {
                CollisionAvoidance_GridQuad *gridQuad = gridQuads[j];
                if (gridQuad)
                    gridQuad->Clear();
            }            
        }        
    }
    else
    {
        for (int i=1; i<=waypoint_names.size(); i++) //i=0; i<waypoint_names.size(); i++)
        {
            Ogre::String waypointName = waypoint_names[i];
            Ogre::String raceWaypointKey = raceID + "|||" + waypointName;
            std::map<Ogre::String, std::vector<CollisionAvoidance_GridQuad*>>::iterator iter;
            iter = raceGridQuads.find(raceWaypointKey);
            if (iter != raceGridQuads.end())
            {
                std::vector<CollisionAvoidance_GridQuad*>& gridQuads = iter->second;//raceGridQuads[raceWaypointKey];        
                for (int j=0; j<gridQuads.size(); j++)
                {
                    CollisionAvoidance_GridQuad *gridQuad = gridQuads[j];
                    if (gridQuad)
                        gridQuad->Clear();
                }
            }
        }
    }
    
    Beam** trucks = m_sim_controller->GetBeamFactory()->getTrucks();
    int numTrucks = m_sim_controller->GetBeamFactory()->getTruckCount();
    
    for (int t = 0; t < numTrucks; t++)
    {
        if (!trucks[t])
            continue;
        if (trucks[t]->vehicle_ai)// && (trucks[t]->vehicle_ai->IsActive()))
        {
            trucks[t]->vehicle_ai->predictedVehiclePath.clear();//cosmic vole June 25 2017
        }
    }
}

//cosmic vole August 30 2017
std::vector<CollisionAvoidance_GridQuad*>* VehicleAI::CollisionAvoidance_GetGridQuads(Ogre::String raceID, int waypointID)
{
    //TODO Factor the VehicleAI out here. The Collision Avoidance Grid needs to go in a Race object.
    VehicleAI *truckAI = this;
    int nextWaypointOnFirstLap;
    if (truckAI->num_waypoints_in_lap > 0 && truckAI->num_waypoints_in_lap <= truckAI->free_waypoints)
    {
        nextWaypointOnFirstLap = ((waypointID - 1) % truckAI->num_waypoints_in_lap) + 1;
        //lastWaypointOnFirstLap = nextWaypointOnFirstLap - 1;
        //Wrap around - this will work even if it's a single lap race on a circuit. It just won't work on a drag strip, so set num_waypoints_in_lap to 0 for that. cosmic vole June 25 2017
        //if (lastWaypointOnFirstLap < 1)
        //    lastWaypointOnFirstLap += truckAI->num_waypoints_in_lap;            
    }
    else
    {
        nextWaypointOnFirstLap = waypointID;
        //lastWaypointOnFirstLap = waypointID - 1;
    }
    
    Ogre::String raceWaypointKey("");
    
    if (nextWaypointOnFirstLap > 0)//BUG / Problem waypoint 0 is not supported - see GenerateLanes() - cosmic vole June 25 2017 = 0)
    {
        Ogre::String nextAgentWaypointName = truckAI->waypoint_names[nextWaypointOnFirstLap];
        raceWaypointKey = raceID + "|||" + nextAgentWaypointName;
        //endLaneIndex = raceWaypointToLanePointIndex[raceIDStr + "|||" + nextAgentWaypointName];
        //if (nextAgentWaypointID >= 1)
        //{
        //    Ogre::String lastAgentWaypointName = waypoint_names[lastWaypointOnFirstLap];//nextAgentWaypointID-1];
        //    startLaneIndex = raceWaypointToLanePointIndex[raceIDStr + "|||" + lastAgentWaypointName];//!< Maps raceID + "|||" + waypoint name to an index into that race's entry in raceLanePoints. cosmic vole May 18 2017
        //}        
    }
    else
    {
        LOG("ERROR in CollisionAvoidance_GetGridQuads() Waypoint: " + TOSTRING(waypointID) + " skipped because nextWaypointOnFirstLap = " + TOSTRING(nextWaypointOnFirstLap) + ".");
        return nullptr;
    }
    
    std::map<Ogre::String, std::vector<CollisionAvoidance_GridQuad*>>::iterator iter;
    iter = raceGridQuads.find(raceWaypointKey);
    //OLD - Lazy load the race track's collision avoidance grid of lanes if it hasn't already been generated - cosmic vole June 24 2017 NOTE on some tracks this may cause lag!
    if (iter == raceGridQuads.end())
    {
        //**** TODO This is wrong, needs to be static / held in a Race object. cosmic vole June 27 2017
        CollisionAvoidance_GenerateLanes(-1, -1, truckAI->waypoints, truckAI->road_edge_points_left, truckAI->road_edge_points_right);
        iter = raceGridQuads.find(raceWaypointKey);
        if (iter == raceGridQuads.end())
        {
            LOG("ERROR in CollisionAvoidance_GetGridQuads() Waypoint: " + TOSTRING(waypointID) + " skipped because grid quads not found for: '" + raceWaypointKey + "'.");
            return nullptr;
        }
    }
    std::vector<CollisionAvoidance_GridQuad*>& gridQuads = iter->second;
    return &gridQuads;
}

//cosmic vole August 30 2017
int VehicleAI::CollisionAvoidance_GetLaneNumber(int waypoint, bool useAdjustedWaypoints)
{
    if (waypoint <= 0 || waypoint > free_waypoints)
    {
        LOG("In CollisionAvoidance_GetLaneNumber() waypoint index out of range: " + TOSTRING(waypoint) + ".");
        return -1;
    }
    std::vector<CollisionAvoidance_GridQuad*>* pGridQuads = CollisionAvoidance_GetGridQuads(TOSTRING(raceID), waypoint);
    if (!pGridQuads || pGridQuads->size() < 1)
    {
        LOG("In CollisionAvoidance_GetLaneNumber() no grid quads found for waypoint: " + TOSTRING(waypoint) + ".");
        return -1;
    }
    CollisionAvoidance_GridQuad *pQuad = (*pGridQuads)[0];
    Vector3 wpt;
    if (useAdjustedWaypoints)
        wpt = adjusted_waypoints[waypoint];
    else
        wpt = waypoints[waypoint];
    if (pQuad->ContainsPoint(wpt))
        return pQuad->GetLaneNumber();
    CollisionAvoidance_GridQuad *pLeft = pQuad->GetNeighbourLeft();
    while (pLeft)
    {
        if (pLeft->ContainsPoint(wpt))
            return pLeft->GetLaneNumber();
        pLeft = pLeft->GetNeighbourLeft();
    }
    CollisionAvoidance_GridQuad *pRight = pQuad->GetNeighbourRight();
    while (pRight)
    {
        if (pRight->ContainsPoint(wpt))
            return pRight->GetLaneNumber();
        pRight = pRight->GetNeighbourRight();
    }
    LOG("In CollisionAvoidance_GetLaneNumber() no quad found containing waypoint: " + TOSTRING(wpt) + ".");
    return -1;
}

struct Vector3Compare
{
    bool operator()(const Vector3& v0, const Vector3& v1) const
    {
        if (v0.x != v1.x)
            return (v0.x < v1.x);
        else if (v0.y != v1.y)
            return (v0.y < v1.y);
        else if (v0.z != v1.z)
            return v0.z < v1.z;
        else
            return false;
    }
};

void VehicleAI::CollisionAvoidance_DebugVehiclePath(int truckNum)
{
    //#if 0                
    //Debug projected positions of the vehicles - cosmic vole March 10 2017
    Beam *curTruck;
    if (truckNum < 0)
    {
        curTruck = m_sim_controller->GetBeamFactory()->getCurrentTruck();
    }
    else
    {
        curTruck = m_sim_controller->GetBeamFactory()->getTruck(truckNum);
    }
    if (!curTruck)
        return;
    
    static std::map<Ogre::Vector3, SceneNode*, Vector3Compare> debugCol;
    VehicleAI *truckAI = curTruck->getVehicleAI();
    if (!truckAI)
        return;
    std::vector<PredictedTruckPosition>& path = truckAI->predictedVehiclePath;
    //static std::vector<SceneNode*> debugCol;// = std::vector<SceneNode*>();
    //Hide all the existing markers first TODO will be better to keep them in a map and just change their color
    /*
    for (int i = 0; i < debugCol.size(); i++)
    {
        SceneNode *node = debugCol.at(i);
        Entity *entity = (Entity *)node->getAttachedObject(0);
        node->setPosition(Vector3::ZERO);
    }
    */
    
    
    for (int i = 0; i < path.size(); i++)
    {
        CollisionAvoidance_GridQuad *gridQuad = path[i].pGridQuad;
        if (!gridQuad)// || drawnQuad[gridQuad])
            continue;

        static MaterialPtr debugMaterialAgent;
        static MaterialPtr debugMaterialOther;
        static MaterialPtr debugMaterialCollision;
        static MaterialPtr debugMaterialPath;
        static bool madeMaterials = false;
        //SceneNode *nodeAgent, *nodeOther;
        //Entity *entityAgent, *entityOther;
        static int reuseNode = -1;

        int count = debugCol.size();
        if (count>20000)//3000)//12000)
        {
            if (reuseNode < 0)
            {
                reuseNode = 0;
            }
            else if (reuseNode >= count-8)
            {
                reuseNode = 0;
            }
            //int toDel = count - 20000;
            /* Argh this deletion stuff currently seg faults - WHY?! cosmic vole
            for (int k=0;k<toDel; k++)
            {
                SceneNode * node = debugCol.at(k);//debugCol[k];
                //gEnv->sceneManager->destroyMovableObject(node->getAttachedObject(0)); Seg faults!
                node->detachObject((unsigned short)0);
                //gEnv->sceneManager->destroySceneNode(node);
                //debugCol.erase(debugCol.begin());
            }
            */
            
        }
        else
        {
            reuseNode = -1;
        }
        
        if (!madeMaterials)
        {
            madeMaterials = true;
            debugMaterialAgent = MaterialManager::getSingleton().create("debugMaterialAgent2", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
            debugMaterialAgent->setSelfIllumination(0.3f,0.0f,0.6f);
            debugMaterialAgent->setAmbient(0.3f,0.0f,0.6f);
            debugMaterialAgent->setDiffuse(0.3f,0.0f,0.6f,0.1f);
            debugMaterialOther = MaterialManager::getSingleton().create("debugMaterialOther2", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
            debugMaterialOther->setSelfIllumination(0.7f,0.5f,0.0f);
            debugMaterialOther->setAmbient(0.7f,0.5f,0.0f);
            debugMaterialOther->setDiffuse(0.7f,0.5f,0.0f,0.9f);
            debugMaterialCollision = MaterialManager::getSingleton().create("debugMaterialCollision2", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
            debugMaterialCollision->setSelfIllumination(0.7f,0.0f,0.0f);
            debugMaterialCollision->setAmbient(0.7f,0.0f,0.0f);
            debugMaterialCollision->setDiffuse(0.7f,0.0f,0.0f,0.9f);
            //debugMaterialPath = MaterialManager::getSingleton().create("debugMaterialPath", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
            //debugMaterialPath->setSelfIllumination(0.0f,0.6f,0.1f);
            //debugMaterialPath->setAmbient(0.0f,0.6f,0.1f);
            //debugMaterialPath->setDiffuse(0.0f,0.6f,0.1f,0.9f);
        }
        Vector3 coord0, coord1, coord2, coord3;
        Vector3 debugRect0;// = agentRect0_w;
        Vector3 debugRect1;// = agentRect1_w;
        Vector3 debugRect2;// = agentRect2_w;
        Vector3 debugRect3;// = agentRect3_w;
        
        //debugCollision(reuseNode, debugCol, debugRect0, std::string("debugMaterialPath"));
        //debugCollision(reuseNode, debugCol, debugRect1, std::string("debugMaterialPath"));
        //debugCollision(reuseNode, debugCol, debugRect2, std::string("debugMaterialPath"));
        //debugCollision(reuseNode, debugCol, debugRect3, std::string("debugMaterialPath"));
        
        
        int numQuadsInRow = 1;
        CollisionAvoidance_GridQuad *pRight = gridQuad->GetNeighbourRight();
        while (pRight)
        {
            numQuadsInRow++;
            //if (!drawnQuad[pRight])
            {
                //drawnQuad[pRight] = true;
                pRight->GetCoordinates(coord0, coord1, coord2, coord3);
                debugRect0 = coord0;
                debugRect1 = coord1;
                debugRect2 = coord2;
                debugRect3 = coord3;
                debugRect0.y = debugRect1.y = debugRect2.y = debugRect3.y = /*mAgentAbsPosition.y*/curTruck->getPosition().y + 0.2f;
                if (pRight->HasCollisionWith(curTruck->trucknum))
                {
                    debugCol[coord0] = debugCollision(debugCol[coord0], debugRect0, std::string("debugMaterialCollision2"));
                    debugCol[coord1] = debugCollision(debugCol[coord1], debugRect1, std::string("debugMaterialCollision2"));
                    debugCol[coord2] = debugCollision(debugCol[coord2], debugRect2, std::string("debugMaterialCollision2"));
                    debugCol[coord3] = debugCollision(debugCol[coord3], debugRect3, std::string("debugMaterialCollision2"));

                }
                else if (pRight->Contains(curTruck->trucknum))
                {
                    debugCol[coord0] = debugCollision(debugCol[coord0], debugRect0, std::string("debugMaterialAgent2"));
                    debugCol[coord1] = debugCollision(debugCol[coord1], debugRect1, std::string("debugMaterialAgent2"));
                    debugCol[coord2] = debugCollision(debugCol[coord2], debugRect2, std::string("debugMaterialAgent2"));
                    debugCol[coord3] = debugCollision(debugCol[coord3], debugRect3, std::string("debugMaterialAgent2"));                    
                }
                else
                {
                    //debugCol[coord0] = debugCollision(debugCol[coord0], debugRect0, std::string("debugMaterialOther2"));
                    debugCol[coord1] = debugCollision(debugCol[coord1], debugRect1, std::string("debugMaterialOther2"));
                    debugCol[coord2] = debugCollision(debugCol[coord2], debugRect2, std::string("debugMaterialOther2"));
                    //debugCol[coord3] = debugCollision(debugCol[coord3], debugRect3, std::string("debugMaterialOther2"));                        
                }                
            }
            pRight = pRight->GetNeighbourRight();
        }
        
        CollisionAvoidance_GridQuad *pLeft = gridQuad->GetNeighbourLeft();
        while (pLeft)
        {
            numQuadsInRow++;
            //if (!drawnQuad[pLeft])
            {
                //drawnQuad[pLeft] = true;
                pLeft->GetCoordinates(coord0, coord1, coord2, coord3);
                debugRect0 = coord0;
                debugRect1 = coord1;
                debugRect2 = coord2;
                debugRect3 = coord3;
                debugRect0.y = debugRect1.y = debugRect2.y = debugRect3.y = /*mAgentAbsPosition.y*/curTruck->getPosition().y + 0.2f;
                if (pLeft->HasCollisionWith(curTruck->trucknum))
                {
                    debugCol[coord0] = debugCollision(debugCol[coord0], debugRect0, std::string("debugMaterialCollision2"));
                    debugCol[coord1] = debugCollision(debugCol[coord1], debugRect1, std::string("debugMaterialCollision2"));
                    debugCol[coord2] = debugCollision(debugCol[coord2], debugRect2, std::string("debugMaterialCollision2"));
                    debugCol[coord3] = debugCollision(debugCol[coord3], debugRect3, std::string("debugMaterialCollision2"));

                }
                else if (pLeft->Contains(curTruck->trucknum))
                {
                    debugCol[coord0] = debugCollision(debugCol[coord0], debugRect0, std::string("debugMaterialAgent2"));
                    debugCol[coord1] = debugCollision(debugCol[coord1], debugRect1, std::string("debugMaterialAgent2"));
                    debugCol[coord2] = debugCollision(debugCol[coord2], debugRect2, std::string("debugMaterialAgent2"));
                    debugCol[coord3] = debugCollision(debugCol[coord3], debugRect3, std::string("debugMaterialAgent2"));                                        
                }
                else
                {
                    debugCol[coord0] = debugCollision(debugCol[coord0], debugRect0, std::string("debugMaterialOther2"));
                    //debugCol[coord1] = debugCollision(debugCol[coord1], debugRect1, std::string("debugMaterialOther2"));
                    //debugCol[coord2] = debugCollision(debugCol[coord2], debugRect2, std::string("debugMaterialOther2"));
                    debugCol[coord3] = debugCollision(debugCol[coord3], debugRect3, std::string("debugMaterialOther2"));                        
                }                
            }
            pLeft = pLeft->GetNeighbourLeft();
        }
        
        //if (!drawnQuad[gridQuad])
        {
            gridQuad->GetCoordinates(coord0, coord1, coord2, coord3);
            debugRect0 = coord0;
            debugRect1 = coord1;
            debugRect2 = coord2;
            debugRect3 = coord3;
            debugRect0.y = debugRect1.y = debugRect2.y = debugRect3.y = /*mAgentAbsPosition.y*/curTruck->getPosition().y + 0.2f;

            if (gridQuad->HasCollisionWith(curTruck->trucknum))
            {
                debugCol[coord0] = debugCollision(debugCol[coord0], debugRect0, std::string("debugMaterialCollision2"));
                debugCol[coord1] = debugCollision(debugCol[coord1], debugRect1, std::string("debugMaterialCollision2"));
                debugCol[coord2] = debugCollision(debugCol[coord2], debugRect2, std::string("debugMaterialCollision2"));
                debugCol[coord3] = debugCollision(debugCol[coord3], debugRect3, std::string("debugMaterialCollision2"));

            }
            else
            {
                debugCol[coord0] = debugCollision(debugCol[coord0], debugRect0, std::string("debugMaterialAgent2"));
                debugCol[coord1] = debugCollision(debugCol[coord1], debugRect1, std::string("debugMaterialAgent2"));
                debugCol[coord2] = debugCollision(debugCol[coord2], debugRect2, std::string("debugMaterialAgent2"));
                debugCol[coord3] = debugCollision(debugCol[coord3], debugRect3, std::string("debugMaterialAgent2"));
            }
        }
        
        
        //drawnQuad[gridQuad] = true;
        if (numQuadsInRow < 3)
        {
            LOG("Less quads in row: " + TOSTRING(numQuadsInRow));
            //return;
        }
    }
    //End Debug projected positions
    //#endif
}

//cosmic vole July 2 2017
void VehicleAI::CollisionAvoidance_ShowDebugMsg(bool isLaneOccupied, bool hasCollision, bool collisionWithTruckInFront, bool canAvoidTruck, int currentLane, int targetLane, Ogre::UTFString& info, float dt)
{
    //At the moment this just uses the same "flash message" feature that GameScript has.
    //We could add our own debug text overlay to bin/resources/overlays/various.overlay cosmic vole July 2 2017
    static UTFString lastDebugInfo = L"";
    static UTFString lastLogged = L"";
    static UTFString lastDebugInfoLogged = L"";
    static UTFString lastLaneOccLogged = L"";
    static int lastLaneLogged = -1;
    static float laneOccTimeout = -1.0f;
    if (info == L"" && lastDebugInfo != L"Truck path is empty!")
    {
        info = lastDebugInfo;
    }
    else
    {
        lastDebugInfo = info;
    }
    Ogre::String sLaneOcc = (isLaneOccupied) ? "*" : " ";
    Ogre::String sCarFrnt = (hasCollision)?((collisionWithTruckInFront) ? "*" : "x"):" ";
    Ogre::String sCanAvoid = (hasCollision)?((canAvoidTruck)? "*" : "x"):" ";
    int laneDelta = targetLane - currentLane;
    Ogre::String sTargLane = 
        (laneDelta > 1) ? ">>" : 
        (laneDelta > 0) ? "> " :
        (laneDelta < -1) ? "<<" :
        (laneDelta < 0) ? "< " :
        "__";
    if (targetLane < 0)
        sTargLane = "  ";
    //Enable a timeout on the "lane occupied" symbol, as it rarely seems to show up at the moment. July 4 2017
    Ogre::String sDispLaneOcc = sLaneOcc;
    if (isLaneOccupied)
    {
        laneOccTimeout = 0.0f;
    }
    else if (laneOccTimeout > 0.0f)
    {
        laneOccTimeout += dt;
        if (laneOccTimeout > 1700.0f)
        {
            laneOccTimeout = -1.0f;
        }
        else
        {
            sDispLaneOcc = "*";
        }
    }
#ifdef USE_MYGUI
    wchar_t txt[512] = L"";
    UTFString fmt = L"LaneOcc: %ls CarInFrnt: %ls CanAvoid: %ls CurLane: %i TargLane: %ls\nInfo: %ls                          ";
    swprintf(txt, 512, fmt.asWStr_c_str(), UTFString(sDispLaneOcc).asWStr_c_str(), UTFString(sCarFrnt).asWStr_c_str(), UTFString(sCanAvoid).asWStr_c_str(), currentLane, UTFString(sTargLane).asWStr_c_str(), UTFString(info).asWStr_c_str());
    UTFString sTxt = UTFString(txt);
    if (sTxt != lastLogged)
    {
        RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, txt, "script_code_red.png");
        RoR::App::GetGuiManager()->PushNotification("", txt);
    }    
    if ((hasCollision || info != lastDebugInfoLogged || currentLane != lastLaneLogged || lastLaneOccLogged != sLaneOcc) && sTxt != lastLogged)
    {
        lastDebugInfoLogged = info;
        lastLogged = sTxt;
        lastLaneLogged = currentLane;
        lastLaneOccLogged = sLaneOcc;
        LOG(sTxt);
    }
#endif // USE_MYGUI    
}

void VehicleAI::updateWaypoint()
{
    if (beam == m_sim_controller->GetBeamFactory()->getCurrentTruck())
    {
        RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, "Reached waypoint: " + waypoint_names[current_waypoint_id] + " Truck: " + TOSTRING(beam->trucknum) + " Coords: " + TOSTRING(current_waypoint), "note.png");
        LOG("Reached waypoint: " + waypoint_names[current_waypoint_id] + " Truck: " + TOSTRING(beam->trucknum) + " Coords: " + TOSTRING(current_waypoint));
        LOG("Waypoint speed: " + TOSTRING(adjusted_waypoint_speed[current_waypoint_id-1]));
        LOG("Truck speed: " + TOSTRING(beam->getVelocity().length() * 3.6f));
    }
    
    lastWaypointNavigated = current_waypoint; // We need this as waypoints can now be skipped. cosmic vole April 12 2017
    Vector3 position = beam->getPosition();
    vehiclePositionsAtWaypoints[current_waypoint_id] = position; // Record exact position for debugging. cosmic vole August 16 2017
    
    //See if it's time to re-enable collision avoidance - cosmic vole March 4 2017
    if (collision_waypoint_id.compare(waypoint_names[current_waypoint_id]) == 0)
    {
        collision_waypoint_id = "";
    }
    int event = waypoint_events[current_waypoint_id];
    if (event)
    {
        switch (event)
        {
        case AI_LIGHTSTOGGLE:
            beam->lightsToggle();
            break;
        case AI_BEACONSTOGGLE:
            beam->beaconsToggle();
            break;
        default:
            break;
        }
    }

    float speed = adjusted_waypoint_speed[current_waypoint_id-1];
    if (speed)
    {
        maxspeed = speed;
    }

    float power = waypoint_power[current_waypoint_id-1];
    if (power)
        acc_power = power;


    Vector3 agentPos = beam->getPosition();
    float velocity = beam->getVelocity().length();
   
    //We now look ahead an extra waypoint if the vehicle is moving fast
    //and the waypoints have been placed close enough together - cosmic vole March 13 2017
    int skipped = -1;
    float speed2 = maxspeed;
    float power2 = acc_power;
    do
    {
        current_waypoint_id++;
        skipped++;
        if (current_waypoint_id > free_waypoints)
        {
            current_waypoint_id = 0;
            is_enabled = false;
            beam->parkingbrakeToggle();
            return;
        }
        if (skipped > 0)
        {
            float speed3 = adjusted_waypoint_speed[current_waypoint_id-1];
            if (speed3)
                speed2 += speed3;
            else
                speed2 += speed2;

            float power3 = waypoint_power[current_waypoint_id-1];
            if (power3)
                power2 += power3;
            else
                power2 += power2;
            vehiclePositionsAtWaypoints[current_waypoint_id-1] = Vector3::ZERO;
        }
        current_waypoint = adjusted_waypoints[current_waypoint_id];//cosmic vole changed June 25 2017   waypoints[current_waypoint_id];
        
    }
    while (((velocity >= 20.0f/*15.0f*/ || (/*time_since_last_collision_avoid >= 5.0f &&*/ time_since_last_collision_avoid < /*8.0f*/3.0f)) && agentPos.distance(current_waypoint) < 20.0f/*25.0f*/) ||
 ((velocity >= 25.0f/*20.0f*/ /*|| time_since_last_collision_avoid < *1.9f 2.1f 3.5f*5.0f*/) && agentPos.distance(current_waypoint) < 30.0f/*35.0f*/) ||
 (velocity >= 29.0f && agentPos.distance(current_waypoint) < 40.0f));
    
    //Use averaged speed and power values if we have skipped waypoints. cosmic vole. TODO may be safer to take minimum?
    if (skipped > 0)
    {
        maxspeed = speed2 / (skipped + 1);
        acc_power = power2 / (skipped + 1);
    }
    
    //Debugging - draw blue spheres on the track for the original and adjusted waypoints - cosmic vole June 30th 2017
    if (beam == m_sim_controller->GetBeamFactory()->getCurrentTruck())
    {
        Vector3 debugAdjustedWpt = current_waypoint;
        Vector3 debugOrigWpt = waypoints[current_waypoint_id];
        debugOrigWpt.y = debugAdjustedWpt.y = beam->getPosition().y + 0.2f;
        debugPoints(debugAdjustedWpt, debugOrigWpt, debugAdjustedWpt.y);//current_waypoint.y);
    }
}

void VehicleAI::update(float dt, int doUpdate)
{
    //First calculate average acceleration to aid collision avoidance - cosmic vole April 7 2017
    accel_time += dt;
    if (accel_time >= 0.5f)
    {
        Vector3 newVelocity = beam->getVelocity();
        average_accel = newVelocity - accel_velocity;
        average_accel = average_accel / accel_time;
        accel_time = 0.0f;
        accel_velocity = newVelocity;
    }
    
    //Detect vehicle crashes to debug the AI - cosmic vole August 12 2017
    float health = beam->getVehicleHealth();
    if (last_health - health > 8.0f)
    {
        if (last_crash_time >= 10.0f || last_crash_time < 0.0f)
        {
            LOG("Truck " + TOSTRING(beam->trucknum) + " CRASHED. Health: " + TOSTRING(health) + ".");
            LOG("Waypoints up to crash:");
            int start = 1;
            if (num_waypoints_in_lap > 0 && current_waypoint_id > num_waypoints_in_lap)
            {
                start = current_waypoint_id - num_waypoints_in_lap;
            }
            for (int i = start; i <= current_waypoint_id; i++)
            {
                if ((i == current_waypoint_id) || (vehiclePositionsAtWaypoints[i] != Vector3::ZERO))
                    LOG( TOSTRING(adjusted_waypoints[i].x) + "," + TOSTRING(adjusted_waypoints[i].z) );
            }
            LOG("\nPositions at waypoints:");
            for (int i = start; i < current_waypoint_id; i++)
            {
                if (vehiclePositionsAtWaypoints[i] != Vector3::ZERO)
                    LOG( TOSTRING(vehiclePositionsAtWaypoints[i].x) + "," + TOSTRING(vehiclePositionsAtWaypoints[i].z) );
            }
            if (adjusted_waypoint_speed[current_waypoint_id] > 80.0f)
                adjusted_waypoint_speed[current_waypoint_id] = 80.0f;
        }
        last_crash_time = 0.0f;
        last_health = health;
    }
    else if (last_crash_time >= 0.0f)
    {
        last_crash_time += dt;
    }
    
    if (is_waiting)
    {
        wait_time -= dt;
        if (wait_time < 0)
        {
            is_waiting = false;
        }
        return;
    }
    
    //Enable analog control of steering, necessary for some of my AI changes - cosmic vole April 12 2017
    beam->hydroSpeedCoupling = false;

    Vector3 mAgentPosition = beam->getPosition();
    //Keep the absolute position in a separate variable as the other one will be normalized. cosmic vole October 10 2016.
    Vector3 mAgentAbsPosition = mAgentPosition;
    float curDistance;

    if (lastWaypointNavigated == Vector3::ZERO)
    {
        lastWaypointNavigated = mAgentPosition;
    }

    float waypoint_dist = current_waypoint.distance(mAgentPosition);
    bool reachedWaypoint = false;//(waypoint_dist < 6.8f);//6.5f)//5.5f)//7)//9)//5)
    bool offCourse = false;
    float offCourseDist = 0.0f;
    if (!reachedWaypoint && free_waypoints > 0)
    {
        //Vector3 lastWaypoint;
        Vector3 nextWaypoint;
        //Vector3 &a, &b;
        Vector3 trackDirection = Vector3::ZERO;
        Vector3 trackCentre = Vector3::ZERO;
        //if (current_waypoint_id > 1)
        {
            //lastWaypoint = waypoints[current_waypoint_id - 1];
            trackDirection = current_waypoint - lastWaypointNavigated;//lastWaypoint;
            trackCentre = getClosestPointOnLine(lastWaypointNavigated, current_waypoint, mAgentPosition, false);
            //a = lastWaypoint;
            //b = current_waypoint;
        }
        /*
        else if (current_waypoint_id < free_waypoints)
        {
            nextWaypoint = waypoints[current_waypoint_id + 1];
            trackDirection = nextWaypoint - current_waypoint;
            trackCentre = getClosetPointOnLine(current_waypoint, nextWaypoint, mAgentPosition, false);
            //a = current_waypoint;
            //b = nextWaypoint;
        }
        */
        if (trackDirection != Vector3::ZERO)
        {
            Vector3 acrossTrack = Ogre::Quaternion(Ogre::Degree(90), Ogre::Vector3::UNIT_Y) * trackDirection;
            trackDirection.normalise();
            acrossTrack.normalise();
            Vector3 toTarget = current_waypoint - mAgentPosition;
            float distAlongTrack = toTarget.dotProduct(trackDirection);
            float distAcrossTrack = toTarget.dotProduct(acrossTrack);
            if (abs(distAcrossTrack) > 0.2f)//3.0f)//3.8f)//4.0f);//4.5f)
            {
                offCourse = true;
            }
            offCourseDist = (mAgentPosition - trackCentre).length();//distAcrossTrack;
            if (abs(offCourseDist) > 1.0f)
            {
                //RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, 
                //    "Truck: " + TOSTRING(beam->trucknum) + ". dist to track centre: " + TOSTRING(offCourseDist) + ", sum errors: " + TOSTRING(sumErrorsAcrossTrack) + ", distAcrossTrack: " + TOSTRING(distAcrossTrack) + ".", "note.png");

            }
            if (abs(distAlongTrack) < 6.5f && abs(distAcrossTrack) < 25.0f)//10.0f) //TODO check actual track bounds and also checkpoint locations
            {
                reachedWaypoint = true;
                
                //RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, 
                //    "Waypoint reached along track: " + TOSTRING(beam->trucknum) + ". distAlongTrack: " + TOSTRING(distAlongTrack) + ", distAcrossTrack: " + TOSTRING(distAcrossTrack) + ", DIST: " + TOSTRING(waypoint_dist) + "." + ". dist to track centre: " + TOSTRING(offCourseDist), "note.png");
                    
                if (beam->hydroSpeedCoupling)
                {
//RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, 
//                    "hydroSpeedCoupling is true (digital steering control)", "note.png");                    
                }
                else
                {
//RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, 
//                    "hydroSpeedCoupling is false (analog steering control)", "note.png");                    
                }
            }
        }
        
    }
    if (!reachedWaypoint)
    {
        reachedWaypoint = (waypoint_dist < 6.8f);//6.5f)//5.5f)//7)//9)//5)
    }
    if (reachedWaypoint)
    {
        
        turn_time = 0.0f;
        updateWaypoint();
        //Bug fix for auto reset. If the waypoints are very close together, the reset can fire off if we don't keep checking this here. cosmic vole October 10 2016
        if (is_enabled && stuck_reset_delay > 0.0f)
        {
            curDistance = mAgentAbsPosition.distance(stuck_position);
            if (curDistance >= stuck_cancel_distance)
            {
                is_stuck = false;
                stuck_time = 0.0f;
                stuck_position = mAgentPosition;
            }
        }
        return;
    }

    Vector3 TargetPosition = current_waypoint;//adjusted_waypoints[current_waypoint_id];//cosmic vole changed from current_waypoint; June 21 2017
    TargetPosition.y = 0; //Vector3 > Vector2
    Quaternion TargetOrientation = Quaternion::ZERO;

    mAgentPosition.y = 0; //Vector3 > Vector2
    float agentRotation = beam->getHeadingDirectionAngle();
    Quaternion mAgentOrientation = Quaternion(Radian(agentRotation), Vector3::NEGATIVE_UNIT_Y);
    mAgentOrientation.normalise();

    Vector3 mVectorToTarget = TargetPosition - mAgentPosition; // A-B = B->A
    mAgentPosition.normalise();
    
    //OLD CODE
    Vector3 mAgentHeading = mAgentOrientation * mAgentPosition;
    Vector3 mTargetHeading = TargetOrientation * TargetPosition;
    mAgentHeading.normalise();
    mTargetHeading.normalise();

    bool usePIDControl = true;//cosmic vole added experimental PID control April 12 2017 - TODO possibly make this configurable?

    // Compute new torque scalar (-1.0 to 1.0) based on heading vector to target.
    Vector3 mSteeringForce = mAgentOrientation.Inverse() * mVectorToTarget;
    
    if (fabsf(mSteeringForce.x) > 6.0f)//4.0f) //Steering force is usually under 4. When a vehicle's swerving / crashing it can go as high as 9.0 or even 19.0.
    {
        LOG("Steering force large for truck " + TOSTRING(beam->trucknum) + " Force: " + TOSTRING(mSteeringForce.x) +
        " Current waypoint [" + TOSTRING(current_waypoint_id) + "]: " + TOSTRING(current_waypoint) + " Truck pos: " + TOSTRING(beam->getPosition()) + " Truck dir: " + TOSTRING(beam->getDirection()) + " Prev waypoint: " +
        TOSTRING(adjusted_waypoints[current_waypoint_id - 1]) + " Lane change from: " + TOSTRING(lane_change_from) + " Lane change to: " + TOSTRING(lane_change_to) + " Start wpt: " + TOSTRING(lane_change_start_wpt) + " End wpt: " + TOSTRING(lane_change_end_wpt) + ".");
    }
    
    //cosmic vole - have to set default speed as collision avoidance code may have reduced it March 4 2017
    float speed = adjusted_waypoint_speed[current_waypoint_id];//(current_waypoint_id>1)? adjusted_waypoint_speed[current_waypoint_id-1] : 0.0f;
    if (speed)
        maxspeed = speed;
    else
        maxspeed = 200.0f;//TODO what to use?

    float power = waypoint_power[current_waypoint_id];//(current_waypoint_id>1)? waypoint_power[current_waypoint_id-1] : 0.0f;
    if (power)
        acc_power = power;
    else
        acc_power = 1.0f;
        
    if (offCourse && abs(offCourseDist) > /*2.0f*//*3.0f*/6.0f)// && time_since_last_collision_avoid > 4.0f)
    {
        //Reduce speed and power flexibly based on how far off course we are - TODO this needs averaging over time as part of our stability metric
        float adjust = abs(offCourseDist);
        //TODO this shouldn't really be applied when they are overtaking / collision avoiding as it currently may conflict with speed / power
        //controls in place around that code
        if (adjust > 3.0f)
        {
            adjust = 3.0f;
        }
        //adjust = adjust * 0.24f; //0 --> 0.72
        adjust = adjust * 0.1f;
        adjust = 1.0f - adjust; //0.28 -> 1.0
        
        //TODO sanity check these against speed, stability etc
        //maxspeed *= adjust;//0.3f;//0.5f;
        acc_power *= adjust;//0.2f;//0.4f;
        /*
        if (maxspeed < 4.0f)
        {
            maxspeed = 4.0f;
        }
        */
        if (beam->getVelocity().length() < 5.0f && acc_power < 0.4f)//0.3f)
        {
            acc_power = 0.4f;//0.3f;
        }
        //usePIDControl = false;
    }
    

    //cosmic vole added auto reset code for stuck vehicles October 9 2016
    if (is_enabled && stuck_reset_delay > 0.0f)
    {
        float curDistance = mAgentAbsPosition.distance(stuck_position);
        if (curDistance >= stuck_cancel_distance)
        {
            is_stuck = false;
            stuck_time = 0.0f;
            stuck_position = mAgentAbsPosition;
        }
        else
        {
            is_stuck = true;
            stuck_time += dt;
            if (stuck_time >= stuck_reset_delay)
            {
                //RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, 
                //    "Resetting vehicle: " + TOSTRING(beam->trucknum) + ". Got stuck at (" + TOSTRING(stuck_position.x) + ", " + TOSTRING(stuck_position.y) + ", " + TOSTRING(stuck_position.z) + "); cur. pos (" + 
                //    TOSTRING(mAgentAbsPosition.x) + ", " + TOSTRING(mAgentAbsPosition.y) + ", " + TOSTRING(mAgentAbsPosition.z) + "); for time " + TOSTRING(stuck_time) + ". Dist. moved: " + TOSTRING(curDistance) + ".", "note.png");
                //Reset the vehicle's position to the last waypoint - TODO we may or may not want to repair the vehicle
                Vector3 resetPos;
                if (current_waypoint_id <= free_waypoints)
                {
                    //if (current_waypoint_id > 0)
                    //{
                    //    resetPos = waypoints[current_waypoint_id-1];
                    //}
                    //else
                    //{
                    resetPos = beam->nodes[0].initial_pos;//initial_node_pos[0];//waypoints[current_waypoint_id];
                    if (resetPos.squaredDistance(beam->nodes[0].AbsPosition) < 9.0)
                    {
                        //It's already on the spawn point so don't reset
                        resetPos = Vector3::ZERO;
                    }
                    //}

                    //When the vehicle's at the start, the 0th waypoint is Vector3::ZERO. TODO we should probably detect this better.
                    if (resetPos != Vector3::ZERO)
                    {
                        //We will aim to reset to the last waypoint visited, but
                        //before we move to the waypoint, find a clear space on the track from that point back
                        float maxRelocateDistanceSq = 100.0 * 100.0;//If there's no clear track within 100 metres, this truck will just sit and wait
                        int i = current_waypoint_id-1;
                        bool found_space = false;
                        for (int j=0; !found_space && j<free_waypoints; j++)
                        {
                            //0th waypoint is not a point on the track, we need to move round to the very last waypoint - TODO Jan 2017 it actually needs to be the last waypoint of the first lap rather than the end of the race...
                            if (i<=0)
                            {
                                i = free_waypoints;
                            }
                            int iprev = i-1;
                            if (i == 1)
                            {
                                iprev = free_waypoints;
                            }
                            Vector3 thisWaypoint = waypoints[i];
                            if (j > 0 && thisWaypoint.squaredDistance(mAgentAbsPosition) > maxRelocateDistanceSq)
                            {
                                //Too far away. Give up and wait for the track to clear.
                                break;
                            }
                            Beam** trucks = m_sim_controller->GetBeamFactory()->getTrucks();
                            int numTrucks = m_sim_controller->GetBeamFactory()->getTruckCount();

                            do
                            {
                                //Assume this new position is OK until we've checked all the trucks
                                found_space = true;
                                for (int t=0; t<numTrucks; t++)
                                {
                                    if (trucks[t] == nullptr || trucks[t]->trucknum == beam->trucknum)
                                    {
                                        continue;
                                    }
                                    //TODO something's not working quite right with this as a car respawned right inside the player's car today! cosmic vole October 11 2016
                                    if (thisWaypoint.squaredDistance(trucks[t]->getPosition()) < 9.0)
                                    {
                                        //That truck's too close
                                        found_space = false;
                                        break;
                                    }
                                }
                                if (!found_space)
                                {
                                    Vector3 prevWaypoint = waypoints[iprev];
                                    //If we've reversed past the previous waypoint, we need to continue our outer loop to start looking at the other waypoints (busy track!)
                                    if (waypoints[i].squaredDistance(thisWaypoint) - waypoints[i].squaredDistance(prevWaypoint) > 0.1)
                                    {
                                        break;
                                    }
                                    Vector3 headBack = prevWaypoint - waypoints[i];//thisWaypoint;
                                    if (headBack.squaredLength() < 0.0001f)
                                    {
                                        //The two waypoints coincide. We need to keep looking at the other ones
                                        break;
                                    }
                                    headBack = headBack.normalise();//Should be 1 metre length which is good for our purposes
                                    thisWaypoint += headBack;
                                    if (j > 0 && thisWaypoint.squaredDistance(mAgentAbsPosition) > maxRelocateDistanceSq)
                                    {
                                        //Too far away. Give up and wait for the track to clear.
                                        break;
                                    }
                                }
                            }
                            while (!found_space);

                            if (found_space)
                            {
                                resetPos = thisWaypoint;
                                //Don't forget to update the current waypoint for the AI!
                                current_waypoint_id = i+1;
                                if (current_waypoint_id > free_waypoints)
                                {
                                    //BUG / TODO review this. It will create an infinite loop which is not the normal behaviour!
                                    current_waypoint_id = 1;
                                }
                                //As we've changed the waypoint, at the very least we need to recalc the Target Heading so we know how to rotate the car into it's new position
                                TargetPosition = current_waypoint = waypoints[current_waypoint_id];
                                TargetPosition.y = 0; //Vector3 > Vector2
                                Quaternion TargetOrientation = Quaternion::ZERO;
                                //mAgentPosition.y = 0; //Vector3 > Vector2
                                //float agentRotation = beam->getHeadingDirectionAngle();
                                //Quaternion mAgentOrientation = Quaternion(Radian(agentRotation), Vector3::NEGATIVE_UNIT_Y);
                                //mAgentOrientation.normalise();
                                //Vector3 mVectorToTarget = TargetPosition - mAgentPosition; // A-B = B->A
                                //mAgentPosition.normalise();
                                //Vector3 mAgentHeading = mAgentOrientation * mAgentPosition;
                                mTargetHeading = TargetOrientation * TargetPosition;
                                //mAgentHeading.normalise();
                                mTargetHeading.normalise();
                                //We've finally found our respawn point. Good to go!
                                break;
                            }
                            i--;
                        }

                        if (found_space && resetPos != Vector3::ZERO)
                        {
                            //Vector3 mSpawnRotationXZ = Vector3(0.0f, 0.0f, 1.0f);
                            Vector3 beamDir = beam->getDirection();
                            beamDir.normalise();
                            Degree pitchAngle = Radian(asin(beamDir.dotProduct(Vector3::UNIT_Y)));
                            //360 degree Pitch using atan2 and pythag: http://stackoverflow.com/questions/3755059/3d-accelerometer-calculate-the-orientation
                            float beamDirY = beamDir.dotProduct(Vector3::UNIT_Y);
                            float beamDirZ = beamDir.dotProduct(Vector3::UNIT_Z);
                            //Yaw: return atan2(idir.dotProduct(Vector3::UNIT_X), (idir).dotProduct(-Vector3::UNIT_Z));
                            Degree pitchAngle2 = Radian(atan2(beamDir.dotProduct(-Vector3::UNIT_X), sqrt(beamDirY * beamDirY + beamDirZ * beamDirZ)));
                            Degree rollAngle = Radian(atan2(beamDirY, beamDirZ));
                            Vector3 mTargetHeadingXZ = mTargetHeading;
                            mAgentHeading.y = 0.0f;
                            mTargetHeadingXZ.y = 0.0f;

                            //agentRotation gives the vehicle's current rotation - shouldn't be relevant when we do resetAngle()
                            //we can also get the spawnRotation but that is already accounted for in resetAngle()
                            Degree targetRotation = Radian(atan2(mTargetHeading.dotProduct(Vector3::UNIT_X), mTargetHeading.dotProduct(-Vector3::UNIT_Z)));


                            Vector3 rollv=beam->nodes[beam->cameranodepos[0]].RelPosition-beam->nodes[beam->cameranoderoll[0]].RelPosition;
                            rollv.normalise();
                            rollAngle=Radian(asin(rollv.dotProduct(Vector3::UNIT_Y)));

                            //Find the up vector to tell if upside down or not
                            Vector3 cam_pos  = beam->nodes[beam->cameranodepos[0]].RelPosition;
                            Vector3 cam_roll = beam->nodes[beam->cameranoderoll[0]].RelPosition;
                            Vector3 cam_dir  = beam->nodes[beam->cameranodedir[0]].RelPosition;

                            //Vector3 rollv = (cam_pos - cam_roll).normalisedCopy();
                            Vector3 dirv  = (cam_pos - cam_dir ).normalisedCopy();
                            Vector3 upv   = dirv.crossProduct(-rollv);

                            //Quaternions to reorientate the vehicle (experimental based on http://www.ogre3d.org/tikiwiki/Quaternion+and+Rotation+Primer#Q_How_can_I_make_my_objects_stand_upright_after_a_bunch_of_rotations_)
                            //Also based on https://github.com/opengl-tutorials/ogl/blob/master/common/quaternion_utils.cpp
                            //Vector3 localY = mNode->getOrientation() * Vector3::UNIT_Y;
                            // Get rotation to original facing                                          
                            Quaternion quatRestoreFwd = dirv.getRotationTo(TargetPosition - resetPos);//UNIT_Z worked great: Vector3::UNIT_Z);//currentFacing.getRotationTo(mInitFacing);
                            // Because of the 1rst rotation, the up is probably completely screwed up.
                            // Find the rotation between the "up" of the rotated object, and the desired up
                            Vector3 newUp = quatRestoreFwd * upv;//Vector3::UNIT_Y;
                            Quaternion quatRestoreUp = newUp.getRotationTo(Vector3::UNIT_Y);//upv.getRotationTo(Vector3::UNIT_Y);
                            Quaternion quatRestore = quatRestoreUp * quatRestoreFwd;//remember, in reverse order.


                            //RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, 
                            //    "Vehicle: " + TOSTRING(beam->trucknum) + "  Yaw: " + TOSTRING(Radian(agentRotation).valueDegrees()) + " Pitch: " + TOSTRING(pitchAngle.valueDegrees()) + " Pitch (alt calc): " + TOSTRING(pitchAngle2.valueDegrees()) + " Roll: " + TOSTRING(rollAngle.valueDegrees()) + " Target Yaw: " + TOSTRING(targetRotation.valueDegrees()) + " cur. pos (" + 
                            //    TOSTRING(mAgentAbsPosition.x) + ", " + TOSTRING(mAgentAbsPosition.y) + ", " + TOSTRING(mAgentAbsPosition.z) + "); for time " + TOSTRING(stuck_time) + ". Dist. moved: " + TOSTRING(curDistance) + ".", "note.png");

                            //resetAngle() WON'T WORK if the truck is upside down or on its side as it uses a matrix made from Euler angles and only deals with yaw!
                            //Doesn't seem to work beam->resetAngle(targetRotation);// + mAgentHeading.angleBetween(mTargetHeadingXZ).valueDegrees());//mSteeringForce.x);//mTargetHeading.x);//beam->getRotation());
                            beam->updateFlexbodiesFinal();
                            //I think the reason this isn't quite right is we need to know the heading of the target FROM resetPos
                            //beam->displace(Vector3::ZERO, mAgentHeading.angleBetween(mTargetHeadingXZ).valueDegrees());//targetRotation);


                            //TODO Do this instead (this will be slow so we may need to look at deferring it to a later time slice??):
                            node_t* nodes = beam->nodes;
                            int nodeCount = beam->free_node;
                            //This code is taken from LoadTruck(), but we've got problems here. We can't easily base the rotations on each node's initial position because the nodes may be deformed.
                            //I am not sure there's an obvious way to tell whether the whole vehicle is rotated (e.g.) upside down or just some of the nodes are rotated relative to the others!
                            //We need the full quaternion direction the vehicle's nodes are currently pointing and to first reverse that rotation, then to apply the new one

                            //Something like this: look at calcAnimators() from line 2389 as well to work out if vehicle upside down etc.
                            //Also look at RORFrameListener.cpp, inside advanced repair, how it steers and moves the nodes of the whole truck (based on input events) using beam->displace(): curr_truck->displace(translation * scale, rotation * scale); line 780

                            // Set origin of rotation to camera node
                            Vector3 origin = nodes[0].AbsPosition;

                            if (beam->cameranodepos[0] >= 0 && beam->cameranodepos[0] < nodeCount)//MAX_NODES)
                            {
                                origin = nodes[beam->cameranodepos[0]].AbsPosition;
                            }
                            for (int i=0; i<nodeCount; i++)
                            {
                                nodes[i].AbsPosition = quatRestore * nodes[i].AbsPosition;//spawn_position + spawn_rotation * (nodes[i].AbsPosition - spawn_position);
                                nodes[i].RelPosition = nodes[i].AbsPosition - origin;
                            }

                            beam->resetPosition(resetPos.x, resetPos.z, false, 0.0f);//resetPos.y);//false, 0.0f);//resetPos.y);
                            if (!beam->engine->isRunning())
                            {
                                beam->engine->start();
                            }
                            if (beam->parkingbrake)
                            {
                                beam->parkingbrakeToggle();
                            }

                        }
                    }

                    mAgentPosition = beam->getPosition();
                    is_stuck = false;
                    stuck_time = 0.0f;
                    stuck_position = mAgentPosition;
                    return;
                }

            }
        }
    }
    
    
    Vector3 agentVelocity = beam->getVelocity();
    //We will only move or add waypoints for a predicted collision if we haven't already done this
    //If we already did it, we still run collision detection again to adjust velocities etc.
    //bool canAdjustWaypoint = false;//collision_waypoint_id.empty();
    //bool hasCarOnLeft = false;
    //bool hasCarOnRight = false;
    bool hasStationaryObstacleInFront = false; //TODO This doesn't get assigned here anymore but is needed
  
    float steeringForceRaw = mSteeringForce.x;
    mSteeringForce.normalise();

    float mYaw = mSteeringForce.x;
    float mPitch = mSteeringForce.z;
    //float mRoll   = mTargetVO.getRotationTo( mAgentVO ).getRoll().valueRadians();

    if (mPitch > 0)
    {
        if (mYaw > 0)
            mYaw = 1;
        else if (mYaw < 0)//Allow zero yaw. cosmic vole.
            mYaw = -1;
    }
    
    float navigationYaw = mYaw; //cosmic vole - we screw around with yaw, so remember where we are trying to go
    
    float maxsteermult = 11.0f;//was 11
    
    //Track the vehicle's stability as it follows the waypoints - cosmic vole August 14 2017
    stability_time += dt;

    if (steeringForceRaw < -4.0f)
    {
        if (steeringForceRaw < stability_max_negative_steer)
            stability_max_negative_steer = steeringForceRaw;
        stability_last_negative_steer = steeringForceRaw;
        if (stability_last_steer > 4.0f)
            stability_plus_minus_transitions++;
        stability_last_steer = steeringForceRaw;
        if (stability_plus_minus_transitions < 1)
            maxsteermult = 13.0f;
            
    }
    else if (steeringForceRaw > 4.0f)
    {
        if (steeringForceRaw > stability_max_positive_steer)
            stability_max_positive_steer = steeringForceRaw;
        stability_last_positive_steer = steeringForceRaw;
        if (stability_last_steer < -4.0f)
            stability_plus_minus_transitions++;
        stability_last_steer = steeringForceRaw;
        if (stability_plus_minus_transitions < 1)
            maxsteermult = 13.0f;
    }
    float agentSpeed = agentVelocity.length();
    if (agentSpeed > stability_max_speed)
        stability_max_speed = agentSpeed;
    
    if (!stability_lost_control && stability_max_speed * 3.6f >= 14.0f)//16.0f)
    {
        //TEMP TEST Take a look at the truck. TODO remove this!
        static float time_since_last_truck_change = -1.0f;
        if (time_since_last_truck_change >= 0.0f)
            time_since_last_truck_change += dt;
        if ((fabsf(steeringForceRaw) > 8.0f/*6.0f*//*4.6f && stability_plus_minus_transitions >= 1*/) && (time_since_last_truck_change >= 60.0f || time_since_last_truck_change < 0.0f))
        {
            //BeamFactory::getSingleton().setCurrentTruck(beam->trucknum);
            time_since_last_truck_change = 0.0f;
        }
        if (stability_plus_minus_transitions >= 1 || fabsf(steeringForceRaw) > /*6.0f*//*8.0f*/9.5f)//2)//3)
        {
            LOG("Truck " + TOSTRING(beam->trucknum) + " lost stability. Steering transitions: " + TOSTRING(stability_plus_minus_transitions) +
            " Max speed: " + TOSTRING(stability_max_speed) + " Time: " + TOSTRING(stability_time) + ".");
            stability_lost_control = true;
            float oldmaxspeed = maxspeed;
            if (stability_skip_to_waypoint < current_waypoint_id)
            {
                stability_skip_to_waypoint = current_waypoint_id + 1;//3;//2;
                current_waypoint_id = stability_skip_to_waypoint;
                if (current_waypoint_id > free_waypoints)
                    current_waypoint_id = free_waypoints;
                maxspeed = MIN(adjusted_waypoint_speed[current_waypoint_id-1], adjusted_waypoint_speed[current_waypoint_id]);
                if (!maxspeed || maxspeed > oldmaxspeed)
                    maxspeed = oldmaxspeed;
                if (maxspeed > 80.0f)
                    maxspeed = 80.0f;
                adjusted_waypoint_speed[current_waypoint_id] = maxspeed;
                LOG("Skipping to waypoint " + TOSTRING(current_waypoint_id) + " and reducing engine power: " + TOSTRING(acc_power) + ".");
            }
            if (acc_power > 0.4f)//0.2f)
                acc_power = 0.4f;//0.2f;
            //Attempt to damp the steering
            steeringForceRaw *= 0.9f;//0.6f;//0.5f;
            //usePIDControl = false;
            //beam->hydroSpeedCoupling = true;// re-enable digital steering input as we don't want to instantaneously jump to full lock            
            
        }
    }
    else if (stability_lost_control && stability_time >= 3.0f)
    {
        stability_lost_control = false;
        acc_power = waypoint_power[current_waypoint_id];
        if (acc_power <= 0.0f)
            acc_power = 1.0f;
    }
    
    if (stability_time >= 3.0f)
    {
        stability_time = 0.0f;
        stability_max_speed = 0.0f;
        stability_plus_minus_transitions = 0;
        stability_last_negative_steer = 0.0f;
        stability_last_positive_steer = 0.0f;
        stability_max_negative_steer = 0.0f;
        stability_max_positive_steer = 0.0f;
        stability_last_steer = 0.0f;
    }



    //Always reduce power when steering hard at speed - cosmic vole March 11 2017
    //TODO FIX this I think it makes the car shake Also need to pick apart how hydrodirstate relates to what way the wheels are pointing!
    /*
    if (agentVelocity.length() > 6.4f && fabsf(beam->hydrodirstate) > 0.20f && acc_power > 0.8f)//acc_power > 0.0f)// was > 0.22f 0.15f)
    {
        acc_power = 0.8f;//0.0f;//0.15f;
    }
    else if (agentVelocity.length() > 6.4f && fabsf(beam->hydrodirstate) > 0.35f && acc_power > 0.5f)//acc_power > 0.0f)// was > 0.22f 0.15f)
    {
        acc_power = 0.5f;//0.2f;//0.0f;//0.15f;
    }
    
     */
 
    //TODO refine this. We really need to know whether this is countersteering due to oversteer
    //bool directionChanged = (last_steering_yaw < 0.0f && mYaw > 0.0f || last_steering_yaw > 0.0f && mYaw < 0.0f);
    
    //cosmic vole added last steering direction March 11 2017
    //last_steering_yaw = mYaw;
    
    //cosmic vole added steering slowdown March 8 2017
    bool skidding = false;
    float maxSlip = 0.0f;
    for (int i=0; i<beam->free_wheel; i++)
    {
        // ignore wheels without data
        if (beam->wheels[i].lastContactInner == Vector3::ZERO && beam->wheels[i].lastContactOuter == Vector3::ZERO)
            continue;
        float slip = beam->wheels[i].lastSlip;
        if (slip > maxSlip)
        {
            maxSlip = slip;
        }
        skidding = skidding || beam->wheels[i].isSkiding || slip > 7.25f;//6.9f;//7.1f 7.5f;//7.0f;//6.5f;
    }
    if (agentVelocity.length() < 7.0f)
    {
        skidding = false;
    }
    direction_changed = direction_changed || (skidding || (skid_time > 0.0f && skid_time <= 1.4f)) && ((last_skid_hydrodirstate < -0.1f && mYaw > 0.1f) || (last_skid_hydrodirstate > 0.1f && mYaw < -0.1f));
    if (skidding)
    {
        skid_time += dt;//0.0f;
        last_steering_yaw = mYaw;
        last_skid_hydrodirstate = beam->hydrodirstate;
        //Speed up steering
        //if (is_counter_steering)
        //{
        //    steering_time = 0.0f;
        //}
        //Reduce power (for RWD)
        if (acc_power > 0.8f)//0.0f)//0.1f)//was 0.3f
        {
//            acc_power = 0.8f;//0.0f;//0.1f;
        }
        /*
        if (directionChanged)// && !is_counter_steering)
        {
            if (!is_counter_steering)
                RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, "Counter steering started!\n", "note.png");
            is_counter_steering = true;//!is_counter_steering;//true;
        }
        */
    }
    else
    {
        if (skid_time > 0.00001f)
        {
            skid_time += dt;
            if (skid_time > 1.3f)
            {
                skid_time = 0.0f;
                //RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, "Counter steering stopped!\n", "note.png");
                is_counter_steering = false;
                direction_changed = false;
            }
            else if (skid_time < 0.85f && acc_power > 0.8f && agentVelocity.length() >= 7.0f)
            {
//               acc_power = 0.8f;//0.0f;//0.1f;
            }
        }
//        if (is_counter_steering && skid_time <= 0.00001f)//(skid_time > 0.6f || (last_skid_hydrodirstate < -0.3f && mYaw > 0.3f) || (last_skid_hydrodirstate > 0.3f && mYaw < -0.3f) || fabsf(beam->hydrodirstate < 0.1f) ))
 //       {
//            RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, "Counter steering stopped!\n", "note.png");
 //           is_counter_steering = false;
            //skid_time = 0.0f;
 //       }
    }
    
    if (direction_changed && skid_time > 0.33f)//0.35f)//0.4f)
    {
        //if (!is_counter_steering)
        //   RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, "Counter steering started!\n", "note.png");
        is_counter_steering = true;
        if (skid_time > 0.40f)//0.42f)
        {
            //Speed up steering to control the skid
            steering_time = 0.0f;
        }
        else
        {
            if (steering_time > 0.1f)
            {
                steering_time = 0.05f;//0.1f;
            }
        }
    }



/*    
    if (directionChanged)// && !is_counter_steering)
    {
        if (!is_counter_steering)
            RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, "Counter steering started!\n", "note.png");
        is_counter_steering = true;//!is_counter_steering;//true;
        //steering_time = 0.0f;
    }
    else
    {
        skid_time += dt;
        if (is_counter_steering && (skid_time > 1.4f || (last_skid_hydrodirstate < -0.1f && mYaw > 0.1f) || (last_skid_hydrodirstate > 0.1f && mYaw < -0.1f) || fabsf(beam->hydrodirstate < 0.05f) ))
        {
            RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, "Counter steering stopped!\n", "note.png");
            is_counter_steering = false;
            skid_time = 0.0f;
        }
    }
    
    //Speed up steering
    if (is_counter_steering)
    {
        //steering_time = 0.0f;
    }
*/

/* Commented out for now - cosmic vole June 24 2017
    //Collision avoidance for immediate vicinity

    if (hasCarOnLeft && mYaw > 0.0f)//< 0.0f)
    {
        mYaw = -0.2f;//0.2f;//0.1f;
        usePIDControl = false;
        beam->hydroSpeedCoupling = false;//enable analog steering
        //usePIDControl = true;
        //steeringForceRaw = -0.1f;
    }
    else if (hasCarOnRight && mYaw < 0.0f)//> 0.0f)
    {
        mYaw = 0.2f;//0.2f;//-0.1f;
        usePIDControl = false;
        beam->hydroSpeedCoupling = false;//enable analog steering
        //usePIDControl = false;
        //steeringForceRaw = 0.1f;
    }
*/    
    if (hasStationaryObstacleInFront && beam->getVelocity().length() < 5.0f)
    {
        int curGear = beam->engine->getGear();
        if (curGear != -1 )
        {
            beam->engine->shiftTo(-1);
        }
        //acc_power = -acc_power;
        if (acc_power < 0.6f)
        {
            acc_power = 0.6f;
        }
        mYaw = -navigationYaw;//mYaw;
        usePIDControl = false;
    }
    else
    {
        int curGear = beam->engine->getGear();
        if (curGear < 0 )
        {
            beam->engine->shiftTo(1);
        }        
    }

    //Roll correction - cosmic vole TODO make this configurable per track - and poss try to detect a banked slope (disable if at least 3 wheels touching ground) March 12 2017
    Vector3 rollv=beam->nodes[beam->cameranodepos[0]].RelPosition-beam->nodes[beam->cameranoderoll[0]].RelPosition;
    rollv.normalise();
    float rollAngle0 =Radian(asin(rollv.dotProduct(Vector3::UNIT_Y))).valueDegrees();
    //TODO this is WRONG. It fluctuates constantly
    float rollAngle = Radian(atan2(rollv.dotProduct(Vector3::UNIT_X), rollv.dotProduct(Vector3::UNIT_Y))).valueDegrees();
    float rollAngle2 = rollAngle;
    if (rollAngle2 > 180.0f)
    {
        rollAngle2 = rollAngle2 - 360.0f;
    }
    
    //static int rollMsgCount = 0;
    if (allowRollCorrection && fabsf(rollAngle0) > 5.0f)
    {
        //if (rollMsgCount == 0)
        //    RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, "Roll detected. angle0: " + TOSTRING(rollAngle0) + " angle: " + TOSTRING(rollAngle) + " angle2: " + TOSTRING(rollAngle2) + " \n", "note.png");
        //rollMsgCount++;
        //if (rollMsgCount > 5)
        //{
        //    rollMsgCount = 0;
        //}
        //Roll correction for angles less than 12 degrees (e.g. 10.5) does help in some situations but more often than not
        //it was making the vehicle crash - especially if it was close to the edge of the track as it just made it steer off course
        //If we want to re-enable it for small angles it's probably worth checking how far off course the vehicle is (angle and distance from center of track).
        //cosmic vole August 17 2017.
        if (rollAngle0 > /*12.0f*//*14.0f*//*13.5f*//*12.5f*//*10.5f*/13.25f && rollAngle0 < 180.0f)
        {
            LOG("Roll correction enabled for Truck " + TOSTRING(beam->trucknum) + " Roll angle: " + TOSTRING(rollAngle0) + " Steer. Force: " + TOSTRING(steeringForceRaw) + "." );
            //If angle is anticlockwise, looking from rear of vehicle, it has its right side raised, so we need to steer left
            mYaw = -1.0f;
            beam->hydroSpeedCoupling = true;// re-enable digital steering input as we don't want to instantaneously jump to full lock
            usePIDControl = false;
        }
        else if (rollAngle0 < /*-12.0f*//*-15.0f*//*-13.5f*//*-12.5f*//*-10.5f*/-13.25f && rollAngle0 > -180.0f)
        {
            LOG("Roll correction enabled for Truck " + TOSTRING(beam->trucknum) + " Roll angle: " + TOSTRING(rollAngle0) + " Steer. Force: " + TOSTRING(steeringForceRaw) + "." );
            //If angle is clockwise, looking from rear of vehicle, it has its left side raised, so we need to steer right
            mYaw = 1.0f;
            beam->hydroSpeedCoupling = true;// re-enable digital steering input as we don't want to instantaneously jump to full lock
            usePIDControl = false;
        }
        /*
        else if (rollAngle0 > 7.5f && rollAngle0 < 180.0f)
        {
            //If angle is anticlockwise, looking from rear of vehicle, it has its right side raised, so we need to steer left
            if (mYaw > -0.9f)
                mYaw = -0.9f;
            beam->hydroSpeedCoupling = true;// re-enable digital steering input as we don't want to instantaneously jump to full lock
            usePIDControl = false;
        }
        else if (rollAngle0 < -7.5f && rollAngle0 > -180.0f)
        {
            //If angle is clockwise, looking from rear of vehicle, it has its left side raised, so we need to steer right
            if (mYaw < 0.9f)
                mYaw = 0.9f;
            beam->hydroSpeedCoupling = true;// re-enable digital steering input as we don't want to instantaneously jump to full lock
            usePIDControl = false;
        }
        */
        /*
        if (fabsf(steeringForceRaw) > 8.0f)
        {
            if (rollAngle0 < -5.0f && rollAngle0 > -180.0f && steeringForceRaw < -8.0f)
            {
                LOG("Roll correction enabled for Truck " + TOSTRING(beam->trucknum) + " Roll angle: " + TOSTRING(rollAngle0) + " Steer. Force: " + TOSTRING(steeringForceRaw) + "." );
                mYaw = 1.0f;
                beam->hydroSpeedCoupling = true;// re-enable digital steering input as we don't want to instantaneously jump to full lock
                usePIDControl = false;
            }
            else if (rollAngle0 > 5.0f && rollAngle0 < 180.0f && steeringForceRaw > 8.0f)
            {
                LOG("Roll correction enabled for Truck " + TOSTRING(beam->trucknum) + " Roll angle: " + TOSTRING(rollAngle0) + " Steer. Force: " + TOSTRING(steeringForceRaw) + "." );
                mYaw = -1.0f;
                beam->hydroSpeedCoupling = true;// re-enable digital steering input as we don't want to instantaneously jump to full lock
                usePIDControl = false;
            }
            
            
        }
        */
        
        
        //Detect a crash (for debugging AI) if the vehicle rolls past 80 degrees - cosmic vole August 12 2017
        if (fabsf(rollAngle0 > 80.0f))
        {
            if (last_crash_time >= 10.0f || last_crash_time < 0.0f)
            {
                last_health = beam->getVehicleHealth();
                LOG("Truck " + TOSTRING(beam->trucknum) + " CRASHED. Health: " + TOSTRING(last_health) + " Roll angle: " + TOSTRING(rollAngle0) + ".");
                LOG("Waypoints up to crash:");
                int start = 1;
                if (num_waypoints_in_lap > 0 && current_waypoint_id > num_waypoints_in_lap)
                {
                    start = current_waypoint_id - num_waypoints_in_lap;
                }
                for (int i = start; i <= current_waypoint_id; i++)
                {
                    if ((i == current_waypoint_id) || (vehiclePositionsAtWaypoints[i] != Vector3::ZERO))
                        LOG( TOSTRING(adjusted_waypoints[i].x) + "," + TOSTRING(adjusted_waypoints[i].z) );
                }
                LOG("\nPositions at waypoints:");
                for (int i = start; i < current_waypoint_id; i++)
                {
                    if (vehiclePositionsAtWaypoints[i] != Vector3::ZERO)
                        LOG( TOSTRING(vehiclePositionsAtWaypoints[i].x) + "," + TOSTRING(vehiclePositionsAtWaypoints[i].z) );
                }
                if (adjusted_waypoint_speed[current_waypoint_id] > 80.0f)
                    adjusted_waypoint_speed[current_waypoint_id] = 80.0f;
            }
            last_crash_time = 0.0f;
        }
    }
    
 /*   
    if (steering_time > 0.0f)
    {
        steering_time -= dt;
        mYaw = beam->hydrodirstate;
        usePIDControl = false;
        if (steering_time < 0.0f)
            steering_time = 0.0f;
    }
    else
    {
        steering_time = steering_delay;
    }
*/
    

    if (usePIDControl)
    {
        //Apply PID Control algorithm - cosmic vole April 11 2017
        //TODO I think the offCourseDist calc may be wrong - we need to do linear interp between the waypoints rather than check against next waypoint dist
        //if (abs(offCourseDist) > 0.0f)
        //{
        //    mYaw *= offCourseDist;
        //}
        //float err = abs(offCourseDist);
        //Big off track distances were causing excessive steering input, so try reducing the error logarithmically
        //err = (float)log(1.0f + error);
        //err *= mYaw;
        float err = steeringForceRaw;
        mYaw = GetPIDControlFactor(/*abs(offCourseDist) * mYaw*/err, dt);
        if (mYaw > 1.0f)
        {
            mYaw = 1.0f;
        }
        else if (mYaw < -1.0f)
        {
            mYaw = -1.0f;
        }
                       
    }
    
    //Limit speed of analog steering - cosmic vole
    if (beam->hydroSpeedCoupling == false && dt > 0.0f)
    {
        float change = mYaw - beam->hydrodirstate;
        float speed = change / dt;
        //float maxsteerspeed = 1.0f;//was 2.0f which was a barely noticable slowdown
        float maxsteerspeed = maxsteermult / (10.0 + fabs(beam->WheelSpeed / 2.0));
        if (maxsteerspeed < 0.5f)
        {
            maxsteerspeed = 0.5f;
        }
        if (speed > maxsteerspeed)//1.5 may be better than 2.0 TODO - also note it relates to wheel speed in BeamForcesEuler.cpp
        {
            change = maxsteerspeed * dt;
            mYaw = change + beam->hydrodirstate;
        }
        else if (speed < -maxsteerspeed)
        {
            change = -maxsteerspeed * dt;
            mYaw = change + beam->hydrodirstate;
        }        
    }

    // actually steer
    beam->hydrodircommand = mYaw;//mYaw
    
    //Cosmic Vole TODO If actual body roll due to turning (i.e. we're not just on a banked slope etc) is greater than a dangerous amount, steer into the roll to try to stop the vehicle tipping over

    if (beam->engine)
    {
        // start engine if not running
        if (!beam->engine->isRunning())
            beam->engine->start();

        float kmh_wheel_speed = beam->getWheelSpeed() * 3.6;
        
        float targetSpeed = maxspeed;
        if ((lane_change_to >= 0 || lane_change_from >= 0) && lane_change_end_wpt >= current_waypoint_id && lane_change_start_wpt <= current_waypoint_id && targetSpeed < 85.0f)
        {
            //Slow the vehicles down just a bit when they're not following the racing line.
            //This shouldn't stop overtakes happening as all the bots will do it at the same time.
            //cosmic vole August 17 2017
            if (targetSpeed > 31.0f)//35.0f)//40.0f)
            {
                targetSpeed -= 2.0f;//1.0f;//1.5f;//3.0f;//2.5f;
            }
            else
            {
                targetSpeed -= 0.5f;//0.8f;//1.0f;
            }
        }

        if (!stability_lost_control && (abs(mYaw) < 0.5f || fabsf(steeringForceRaw) < 2.0f))
        {
            if (kmh_wheel_speed < targetSpeed - 1)
            {
                beam->brake = 0;
                beam->engine->autoSetAcc(acc_power);
            }
            else if (kmh_wheel_speed > targetSpeed + 1)//1.5f) //2.0f)//1)
            {
                if (kmh_wheel_speed > targetSpeed + 9.0f)
                {
                    beam->brake = beam->brakeforce;
                }
                else if (kmh_wheel_speed > targetSpeed + 7.0f)
                {
                    //This was the default braking force used. Originally / 3 at all speeds. cosmic vole August 21 2017.
                    beam->brake = beam->brakeforce / 1.5f;// / 3
                }
                else
                {
                    beam->brake = beam->brakeforce / 4.5f;
                }
                
                beam->engine->autoSetAcc(0);
            }
            else
            {
                beam->brake = 0;
                beam->engine->autoSetAcc(0);
            }
        }
        else
        {
            if (kmh_wheel_speed < targetSpeed - 1)
            {
                beam->brake = 0;
                beam->engine->autoSetAcc(acc_power / 3);
            }
            else if (kmh_wheel_speed > targetSpeed + 1)//1.5f) //2.0f)//1)
            {
                //cosmic vole - don't apply brakes when skidding March 8 2017
                if (stability_lost_control || skidding || (skid_time > 0.0f && skid_time < 0.8f))
                {
                    beam->brake = /*0.0f;*/beam->brakeforce / 5.5f;//6.0f;//8;//0.0f;
                }
                if (kmh_wheel_speed > targetSpeed + 7.0f)
                {
                    //This was the default braking force used. Originally / 2 at all speeds. cosmic vole August 21 2017.
                    beam->brake = beam->brakeforce / 3.25f;// / 3,  / 2;
                }
                else
                {
                    beam->brake = beam->brakeforce / 4.5f;
                }
                beam->engine->autoSetAcc(0);
            }
            else
            {
                beam->brake = 0;
                beam->engine->autoSetAcc(0);
            }
        }
    }


#if 0
    // this is for debugging purposes
    static SceneNode *n = 0;
    if (!n)
    {
        Entity *e = gEnv->sceneManager->createEntity("axes.mesh");
        n = gEnv->sceneManager->getRootSceneNode()->createChildSceneNode();
        n->attachObject(e);
    }
    n->setPosition(beam->getPosition() + mVectorToTarget);
    n->setOrientation(mAgentOrientation);

#endif

    /* Old code
        // accelerate / brake
        float maxvelo = 1;

        maxvelo = std::max<float>(0.2f, 1 - fabs(mYaw)) * 50;

        maxvelo += std::max<float>(5, std::min<float>(mVectorToTarget.length(), 50)) - 50;

        if (maxvelo < 0)
            maxvelo = 0;

        float pitch = 0.0f;
        // pitch
        if (cameranodepos[0] >= 0 && cameranodepos[0] < MAX_NODES)
        {
            Vector3 dir = nodes[cameranodepos[0]].RelPosition - nodes[cameranodedir[0]].RelPosition;
            dir.normalise();
            float angle = asin(dir.dotProduct(Vector3::UNIT_Y));
            if (angle < -1) angle = -1;
            if (angle > 1) angle = 1;

            pitch = Radian(angle).valueDegrees();
        }

        //More power for uphill
        float power = 80 + pitch;
        power = power / 100;


        //String txt = "brakePower: "+TOSTRING(brakePower);//+" @ "+TOSTRING(maxvelo)
        //RoR::App::GetConsole()->putMessage(Console::CONSOLE_MSGTYPE_SCRIPT, Console::CONSOLE_SYSTEM_NOTICE, txt, "note.png");


        if (engine)
        {
            if (mVectorToTarget.length() > 5.0f)
            {
                if (pitch < -5 && WheelSpeed > 10)
                {
                    if (pitch < 0) pitch = -pitch;
                    brake = pitch * brakeforce / 90;
                    engine->autoSetAcc(0);
                }
                else
                {
                    if (WheelSpeed < maxvelo - 5)
                    {
                        brake = 0;
                        engine->autoSetAcc(power);
                    }
                    else if (WheelSpeed > maxvelo + 5)
                    {
                        brake = brakeforce / 3;
                        engine->autoSetAcc(0);
                    }
                    else
                    {
                        brake = 0;
                        engine->autoSetAcc(0);
                    }
                }
                return false;
            }
            else
            {
                engine->autoSetAcc(0);
                brake = brakeforce;
                return true;
            }
        }
        else
        {
            return true;
        }
        */
		
        //Don't do this! It's a crazy experiment! **** ACTUALLY make it configurable as a Retro Arcade Collision Avoidance
        //It does work but spawns the vehicles slightly in the air (can we get ground height at the four wheels xz positions and calc a position and quaternion from that?)
        //(The reason they spawn in the air is partly it adds an offset.y but mainly because resetPosition() doesn't try and rotate, it just finds the max ground height and offsets everything by that).
        //and we'd only want to do it if they've been stuck for a long time
        //and maybe progressively resolve with an increasing safety margin from say 1 to 20 instead of 20 straight away.
        //ACTUALLY - a smoother option is 1st get the target position (including safety margin) and frame by frame, smoothly move towards that until it's
        //reached or no further collisions are detected. We can even just use the target distance as an indication of whether any action is needed at all.
        //A better option is use the logic it has to see what direction is away from the collision, and if the vehicles are trying to move apart in that
        //direction (how to tell? have to combine drive direction with steering direction) after some delay, turn off collision detection temporarily between the two vehicles
#if 0
        beam->resolveCollisions(50.0f, false);//true);
#endif
}

/*
 * returns true if the two 2D lines intersect using the x and z coordinates
 * cosmic vole February 28 2017
 * 
 * */
bool lineIntersectionXZ(Vector3 start1, Vector3 end1, Vector3 start2, Vector3 end2, bool& intersectionInBounds, bool& linesAreParallel, Vector3& intersection)
{
    //TODO consider converting this to using homogenous coordinates and dot product - it may be a lot simpler!
    bool intersects = false;
    intersection = Vector3::ZERO;
    intersectionInBounds = false;
    linesAreParallel = false;
    float m1, m2, c1, c2; //z = mx + c
    float dx1 = end1.x - start1.x;
    float dz1 = end1.z - start1.z;
    float dx2 = end2.x - start2.x;
    float dz2 = end2.z - start2.z;
    bool line1Vertical = (dx1 == 0.0f);
    bool line2Vertical = (dx2 == 0.0f);
    bool line1Horizontal = (dz1 == 0.0f);
    bool line2Horizontal = (dz2 == 0.0f);
    float min1x = MIN(start1.x, end1.x);
    float max1x = MAX(start1.x, end1.x);
    float min1z = MIN(start1.z, end1.z);
    float max1z = MAX(start1.z, end1.z);
    float min2x = MIN(start2.x, end2.x);
    float max2x = MAX(start2.x, end2.x);
    float min2z = MIN(start2.z, end2.z);
    float max2z = MAX(start2.z, end2.z);
  
    static int counter = 0;
    counter++;
    if (counter > 16000)
    {
        counter = 1;
    }
  
    if (line1Horizontal && line2Horizontal)
    {
        //Both lines are horizontal
        linesAreParallel = true;
        intersects = start1.z == start2.z;
        if (intersects)
        {
            //See if the lines overlap
            //If they do, we'll return the first point of overlap
            if (start2.x >= min1x && start2.x <= max1x)
            {
                intersectionInBounds = true;
                intersection = start2;
            }
            else if (start1.x > min2x && start1.x <= max2x)
            {
                intersectionInBounds = true;
                intersection = start1;
            }
            else if (end2.x >= min1x && end2.x <= max1x)
            {
                intersectionInBounds = true;
                intersection = end2;
            }
            else if (end1.x > min2x && end1.x <= max2x)
            {
                intersectionInBounds = true;
                intersection = end1;
            }
            //intersects is true
            return true;
        }
        else
        {
            //lines are parallel and never intersect
            return false;
        }
    }// end if (line1Horizontal && line2Horizontal)
    
    if (line1Vertical && line2Vertical)
    {
        //Both lines are vertical
        linesAreParallel = true;
        intersects = start1.x == start2.x;
        if (intersects)
        {
            //See if the lines overlap
            //If they do, we'll return the first point of overlap
            if (start2.z >= min1z && start2.z <= max1z)
            {
                intersectionInBounds = true;
                intersection = start2;
            }
            else if (start1.z > min2z && start1.z <= max2z)
            {
                intersectionInBounds = true;
                intersection = start1;
            }
            else if (end2.z >= min1z && end2.z <= max1z)
            {
                intersectionInBounds = true;
                intersection = end2;
            }
            else if (end1.z > min2z && end1.z <= max2z)
            {
                intersectionInBounds = true;
                intersection = end1;
            }            
            //intersects is true
            return true;
        }
        else
        {
            //lines are parallel and never intersect
            return false;
        }
    }// end if (line1Vertical && line2Vertical)

    
    if (!line1Vertical)
    {
        //Line 1 is not vertical
        //Find equation of line 1
        m1 = dz1 / dx1;
        //z = mx + c => c = z - mx
        c1 = start1.z - m1 * start1.x;

        if (line2Vertical)
        {
            intersection.x = start2.x;
            intersection.z = m1 * intersection.x + c1;
            intersects = true;
        }
        else
        {
            //Neither line is vertical
            //Find equation of line 2
            m2 = dz2 / dx2;
            c2 = start2.z - m2 * start2.x;
            float dm = m1 - m2;
            float dc = c2 - c1;
            if (dm == 0.0f)
            {
                //The lines are parallel, at an angle to both axes
                linesAreParallel = true;
                if (dc == 0.0f)
                {
                    //All points are on the same line
                    //See if the lines overlap
                    //If they do, we'll return the first point of overlap
                    if (start2.x >= min1x && start2.x <= max1x)
                    {
                        intersectionInBounds = true;
                        intersection = start2;
                    }
                    else if (start1.x > min2x && start1.x <= max2x)
                    {
                        intersectionInBounds = true;
                        intersection = start1;
                    }
                    else if (end2.x >= min1x && end2.x <= max1x)
                    {
                        intersectionInBounds = true;
                        intersection = end2;
                    }
                    else if (end1.x > min2x && end1.x <= max2x)
                    {
                        intersectionInBounds = true;
                        intersection = end1;
                    }
                    //intersects is true
                    return true;                    
                }
                else
                {
                    //The lines are parallel and never intersect
                    return false;
                }
            }
            intersection.x = dc / dm;
            intersection.z = m1 * intersection.x + c1;
            intersects = true;
        }
    }
    else
    {
        //line 1 is vertical; line 2 is not
        //Find equation of line 2
        m2 = dz2 / dx2;
        c2 = start2.z - m2 * start2.x;
        
        intersection.x = start1.x;
        intersection.z = m2 * intersection.x + c2;
        intersects = true;
    }

    if (intersects)
    {
        //If we got here, the lines intersect at a single point and are not parallel
        //Now test if the point is within the specified bounds
        if (intersection.x >= min1x && intersection.x <= max1x &&
            intersection.z >= min1z && intersection.z <= max1z &&
            intersection.x >= min2x && intersection.x <= max2x &&
            intersection.z >= min2z && intersection.z <= max2z)
        {
            intersectionInBounds = true;
        }
    }
        //if ((intersectionInBounds && counter <= 160)) RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, 
        //"lineIntersectionXZ() start1:(" + TOSTRING(start1.x) + ", " + TOSTRING(start1.z) + ") end1:(" + TOSTRING(end1.x) + ", " + TOSTRING(end1.z) + ");\n start2:(" + 
        //TOSTRING(start2.x) + ", " + TOSTRING(start2.z) + ") end2:(" + TOSTRING(end2.x) + ", " + TOSTRING(end2.z) + "); Intersects: " + TOSTRING(intersects) + "\nInBoundary: " + TOSTRING(intersectionInBounds) + ".\n", "note.png");
    return intersects;
}

/*
 * returns true if the two 2D lines intersect using the x and z coordinates
 * cosmic vole August 6 2017
 * 
 * */
bool lineIntersectionXZ(Vector3 start1, Vector3 end1, bool allowOutOfBounds1, Vector3 start2, Vector3 end2, bool allowOutOfBounds2, bool& inBounds1, bool& inBounds2, bool& linesAreParallel, Vector3& intersection)
{
    //TODO consider converting this to using homogenous coordinates and dot product - it may be a lot simpler!
    bool intersects = false;
    intersection = Vector3::ZERO;
    inBounds1 = false;
    inBounds2 = false;
    linesAreParallel = false;
    float m1, m2, c1, c2; //z = mx + c
    float dx1 = end1.x - start1.x;
    float dz1 = end1.z - start1.z;
    float dx2 = end2.x - start2.x;
    float dz2 = end2.z - start2.z;
    bool line1Vertical = (dx1 == 0.0f);
    bool line2Vertical = (dx2 == 0.0f);
    bool line1Horizontal = (dz1 == 0.0f);
    bool line2Horizontal = (dz2 == 0.0f);
    float min1x = MIN(start1.x, end1.x);
    float max1x = MAX(start1.x, end1.x);
    float min1z = MIN(start1.z, end1.z);
    float max1z = MAX(start1.z, end1.z);
    float min2x = MIN(start2.x, end2.x);
    float max2x = MAX(start2.x, end2.x);
    float min2z = MIN(start2.z, end2.z);
    float max2z = MAX(start2.z, end2.z);
  

    if (line1Horizontal && line2Horizontal)
    {
        //Both lines are horizontal
        linesAreParallel = true;
        intersects = start1.z == start2.z;
        if (intersects)
        {
            //See if the line segments overlap
            //If they do, we'll return the first point of overlap
            if (start2.x >= min1x && start2.x <= max1x)
            {
                inBounds1 = inBounds2 = true;
                intersection = start2;
            }
            else if (start1.x > min2x && start1.x <= max2x)
            {
                inBounds1 = inBounds2 = true;
                intersection = start1;
            }
            else if (end2.x >= min1x && end2.x <= max1x)
            {
                inBounds1 = inBounds2 = true;
                intersection = end2;
            }
            else if (end1.x > min2x && end1.x <= max2x)
            {
                inBounds1 = inBounds2 = true;
                intersection = end1;
            }
            else
            {
                //The line segments do not overlap.
                if (allowOutOfBounds1)
                {
                    inBounds1 = false;
                    inBounds2 = true;
                    intersection = start2;
                }
                else if (allowOutOfBounds2)
                {
                    inBounds1 = true;
                    inBounds2 = false;
                    intersection = start1;
                }
                else
                {
                    //The arguments do not permit this case
                    //so we cannot choose an intersection point
                    inBounds1 = inBounds2 = false;
                    return false;
                }
            }
            //intersects is true
            return true;
        }
        else
        {
            //lines are parallel and never intersect
            return false;
        }
    }// end if (line1Horizontal && line2Horizontal)
    
    if (line1Vertical && line2Vertical)
    {
        //Both lines are vertical
        linesAreParallel = true;
        intersects = start1.x == start2.x;
        if (intersects)
        {
            //See if the lines overlap
            //If they do, we'll return the first point of overlap
            if (start2.z >= min1z && start2.z <= max1z)
            {
                inBounds1 = inBounds2 = true;
                intersection = start2;
            }
            else if (start1.z > min2z && start1.z <= max2z)
            {
                inBounds1 = inBounds2 = true;
                intersection = start1;
            }
            else if (end2.z >= min1z && end2.z <= max1z)
            {
                inBounds1 = inBounds2 = true;
                intersection = end2;
            }
            else if (end1.z > min2z && end1.z <= max2z)
            {
                inBounds1 = inBounds2 = true;
                intersection = end1;
            }
            else
            {
                //The line segments do not overlap.
                if (allowOutOfBounds1)
                {
                    inBounds1 = false;
                    inBounds2 = true;
                    intersection = start2;
                }
                else if (allowOutOfBounds2)
                {
                    inBounds1 = true;
                    inBounds2 = false;
                    intersection = start1;
                }
                else
                {
                    //The arguments do not permit this case
                    //so we cannot choose an intersection point
                    inBounds1 = inBounds2 = false;
                    return false;
                }
            }            
            //intersects is true
            return true;
        }
        else
        {
            //lines are parallel and never intersect
            return false;
        }
    }// end if (line1Vertical && line2Vertical)

    
    if (!line1Vertical)
    {
        //Line 1 is not vertical
        //Find equation of line 1
        m1 = dz1 / dx1;
        //z = mx + c => c = z - mx
        c1 = start1.z - m1 * start1.x;

        if (line2Vertical)
        {
            intersection.x = start2.x;
            intersection.z = m1 * intersection.x + c1;
            intersects = true;
        }
        else
        {
            //Neither line is vertical
            //Find equation of line 2
            m2 = dz2 / dx2;
            c2 = start2.z - m2 * start2.x;
            float dm = m1 - m2;
            float dc = c2 - c1;
            if (dm == 0.0f)
            {
                //The lines are parallel, at an angle to both axes
                linesAreParallel = true;
                if (dc == 0.0f)
                {
                    //All points are on the same line
                    //See if the lines overlap
                    //If they do, we'll return the first point of overlap
                    if (start2.x >= min1x && start2.x <= max1x)
                    {
                        inBounds1 = inBounds2 = true;
                        intersection = start2;
                    }
                    else if (start1.x > min2x && start1.x <= max2x)
                    {
                        inBounds1 = inBounds2 = true;
                        intersection = start1;
                    }
                    else if (end2.x >= min1x && end2.x <= max1x)
                    {
                        inBounds1 = inBounds2 = true;
                        intersection = end2;
                    }
                    else if (end1.x > min2x && end1.x <= max2x)
                    {
                        inBounds1 = inBounds2 = true;
                        intersection = end1;
                    }
                    else
                    {
                        //The line segments do not overlap.
                        if (allowOutOfBounds1)
                        {
                            inBounds1 = false;
                            inBounds2 = true;
                            intersection = start2;
                        }
                        else if (allowOutOfBounds2)
                        {
                            inBounds1 = true;
                            inBounds2 = false;
                            intersection = start1;
                        }
                        else
                        {
                            //The arguments do not permit this case
                            //so we cannot choose an intersection point
                            inBounds1 = inBounds2 = false;
                            return false;
                        }
                    }                    
                    //intersects is true
                    return true;                    
                }
                else
                {
                    //The lines are parallel and never intersect
                    return false;
                }
            }
            intersection.x = dc / dm;
            intersection.z = m1 * intersection.x + c1;
            intersects = true;
        }
    }
    else
    {
        //line 1 is vertical; line 2 is not
        //Find equation of line 2
        m2 = dz2 / dx2;
        c2 = start2.z - m2 * start2.x;
        
        intersection.x = start1.x;
        intersection.z = m2 * intersection.x + c2;
        intersects = true;
    }

    if (intersects)
    {
        //If we got here, the lines intersect at a single point and are not parallel
        //Now test if the point is within the specified bounds
        if (intersection.x >= min1x && intersection.x <= max1x &&
            intersection.z >= min1z && intersection.z <= max1z)
        {
            inBounds1 = true;
        }
        if (intersection.x >= min2x && intersection.x <= max2x &&
            intersection.z >= min2z && intersection.z <= max2z)
        {
            inBounds2 = true;
        }
        if (!inBounds1 && !allowOutOfBounds1)
            intersects = false;
        if (!inBounds2 && !allowOutOfBounds2)
            intersects = false;
    }
        //if ((intersectionInBounds && counter <= 160)) RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, 
        //"lineIntersectionXZ() start1:(" + TOSTRING(start1.x) + ", " + TOSTRING(start1.z) + ") end1:(" + TOSTRING(end1.x) + ", " + TOSTRING(end1.z) + ");\n start2:(" + 
        //TOSTRING(start2.x) + ", " + TOSTRING(start2.z) + ") end2:(" + TOSTRING(end2.x) + ", " + TOSTRING(end2.z) + "); Intersects: " + TOSTRING(intersects) + "\nInBoundary: " + TOSTRING(intersectionInBounds) + ".\n", "note.png");
    return intersects;
}

/*
 * returns true if the 2D line (given by the X and Z coordinates of lineStart and lineEnd) intersects or is contained within the 2D quadrilateral
 * defined by the X and Z coordinates of rectPoint1 to rectPoint4
 * cosmic vole May 20 2017
 * 
 * */
bool lineIntersectsQuad(Ogre::Vector3 lineStart, Ogre::Vector3 lineEnd, Ogre::Vector3 quadPoint1, Ogre::Vector3 quadPoint2, Ogre::Vector3 quadPoint3, Ogre::Vector3 quadPoint4)
{
    bool intersectionInBounds;
    bool linesAreParallel;
    Vector3 intersection;
    //We require the quad points are listed in a circuit around the shape. If they're in the wrong order, we'd end up testing a diagonal instead of an edge. cosmic vole May 20 2017.
    if ((lineIntersectionXZ(quadPoint1, quadPoint2, lineStart, lineEnd, intersectionInBounds, linesAreParallel, intersection) && intersectionInBounds) ||
         (lineIntersectionXZ(quadPoint2, quadPoint3, lineStart, lineEnd, intersectionInBounds, linesAreParallel, intersection) && intersectionInBounds) ||
         (lineIntersectionXZ(quadPoint3, quadPoint4, lineStart, lineEnd, intersectionInBounds, linesAreParallel, intersection) && intersectionInBounds) ||
         (lineIntersectionXZ(quadPoint4, quadPoint1, lineStart, lineEnd, intersectionInBounds, linesAreParallel, intersection) && intersectionInBounds))
         return true;
    //Finally, test if both the points are contained completely within the quad
    return PointIsInsideQuad(lineStart, quadPoint1, quadPoint2, quadPoint3, quadPoint4) &&
        PointIsInsideQuad(lineEnd, quadPoint1, quadPoint2, quadPoint3, quadPoint4);
}

bool QuadsIntersect(Ogre::Vector3 quad1Point1, Ogre::Vector3 quad1Point2, Ogre::Vector3 quad1Point3, Ogre::Vector3 quad1Point4, Ogre::Vector3 quad2Point1, Ogre::Vector3 quad2Point2, Ogre::Vector3 quad2Point3, Ogre::Vector3 quad2Point4)
{
    //First test if any vertex of each quad is contained inside the other quad. This will cover the case where one quad is completely inside the other as well.
    if (PointIsInsideQuad(quad1Point1, quad2Point1, quad2Point2, quad2Point3, quad2Point4) ||
        PointIsInsideQuad(quad1Point2, quad2Point1, quad2Point2, quad2Point3, quad2Point4) ||
        PointIsInsideQuad(quad1Point3, quad2Point1, quad2Point2, quad2Point3, quad2Point4) ||
        PointIsInsideQuad(quad1Point4, quad2Point1, quad2Point2, quad2Point3, quad2Point4) ||
        PointIsInsideQuad(quad2Point1, quad1Point1, quad1Point2, quad1Point3, quad1Point4) ||
        PointIsInsideQuad(quad2Point2, quad1Point1, quad1Point2, quad1Point3, quad1Point4) ||
        PointIsInsideQuad(quad2Point3, quad1Point1, quad1Point2, quad1Point3, quad1Point4) ||
        PointIsInsideQuad(quad2Point4, quad1Point1, quad1Point2, quad1Point3, quad1Point4))
        return true;
    //Now we have to test whether any of the sides intersect    
    return (lineIntersectsQuad(quad1Point1, quad1Point2, quad2Point1, quad2Point2, quad2Point3, quad2Point4) ||
        lineIntersectsQuad(quad1Point2, quad1Point3, quad2Point1, quad2Point2, quad2Point3, quad2Point4) ||
        lineIntersectsQuad(quad1Point3, quad1Point4, quad2Point1, quad2Point2, quad2Point3, quad2Point4) ||
        lineIntersectsQuad(quad1Point4, quad1Point1, quad2Point1, quad2Point2, quad2Point3, quad2Point4));
}

bool PointIsInsideQuad(Ogre::Vector3 point, Ogre::Vector3 quadPoint1, Ogre::Vector3 quadPoint2, Ogre::Vector3 quadPoint3, Ogre::Vector3 quadPoint4)
{
    int sidesCrossed = 0;
    for (int i=0; i<4; i++)
    {
        Vector3 v0 = quadPoint4;
        Vector3 v1 = quadPoint1;
        //TODO consider replacing this ugliness with a variable argument list!
        switch (i)
        {
            case 0:
            v0 = quadPoint1;
            v1 = quadPoint2;
            break;
            case 1:
            v0 = quadPoint2;
            v1 = quadPoint3;
            break;
            case 2:
            v0 = quadPoint3;
            v1 = quadPoint4;
            break;
            default:
            //Already initialized (to keep the compiler happy)
            break;
        }
        if (((v0.z <= point.z) && (v1.z > point.z)) || ((v0.z > point.z) && (v1.z <= point.z)))
        {
            float m = (point.z - v0.z) / (v1.z - v0.z);
            float intersection = v0.x + m * (v1.x - v0.x);
            if (point.x < intersection)
            {
                sidesCrossed++;
            }
        }
    }
    return sidesCrossed % 2 != 0;
}

float GetLaneOverlap(float& leftOverlap, float& rightOverlap, Ogre::Vector3 quadPoint1, Ogre::Vector3 quadPoint2, Ogre::Vector3 quadPoint3, Ogre::Vector3 quadPoint4, Ogre::Vector3 laneLeft, Ogre::Vector3 laneRight, Ogre::Vector3 laneCenter, bool debug)
{
    if (laneCenter == Vector3::ZERO)
        laneCenter = (laneLeft + laneRight) * 0.5f;
    leftOverlap = rightOverlap = 0.0f;
    Vector3 acrossLane = laneRight - laneLeft;
    float laneWidth = acrossLane.length();
    if (laneWidth == 0.0f)
        return 0.0f;
    float halfWidth = laneWidth * 0.5f;
    acrossLane.normalise();
    //TODO We should probably only be projecting the vehicle's rear two points (closest two points?) onto the lane grid quads' rear edge, and the vehicle's front two points onto the front edge,
    //TODO otherwise a change of direction can cause the vehicle to falsely seem to cross the lane boundary.
    //We project all the quad's points (relative to the lane center) onto acrossLane and find the left most and right most
    float dot1 = (quadPoint1 - laneCenter).dotProduct(acrossLane);
    float dot2 = (quadPoint2 - laneCenter).dotProduct(acrossLane);
    float dot3 = (quadPoint3 - laneCenter).dotProduct(acrossLane);
    float dot4 = (quadPoint4 - laneCenter).dotProduct(acrossLane);
    //BUG! TODO This isn't quite right - if points are way past the edges of the lane they'll be counted as a big overlap! Need to calc actual edge overlaps
    float quadStart = dot1;
    float quadEnd = dot1;
    if (dot2 < quadStart)
        quadStart = dot2;
    else if (dot2 > quadEnd)
        quadEnd = dot2;
    if (dot3 < quadStart)
        quadStart = dot3;
    else if (dot3 > quadEnd)
        quadEnd = dot3;
    if (dot4 < quadStart)
        quadStart = dot4;
    else if (dot4 > quadEnd)
        quadEnd = dot4;
    //Now clip the quad's bounds to the lane edges
    if (quadStart < -halfWidth)
        quadStart = -halfWidth;
    if (quadStart > halfWidth)
        quadStart = halfWidth;
    if (quadEnd < -halfWidth)
        quadEnd = -halfWidth;
    if (quadEnd > halfWidth)
        quadEnd = halfWidth;
    
    if (quadStart < 0.0f) //0.0 represents the center of the lane here
    {
        float leftOverlapEnd = (quadEnd > 0.0f) ? 0.0f : quadEnd;
        leftOverlap = fabsf(leftOverlapEnd - quadStart);
    }
    else
    {
        leftOverlap = 0.0f;
    }
    if (quadEnd >= 0.0f)
    {
        float rightOverlapStart = (quadStart < 0.0f) ? 0.0f : quadStart;
        rightOverlap = fabsf(quadEnd - rightOverlapStart);
    }
    else
    {
        rightOverlap = 0.0f;
    }
    
    float totalOverlap = fabsf(quadEnd - quadStart);
    totalOverlap = totalOverlap / laneWidth;
    return totalOverlap;
}


float GetLaneOverlap(Ogre::Vector3 quadPoint1, Ogre::Vector3 quadPoint2, Ogre::Vector3 quadPoint3, Ogre::Vector3 quadPoint4, Ogre::Vector3 laneRearLeft, Ogre::Vector3 laneRearRight, Ogre::Vector3 laneFrontLeft, Ogre::Vector3 laneFrontRight)
{
    //1. We draw a line down the center of the lane (by connecting the midpoints of the front and rear edges).
    //2. We pick a corner of our vehicle. We cast a line from it to the closest point on the lane center line. It will meet the center line at 90 degrees (unless clamping has occurred).
    //3. We find the other intersection points between the line in 2. and the 4 sides of the vehicle. At most 3 sides can intersect, but there should only really be 2 points (except if the line is exactly along a side - then we want the corners only), but rounding errors may make it seem like more.
    //4. We clamp the points in 3. to the lane edges. We need to find the start and end point.
    //5. We add the smallest gap (left or right side of the lane) from the lane edge to our point(s) to the distance between our points (if more than one). This will be our overlap.
    //6. We repeat this procedure for the other 3 corners of the vehicle. In the unusual case where the lane quad's completely contained inside a vehicle, clamping to the lane corners should make it work (TODO how to do this and still be sure they're inside the vehicle?).
    //7. We will need to either average the overlap amounts or, preferably, take the maximum.
    Vector3 centerRear = (laneRearLeft + laneRearRight) * 0.5f;
    Vector3 centerFront = (laneFrontLeft + laneFrontRight) * 0.5f;
    Vector3 pointOnCenter1 = getClosestPointOnLine(centerRear, centerFront, quadPoint1, true);
    bool intersects, inBounds1, inBounds2, linesAreParallel;
    //If these intersections aren't in the lane's bounds, we need to clamp them! They WON'T be in the bounds of quadPoint1 -> pointOnCenter1.
    Vector3 pointOnLaneLeftEdge, pointOnLaneRightEdge;
    intersects = lineIntersectionXZ(quadPoint1, pointOnCenter1, true, laneRearLeft, laneFrontLeft, true, inBounds1, inBounds2, linesAreParallel, pointOnLaneLeftEdge);
    if (!intersects)
        pointOnLaneLeftEdge = getClosestPointOnLine(laneRearLeft, laneFrontLeft, pointOnCenter1, true);
    else if (!inBounds2)
        pointOnLaneLeftEdge = getClosestPointOnLine(laneRearLeft, laneFrontLeft, pointOnLaneLeftEdge, true);
    intersects = lineIntersectionXZ(quadPoint1, pointOnCenter1, true, laneRearRight, laneFrontRight, true, inBounds1, inBounds2, linesAreParallel, pointOnLaneRightEdge);
    if (!intersects)
        pointOnLaneRightEdge = getClosestPointOnLine(laneRearRight, laneFrontRight, pointOnCenter1, true);
    else if (!inBounds2)
        getClosestPointOnLine(laneRearRight, laneFrontRight, pointOnLaneRightEdge, true);
    //! TODO what if quadPoint1 itself is outside the quad front / rear? Clamping to the front / rear edge might not work if the clamped point is outside the vehicle!
    
    //TODO if these intersections aren't in the vehicle's bounds, we need reject them! They WON'T be in the bounds of quadPoint1 -> pointOnCenter1.
    Vector3 quadOtherPoint1, quadOtherPoint2, quadOtherPoint3;
    bool gotOtherPt1 = lineIntersectionXZ(quadPoint1, pointOnCenter1, true, quadPoint2, quadPoint3, false, inBounds1, inBounds2, linesAreParallel, quadOtherPoint1);
    bool gotOtherPt2 = lineIntersectionXZ(quadPoint1, pointOnCenter1, true, quadPoint3, quadPoint4, false, inBounds1, inBounds2, linesAreParallel, quadOtherPoint2);
    bool gotOtherPt3 = lineIntersectionXZ(quadPoint1, pointOnCenter1, true, quadPoint4, quadPoint1, false, inBounds1, inBounds2, linesAreParallel, quadOtherPoint3);

    
    //float centerLength = (centerFront - centerRear).length();
    //float alongCenterLine = (centerLength == 0.0f) ? 0.0f : ((pointOnCenter - centerRear).length() / centerLength);
    
    return 0.0f;
}

/*
void findIntersectionZLimits(Vector3& intersection, float& closestIntersectionZ, float& furthestIntersectionZ, float& agentDistAtClosest, float& otherDistAtClosest)
{
    if (intersection.z < closestIntersectionZ)
    {
        closestIntersectionZ = intersection.z;
        agentDistAtClosest = (intersection - agentAvgPosition).length();
        otherDistAtClosest = (intersection - otherAvgPosition).length();
    }
    if (intersectionFL.z > furthestIntersectionZ)
    {
        furthestIntersectionZ = intersection.z);
        //agentDistAtFurthest = (intersection - agentAvgPosition).length();
        //otherDistAtFurthest = (intersection - otherAvgPosition).length();
    }
}
*/

Vector3 rotate(Vector3& point, Quaternion rotation, Vector3& centre)
{
    Vector3 newPoint = point;
    newPoint -= centre;
    newPoint = rotation * newPoint;
    newPoint += centre;
    return newPoint;
}

Vector3 getClosestPointOnLine(Vector3& A, Vector3& B, Vector3& p, bool clampToBounds)
{
    Vector3 AP = p - A;
    Vector3 AB = B - A;
    float ab2 = AB.x * AB.x + AB.y * AB.y;
    float ap_ab = AP.x * AB.x + AP.y * AB.y;
    float t = ap_ab / ab2;
    if (clampToBounds)
    {
         if (t < 0.0f) t = 0.0f;
         else if (t > 1.0f) t = 1.0f;
    }
    Vector3 closest = A + AB * t;
    return closest;
}

//PID Control for steering - based on http://www.habrador.com/tutorials/pid-controller/1-car-follow-path/
//cosmic vole April 11 2017
float VehicleAI::GetPIDControlFactor(float errorAcrossTrack, float dt)
{
    //The PID control factor to return
    float factor = 0.0f;
    //P
    factor = PID_P * errorAcrossTrack;

    //I
    sumErrorsAcrossTrack += dt * errorAcrossTrack;

    float averageAmount = 20.0f;//40.0f;//20.0f;
    sumErrorsAcrossTrack = sumErrorsAcrossTrack + ((errorAcrossTrack - sumErrorsAcrossTrack) / averageAmount);

    factor += PID_I * sumErrorsAcrossTrack;

    //D
    float d_dt_error = (errorAcrossTrack - lastErrorAcrossTrack) / dt;

    factor += PID_D * d_dt_error;

    lastErrorAcrossTrack = errorAcrossTrack;

    return factor;    
}

#define DEBUG_COLLISION_AVOIDANCE 1

//For debugging - adds a sphere to the scene and the supplied vector, used to debug collision avoidance - cosmic vole
void debugCollision(int& reuseNode, std::vector<SceneNode*>& debugCol, Vector3 position, std::string materialName)
{
#if  DEBUG_COLLISION_AVOIDANCE
    SceneNode *node;
    Entity *entity;
    if (reuseNode >= 0 && reuseNode < debugCol.size())
    {
        node = debugCol.at(reuseNode);
        entity = (Entity *)node->getAttachedObject(0);
        reuseNode++;
    }
    else
    {
        node = gEnv->sceneManager->getRootSceneNode()->createChildSceneNode();
        entity = gEnv->sceneManager->createEntity(SceneManager::PT_SPHERE);
        node->attachObject(entity);
        reuseNode = -1;
    }

    //float radius = entityAgent->getBoundingRadius();
    float entityWidth = 2.0f * entity->getBoundingRadius();//100.0f;//entityAgent->getBoundingBox().getSize().x;
    //float width = (agentFrontRight - agentFrontLeft).length();
    //float length = (agentFrontLeft - agentRearLeft).length();
    //nodeAgent->setScale(Vector3(width/entityWidth,width*0.5f/entityWidth,length/entityWidth));
    node->setScale(Vector3(0.15f/entityWidth,0.15f/entityWidth,0.15f/entityWidth));
    node->setPosition(position);//agentPosWorld_);
    //nodeAgent->setOrientation(agentOrientation);
    entity->setMaterialName(materialName);
    if (reuseNode < 0)
        debugCol.push_back(node);
#endif                                
}

//For debugging - adds a sphere to the scene and the supplied vector, used to debug collision avoidance - cosmic vole
SceneNode* debugCollision(SceneNode* node, Vector3 position, std::string materialName)
{
#if  DEBUG_COLLISION_AVOIDANCE
    Entity *entity;
    if (node)
    {
        entity = (Entity *)node->getAttachedObject(0);
    }
    else
    {
        node = gEnv->sceneManager->getRootSceneNode()->createChildSceneNode();
        entity = gEnv->sceneManager->createEntity(SceneManager::PT_SPHERE);
        node->attachObject(entity);
    }
    float entityWidth = 2.0f * entity->getBoundingRadius();
    node->setScale(Vector3(0.15f/entityWidth,0.15f/entityWidth,0.15f/entityWidth));
    node->setPosition(position);
    entity->setMaterialName(materialName);
    //SubEntity *subEntity = entity->getSubEntity(0);
    //if (subEntity)
    //    subEntity->setMaterialName(materialName);
#endif
    return node;
}


void debugPoints(Vector3 point1, Vector3 point2, float minY = 0.5f)
{
    /*
    if (point1.y < minY)
        point1.y = minY;
    if (point2.y < minY)
        point2.y = minY;
    if (point1.y > 10.0f)
        point1.y = 10.0f;
    if (point2.y > 10.0f)
        point2.y = 10.0f;
         */
    static SceneNode *node1 = nullptr;
    static SceneNode *node2 = nullptr;
    static MaterialPtr material1;
    static MaterialPtr material2;
    Entity *entity1, *entity2;
    if (node1 == nullptr)
    {
        material1 = MaterialManager::getSingleton().create("debugMaterialWaypt1", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        material1->setSelfIllumination(0.1f,0.0f,0.8f);
        material1->setAmbient(0.1f,0.0f,0.8f);
        material1->setDiffuse(0.1f,0.0f,0.8f,0.9f);
        
        material2 = MaterialManager::getSingleton().create("debugMaterialWaypt2", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        material2->setSelfIllumination(0.0f,0.5f,0.8f);
        material2->setAmbient(0.0f,0.5f,0.8f);
        material2->setDiffuse(0.0f,0.5f,0.8f,0.9f);
        
        node1 = gEnv->sceneManager->getRootSceneNode()->createChildSceneNode();
        entity1 = gEnv->sceneManager->createEntity(SceneManager::PT_SPHERE);
        node1->attachObject(entity1);
        entity1->setMaterialName("debugMaterialWaypt1");
        
        node2 = gEnv->sceneManager->getRootSceneNode()->createChildSceneNode();
        entity2 = gEnv->sceneManager->createEntity(SceneManager::PT_SPHERE);
        node2->attachObject(entity2);
        entity2->setMaterialName("debugMaterialWaypt2");
    }
    else
    {
        entity1 = (Entity *)node1->getAttachedObject(0);
        entity2 = (Entity *)node2->getAttachedObject(0);
    }
    static float entityWidth = 2.0f * entity1->getBoundingRadius();//100.0f;//entityAgent->getBoundingBox().getSize().x;
    node1->setScale(Vector3(0.15f/entityWidth,0.15f/entityWidth,0.15f/entityWidth));
    node1->setPosition(point1);
    
    entityWidth = 2.0f * entity2->getBoundingRadius();//100.0f;//entityAgent->getBoundingBox().getSize().x;
    node2->setScale(Vector3(0.15f/entityWidth,0.15f/entityWidth,0.15f/entityWidth));
    node2->setPosition(point2);
}

#endif // USE_ANGELSCRIPT
