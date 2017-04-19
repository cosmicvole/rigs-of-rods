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

#include "VehicleAI.h"
#include "Beam.h"
#include "BeamFactory.h"
#include "BeamEngine.h"
#include "GUI_GameConsole.h"

#include "MainThread.h"
//cosmic vole added AI driver model November 21 2016
#include "CharacterFactory.h"
#include "PlayerColours.h"

#include "scriptdictionary/scriptdictionary.h"

using namespace Ogre;

VehicleAI::VehicleAI(Beam* b)
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
    stuck_reset_delay = 11.0;//Time in seconds until a stuck vehicle resets. Was 12
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
    accel_velocity = b->getVelocity();
    average_accel = Vector3::ZERO;
    //cosmic vole added AI driver model November 21 2016
    character = nullptr;
    //PID control pararmeters - cosmic vole April 11 2017
    PID_P = 0.075f;//0.065f;//2.0f;//7.5f;//3.9f;//0.001 - it steers, VERY smoothly but VERY slowly. 1.5f;//13.0f;//2.5f;//0.08f;//0.02f;//0.23f; //0.5 0 0 gives huge zig zags - actually 4.0 is better, 20.0 oscillates intermittenly a lot (same as 1500! maxed out), 15 a bit!, 0.01 0 0 it hardly steers. 0.0001 it goes straight ahead pretty much.
    PID_I = 0.015f;//0.3f;//0.001f;//.007f .003f;//0.001f;//0.16f;
    PID_D = 0.03f;//0.0005f;//0.0005f 001f;//0.03f;
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
    //0.075 0.015 0.03 maxsteerspeed = 11.0 Mostly good but crashed on bend just before pit lane entrance. Game was lagging a lot. TODO Also check "off course" logic.
    
    lastErrorAcrossTrack = 0.0f;
    sumErrorsAcrossTrack = 0.0f;
    lastWaypointNavigated = Vector3::ZERO;

    beam = b;
}

VehicleAI::~VehicleAI()
{
}

void VehicleAI::SetActive(bool value)
{
    is_enabled = value;
    //cosmic vole added AI Character driver model November 21 2016
    if (value && BSETTING("ShowAIDrivers", true) && character == nullptr)
    {
        int aiColour = PlayerColours::getSingleton().getRandomColourNum();//Ogre::ColourValue(frand(), frand(), frand(), 1.0f);
        character = CharacterFactory::getSingleton().createAIInstance(beam->trucknum, (int)aiColour);
        character->setBeamCoupling(true, beam);
    }
    else if (!value && character != nullptr)
    {
        CharacterFactory::getSingleton().removeAIInstance(beam->trucknum);
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
}

void VehicleAI::AddWaypoints(AngelScript::CScriptDictionary& d)
{
    for (auto dict : d.dict)
    {
        String id = dict.first;
        Vector3* point = (Vector3 *)dict.second.valueObj;
        AddWaypoint(id, *(point));
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
    
    //As position is actually a key in the hash maps, we need to renumber all the waypoints
    //that come after the one we've just inserted, although we don't rename them. cosmic vole March 7 2017
    for (int i=free_waypoints; i>position; i--)
    {
        int h = i-1;
        Vector3 pointToMove = waypoints[h];
        waypoints[i] = pointToMove;
        String idToMove = waypoint_names[h];
        float speedToMove = waypoint_speed[h];
        float powerToMove = waypoint_power[h];
        int eventToMove = waypoint_events[h];
        waypoint_ids.emplace(idToMove, i);
        waypoint_names.emplace(i, idToMove);
        if (speedToMove)
            waypoint_speed[i] = speedToMove;
        if (powerToMove)
            waypoint_power[i] = powerToMove;
        if (eventToMove)
            waypoint_events[i] = eventToMove;
    }
    waypoints[position] = point;
    waypoint_ids[id] = position;
    waypoint_names[position] = id;
    
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
            break;
        case AI_POWER:
            waypoint_power.emplace(waypointid, value);
            break;
        default:
            break;
        }
    }
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

//cosmic vole March 16 2017
void VehicleAI::FindRoadEdgePoints(Vector3& location, Vector3& leftEdgePoint, Vector3& rightEdgePoint)
{
    float minDist = -1.0f;
    int minDistIndex = -1;
    int start = 0;
    int end = road_edge_points_left.size()-1;
    if (end < 0)
    {
        leftEdgePoint = rightEdgePoint = Vector3(Vector3::ZERO);
        return;
    }
    for (int i=start; i<=end; i++)
    //while (true)
    {
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
        Vector3& curLeftStart = road_edge_points_left[i];
        Vector3& curRightStart = road_edge_points_right[i];
        
        if (end == start)
        {
            leftEdgePoint = curLeftStart;
            rightEdgePoint = curRightStart;
            return;
        }
        
        Vector3 atStart = getClosetPointOnLine(curLeftStart, curRightStart, location, true);
        float distStart = atStart.distance(location);
        
        Vector3& curLeftEnd = road_edge_points_left[j];
        Vector3& curRightEnd = road_edge_points_right[j];
        Vector3 atEnd = getClosetPointOnLine(curLeftEnd, curRightEnd, location, true);
        float distEnd = atEnd.distance(location);
        
        float avgDist = distStart + distEnd;
        if (avgDist < minDist || minDist < 0.0f)
        {
            minDist = avgDist;
            minDistIndex = i;
        }
         
        /* Binary search stuff - won't work as parts of the track may get closer and further away erratically - cosmic vole March 29 2017
        if (end - start == 1)
        {
            //No further points to search. Just interpolate between the four edge points. TODO one day this could be improved with curves. cosmic vole March 29 2017
            Vector3 midpointStart = curLeftStart + ((curRightStart - curLeftStart) * 0.5f);
            Vector3 midpointEnd = curLeftEnd + ((curRightEnd - curRightStart) * 0.5f);
            Vector3 closestTrackCenter = getClosetPointOnLine(midpointStart, midpointEnd, location);
            leftEdgePoint = getClosetPointOnLine(curLeftStart, curLeftEnd, closestTrackCenter);
            rightEdgePoint = getClosetPointOnLine(curRightStart, curRightEnd, closestTrackCenter);
            return;
        }
        else
        {
            //Binary search
            int midIndex = (end + start) / 2;
            if (distEnd < distStart)
            {
                start = midIndex;
            }
            else
            {
                end = midIndex;
            }
        }
*/
    }
    
    //If we got here, we've found the closest four edge points on the track and we'll use linear interpolation along the track sides to get the results
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
        Vector3& curLeftStart = road_edge_points_left[i];
        Vector3& curRightStart = road_edge_points_right[i];
        Vector3& curLeftEnd = road_edge_points_left[j];
        Vector3& curRightEnd = road_edge_points_right[j];
        
        //We find our closest track edge points by finding the closest point on the track center line to the supplied location
        //and then returning the closest points on the left and right side of the track to that point, where the left and right
        //side of the track are just straight lines between the edge points we have already found.
        //At some point this could be improved by fitting curves to the edge points instead of straight lines, but that's probably
        //OTT I would think. cosmic vole March 29 2017
        Vector3 midpointStart = curLeftStart + ((curRightStart - curLeftStart) * 0.5f);
        Vector3 midpointEnd = curLeftEnd + ((curRightEnd - curRightStart) * 0.5f);
        Vector3 closestTrackCenter = getClosetPointOnLine(midpointStart, midpointEnd, location, true);
        
        leftEdgePoint = getClosetPointOnLine(curLeftStart, curLeftEnd, closestTrackCenter, true);
        rightEdgePoint = getClosetPointOnLine(curRightStart, curRightEnd, closestTrackCenter, true);        
    
}

void VehicleAI::updateWaypoint()
{
    if (beam == BeamFactory::getSingleton().getCurrentTruck()) RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, "Reached waypoint: " + waypoint_names[current_waypoint_id] + " Truck: " + TOSTRING(beam->trucknum), "note.png");
    
    lastWaypointNavigated = current_waypoint; // We need this as waypoints can now be skipped. cosmic vole April 12 2017
    
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

    float speed = waypoint_speed[current_waypoint_id-1];
    if (speed)
        maxspeed = speed;

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
            float speed3 = waypoint_speed[current_waypoint_id-1];
            if (speed3)
                speed2 += speed3;
            else
                speed2 += speed2;

            float power3 = waypoint_power[current_waypoint_id-1];
            if (power3)
                power2 += power3;
            else
                power2 += power2;
        }
        current_waypoint = waypoints[current_waypoint_id];
        
    }
    while (((velocity >= 20.0f/*15.0f*/ || (/*time_since_last_collision_avoid >= 5.0f &&*/ time_since_last_collision_avoid < /*8.0f*/3.0f)) && agentPos.distance(current_waypoint) < 20.0f/*25.0f*/) || ((velocity >= 25.0f/*20.0f*/ /*|| time_since_last_collision_avoid < *1.9f 2.1f 3.5f*5.0f*/) && agentPos.distance(current_waypoint) < 30.0f/*35.0f*/));
    
    //Use averaged speed and power values if we have skipped waypoints. cosmic vole. TODO may be safer to take minimum?
    if (skipped > 0)
    {
        maxspeed = speed2 / (skipped + 1);
        acc_power = power2 / (skipped + 1);
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
            trackCentre = getClosetPointOnLine(lastWaypointNavigated, current_waypoint, mAgentPosition, false);
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
            if (abs(distAlongTrack) < 6.5f && abs(distAcrossTrack) < 10.0f) //TODO check actual track bounds and also checkpoint locations
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

    Vector3 TargetPosition = current_waypoint;
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

    //NEW CODE with Slerp to smooth out direction changes - cosmic vole March 11 2017
    //float waypoint_dist = current_waypoint.distance(mAgentAbsPosition);
    //if (false)//
    //if (waypoint_dist > 7.5f)
    if (false) //TODO this TOTALLY doesn't work even with a REALLY slow turn speed value, the car laps slowly and spins out / rolls
    {
        turn_time += dt;
        Vector3 agentDir = beam->getDirection();
        agentDir.normalise();
        //Get agent current rotation - the same as beam->getHeadingDirectionAngle() except this handles where the truck doesn't have the usual camera nodes - cosmic vole
        float agentRotation = atan2(agentDir.dotProduct(Vector3::UNIT_X), (agentDir).dotProduct(-Vector3::UNIT_Z));
        //Turn it into a quaternion for current agent orientation
        Quaternion agentOrientation = Quaternion(Radian(agentRotation), Vector3::NEGATIVE_UNIT_Y);
        agentOrientation.normalise();//TODO does Ogre do this for us???
        Vector3 agentTargetVector = current_waypoint - beam->getPosition();
        Vector3 agentTargetHeading = agentTargetVector;
        agentTargetHeading.normalise();        
        //Get the desired agent rotation to face the target - cosmic vole
        float agentTargetRotation = atan2(agentTargetHeading.dotProduct(Vector3::UNIT_X), agentTargetHeading.dotProduct(-Vector3::UNIT_Z));
        //Turn it into a quaternion for desired agent orientation to target
        Quaternion agentTargetOrientation = Quaternion(Radian(agentTargetRotation), Vector3::NEGATIVE_UNIT_Y);
        agentTargetOrientation.normalise();
        float turnSpeed;//TODO increase turn speed and even turn off interpolation if waypoint is very close
/* works, just about - about a 2:50 first lap for Taunus
        if (waypoint_dist < 15.0f)
            turnSpeed = 2.9f;
        else if (waypoint_dist < 25.0f)
            turnSpeed = 2.8f;//2.7 3.0;//4.0f;
        else if (waypoint_dist < 35.0f)
            turnSpeed = 1.6f;//1.4 2.5f;//3.0f;
        else turnSpeed = 0.7f;//0.7 0.4f;//0.8f;
        if (beam->getVelocity().length() >= 24.0f)
            turnSpeed *= 0.6f;//0.5f;
        if (beam->getVelocity().length() >= 20.0f)
            turnSpeed *= 0.75f;
        else if (beam->getVelocity().length() >= 15.0f)
            turnSpeed *= 0.8f;//0.9f;
*/
        if (waypoint_dist < 21.0f)//19, 26, 36 worked ok on first lap but not second
            turnSpeed = 2.9f;//3.1f;
        else if (waypoint_dist < 27.0f)
            turnSpeed = 2.7f;//3.0f;//2.7 3.0;//4.0f;
        else if (waypoint_dist < 37.0f)
            turnSpeed = 2.4f;//1.8f;//1.4 2.5f;//3.0f;
        else turnSpeed = 1.9f;//1.1f;//0.7 0.4f;//0.8f;
        /*
        if (beam->getVelocity().length() >= 24.0f)
            turnSpeed *= 0.54f;//0.56 0.5f;
        if (beam->getVelocity().length() >= 20.0f)
            turnSpeed *= 0.73f;//0.75
        else if (beam->getVelocity().length() >= 15.0f)
            turnSpeed *= 0.8f;//0.9f;
        */
        turnSpeed *= 50.0f;//40.0f 0.4f;//3.0f; * 10.0f is actually almost acceptable. Anything lower causes general slowness, sloppiness and zigzagging!
        float agentRotProgress = turnSpeed * turn_time; //(time-agentTurnStartTime);
        if (agentRotProgress > 1.0f)
            agentRotProgress = 1.0f;
        Quaternion newAgentOrientation = Quaternion::Slerp(agentRotProgress, agentOrientation, agentTargetOrientation, true);
        //q2 = r*q1 and q2*q1' = r
        //newAgentOrientation = newAgentRotation * agentOrientation
        //=> newAgentOrientation * agentOrientation.Inverse() == newAgentRotation
        Quaternion newAgentRotation = newAgentOrientation * agentOrientation.Inverse();
        newAgentRotation.normalise();
        Vector3 agentTargetDir = newAgentRotation * agentDir;
        agentTargetDir.normalise();
        mVectorToTarget = agentTargetDir;
    }
    
    // Compute new torque scalar (-1.0 to 1.0) based on heading vector to target.
    Vector3 mSteeringForce = mAgentOrientation.Inverse() * mVectorToTarget;
    
    //cosmic vole - have to set default speed as collision avoidance code may have reduced it March 4 2017
    float speed = (current_waypoint_id>1)? waypoint_speed[current_waypoint_id-1] : 0.0f;
    if (speed)
        maxspeed = speed;
    else
        maxspeed = 200.0f;//TODO what to use?

    float power = (current_waypoint_id>1)? waypoint_power[current_waypoint_id-1] : 0.0f;
    if (power)
        acc_power = power;
    else
        acc_power = 1.0f;
        
    if (offCourse && abs(offCourseDist) > /*2.0f*/3.0f && time_since_last_collision_avoid > 4.0f)
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
        if (maxspeed < 4.0f)
        {
            maxspeed = 4.0f;
        }
        if (beam->getVelocity().length() < 5.0f && acc_power < 0.4f)//0.3f)
        {
            acc_power = 0.4f;//0.3f;
        }
        usePIDControl = false;
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
                    resetPos = beam->initial_node_pos[0];//waypoints[current_waypoint_id];
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
                            Beam** trucks = BeamFactory::getSingleton().getTrucks();
                            int numTrucks = BeamFactory::getSingleton().getTruckCount();

                            do
                            {
                                //Assume this new position is OK until we've checked all the trucks
                                found_space = true;
                                for (int t=0; t<numTrucks; t++)
                                {
                                    if (t == beam->trucknum)
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
                            //Nice tutorial says how to get character upright after a bunch of rotations, using quaternions http://www.ogre3d.org/tikiwiki/Quaternion+and+Rotation+Primer#Q_How_can_I_make_my_objects_stand_upright_after_a_bunch_of_rotations_
                            //Good quaternion cheat sheet http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-17-quaternions/

                            //if (cameranodepos[0] >= 0 && cameranodepos[0] < MAX_NODES)
                            //{
                            //    // pitch
                            //    Vector3 dir = nodes[cameranodepos[0]].RelPosition - nodes[cameranodedir[0]].RelPosition;
                            //    dir.normalise();
                            //    //TODO check this handles 360 degree rotation - look into using asin2 with y and z offsets if necessary
                            //    float angle = asin(dir.dotProduct(Vector3::UNIT_Y));
                            //    if (angle < -1) angle = -1;
                            //    if (angle > 1) angle = 1;

                            //    pitch = Radian(angle).valueDegrees();

                            //How pitch angle is calced for anti rollback:
                            //Vector3 dirDiff = curr_truck->getDirection();
                            //Degree pitchAngle = Radian(asin(dirDiff.dotProduct(Vector3::UNIT_Y)));

                            //    //TODO not sure how to calc roll yet but we may not need it as 2 axes may be enough to right the vehicle
                            //}

                            //Isn't this susceptible to gimbal lock?
                            //Quaternion rot = Quaternion(Degree(rz), Vector3::UNIT_Z) * Quaternion(Degree(ry), Vector3::UNIT_Y) * Quaternion(Degree(rx), Vector3::UNIT_X);

                            //CHECK OUT THIS CameraBehaviorVehicle::mousePressed Seems to do exactly what we need! Line 96 on...

                            // Set origin of rotation to camera node
                            Vector3 origin = nodes[0].AbsPosition;

                            if (beam->cameranodepos[0] >= 0 && beam->cameranodepos[0] < MAX_NODES)
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
    
    //cosmic vole started experimental collision avoidance code February 25 2017
    Vector3 agentVelocity = beam->getVelocity();
    //We will only move or add waypoints for a predicted collision if we haven't already done this
    //If we already did it, we still run collision detection again to adjust velocities etc.
    bool canAdjustWaypoint = collision_waypoint_id.empty();
    //if (collision_waypoint_id.empty())//TEST && 
    bool hasCarOnLeft = false;
    bool hasCarOnRight = false;
    bool hasStationaryObstacleInFront = false;
    
    if (agentVelocity.length() > 0.0001f)
    {
        float PI = 3.141592654;//!!TODO REMOVE THIS!!!!! cosmic vole
        Beam** trucks = BeamFactory::getSingleton().getTrucks();
        int numTrucks = BeamFactory::getSingleton().getTruckCount();
        float lookAhead = 100.0f;//50.0f;
        static int numWaypointsAdded = 0;
        //We sort the trucks by distance from this vehicle to try to avoid collisions with the nearest first - cosmic vole March 9 2017
        std::vector<Beam*> trucksByDist(trucks, trucks+numTrucks);
        truckDistanceSort comparer;
        comparer.fromTruck = beam;
        std::sort(trucksByDist.begin(), trucksByDist.end(), comparer);
        //float amountToMoveFinal = 0.0f;
        float timeToCollision = -1.0f;
        //Adjust time in seconds to wait between subsequent steering commands.
        //We slow down the steering a bit at higher speeds to reduce the chance of spins and rolls. cosmic vole March 11 2017
        //Actually, much smoother and faster without all this and using Slerp! Commented out. cosmic vole March 12 2017
/*        
        if (agentVelocity.length() > 9.0f)//10.0f
        {
            if (agentVelocity.length() > 12.0f)//13.0f
            {
                if (agentVelocity.length() > 18.0f)//20.0f
                {
                    if (agentVelocity.length() > 25.0f)
                    {
                        steering_delay = 0.41f;//0.42 0.436f;//0.48f;//0.3f
                    }
                    else
                    {
                        steering_delay = 0.31f;//0.32 0.331f;//0.29f;//0.45f;//0.23f;
                    }
                }
                else
                {
                    steering_delay = 0.20f;//0.213f;//0.12f;//0.40f;//0.16f;
                }
            }
            else
            {
                steering_delay = 0.110f;//0.09f;//0.30f;//0.13f;
            }
        }
        else*/
        {
            steering_delay = 0.025f;//0.05f;
        }
        
        for (int t=0; t<numTrucks; t++)
        {
            //if (t == beam->trucknum)
            //{
            //    continue;
            //}
            Beam* otherTruck = trucksByDist[t];//trucks[t];
            if (otherTruck->trucknum == beam->trucknum)
            {
                continue;
            }
            Vector3 otherPosition = otherTruck->getPosition();
            if (/*mAgentAbsPosition*/beam->getPosition().squaredDistance(otherPosition) < 2250.0f)//1600.0f)//400.0f)
            {
                VehicleAI* otherAI = otherTruck->getVehicleAI();
                Vector3 otherVelocity = otherTruck->getVelocity(); //TODO there's also absVelocity and relVelocity in the beam
                                    
                //if (otherAI)// && otherAI->IsActive())
                {
                    Vector3 agentTargetVector = current_waypoint - beam->getPosition();
                    Vector3 agentTargetHeading = agentTargetVector;
                    agentTargetHeading.normalise();
                    Vector3 otherTargetVector = (otherAI) ? (otherAI->current_waypoint - otherPosition) : otherTruck->getDirection();
                    //if ((otherAI->current_waypoint == current_waypoint && otherTargetVector.length() > agentTargetVector.length()) || (current_waypoint - otherAI->current_waypoint_id > free_waypoints * 0.5f)))
                    //{
                    //}
                    //TODO if the other truck is too close to its waypoint we need to look at the next waypoint along as well
                    Vector3 otherTargetHeading = otherTargetVector;
                    //otherTargetHeading.y = 0.0f;
                    otherTargetHeading.normalise();
                    float agentSpeedToTarget = agentVelocity.dotProduct(agentTargetHeading);
                    float otherSpeedToTarget = otherVelocity.dotProduct(otherTargetHeading);
                    //Make sure direction is not lost in the velocities
                    if (agentSpeedToTarget < 0.001f)
                        agentSpeedToTarget = 0.001f;
                    if (otherSpeedToTarget < 0.001f)
                        otherSpeedToTarget = 0.001f;
                    Vector3 agentVelocityToTarget = agentTargetHeading * agentSpeedToTarget;
                    Vector3 otherVelocityToTarget = otherTargetHeading * otherSpeedToTarget;
                    Vector3 agentDir = beam->getDirection();
                    agentDir.normalise();
                    //agentDir.getRotationTo(agentTargetHeading)
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

                    Vector3 otherDir = otherTruck->getDirection();
                    otherDir.normalise();
                    //Get other truck current rotation - the same as beam->getHeadingDirectionAngle() except this handles where the truck doesn't have the usual camera nodes - cosmic vole
                    float otherRotation = atan2(otherDir.dotProduct(Vector3::UNIT_X), (otherDir).dotProduct(-Vector3::UNIT_Z));
                    //Turn it into a quaternion for other truck's current orientation
                    Quaternion otherOrientation = Quaternion(Radian(otherRotation), Vector3::NEGATIVE_UNIT_Y);
                    otherOrientation.normalise();//TODO does Ogre do this for us???
                    //Get the desired other rotation to face the target - cosmic vole
                    float otherTargetRotation = atan2(otherTargetHeading.dotProduct(Vector3::UNIT_X), otherTargetHeading.dotProduct(-Vector3::UNIT_Z));
                    //Turn it into a quaternion for desired other truck orientation to target
                    Quaternion otherTargetOrientation = Quaternion(Radian(otherTargetRotation), Vector3::NEGATIVE_UNIT_Y);
                    otherTargetOrientation.normalise();
                    
                    
                    //float velocityToTarget = otherVelocity.dotProduct(otherTargetHeading);
                    //otherVelocity = otherVelocity.normalise() * velocityToTarget;
                    //float otherSpeed = otherVelocity.length();
                    //TODO need to estimate current truck acceleration to get a good estimate of time to waypoint
                    //float otherTimeToTarget = (otherSpeed > 0.0f) ? (otherSpeed / otherVelocity.length()) : -1.0f;
                    //Fix for immobile vehicles:
                    if (otherVelocity.length() <= 0.001f)
                    {
                        //otherVelocity = otherTargetHeading;
                    }
                    //otherVelocity.normalise();
                    //agentVelocity.normalise();
                    if (true)//otherTimeToTarget > 0.0f)
                    {
                        //We generate a rectangular area along the opponent truck's path and this agent (beam)'s path
                        //then find where the paths cross. A collision may or may not occur where the rectangles
                        //intersect. If the vehicles are single points, we can find only one place where they can collide where their direction vectors intersect.
                        //We can then solve simultaneous constant acceleration equations to estimate the time this occurs.
                        //Exceptions are if the motion is parallel or colinear. If parallel, but the rectangles still intersect, we want to know at what time 1 vehicle
                        //will draw alongside (and begin to pass) the other one. If colinear, we want the earliest point at which it catches up, which is a collision
                        //We need to bear in mind that the truck in front may suddenly decelerate, due to another collision, or stop accelerating due to reaching waypoint speed
                        //or due to wind resistance, so we need to allow a decent time margin of safety if possible.

                        Vector3 agentRect0, agentRect1, agentRect2, agentRect3;
                        Vector3 otherRect0, otherRect1, otherRect2, otherRect3;
                        
                        
                        //TODO add a slight safety margin to width
                        //TODO do we want to collision detect just from the front of the current truck, or from the rear forwards (to steer away from a truck rubbing alongside)?
                        //At the moment we define the rectangle from the front of the truck
                        Vector3 agentFrontLeft, agentFrontRight, agentRearLeft, agentRearRight;
                        Vector3 otherFrontLeft, otherFrontRight, otherRearLeft, otherRearRight;
                        beam->getCorners2D(agentFrontLeft, agentFrontRight, agentRearLeft, agentRearRight, 1.25f);//1.30f);
                        otherTruck->getCorners2D(otherFrontLeft, otherFrontRight, otherRearLeft, otherRearRight, 1.25f);//1.30f);//1.1f 1.3f works quite well and 1.45f even better on start grid (1.65 is about the limit) BUT - think there's a bug in the scaling code


                        //*** NEW CODE - brute force - just check the bounding rectangles of the two vehicles at numerous time steps and calc how far to move.

                        //Now we rotate both the rectangles so the agent's one is axis aligned:
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
                        //TODO won't work - that one always rotates around origin and translates after!
                        //matAxisAlign.makeTransform(-mAgentAbsPosition, Vector3::UNIT_SCALE, quatAxisAlign);
                        //Ogre like OpenGL works right to left for matrix multiplication
                        matAxisAlign = /*Matrix4::getTrans(mAgentAbsPosition) * */ Matrix4(quatAxisAlign) * Matrix4::getTrans(-mAgentAbsPosition);
                        matWorldAlign = matAxisAlign.inverse();
                        Quaternion quatAlignToWorld = quatAxisAlign.Inverse();
                        quatAlignToWorld.normalise();
                        
                        //***!!!! BUG need to subtract centre of rotation first because quatAxisAlign is a rotation! maybe better to make it into a matrix
                        agentRect0 = matAxisAlign * agentFrontLeft;//rotate(agentFrontLeft, quatAxisAlign, mAgentAbsPosition);//quatAxisAlign * agentFrontLeft;
                        agentRect1 = matAxisAlign * agentFrontRight;//rotate(agentFrontRight, quatAxisAlign, mAgentAbsPosition);//quatAxisAlign * agentFrontRight;
                        agentRect2 = matAxisAlign * agentRearLeft;//rotate(agentRearLeft, quatAxisAlign, mAgentAbsPosition);//quatAxisAlign * agentRearLeft;
                        agentRect3 = matAxisAlign * agentRearRight;//rotate(agentRearRight, quatAxisAlign, mAgentAbsPosition);//quatAxisAlign * agentRearRight;
                        
                        otherRect0 = matAxisAlign * otherFrontLeft;//rotate(otherFrontLeft, quatAxisAlign, mAgentAbsPosition);//quatAxisAlign * otherFrontLeft;
                        otherRect1 = matAxisAlign * otherFrontRight;//rotate(otherFrontRight, quatAxisAlign, mAgentAbsPosition);//quatAxisAlign * otherFrontRight;
                        otherRect2 = matAxisAlign * otherRearLeft;//rotate(otherRearLeft, quatAxisAlign, mAgentAbsPosition);//quatAxisAlign * otherRearLeft;
                        otherRect3 = matAxisAlign * otherRearRight;//rotate(otherRearRight, quatAxisAlign, mAgentAbsPosition);//quatAxisAlign * otherRearRight;
                        
                        Vector3 agentVelocAlign = quatAxisAlign * agentVelocity;
                        Vector3 otherVelocAlign = quatAxisAlign * otherVelocity;
                        
                        Vector3 agentPosAlign = matAxisAlign * mAgentAbsPosition;//mAgentAbsPosition;//Agent position is the centre of rotation so doesn't change! quatAxisAlign * mAgentAbsPosition;
                        Vector3 otherPosAlign = matAxisAlign * otherPosition;//rotate(otherPosition, quatAxisAlign, mAgentAbsPosition);//quatAxisAlign * otherPosition;
                        
                        //Calculate average current positions of the two vehicles, aligned to the agent's vehicle's axis
                        Vector3 agentAvgPosition = (agentRect0 + agentRect1 + agentRect2 + agentRect3) * 0.25f;
                        Vector3 otherAvgPosition = (otherRect0 + otherRect1 + otherRect2 + otherRect3) * 0.25f;

                        //Check if the opponent is moving head on towards the agent
                        //!! TODO we also need to consider that the vehicle could be reversing! Not handled yet. Or sliding forwards upside down!
                        //This finds the angle measured anticlockwise from mAgentHeading to otherTargetHeading
                        float approach = fmod(atan2(otherTargetHeading.z-mAgentHeading.z,otherTargetHeading.x-mAgentHeading.x) * 180.0f/PI, 360.0f);
                        //float approach = Ogre::Degree(mAgentHeading.angleBetween(otherTargetHeading);
                        bool isHeadOn = !(approach > 90.0f && approach < 270.0f);//(otherAvgFront).squaredDistance((agentRect0 + agentRect1) * 0.5f) > (otherAvgBack).squaredDistance((agentRect0 + agentRect1) * 0.5f);
                        bool isFromRight = approach > 0.0f && approach < 180.0f;//other vehicle is pointing towards the LHS of the agent's path
                        bool isFromLeft = approach > 180.0f && approach < 360.0f;//other vehicle is pointing towards the RHS of the agent's path

                        //Now, if the other vehicle is actually behind our (agent's) vehicle (TODO handle reversing), it is THEIR responsibility,
                        //not ours, to avoid the collision. This helps prevent two AIs moving in the same direction.
                        //if (/*otherAI && otherAI->IsActive() &&*/ !isHeadOn)
                        {
                            /*
                            if (canAdjustWaypoint)
                            {
                                if (agentRect0.z > agentRect2.z)
                                {
                                    RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, 
                                "Axis aligned z is bigger at front", "note.png");   
                                }
                                else if (agentRect0.z < agentRect2.z)
                                {
                                    RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, 
                                "Axis aligned z is smaller at front", "note.png");                                    
                                }
                            }
                            *///TODO if they're back to back, who does the collision checking? Only really relevant if at least one is reversing
                            bool debugIsBackToFront = false;
                            if (agentRect0.z < agentRect2.z)
                            {
                                RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, 
                            "Axis aligned z is smaller at front", "note.png");
                                debugIsBackToFront = true;
                            }
                            //TODO this is TOTALLY BROKEN! Debugging, it seems back to front but the above condition is false
                            //On the grid of f1 track, a bigger z on track gets classified as being in front, where the track is pointing towards negative z! 
                            //I think it's possibly because back and front are reversed by beam::getCorners2D()
                            /*
                            if (otherPosAlign.z > agentPosAlign.z && !debugIsBackToFront)//if (otherAvgPosition.z < agentAvgPosition.z)
                            {
                                RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, 
                            "otherPos is behind agentPos. agentPosAlign: (" + TOSTRING(agentPosAlign.x) + "," + TOSTRING(agentPosAlign.z) +
                            ") otherPosAlign: (" + TOSTRING(otherPosAlign.x) + "," + TOSTRING(otherPosAlign.z) + ")\n" +
                            " agentPosWrld: (" + TOSTRING(mAgentAbsPosition.x) + "," + TOSTRING(mAgentAbsPosition.z) +
                            ") otherPosWrld: (" + TOSTRING(otherPosition.x) + "," + TOSTRING(otherPosition.z) + ")\n", "note.png");
                                continue;
                            }
                            else
                            {
                                //RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, 
                            //"otherPos is ahead of agentPos. agentPosAlign: (" + TOSTRING(agentPosAlign.x) + "," + TOSTRING(agentPosAlign.z) +
                            //") otherPosAlign: (" + TOSTRING(otherPosAlign.x) + "," + TOSTRING(otherPosAlign.z) + ")\n" +
                            //" agentPosWrld: (" + TOSTRING(mAgentAbsPosition.x) + "," + TOSTRING(mAgentAbsPosition.z) +
                            //") otherPosWrld: (" + TOSTRING(otherPosition.x) + "," + TOSTRING(otherPosition.z) + ")\n", "note.png");
                            }
                            */
                            float distBetween = mAgentAbsPosition.distance(otherPosition);
                            Vector3 otherFromAgent = otherPosition - mAgentAbsPosition;
                            otherFromAgent.normalise();
                            //float angle = Radian(atan2(Vector3(-agentDir.z, 0.0f, agentDir.x).dotProduct(otherFromAgent), agentDir.dotProduct(otherFromAgent))).valueDegrees();//agentDir.angleBetween(otherFromAgent).valueDegrees();
                            //float angle = Radian(atan2(Vector3(-otherFromAgent.z, otherFromAgent.y, otherFromAgent.x).dotProduct(agentDir), otherFromAgent.dotProduct(agentDir))).valueDegrees();//agentDir.angleBetween(otherFromAgent).valueDegrees();
                            float agentDirAngle = Radian(atan2(agentDir.dotProduct(Vector3::UNIT_X), (agentDir).dotProduct(Vector3::UNIT_Z))).valueDegrees();
                            float angleToOther = Radian(atan2(otherFromAgent.dotProduct(Vector3::UNIT_X), (otherFromAgent).dotProduct(Vector3::UNIT_Z))).valueDegrees();
                            if (agentDirAngle < 0.0f)
                                agentDirAngle = 360.0f + agentDirAngle;
                            if (angleToOther < 0.0f)
                                angleToOther = 360.0f + angleToOther;
                            float angle = angleToOther - agentDirAngle;
                            if (angle < 0.0f)
                                angle = 360.0f + angle;
                            //angle = Radian(angle).valueDegrees();
                            /*
                            if (angle < 0.0f)
                            {
                                angle = 360.0f + angle;
                            }
                            */
                            
                            //angle = angle % 360.0f;
                            //Vector3 pointInFront = mAgentAbsPosition + (agentDir * distBetween * 2.0f);
                            //if (otherPosition.distance(pointInFront) > mAgentAbsPosition.distance(pointInFront))
                            
                            if (distBetween < 2.5f)//3.0f)//2.5f)//2.0f)
                            {
                                //LOG("agentDirAngle: " + TOSTRING(agentDirAngle) + " angleToOther: " + TOSTRING(angleToOther) + "\n" +
                            //"angle: " + TOSTRING(angle) + " agentDir: (" + TOSTRING(agentDir.x) + ", " + TOSTRING(agentDir.z) + ") otherFromAgent: (" + TOSTRING(otherFromAgent.x) + ", " + TOSTRING(otherFromAgent.z) + ").\n");
                                if (fabsf(angle) > 30.0f && fabsf(angle) < 150.0f)//40.0f && fabsf(angle) < 130.0f)
                                {
                                    //TODO I'm not yet sure which is left vs right (are angles clockwise or anti???)
                                    hasCarOnRight = true;
                                    //LOG("Has car on right.\n");
                                }
                                else if (fabsf(angle) > 210.0f && fabsf(angle) < 330.0f)//220.0f && fabsf(angle) < 320.0f)
                                {
                                    hasCarOnLeft = true;
                                    //LOG("Has car on left.\n");
                                }
                            }
                            if (distBetween < 4.0f && (fabsf(angle) <= 30.0f || fabsf(angle) >= 330.0f))
                            {
                                //LOG("Has car in front.\n");
                                if (otherVelocity.length() < 5.0f)//2.0f)
                                {
                                    hasStationaryObstacleInFront = true;
                                }
                            }
                            
                            if (fabsf(angle) > 90.0f && fabsf(angle) < 270.0f)
                            {
                                //if (distBetween < 10.0f) LOG("Has car behind.\n");
                                //Other truck is behind us
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
                                        continue;                                
                            }
                            else
                            {
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
                        }
                        //In the case of a head-on - if they just both move left, we should be all good. Otherwise we need some deterministic randomness or comms.

                        //Variables used in our collision avoidance simulation
                        bool collisionOccurs = false;
                        float timeLookAhead = 3.8f;//3.0f;//7.0f 5.0f;//3 second time lookahead. At 200 kph = 56 m/s, vehicle will travel 168 metres! At 60 kph = 17 m/s, 50 metres
                        //Increase time to look ahead if vehicle is moving faster - cosmic vole April 7 2017
                        if (agentVelocity.length() >= 30.0f)
                            timeLookAhead = 3.8f;
                        else if (agentVelocity.length() >= 24.0f)
                            timeLookAhead = 2.9f;//3.0f;
                        else if (agentVelocity.length() >= 20.0f)
                            timeLookAhead = 1.8f;//2.0f;
                        else if (agentVelocity.length() >= 15.0f)
                            timeLookAhead = 1.5f;//1.4f 1.6f;
                        else
                            timeLookAhead = 1.4f;//1.3f;//1.2f
                        
                        Vector3 lastNewWaypointPos = Vector3::ZERO;
                        Vector3 lastNewWaypointPosAlign = Vector3::ZERO;
                        Vector3 nextAgentWaypoint = current_waypoint;
                        Vector3 nextAgentWaypointAlign = matAxisAlign * current_waypoint;//rotate(current_waypoint, quatAxisAlign, mAgentAbsPosition);//quatAxisAlign * current_waypoint;
                        int nextAgentWaypointID = current_waypoint_id;
                        Vector3 nextOtherWaypoint = (otherAI)? otherAI->current_waypoint : otherPosition;
                        Vector3 nextOtherWaypointAlign = matAxisAlign * nextOtherWaypoint;//rotate(nextOtherWaypoint, quatAxisAlign, mAgentAbsPosition); //quatAxisAlign * nextOtherWaypoint;//otherAI->current_waypoint;
                        int nextOtherWaypointID = (otherAI) ? otherAI->current_waypoint_id : -1;
                        int firstNewWaypointIndex = -1;
                        float moveNeeded = 0.0f;
                        //TODO accel estimate needs to be more sophisticated and include rpm, topping out and braking as well. Acceleration from a standing start is probably lower too.
                        //TODO more to the point, if the other vehicle is slowed down due to another one ahead of it, it WILL NOT accelerate
                        float agentMaxSpeed = (current_waypoint_id > 1) ? MAX(maxspeed, waypoint_speed[current_waypoint_id-1]) : maxspeed;
                        float otherMaxSpeed = 0.0f;
                        if (otherAI && otherAI->IsActive())
                            otherMaxSpeed = (otherAI->current_waypoint_id > 1) ? MAX(otherAI->maxspeed, otherAI->waypoint_speed[otherAI->current_waypoint_id-1]) : otherAI->maxspeed;
                        float estAgentAccel = (agentSpeedToTarget < agentMaxSpeed) ? average_accel.length() : 0.0f;//1.4f : 0.0f;//1.8f 2.2f 3.5f 2.2f 2.7f : 1.0f;
                        float estOtherAccel = (otherSpeedToTarget < otherMaxSpeed) ? otherAI->average_accel.length() : 0.0f;//1.4f : 0.0f;//1.8f 2.2f 3.5f 2.2f 2.7 : 1.0f;
                        if (agentVelocity.length() < 0.01f)
                            estAgentAccel = 0.1f;
                        if (otherVelocity.length() < 0.01f)
                            estOtherAccel = 0.1f;
                        //if (!otherAI || !otherAI->IsActive())
                        //    estOtherAccel = 0.0f;
                        //If the agent isn't pointing towards the target, we will use Quaternion.Slerp() to gradually turn it as we put down collision avoidance waypoints
                        float agentRotNeeded = fabsf(agentRotation - agentTargetRotation) / (2.0f*PI);
                        float otherRotNeeded = fabsf(otherRotation - otherTargetRotation) / (2.0f*PI);
                        if (agentRotNeeded > 1.0f)
                            agentRotNeeded = 1.0f;
                        if (otherRotNeeded > 1.0f)
                            otherRotNeeded = 1.0f;
                        //This is the estimated turn speed where 1.0 would be a full 360 degree rotation in one second! 2.0 would be two rotations in one second.
                        float estAgentTurnSpeed = (agentRotNeeded < 0.001f) ? 0.0f : (3.0f * agentRotNeeded);//was 2.0f 0.5 so a 360 degree rotation would be done over 2 seconds - cosmic vole
                        float estOtherTurnSpeed = (otherRotNeeded < 0.001f) ? 0.0f : (3.0f * otherRotNeeded);//was 2.0f, 1.3f *
                        float agentRotProgress = 0.0f;
                        float otherRotProgress = 0.0f;
                        float agentTurnStartTime = 0.0f;
                        float otherTurnStartTime = 0.0f;
                        if (!otherAI || !otherAI->IsActive())
                        {
                            otherRotNeeded = 0.0f;
                        }
                        float agentTotalDist = 0.0f;
                        float otherTotalDist = 0.0f;
                        Vector3 agentPosWorld_ = mAgentAbsPosition;
                        Vector3 otherPosWorld_ = otherPosition;
                        Vector3 agentVelocWorld_ = agentVelocity;
                        Vector3 otherVelocWorld_ = otherVelocity;
                        Vector3 agentPosAlign_ = agentPosAlign;
                        Vector3 otherPosAlign_ = otherPosAlign;
                        Vector3 agentDir_ = agentDir;
                        Vector3 otherDir_ = otherDir;
                        Quaternion agentOrientation_ = agentOrientation;
                        Quaternion otherOrientation_ = otherOrientation;
                        Vector3 agentDirAlign_ = quatAxisAlign * agentDir_;
                        agentDirAlign_.normalise();
                        Vector3 otherDirAlign_ = quatAxisAlign * otherDir_;
                        otherDirAlign_.normalise();
                        Vector3 agentRect0_w = agentFrontLeft;
                        Vector3 agentRect1_w = agentFrontRight;
                        Vector3 agentRect2_w = agentRearLeft;
                        Vector3 agentRect3_w = agentRearRight;
                        Vector3 otherRect0_w = otherFrontLeft;
                        Vector3 otherRect1_w = otherFrontRight;
                        Vector3 otherRect2_w = otherRearLeft;
                        Vector3 otherRect3_w = otherRearRight;
                        
                        bool frontCollision = false;
                        bool rearCollision = false;
                        bool leftCollision = false;
                        bool rightCollision = false;
                        
                        static int reuseNode = -1;
                        if (t==0)
                        {
                            reuseNode = 0;
                        }
                        
                        //TODO can probably reduce time step if relative speed of the two vehicles is low
                        //TODO we probably don't want to be doing this every single frame!!! Especially once it's already been run for another vehicle.
                        //!!! MASSIVE BUG with this at the moment - after a lap or so, all vehicles seem to reset onto the same point! Superimposed on one another exactly! and it's not 0,0,0 in world space!
                        //I'm thinking collision avoidance waypoints shouldn't be used for resets, for starters
                        int step = -1;
                        float timeStep = 0.01f;//0.0025 0.005 0.01f;
                        for (float time = 0.01f; time < timeLookAhead; time+=timeStep)
                        {
                            step++;
                            //TODO in this same loop, plot the points of ALL the nearby vehicles, or at least once we know a vehicle's trajectory, we don't need to keep recalculating it
                            Vector3 agentRect0_, agentRect1_, agentRect2_, agentRect3_;
                            Vector3 otherRect0_, otherRect1_, otherRect2_, otherRect3_;
                            //World aligned versions
                            //Vector3 agentRect0_w, agentRect1_w, agentRect2_w, agentRect3_w;
                            //Vector3 otherRect0_w, otherRect1_w, otherRect2_w, otherRect3_w;
                            //TODO update velocities according to estimated accelerations and target speeds
                            //TODO if the lookahead reaches a waypoint for either vehicle, we'll have to tweak velocity direction and magnitude!
                            //TODO We probably DO have to account for this as there could be waypoints very close when we first detect a collision
                            //If the vehicles' initial velocities weren't aligned to their waypoint, we have to gradually rotate them
                            //at a simulated turn speed
                            bool rotated = false;
                            //#if 0
                            if (estAgentTurnSpeed != 0.0f && agentRotProgress < 1.0f)
                            {
                                rotated = true;
                                agentRotProgress = estAgentTurnSpeed * (time-agentTurnStartTime);
                                if (agentRotProgress > 1.0f)
                                    agentRotProgress = 1.0f;
                                Quaternion newAgentOrientation = Quaternion::Slerp(agentRotProgress, agentOrientation_, agentTargetOrientation, true);
                                //q2 = r*q1 and q2*q1' = r
                                //newAgentOrientation = newAgentRotation * agentOrientation
                                //=> newAgentOrientation * agentOrientation.Inverse() == newAgentRotation
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
                            
                            if (estOtherTurnSpeed != 0.0f && otherRotProgress < 1.0f)
                            {
                                rotated = true;
                                otherRotProgress = estOtherTurnSpeed * (time - otherTurnStartTime);
                                if (otherRotProgress > 1.0f)
                                    otherRotProgress = 1.0f;
                                Quaternion newOtherOrientation = Quaternion::Slerp(otherRotProgress, otherOrientation_, otherTargetOrientation, true);
                                //q2 = r*q1 and q2*q1' = r
                                //newAgentOrientation = newAgentRotation * agentOrientation
                                //=> newAgentOrientation * agentOrientation.Inverse() == newAgentRotation
                                Quaternion newOtherRotation = newOtherOrientation * otherOrientation.Inverse();
                                newOtherRotation.normalise();
                                otherDir_ = newOtherRotation * otherDir;
                                otherDir_.normalise();
                                //We will apply rotation to the vehicle, then update the velocity, then re-axis-align everything
                                //Translate starting coordinates to origin, then rotate, then rotate to current position
                                otherRect0_w = otherFrontLeft - otherPosition;
                                otherRect1_w = otherFrontRight - otherPosition;
                                otherRect2_w = otherRearLeft - otherPosition;
                                otherRect3_w = otherRearRight - otherPosition;

                                otherRect0_w = newOtherRotation * otherRect0_w;
                                otherRect1_w = newOtherRotation * otherRect1_w;
                                otherRect2_w = newOtherRotation * otherRect2_w;
                                otherRect3_w = newOtherRotation * otherRect3_w;
                                
                                otherRect0_w += otherPosWorld_;
                                otherRect1_w += otherPosWorld_;
                                otherRect2_w += otherPosWorld_;
                                otherRect3_w += otherPosWorld_;
                                
                                //Now we rotate the velocity by the same amount.
                                //Obviously this isn't accurate physically - we basically discard the components of the velocity
                                //that were in the wrong direction - but if the turn is done slowly enough, it should be good enough
                                //for our collision avoidance purposes (what we don't handle currently is vehicles spinning out) - cosmic vole March 4 2017
                                float newWorldSpeed = otherVelocWorld_.dotProduct(otherDir_);
                                if (newWorldSpeed < 0.001f)
                                    newWorldSpeed = 0.001f;
                                otherVelocWorld_ = otherDir_ * newWorldSpeed;
                            }
                            //#endif
                            
                            //TODO ??!! Shouldn't we be doing this every step as the centre of rotation moves????
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
                                
                                /*Moved:
                                matAxisAlign = Matrix4(quatAxisAlign) * Matrix4::getTrans(-agentPosWorld_);
                                matWorldAlign = matAxisAlign.inverse();

                                agentRect0_ = rotate(agentRect0_w, quatAxisAlign, agentPosWorld_);//quatAxisAlign * agentRect0_w;
                                agentRect1_ = rotate(agentRect1_w, quatAxisAlign, agentPosWorld_);//quatAxisAlign * agentRect1_w;
                                agentRect2_ = rotate(agentRect2_w, quatAxisAlign, agentPosWorld_);//quatAxisAlign * agentRect2_w;
                                agentRect3_ = rotate(agentRect3_w, quatAxisAlign, agentPosWorld_);//quatAxisAlign * agentRect3_w;
                                
                                otherRect0_ = rotate(otherRect0_w, quatAxisAlign, agentPosWorld_);//quatAxisAlign * otherRect0_w;
                                otherRect1_ = rotate(agentRect1_w, quatAxisAlign, agentPosWorld_);//quatAxisAlign * otherRect1_w;
                                otherRect2_ = rotate(agentRect2_w, quatAxisAlign, agentPosWorld_);//quatAxisAlign * otherRect2_w;
                                otherRect3_ = rotate(agentRect3_w, quatAxisAlign, agentPosWorld_);//quatAxisAlign * otherRect3_w;
                                
                                agentVelocAlign = quatAxisAlign * agentVelocWorld_;
                                otherVelocAlign = quatAxisAlign * otherVelocWorld_;
                                agentPosAlign_ = agentPosWorld_;//quatAxisAlign * agentPosWorld_;
                                otherPosAlign_ = rotate(otherPosWorld_, quatAxisAlign, agentPosWorld_);//quatAxisAlign * otherPosWorld_;
                                agentDirAlign_ = quatAxisAlign * agentDir_;
                                agentDirAlign_.normalise();
                                otherDirAlign_ = quatAxisAlign * otherDir_;
                                otherDirAlign_.normalise();
                                nextAgentWaypointAlign = rotate(nextAgentWaypoint, quatAxisAlign, agentPosWorld_);//quatAxisAlign * nextAgentWaypoint;
                                nextOtherWaypointAlign = rotate(nextOtherWaypoint, quatAxisAlign, agentPosWorld_);//quatAxisAlign * nextOtherWaypoint;
                                */
                            }
                            
                            /*
                            Vector3 agentDist = agentVelocAlign * time + 0.5f * estAgentAccel * time * time;
                            Vector3 otherDist = otherVelocAlign * time + 0.5f * estOtherAccel * time * time;
                            agentRect0_ = agentRect0 + agentDist;
                            agentRect1_ = agentRect1 + agentDist;
                            agentRect2_ = agentRect2 + agentDist;
                            agentRect3_ = agentRect3 + agentDist;
                            otherRect0_ = otherRect0 + otherDist;
                            otherRect1_ = otherRect1 + otherDist;
                            otherRect2_ = otherRect2 + otherDist;
                            otherRect3_ = otherRect3 + otherDist;
                            agentPosAlign_ = agentPosAlign + agentDist;
                            otherPosAlign_ = otherPosAlign + otherDist;
                            */
                            //Accelerate the objects
                            Vector3 prevAgentVelocAlign = agentVelocAlign;
                            Vector3 prevOtherVelocAlign = otherVelocAlign;
                            Vector3 prevAgentVelocWorld = agentVelocWorld_;
                            Vector3 prevOtherVelocWorld = otherVelocWorld_;
                            Vector3 agentDist; 
                            Vector3 otherDist;
                            
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
                            if (agentVelocWorld_.length() < maxspeed)
                            {
                                agentDist = prevAgentVelocWorld * timeStep + (0.5f * estAgentAccel * timeStep * timeStep) * agentDir_;
                                agentVelocWorld_ += timeStep * estAgentAccel * agentDir_; 
                            }
                            else
                            {
                                agentDist = prevAgentVelocWorld * timeStep;
                            }
                            
                            /*
                            if (otherAI && otherVelocAlign.length() < otherAI->maxspeed)
                            {
                                otherDist = prevOtherVelocAlign * timeStep + (0.5f * estOtherAccel * timeStep * timeStep) * otherDirAlign_;
                                otherVelocAlign += timeStep * estOtherAccel * otherDirAlign_;
                            }
                            else
                            {
                                otherDist = prevOtherVelocAlign * timeStep;
                            }
                            */
                            if (otherAI && otherVelocWorld_.length() < otherAI->maxspeed)
                            {
                                otherDist = prevOtherVelocWorld * timeStep + (0.5f * estOtherAccel * timeStep * timeStep) * otherDir_;
                                otherVelocWorld_ += timeStep * estOtherAccel * otherDir_;
                            }
                            else
                            {
                                otherDist = prevOtherVelocWorld * timeStep;
                            }
                            
                            agentVelocAlign = quatAxisAlign * agentVelocWorld_;
                            otherVelocAlign = quatAxisAlign * otherVelocWorld_;
                            //Update object positions - BUG / TODO - these incremental changes may be what is distorting the shape???
                            agentRect0_w += agentDist;
                            agentRect1_w += agentDist;
                            agentRect2_w += agentDist;
                            agentRect3_w += agentDist;
                            otherRect0_w += otherDist;
                            otherRect1_w += otherDist;
                            otherRect2_w += otherDist;
                            otherRect3_w += otherDist;
                            agentPosWorld_ += agentDist;
                            otherPosWorld_ += otherDist;
                            
                            matAxisAlign = /*Matrix4::getTrans(mAgentAbsPosition) * */ Matrix4(quatAxisAlign) * Matrix4::getTrans(-agentPosWorld_);
                            matWorldAlign = matAxisAlign.inverse();
                            
                            agentPosAlign_ = matAxisAlign * agentPosWorld_;//quatAlignToWorld * agentPosAlign_;
                            otherPosAlign_ = matAxisAlign * otherPosWorld_;
                            //!! TODO check this! Is the new centre of rotation valid to align to world? What about aligning the velocities above?
                            //otherPosWorld_ = rotate(otherPosAlign_, quatAlignToWorld, agentPosWorld_);//quatAlignToWorld * otherPosAlign_;
                            agentTotalDist += agentDist.length();
                            otherTotalDist += otherDist.length();


                            agentRect0_ = matAxisAlign * agentRect0_w; //rotate(agentRect0_w, quatAxisAlign, agentPosWorld_);//quatAxisAlign * agentRect0_w;
                            agentRect1_ = matAxisAlign * agentRect1_w; //rotate(agentRect1_w, quatAxisAlign, agentPosWorld_);//quatAxisAlign * agentRect1_w;
                            agentRect2_ = matAxisAlign * agentRect2_w;//rotate(agentRect2_w, quatAxisAlign, agentPosWorld_);//quatAxisAlign * agentRect2_w;
                            agentRect3_ = matAxisAlign * agentRect3_w;//rotate(agentRect3_w, quatAxisAlign, agentPosWorld_);//quatAxisAlign * agentRect3_w;
                            
                            otherRect0_ = matAxisAlign * otherRect0_w; //rotate(otherRect0_w, quatAxisAlign, agentPosWorld_);//quatAxisAlign * otherRect0_w;
                            otherRect1_ = matAxisAlign * otherRect1_w; //rotate(agentRect1_w, quatAxisAlign, agentPosWorld_);//quatAxisAlign * otherRect1_w;
                            otherRect2_ = matAxisAlign * otherRect2_w; //rotate(agentRect2_w, quatAxisAlign, agentPosWorld_);//quatAxisAlign * otherRect2_w;
                            otherRect3_ = matAxisAlign * otherRect3_w; //rotate(agentRect3_w, quatAxisAlign, agentPosWorld_);//quatAxisAlign * otherRect3_w;
                            
                            //agentVelocAlign = quatAxisAlign * agentVelocWorld_;
                            //otherVelocAlign = quatAxisAlign * otherVelocWorld_;
                            //agentPosAlign_ = agentPosWorld_;//quatAxisAlign * agentPosWorld_;
                            //otherPosAlign_ = rotate(otherPosWorld_, quatAxisAlign, agentPosWorld_);//quatAxisAlign * otherPosWorld_;
                            agentDirAlign_ = quatAxisAlign * agentDir_;
                            agentDirAlign_.normalise();
                            otherDirAlign_ = quatAxisAlign * otherDir_;
                            otherDirAlign_.normalise();
                            nextAgentWaypointAlign = matAxisAlign * nextAgentWaypoint; //rotate(nextAgentWaypoint, quatAxisAlign, agentPosWorld_);//quatAxisAlign * nextAgentWaypoint;
                            nextOtherWaypointAlign = matAxisAlign * nextOtherWaypoint; //rotate(nextOtherWaypoint, quatAxisAlign, agentPosWorld_);//quatAxisAlign * nextOtherWaypoint;
                                                        
                            //agentVelocWorld_ = quatAlignToWorld * agentVelocAlign;
                            //otherVelocWorld_ = quatAlignToWorld * otherVelocAlign;
                                                        
                            //See if we've reached any of the existing waypoints yet
                            float agentDistToNextWPsq = agentPosAlign_.squaredDistance(nextAgentWaypointAlign);
                            if (agentDistToNextWPsq < 25.0f)
                            {
                                //Update direction vectors to next waypoint
                                nextAgentWaypointID++;
                                if (nextAgentWaypointID <= free_waypoints)
                                {
                                    nextAgentWaypoint = waypoints[nextAgentWaypointID];//TODO may need to renumber waypoints if we insert one, or use names
                                    nextAgentWaypointAlign = matAxisAlign * nextAgentWaypoint; //rotate(nextAgentWaypoint, quatAxisAlign, agentPosWorld_);//quatAxisAlign * nextAgentWaypoint;
                                    agentMaxSpeed = (nextAgentWaypointID > 1) ? waypoint_speed[nextAgentWaypointID-1] : agentMaxSpeed;
                                    
                                    agentTargetVector = nextAgentWaypoint - agentPosWorld_;
                                    agentTargetHeading = agentTargetVector;
                                    agentTargetHeading.normalise();
                                    agentSpeedToTarget = agentVelocWorld_.dotProduct(agentTargetHeading);
                                    //Make sure direction is not lost in the velocities
                                    if (agentSpeedToTarget < 0.001f)
                                        agentSpeedToTarget = 0.001f;
                                    agentVelocityToTarget = agentTargetHeading * agentSpeedToTarget;
                                    
                                    /* Don't think we want to change these (as we're not redoing beam->getCorners()):
**!!! BUG BUG HUGE BUG ARGH ! - The code for turning slerps always from agentOrientation using the original corners!
* BUT - after completing the first turn, our simulated vehicle should NOT be at its initial orientation anymore, so its really wrong!
                                    agentDir = agentDir_;//beam->getDirection();
                                    agentDir.normalise();
                                    //Get agent current rotation - the same as beam->getHeadingDirectionAngle() except this handles where the truck doesn't have the usual camera nodes - cosmic vole
                                    agentRotation = atan2(agentDir.dotProduct(Vector3::UNIT_X), (agentDir).dotProduct(-Vector3::UNIT_Z));
                                    //Turn it into a quaternion for current agent orientation
                                    agentOrientation = Quaternion(Radian(agentRotation), Vector3::NEGATIVE_UNIT_Y);
                                    agentOrientation.normalise();//TODO does Ogre do this for us???
                                    */
                                    //Because of the above HUGE bug, we've decided we DO need to update the orientation
                                    agentDir = agentDir_;//beam->getDirection();
                                    agentDir.normalise();
                                    //Get agent current rotation - the same as beam->getHeadingDirectionAngle() except this handles where the truck doesn't have the usual camera nodes - cosmic vole
                                    float aNewAgentRotation = atan2(agentDir.dotProduct(Vector3::UNIT_X), (agentDir).dotProduct(-Vector3::UNIT_Z));
                                    //Turn it into a quaternion for current agent orientation
                                    agentOrientation_ = Quaternion(Radian(aNewAgentRotation), Vector3::NEGATIVE_UNIT_Y);
                                    agentOrientation_.normalise();
                                    //TODO The above is STILL BROKEN!!!!

                                    
                                     
                                    //Get the desired agent rotation to face the target - cosmic vole
                                    agentTargetRotation = atan2(agentTargetHeading.dotProduct(Vector3::UNIT_X), agentTargetHeading.dotProduct(-Vector3::UNIT_Z));
                                    //Turn it into a quaternion for desired agent orientation to target
                                    agentTargetOrientation = Quaternion(Radian(agentTargetRotation), Vector3::NEGATIVE_UNIT_Y);
                                    agentTargetOrientation.normalise();
                                    
                                    //TODO look up new maxspeed
                                    estAgentAccel = (agentSpeedToTarget < agentMaxSpeed) ? average_accel.length() : 0.0f;//1.4f : 0.0f;
                                    if (agentVelocWorld_.length() < 0.01f)
                                        estAgentAccel = 0.1f;
                                    //If the agent isn't pointing towards the target, we will use Quaternion.Slerp() to gradually turn it as we put down collision avoidance waypoints
                                    agentRotNeeded = fabsf(aNewAgentRotation - agentTargetRotation) / (2.0f*PI);
                                    //This is the estimated turn speed where 1.0 would be a full 360 degree rotation in one second!
                                    estAgentTurnSpeed = (agentRotNeeded < 0.001f) ? 0.0f : (3.0f * agentRotNeeded);// 0.5 so a 360 degree rotation would be done over 2 seconds - cosmic vole
                                    agentRotProgress = 0.0f;
                                    agentTurnStartTime = time;
                                    //TODO update flag for last waypoint
                     
                                }
                            }
                            //We need to do the same for the other vehicle.
                            float otherDistToNextWPSq = otherPosAlign_.squaredDistance(nextOtherWaypointAlign);
                            if (otherAI && otherDistToNextWPSq < 25.0f)
                            {
                                //Update direction vectors to next waypoint - to minimise error it might be worth setting the vehicle position to THIS waypoint as well
                                nextOtherWaypointID++;
                                if (nextOtherWaypointID < otherAI->free_waypoints)
                                {
                                    nextOtherWaypoint = otherAI->waypoints[nextOtherWaypointID];//TODO may need to renumber waypoints if we insert one, or use names
                                    nextOtherWaypointAlign = matAxisAlign * nextOtherWaypoint; //rotate(nextAgentWaypoint, quatAxisAlign, agentPosWorld_);//quatAxisAlign * nextAgentWaypoint;
                                    if (otherAI->IsActive())
                                        otherMaxSpeed = (nextOtherWaypointID > 1) ? otherAI->waypoint_speed[nextOtherWaypointID-1] : otherMaxSpeed;

                                    
                                    otherTargetVector = nextOtherWaypoint - otherPosWorld_;
                                    otherTargetHeading = otherTargetVector;
                                    otherTargetHeading.normalise();
                                    otherSpeedToTarget = otherVelocWorld_.dotProduct(otherTargetHeading);
                                    //Make sure direction is not lost in the velocities
                                    if (otherSpeedToTarget < 0.001f)
                                        otherSpeedToTarget = 0.001f;
                                    otherVelocityToTarget = otherTargetHeading * otherSpeedToTarget;
                                    
                                    /* Don't think we want to change these (as we're not redoing beam->getCorners()):
                                    agentDir = agentDir_;//beam->getDirection();
                                    agentDir.normalise();
                                    //Get agent current rotation - the same as beam->getHeadingDirectionAngle() except this handles where the truck doesn't have the usual camera nodes - cosmic vole
                                    agentRotation = atan2(agentDir.dotProduct(Vector3::UNIT_X), (agentDir).dotProduct(-Vector3::UNIT_Z));
                                    //Turn it into a quaternion for current agent orientation
                                    agentOrientation = Quaternion(Radian(agentRotation), Vector3::NEGATIVE_UNIT_Y);
                                    agentOrientation.normalise();//TODO does Ogre do this for us???
                                    */
                                    //Because of the above HUGE bug, we've decided we DO need to update the orientation
                                    otherDir = otherDir_;//beam->getDirection();
                                    otherDir.normalise();
                                    //Get agent current rotation - the same as beam->getHeadingDirectionAngle() except this handles where the truck doesn't have the usual camera nodes - cosmic vole
                                    float aNewOtherRotation = atan2(otherDir.dotProduct(Vector3::UNIT_X), (otherDir).dotProduct(-Vector3::UNIT_Z));
                                    //Turn it into a quaternion for current agent orientation
                                    otherOrientation_ = Quaternion(Radian(aNewOtherRotation), Vector3::NEGATIVE_UNIT_Y);
                                    otherOrientation_.normalise();
                                     
                                    //Get the desired agent rotation to face the target - cosmic vole
                                    otherTargetRotation = atan2(otherTargetHeading.dotProduct(Vector3::UNIT_X), otherTargetHeading.dotProduct(-Vector3::UNIT_Z));
                                    //Turn it into a quaternion for desired agent orientation to target
                                    otherTargetOrientation = Quaternion(Radian(otherTargetRotation), Vector3::NEGATIVE_UNIT_Y);
                                    otherTargetOrientation.normalise();
                                    
                                    //TODO look up new maxspeed
                                    estOtherAccel = (otherSpeedToTarget < otherMaxSpeed) ? otherAI->average_accel.length() : 0.0f;//1.4f : 0.0f;
                                    if (otherVelocWorld_.length() < 0.01f)
                                        estOtherAccel = 0.1f;
                                    //If the agent isn't pointing towards the target, we will use Quaternion.Slerp() to gradually turn it as we put down collision avoidance waypoints
                                    otherRotNeeded = fabsf(aNewOtherRotation - otherTargetRotation) / (2.0f*PI);
                                    //This is the estimated turn speed where 1.0 would be a full 360 degree rotation in one second!
                                    estOtherTurnSpeed = (otherRotNeeded < 0.001f) ? 0.0f : (3.0f * otherRotNeeded);// 0.5 so a 360 degree rotation would be done over 2 seconds - cosmic vole
                                    otherRotProgress = 0.0f;
                                    otherTurnStartTime = time;
                                    //TODO update flag for last waypoint
                     
                                }
                                
                            }
                            
                            //Debug projected positions of the two vehicles - cosmic vole March 10 2017
                            Beam *curTruck = BeamFactory::getSingleton().getCurrentTruck();
                            static std::vector<SceneNode*> debugCol = std::vector<SceneNode*>();
                            if (curTruck && (curTruck == beam) && (step % 6 == 0))//(curTruck->trucknum == beam->trucknum))
                            {
                                static MaterialPtr debugMaterialAgent;
                                static MaterialPtr debugMaterialOther;
                                static MaterialPtr debugMaterialCollision;
                                static bool madeMaterials = false;
                                SceneNode *nodeAgent, *nodeOther;
                                Entity *entityAgent, *entityOther;
                                //static int reuseNode = -1;

                                int count = debugCol.size();
                                if (count>3000)//12000)
                                {
                                    if (reuseNode < 0)
                                    {
                                        reuseNode = 0;
                                    }
                                    else if (reuseNode >= count-7)
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
                                    debugMaterialAgent = MaterialManager::getSingleton().create("debugMaterialAgent", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
                                    debugMaterialAgent->setSelfIllumination(0.3f,0.0f,0.6f);
                                    debugMaterialAgent->setAmbient(0.3f,0.0f,0.6f);
                                    debugMaterialAgent->setDiffuse(0.3f,0.0f,0.6f,0.9f);
                                    debugMaterialOther = MaterialManager::getSingleton().create("debugMaterialOther", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
                                    debugMaterialOther->setSelfIllumination(0.0f,0.6f,0.1f);
                                    debugMaterialOther->setAmbient(0.0f,0.6f,0.1f);
                                    debugMaterialOther->setDiffuse(0.0f,0.6f,0.1f,0.9f);
                                    debugMaterialCollision = MaterialManager::getSingleton().create("debugMaterialCollision", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
                                    debugMaterialCollision->setSelfIllumination(0.7f,0.0f,0.0f);
                                    debugMaterialCollision->setAmbient(0.7f,0.0f,0.0f);
                                    debugMaterialCollision->setDiffuse(0.7f,0.0f,0.0f,0.9f);

                                }
                                Vector3 debugRect0 = agentRect0_w;
                                Vector3 debugRect1 = agentRect1_w;
                                Vector3 debugRect2 = agentRect2_w;
                                Vector3 debugRect3 = agentRect3_w;
                                debugRect0.y = debugRect1.y = debugRect2.y = debugRect3.y = /*agentPosWorld_.y*/mAgentAbsPosition.y + 0.2f;

                                debugCollision(reuseNode, debugCol, debugRect0, std::string("debugMaterialAgent"));
                                debugCollision(reuseNode, debugCol, debugRect1, std::string("debugMaterialAgent"));
                                debugCollision(reuseNode, debugCol, debugRect2, std::string("debugMaterialAgent"));
                                debugCollision(reuseNode, debugCol, debugRect3, std::string("debugMaterialAgent"));

                                debugRect0 = otherRect0_w;
                                debugRect1 = otherRect1_w;
                                debugRect2 = otherRect2_w;
                                debugRect3 = otherRect3_w;
                                debugRect0.y = debugRect1.y = debugRect2.y = debugRect3.y = /*otherPosWorld_.y*/mAgentAbsPosition.y + 0.2f;
                                
                                debugCollision(reuseNode, debugCol, debugRect0, std::string("debugMaterialOther"));
                                debugCollision(reuseNode, debugCol, debugRect1, std::string("debugMaterialOther"));
                                debugCollision(reuseNode, debugCol, debugRect2, std::string("debugMaterialOther"));
                                debugCollision(reuseNode, debugCol, debugRect3, std::string("debugMaterialOther"));
                                /*
                                if (reuseNode >= 0)
                                {
                                    nodeAgent = debugCol.at(reuseNode);
                                    entityAgent = (Entity *)nodeAgent->getAttachedObject(0);
                                    reuseNode++;
                                }
                                else
                                {
                                    nodeAgent = gEnv->sceneManager->getRootSceneNode()->createChildSceneNode();
                                    entityAgent = gEnv->sceneManager->createEntity(SceneManager::PT_SPHERE);
                                    nodeAgent->attachObject(entityAgent);
                                }

                                //float radius = entityAgent->getBoundingRadius();
                                float entityWidth = 2.0f * entityAgent->getBoundingRadius();//100.0f;//entityAgent->getBoundingBox().getSize().x;
                                //float width = (agentFrontRight - agentFrontLeft).length();
                                //float length = (agentFrontLeft - agentRearLeft).length();
                                //nodeAgent->setScale(Vector3(width/entityWidth,width*0.5f/entityWidth,length/entityWidth));
                                nodeAgent->setScale(Vector3(0.15f/entityWidth,0.15f/entityWidth,0.15f/entityWidth));
                                nodeAgent->setPosition(agentRect0_);//agentPosWorld_);
                                //nodeAgent->setOrientation(agentOrientation);
                                entityAgent->setMaterialName("debugMaterialAgent");
                                if (reuseNode < 0)
                                    debugCol.push_back(nodeAgent);
                                
                                if (reuseNode >= 0)
                                {
                                    nodeOther = debugCol.at(reuseNode);
                                    entityOther = (Entity *)nodeOther->getAttachedObject(0);
                                    reuseNode++;
                                }
                                else
                                {
                                    nodeOther = gEnv->sceneManager->getRootSceneNode()->createChildSceneNode();
                                    entityOther = gEnv->sceneManager->createEntity(SceneManager::PT_CUBE);
                                    nodeOther->attachObject(entityOther);
                                }Material
                                //radius = entityOther->getBoundingRadius();
                           
                                width = (otherFrontRight - otherFrontLeft).length();
                                length = (otherFrontLeft - otherRearLeft).length();
                                nodeOther->setScale(Vector3(width/entityWidth,width*0.5f/entityWidth,length/entityWidth));                            
                                //nodeOther->setScale(Vector3(0.2f/radius,0.2f/radius,0.2f/radius));
                                nodeOther->setPosition(otherPosWorld_);
                                nodeOther->setOrientation(otherOrientation);
                                entityOther->setMaterialName("debugMaterialOther");
                                if (reuseNode < 0)
                                {
                                    debugCol.push_back(nodeOther);
                                }
                                */
                            }
                            //End Debug projected positions
                                                        
                            
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
                            
                            if ((gotNewIntersection || moveIsNeeded) && numWaypointsAdded < 100000)
                            {
                                if (gotNewIntersection)
                                {
                                    collisionOccurs = true;
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
                                    
                                    if (curTruck && (curTruck == beam) && (step % 6 == 0))
                                        debugCollision(reuseNode, debugCol, agentPosWorld_, std::string("debugMaterialCollision"));
                                   
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
                                        float otherSpeedInAgentDir = otherVelocWorld_.dotProduct(otherDir_);
                                        //If a collision is less than 3 metres or less than a second away,
                                        //don't try to swerve, just brake and roll off power - BUG this just makes them all stop at the start of the race! We also want to match forward speed of vehicle in front normally
                                        if (frontCollision && agentVelocWorld_.length() > 5.0f)//7.0f)//9.0f)//agentVelocity.length() > 10.0f)
                                        {
                                            //TODO if it's a front collision and the other vehicle isn't moving and is already directly in front, we need to reverse away
                                            if (canAdjustWaypoint)
                                            {
                                                maxspeed = agentVelocWorld_.length() * 0.6f;//agentVelocity.length() * 0.9f;//0.8f;//0.6f;
                                                if (acc_power > 0.8f) acc_power = 0.8f;//acc_power = 0.75f;//0.6f;
                                            }
                                            //We really need to use the other vehicle's speed in the direction the agent is moving
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
                                        }
                                        if (agentTotalDist < 0.1f || time < 0.2f)//0.3f)//0.6f || time < 0.7f)
                                        {
                                            //!!!!??***** TODO We keep getting false positives here on the v first iteration of the loop
                                            //e.g. time == 0.01f, agentTotalDist 0.1, otherTotalDist 0.01, agentVelocity 10.92
                                            //Maybe when we try to move the rectangles into position after rotation it overlaps them???
                                            
                                            
                                            //Definitely no point in adding waypoints; vehicle is too close
                                            //TODO actually what can happen is it's a side to side collision, in which case it could be avoided by steering the opposite way
                                            
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
                                            if (maxspeed > otherSpeedInAgentDir)// && otherSpeedInAgentDir > 8.0f)//otherVelocWorld_.length() && otherVelocWorld_.length() > 8.0f)//otherVelocity.length() && otherVelocity.length() > 8.0f)//10.0f)
                                            {
                                                //Brake even harder
                                                maxspeed = /*otherVelocWorld_.length()*/otherSpeedInAgentDir * 0.4f;
                                                acc_power = 0.1f;
                                            }
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
                                    if (!canAdjustWaypoint)
                                    {
                                        //break; TODO re-enable this if we start using waypoint adjustment again. At the moment I think it's just stopping collision avoidance from working sometimes!
                                    }
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
                                //TODO theoretically this could be nearer than the current point! Check!
                                //TODO we also need to know the z distance (along track direction), not just total distance
                                //as we don't want an old waypoint left diagonally across the track that makes the vehicle swerve wildly.
                                float distToNextWPsq = newWaypointPosAlign.squaredDistance(nextAgentWaypointAlign);
                                if (distToNextWPsq < 130.0f)//150.0f)
                                {
                                    //We'll just move the existing current_waypoint instead
                                    //amountToMove *= 1.15f;//1.07f 1.2f;//1.3 Move by a bit more as it's not exactly there
                                    float offset = nextAgentWaypointAlign.x - agentPosAlign_.x;//agentAvgPosition.x;
                                    //if (fabsf(offset) < fabsf(amountToMove))
                                    if (canAdjustWaypoint)
                                    {
                                        //nextAgentWaypointAlign.x += amountToMove;//(!moveIsNeeded && nextAgentWaypointAlign.x > agentPosAlign_.x) ? -amountToMove : amountToMove;
                                        //!!! TODO BUG - If a waypoint has already been added in this loop, current_waypoint_id won't point to the existing one
                                        //anymore AND the one it does is likely BEHIND this
#if 0
                                        nextAgentWaypoint = matWorldAlign * nextAgentWaypointAlign; //quatAlignToWorld * nextAgentWaypointAlign;
                                        waypoints[nextAgentWaypointID].x = nextAgentWaypoint.x;
                                        waypoints[nextAgentWaypointID].z = nextAgentWaypoint.z;
                                        collision_waypoint_id = waypoint_names[nextAgentWaypointID];
                                                                            RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, 
                                "Moved Waypoint for Vehicle: " + TOSTRING(beam->trucknum) + "  Other: " + TOSTRING(otherTruck->trucknum) + "\n at pos (" + 
                                TOSTRING(nextAgentWaypoint.x) + ", " + TOSTRING(nextAgentWaypoint.z) +
                                "); amountToMove: " + TOSTRING(amountToMove) + "\n agent pos(" + TOSTRING(mAgentAbsPosition.x) + "," + TOSTRING(mAgentAbsPosition.z) + ")\nPredicted agent pos (" +
                                TOSTRING(agentPosWorld_.x) + "," + TOSTRING(agentPosWorld_.z) + ")\n", "note.png");
#endif                                
                                        break;
                                    }
                                    //else
                                    //{
                                        //The next waypoint SHOULD already move us out of the way
                                        //TODO though bear in mind the other vehicle will turn towards it also!
                                        //Maybe reduce speed a bit as we approach?
                                    //}
                                    moveIsNeeded = false;
                                    moveNeeded = 0.0f;
                                }
                                else
                                {
                                    #if 0
                                    if (distToNextWPsq < 900.0f && canAdjustWaypoint)
                                    {
                                        //If the next waypoint is within 30 metres of this, we will move it a bit as well, if we haven't already
                                        float amountToMove2 = amountToMove * 0.5f;
                                        float offset = nextAgentWaypointAlign.x - agentPosAlign_.x;//agentAvgPosition.x;
                                        if (fabsf(offset) < fabsf(amountToMove2))
                                        {
                                            //TODO this logic isn't quite right - shouldn't be based off agentAvgPosition as we're going to have
                                            //another waypoint. Also, it needs to take into account moveNeeded!
                                            nextAgentWaypointAlign.x += (nextAgentWaypointAlign.x > agentPosAlign_.x) ? -amountToMove2 : amountToMove2;
                                            nextAgentWaypoint = matAlignToWorld * nextAgentWaypointAlign; //quatAlignToWorld * nextAgentWaypointAlign;
                                            waypoints[nextAgentWaypointID].x = nextAgentWaypoint.x;
                                            waypoints[nextAgentWaypointID].z = nextAgentWaypoint.z;
                                        }
                                    }
                                    #endif
                                    //Workaround a major problem. If our new waypoint is closer to the current agent position
                                    //than the threshold for reaching a waypoint, updateWaypoint() will fire immmediately,
                                    //causing endless waypoints to be generated right in front of the vehicle!
                                    //So we defer adding the waypoint until further up the track, moving it over by a bit more to compensate.
                                    if (newWaypointPosAlign.distance(/*quatAxisAlign*/matAxisAlign * mAgentAbsPosition) < 12.0)//5.5)//TODO update value if we change waypoint threshold
                                    {
                                        moveIsNeeded = true;
                                        moveNeeded = amountToMove * 1.01f;//1.2f; TODO the amount it increases needs to depend on distance it's postponed by. Trig probably needed.
                                        continue;
                                    }
                                    //We're going to make a new waypoint now, unless we've already done one
                                    bool isFirstNewWaypoint;
                                    if (firstNewWaypointIndex < 0)
                                    {
                                        isFirstNewWaypoint = true;
                                        firstNewWaypointIndex = current_waypoint_id;
                                    }
                                    //else
                                    //{
                                        //There was already a new waypoint. How far back was it? TODO ? Is z distance better to use here?
                                        //TODO Not just new waypoints, even the last one
                                        Vector3 lastWaypointAlign;
                                        if (nextAgentWaypointID >= 2)
                                        {
                                            lastWaypointAlign = matAxisAlign * waypoints[nextAgentWaypointID-1];
                                            float distToLastWPsq = newWaypointPosAlign.squaredDistance(lastWaypointAlign);//lastNewWaypointPosAlign);
                                            if (distToLastWPsq < 150.0f && time + 0.01f < timeLookAhead)
                                            {
                                                //TODO if we're out of time, position the final waypoint appropriately
                                                //We don't want to go moving earlier waypoints really as we'd have to recheck for collisions there
                                                //so we'll postpone the new waypoint until we reach the required distance from the previous one
                                                //TODO if we do need to move the old waypoints as a smoothing exercise, we need to store a safe zone
                                                //around each point when we collision detect so we know how far we can move it..
                                                if (!moveIsNeeded)
                                                {
                                                    moveIsNeeded = true;
                                                    moveNeeded = amountToMove;// * 1.3f;
                                                }
                                                continue;
                                            }                                            
                                        }
                                        
                                    //}
                                    if (!canAdjustWaypoint)
                                        break;
#if 0                                        
                                    //Now make the new waypoint
                                    char waypointName[255];
                                    snprintf(waypointName, sizeof(waypointName), "TempWaypoint%d", free_waypoints);
                                    std::string name = waypointName;
                                    Vector3 newWaypoint = matWorldAlign * newWaypointPosAlign; //rotate(newWaypointPosAlign, quatAlignToWorld, agentPosWorld_);//quatAlignToWorld * newWaypointPosAlign;
                                    //**!!! TODO Something's not right here - even if amountToMove is 0.1f, the cars still drive many metres off course
                                    //**!!! maybe try outputting a test coord before and after quatAxisAlign and quatWorldAlign???
                                    InsertWaypoint(name, newWaypoint, nextAgentWaypointID);
                                    if (isFirstNewWaypoint)
                                    {
                                        //**!!!!! TODO update this - after we've passed an existing waypoint, we won't want to change current_waypoint
                                        current_waypoint = newWaypoint;
                                        isFirstNewWaypoint = false;
                                    }
                                    //TODO we probably need to disable further collision avoidance until the new / updated waypoints have been reached (with a timeout as well)!
                                    collision_waypoint_id = name;//waypoint_names[nextAgentWaypointID];
                                    numWaypointsAdded++;
                                    nextAgentWaypointID++;
                                    lastNewWaypointPosAlign = newWaypointPosAlign;
                                    lastNewWaypointPos = newWaypoint;
                                    moveIsNeeded = false;
                                    moveNeeded = 0.0f;
                                    //agentRect0 = quatAxisAlign * agentFrontLeft;
                                    char debugQuat[512];
                                    snprintf(debugQuat, sizeof(debugQuat), "Quat test. Truck:%d. FL(%f, %f)\nAxisAlgn FL(%f, %f)\nWrldAlgn FL(%f, %f)\nNew WPt AxAlgn(%f, %f) Wrld(%f %f)\n",
                                        beam->trucknum,
                                        agentFrontLeft.x,
                                        agentFrontLeft.z,
                                        (quatAxisAlign * agentFrontLeft).x,
                                        (quatAxisAlign * agentFrontLeft).z,
                                        (quatAlignToWorld * quatAxisAlign * agentFrontLeft).x,
                                        (quatAlignToWorld * quatAxisAlign * agentFrontLeft).z,
                                        newWaypointPosAlign.x,
                                        newWaypointPosAlign.z,
                                        newWaypoint.x,
                                        newWaypoint.z);
                                //RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, 
                                //debugQuat, "note.png");                                    
                                    RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, 
                                "Added Waypoint for Vehicle: " + TOSTRING(beam->trucknum) + "  Other: " + TOSTRING(otherTruck->trucknum) + "\n at pos (" + 
                                TOSTRING(newWaypoint.x) + ", " + TOSTRING(newWaypoint.y) + ", " + TOSTRING(newWaypoint.z) +
                                "); agent pos(" + TOSTRING(mAgentAbsPosition.x) + "," + TOSTRING(mAgentAbsPosition.z) + ")\nPredicted agent pos (" +
                                TOSTRING(agentPosWorld_.x) + "," + TOSTRING(agentPosWorld_.z) + ")\n", "note.png");
                                    //TODO waypoint speed and power are keyed to current_waypoint_id, so we'll need to update them for the old and new current_waypoint_id and all subsequent waypoints!
                                    //TODO if we ever decide to remove this break, we need to adjust agent's position so it's actually ON this waypoint for the avoidance algorithm to proceed correctly
#endif                                    
                                    break;
                                }
                            }
                        }
                        
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
                        
                        //*** END NEW CODE - cosmic vole March 3 2017
                        
                        //*** OLD CODE - Tried to find min and max times of vehicle's paths crossing. It could work, but may be a waste of time as we still need to run further tests as
                        //we don't know the exact point of collision between those. We can't reject the collision if both time gaps are big as one vehicle could have passed through
                        //the other by that point. Also it was only using two points on each vehicle so further checks would be needed.         
#if 0                        
                        //!! TODO the above corners are based on the directions the two vehicles are *currently* pointing
                        //!! TODO but we need to rotate them here to face towards their targets, otherwise the rectangles will be squashed into parallellograms!
                        //!! TODO In fact actually, if the agent's vehicle is drifting, we need to widen agentRect to be a velocity aligned bounding box
                        Vector3 dirv = agentFrontLeft - agentRearLeft;//TODO 90 degrees from right - left would be better here
                        Quaternion quatRestoreFwd = dirv.getRotationTo(agentVelocity);//agentTargetHeading);//mAgentHeading);//UNIT_Z worked great: Vector3::UNIT_Z);//currentFacing.getRotationTo(mInitFacing);
                        Vector3 newUp = quatRestoreFwd * Vector3::UNIT_Y;//upv
                        Quaternion quatRestoreUp = newUp.getRotationTo(Vector3::UNIT_Y);//upv.getRotationTo(Vector3::UNIT_Y);
                        Quaternion quatRestore = quatRestoreUp * quatRestoreFwd;
                        agentFrontLeft = quatRestore * agentFrontLeft;
                        agentFrontRight = quatRestore * agentFrontRight;
                        agentRearLeft = quatRestore * agentRearLeft;
                        agentRearRight = quatRestore * agentRearRight;
                        
                        dirv = otherFrontLeft - otherRearLeft;
                        quatRestoreFwd = dirv.getRotationTo(otherVelocity);//otherTargetHeading);
                        newUp = quatRestoreFwd * Vector3::UNIT_Y;
                        quatRestoreUp = newUp.getRotationTo(Vector3::UNIT_Y);
                        quatRestore = quatRestoreUp * quatRestoreFwd;
                        otherFrontLeft = quatRestore * otherFrontLeft;
                        otherFrontRight = quatRestore * otherFrontRight;
                        otherRearLeft = quatRestore * otherRearLeft;
                        otherRearRight = quatRestore * otherRearRight;
                                                
                        //Note if the vehicles are head on, left and right will seem reversed for the other truck
                        //TODO these rectangles do potentially leave a blind spot if a vehicle remains alongside and slightly behind and we want to turn 
                        agentRect0 = agentFrontLeft;
                        agentRect1 = agentFrontRight; //TODO Probably best to use velocities here! front-rear worked better than TargetHeading but only when vehicle wasn't damaged
                        agentRect2 = agentRect0 + (/*agentTargetHeading*//*(agentFrontLeft-agentRearLeft).normalise()*/agentVelocity * lookAhead);//(mAgentHeading * lookAhead);
                        agentRect3 = agentRect1 + (/*agentTargetHeading*//*(agentFrontLeft-agentRearLeft).normalise()*/agentVelocity * lookAhead);//(mAgentHeading * lookAhead);
                        //TODO as well as projecting by velocity / heading, we need to run a further rectangle check for just the whole vehicle's
                        //current bounding box. For example a car that has spun out sideways won't get picked up currently.
                        otherRect0 = otherRearLeft;//BUG FIX - the rear of the vehicle in front needs to be avoided too!!! otherFrontLeft;
                        otherRect1 = otherRearRight;//otherFrontRight;
                        otherRect2 = otherRect0 + (/*otherTargetHeading*//*(otherFrontLeft-otherRearLeft).normalise()*/otherVelocity * lookAhead * 0.5f);//TODO the lookAhead here should be different and based on the other vehicle's speed
                        otherRect3 = otherRect1 + (/*otherTargetHeading*//*(otherFrontLeft-otherRearLeft).normalise()*/otherVelocity * lookAhead * 0.5f);
                        
                        //TODO find intersection points of the two rectangles
                        //e.g. https://www.codeproject.com/articles/15573/2d-polygon-collision-detection
                        
                        //If there's an intersection, we need to know how far up the first vehicle's rectangle the intersection happens.
                        //To do this, we can rotate both rectangles by the correct xz angle so that the first rectangle is axis aligned.
                        //Then it's trivial to check which corners of the second rectangle are contained in the first.
                        //If all corners are contained, we can use all four corners as intersection limits.
                        //Otherwise, we need to test each pair of rectangle sides to find intersection points where the lines cross.
                        //The z coordinates of these intersection limits will tell us how far along the first vehicle's path the intersection
                        //danger zone will start and end.
                        //Then we need to use the constant acceleration equations to estimate how close in time the two vehicles will be at those points.
                        //If it covers a long length of track, we could break it in to steps to examine (these steps could be potential waypoints)
                        //If there's a very long drawn out overtake, the vehicles may get very close (maybe close enough to collide) for a very long
                        //distance and time without their centre points touching (though probably corners would pass each other???).
                        
                        //Now we rotate both the rectangles so the agent's one is axis aligned:
                        dirv = agentFrontRight - agentFrontLeft;
                        Quaternion quatAxisAlign = dirv.getRotationTo(Vector3::UNIT_X);
                        newUp = quatAxisAlign * Vector3::UNIT_Y;
                        quatRestoreUp = newUp.getRotationTo(Vector3::UNIT_Y);
                        quatAxisAlign = quatRestoreUp * quatAxisAlign;
                        
                        agentRect0 = quatAxisAlign * agentRect0;
                        agentRect1 = quatAxisAlign * agentRect1;
                        agentRect2 = quatAxisAlign * agentRect2;
                        agentRect3 = quatAxisAlign * agentRect3;
                        
                        otherRect0 = quatAxisAlign * otherRect0;
                        otherRect1 = quatAxisAlign * otherRect1;
                        otherRect2 = quatAxisAlign * otherRect2;
                        otherRect3 = quatAxisAlign * otherRect3;
                        
                        //Calculate average current positions of the two vehicles, aligned to the agent's vehicle's axis
                        Vector3 agentAvgPosition = (agentRect0 + agentRect1 + (quatAxisAlign * agentRearLeft) + (quatAxisAlign * agentRearRight)) * 0.25f;
                        Vector3 otherAvgPosition = (otherRect0 + otherRect1 + (quatAxisAlign * otherRearLeft) + (quatAxisAlign * otherRearRight)) * 0.25f;
                  
                        //What we're interested in is how far along the track the vehicle's paths will start to overlap,
                        //how close in time the vehicles will be along that area of track and if close enough then
                        //by how much they will overlap at various points along the first vehicle's path
                        float closestIntersectionZ = 0.0f;
                        float furthestIntersectionZ = 0.0f;
                        float timeGap = 0.0f;
                        float timeGapThreshold = 0.5f;//Half a second for now
                        Vector3 nearestOverlapStart = Vector3::ZERO;
                        Vector3 nearestOverlapEnd = Vector3::ZERO;
                        Vector3 maxOverlapStart = Vector3::ZERO;
                        Vector3 maxOverlapEnd = Vector3::ZERO;
                        Vector3 furthestOverlapStart = Vector3::ZERO;
                        Vector3 furthestOverlapEnd = Vector3::ZERO;
                        float nearestOverlapAmount = 0.0f;
                        float furthestOverlapAmount = 0.0f;
                        float maxOverlapAmount = 0.0f;
                        //Check if the opponent is moving head on towards the agent
                        //!! TODO we also need to consider that the vehicle could be reversing! Not handled yet. Or sliding forwards upside down!
                        //This finds the angle measured anticlockwise from mAgentHeading to otherTargetHeading
                        float approach = fmod(atan2(otherTargetHeading.z-mAgentHeading.z,otherTargetHeading.x-mAgentHeading.x) * 180.0f/PI, 360.0f);
                        //float approach = Ogre::Degree(mAgentHeading.angleBetween(otherTargetHeading);
                        bool isHeadOn = approach > 90.0f && approach < 270.0f;//(otherAvgFront).squaredDistance((agentRect0 + agentRect1) * 0.5f) > (otherAvgBack).squaredDistance((agentRect0 + agentRect1) * 0.5f);
                        bool isFromRight = approach > 0.0f && approach < 180.0f;//other vehicle is pointing towards the LHS of the agent's path
                        bool isFromLeft = approach > 180.0f && approach < 360.0f;//other vehicle is pointing towards the RHS of the agent's path

                        //Now, if the other vehicle is actually behind our (agent's) vehicle (TODO handle reversing), it is THEIR responsibility,
                        //not ours, to avoid the collision. This helps prevent two AIs moving in the same direction.
                        if (!isHeadOn)
                        {
                            if (otherAvgPosition.z < agentAvgPosition.z)
                            {
                                continue;
                            }                        
                        }
                        //In the case of a head-on - if they just both move left, we should be all good. Otherwise we need some deterministic randomness or comms.
                        
                        if (otherRect0.x < agentRect1.x)
                        {
                            if (otherRect0.x > agentRect0.x)
                            {
                                if (otherRect0.z < agentRect2.z)
                                {
                                    if (otherRect0.z > agentRect0.z)
                                    {
                                        //otherRect0 is inside agentRect
                                    }
                                    else
                                    {
                                        //otherRect0 is directly below agentRect
                                    }
                                }
                            }
                            else
                            {
                                //otherRect0 is somewhere to the left of agentRect
                            }
                        }
                        
                        bool gotIntersection, linesAreParallel;
                        bool intersectsFF, intersectsFL, intersectsFR, intersectsFB;
                        bool intersectsLF, intersectsLL, intersectsLR, intersectsLB;
                        bool intersectsRF, intersectsRL, intersectsRR, intersectsRB;
                        bool intersectsBF, intersectsBL, intersectsBR, intersectsBB;
                        Vector3 intersectionFF, intersectionFL, intersectionFR, intersectionFB;
                        Vector3 intersectionLF, intersectionLL, intersectionLR, intersectionLB;
                        Vector3 intersectionRF, intersectionRL, intersectionRR, intersectionRB;
                        Vector3 intersectionBF, intersectionBL, intersectionBR, intersectionBB;
                        
                        //Test agentRect0->1 against otherRect0->1 (it's the fronts of the two trucks, so they're likely already colliding if there's an intersection, or would as they steer)
                        //Note we ignore return value as we only care if the intersection is within the bounds of the two lines
                        lineIntersectionXZ(agentRect0, agentRect1, otherRect0, otherRect1, intersectsFF, linesAreParallel, intersectionFF);
                        //Test agentRect0->1 against otherRect0->2 (front of agent against LHS of other rect)
                        lineIntersectionXZ(agentRect0, agentRect1, otherRect0, otherRect2, intersectsFL, linesAreParallel, intersectionFL);
                        //Test agentRect0->1 against otherRect1->3 (front of agent against RHS of other rect)
                        lineIntersectionXZ(agentRect0, agentRect1, otherRect1, otherRect3, intersectsFR, linesAreParallel, intersectionFR);
                        //Test agentRect0->1 against otherRect2->3 (front of agent against far end of other rect)
                        lineIntersectionXZ(agentRect0, agentRect1, otherRect2, otherRect3, intersectsFB, linesAreParallel, intersectionFB);

                        
                        //Test agentRect0->2 against otherRect0->1 (LHS of agent rect against front of other truck)
                        lineIntersectionXZ(agentRect0, agentRect2, otherRect0, otherRect1, intersectsLF, linesAreParallel, intersectionLF);
                        //Test agentRect0->2 against otherRect0->2 (LHS of agent against LHS of other rect)
                        lineIntersectionXZ(agentRect0, agentRect2, otherRect0, otherRect2, intersectsLL, linesAreParallel, intersectionLL);
                        //Test agentRect0->2 against otherRect1->3 (LHS of agent against RHS of other rect)
                        lineIntersectionXZ(agentRect0, agentRect2, otherRect1, otherRect3, intersectsLR, linesAreParallel, intersectionLR);
                        //Test agentRect0->2 against otherRect2->3 (LHS of agent against far end of other rect)
                        lineIntersectionXZ(agentRect0, agentRect2, otherRect2, otherRect3, intersectsLB, linesAreParallel, intersectionLB);
                        
                        
                        //Test agentRect1->3 against otherRect0->1 (RHS of agent rect against front of other truck)
                        lineIntersectionXZ(agentRect1, agentRect3, otherRect0, otherRect1, intersectsRF, linesAreParallel, intersectionRF);
                        //Test agentRect1->3 against otherRect0->2 (RHS of agent against LHS of other rect)
                        lineIntersectionXZ(agentRect1, agentRect3, otherRect0, otherRect2, intersectsRL, linesAreParallel, intersectionRL);
                        //Test agentRect1->3 against otherRect1->3 (RHS of agent against RHS of other rect)
                        lineIntersectionXZ(agentRect1, agentRect3, otherRect1, otherRect3, intersectsRR, linesAreParallel, intersectionRR);
                        //Test agentRect1->3 against otherRect2->3 (RHS of agent against far end of other rect)
                        lineIntersectionXZ(agentRect1, agentRect3, otherRect2, otherRect3, intersectsRB, linesAreParallel, intersectionRB);
                        
                        
                        //Test agentRect2->3 against otherRect0->1 (far end of agent rect against front of other truck)
                        lineIntersectionXZ(agentRect2, agentRect3, otherRect0, otherRect1, intersectsBF, linesAreParallel, intersectionBF);
                        //Test agentRect2->3 against otherRect0->2 (far end of agent against LHS of other rect)
                        lineIntersectionXZ(agentRect2, agentRect3, otherRect0, otherRect2, intersectsBL, linesAreParallel, intersectionBL);
                        //Test agentRect2->3 against otherRect1->3 (far end of agent against RHS of other rect)
                        lineIntersectionXZ(agentRect2, agentRect3, otherRect1, otherRect3, intersectsBR, linesAreParallel, intersectionBR);
                        //Test agentRect2->3 against otherRect2->3 (far end of agent against far end of other rect)
                        lineIntersectionXZ(agentRect2, agentRect3, otherRect2, otherRect3, intersectsBB, linesAreParallel, intersectionBB);

                        gotIntersection = gotIntersection || intersectsFF || intersectsFL || intersectsFR || intersectsFB ||
                        intersectsLF || intersectsLL || intersectsLR || intersectsLB ||
                        intersectsRF || intersectsRL || intersectsRR || intersectsRB ||
                        intersectsBF || intersectsBL || intersectsBR || intersectsBB;

                        if (!gotIntersection)
                        {
                            continue;
                        }
                        
                        //Min info we need is, closest and furthest z coordinate of intersection, then within that range,
                        //closest and furthest z coordiantes where time gap between vehicles < safety margin, then within that range,
                        //min and max x coordinate of the other vehicle. If our look-ahead is small enough, that should be enough.
                        //If not we then need to break that down into smaller chunks and get min and max x coordinate for each of those too to work out how far to move over.

                        //First, find closest and furthest z coordinate of intersection. Remember this is aligned to the agent vehicle's own velocity.
                        //Make sure these values are outside the range first !!! TODO check corners inside rect first as well!
                        closestIntersectionZ = agentRect2.z + 10000000.0f;
                        furthestIntersectionZ = agentRect0.z - 10000000.0f;
                        //These will hold approximations of how far the agent and the other vehicle have travelled
                        //at these closest and furthest intersection points and an estimate of the elapsed times at those points.
                        float agentDistAtClosest = 0.0f;
                        float otherDistAtClosest = 0.0f;
                        float agentTimeAtClosest = 0.0f;
                        float otherTimeAtClosest = 0.0f;
                        float agentDistAtFurthest = 0.0f;
                        float otherDistAtFurthest = 0.0f;
                        float agentTimeAtFurthest = 0.0f;
                        float otherTimeAtFurthest = 0.0f;                        
                        //For each of these intersections it's maybe worth finding distance between the intersection point and the start point of the other vehicle projected onto its direction of motion.
                        //This should give us the vehicle's displacement and using avg speed (1/2 u + v), we can work out the time to that point.
                        //Only trouble is, if we don't know what part of the car intersected we have an error of up to one car diagonal length, which is quite a lot.
                        //For now, we always just measure to the centre (average) point of the vehicle as a rough approximation.
                        if (intersectsFF)
                        {
                            closestIntersectionZ = intersectionFF.z;
                            furthestIntersectionZ = intersectionFF.z;
                            agentDistAtClosest = (intersectionFF - agentAvgPosition).length();
                            otherDistAtClosest = (intersectionFL - otherAvgPosition).length();
                        }
                        if (intersectsFL)
                        {
                            findIntersectionZLimits(intersectionFL, closestIntersectionZ, furthestIntersectionZ, agentAvgSpeed, otherAvgSpeed, agentDistAtClosest, otherDistAtClosest, agentDistAtFurthest, otherDistAtFurthest, agentTimeAtClosest, agentTimeAtFurthest, otherTimeAtClosest, otherTimeAtFurthest);
                        }
                        if (intersectsFR)
                        {
                            findIntersectionZLimits(intersectionFR, closestIntersectionZ, furthestIntersectionZ, agentAvgSpeed, otherAvgSpeed, agentDistAtClosest, otherDistAtClosest, agentDistAtFurthest, otherDistAtFurthest, agentTimeAtClosest, agentTimeAtFurthest, otherTimeAtClosest, otherTimeAtFurthest);
                        }
                        if (intersectsFB)
                        {
                            findIntersectionZLimits(intersectionFB, closestIntersectionZ, furthestIntersectionZ, agentAvgSpeed, otherAvgSpeed, agentDistAtClosest, otherDistAtClosest, agentDistAtFurthest, otherDistAtFurthest, agentTimeAtClosest, agentTimeAtFurthest, otherTimeAtClosest, otherTimeAtFurthest);
                        }
                        if (intersectsLF)
                        {
                            findIntersectionZLimits(intersectionLF, closestIntersectionZ, furthestIntersectionZ, agentAvgSpeed, otherAvgSpeed, agentDistAtClosest, otherDistAtClosest, agentDistAtFurthest, otherDistAtFurthest, agentTimeAtClosest, agentTimeAtFurthest, otherTimeAtClosest, otherTimeAtFurthest);
                        }
                        if (intersectsLL)
                        {
                            findIntersectionZLimits(intersectionLL, closestIntersectionZ, furthestIntersectionZ, agentAvgSpeed, otherAvgSpeed, agentDistAtClosest, otherDistAtClosest, agentDistAtFurthest, otherDistAtFurthest, agentTimeAtClosest, agentTimeAtFurthest, otherTimeAtClosest, otherTimeAtFurthest);
                        }
                        if (intersectsLR)
                        {
                            findIntersectionZLimits(intersectionLR, closestIntersectionZ, furthestIntersectionZ, agentAvgSpeed, otherAvgSpeed, agentDistAtClosest, otherDistAtClosest, agentDistAtFurthest, otherDistAtFurthest, agentTimeAtClosest, agentTimeAtFurthest, otherTimeAtClosest, otherTimeAtFurthest);
                        }
                        if (intersectsLB)
                        {
                            findIntersectionZLimits(intersectionLB, closestIntersectionZ, furthestIntersectionZ, agentDistAtClosest, otherDistAtClosest, agentDistAtFurthest, otherDistAtFurthest);
                        }
                        if (intersectsRF)
                        {
                            findIntersectionZLimits(intersectionRF, closestIntersectionZ, furthestIntersectionZ, agentDistAtClosest, otherDistAtClosest, agentDistAtFurthest, otherDistAtFurthest);
                        }
                        if (intersectsRL)
                        {
                            findIntersectionZLimits(intersectionRL, closestIntersectionZ, furthestIntersectionZ, agentDistAtClosest, otherDistAtClosest, agentDistAtFurthest, otherDistAtFurthest);
                        }
                        if (intersectsRR)
                        {
                            findIntersectionZLimits(intersectionRR, closestIntersectionZ, furthestIntersectionZ, agentDistAtClosest, otherDistAtClosest, agentDistAtFurthest, otherDistAtFurthest);
                        }
                        if (intersectsRB)
                        {
                            findIntersectionZLimits(intersectionRB, closestIntersectionZ, furthestIntersectionZ, agentDistAtClosest, otherDistAtClosest, agentDistAtFurthest, otherDistAtFurthest);
                        }
                        if (intersectsBF)
                        {
                            findIntersectionZLimits(intersectionBF, closestIntersectionZ, furthestIntersectionZ, agentDistAtClosest, otherDistAtClosest, agentDistAtFurthest, otherDistAtFurthest);
                        }
                        if (intersectsBL)
                        {
                            findIntersectionZLimits(intersectionBL, closestIntersectionZ, furthestIntersectionZ, agentDistAtClosest, otherDistAtClosest, agentDistAtFurthest, otherDistAtFurthest);
                        }
                        if (intersectsBR)
                        {
                            findIntersectionZLimits(intersectionBR, closestIntersectionZ, furthestIntersectionZ, agentDistAtClosest, otherDistAtClosest, agentDistAtFurthest, otherDistAtFurthest);
                        }
                        if (intersectsBB)
                        {
                            findIntersectionZLimits(intersectionBB, closestIntersectionZ, furthestIntersectionZ, agentDistAtClosest, otherDistAtClosest, agentDistAtFurthest, otherDistAtFurthest);
                        }                        
                        
                        //s=ut+1/2at^2
                        //avg velocity = u + 1/2 at
                        //We need to get the average speed.
                        //See BeamEngine.cpp from line 478 to get an idea how to estimate the acceleration.
                        //or probably better to just cache recent velocities for each truck and extrapolate based on current engine power (from rpm)
                        //and max speed.
                        //TODO - account for acceleration! TODO ?? Are these velocities projected in correct directions?
                        float agentAvgSpeed = fabsf(agentVelocity.length());
                        float otherAvgSpeed = fabsf(otherVelocity.length());
                        if (agentAvgSpeed > 0.0f && otherAvgSpeed > 0.0f)
                        {
                            agentTimeAtClosest = agentDistAtClosest / agentAvgSpeed;
                            otherTimeAtClosest = otherDistAtClosest / otherAvgSpeed;
                            agentTimeAtFurthest = agentDistAtFurthest / agentAvgSpeed;
                            otherTimeAtFurthest = agentDistAtFurthest / agentAvgSpeed;
                        }
                        
                        
                        //TODO This huge number of ifs is horrendous and is probably overcomplicating things. Simpler(?!) code above
                        /*
                        if (intersectsFF)
                        {
                            nearestOverlapStart = nearestOverlapEnd = intersectionFF;
                            if (intersectsFL)
                            {
                                if (intersectsFR)
                                {//FR FF FL - shouldn't be possible unless front of other truck is exactly along the line of the front of the agent truck AND agent is wider or exactly aligned
                                    if (isHeadOn)
                                    {
                                        nearestOverlapStart = intersectsFR;
                                        nearestOverlapEnd = intersectsFL;
                                    }
                                    else
                                    {//For this I think the other truck would be exactly on top of or underneath (or inside) the agent truck
                                        nearestOverlapStart = intersectsFL;
                                        nearestOverlapEnd = intersectsFR;
                                    }
                                    nearestOverlapAmount = nearestOverlapEnd.x - nearestOverlapStart.x;
                                }
                                else if (intersectsFB)
                                {//FB FL FF
                                    if (isFromLeft)
                                    {//For this I think the other truck would be resting sideways exactly across the top or bottom (or through) the agent truck!
                                        nearestOverlapStart = intersectsFB;
                                        nearestOverlapEnd = intersectsFF;
                                    }
                                    else
                                    {
                                        nearestOverlapStart = intersectsFF;
                                        nearestOverlapEnd = intersectsFB;
                                    }
                                }
                                else
                                {//FL FF
                                    if (isFromLeft)
                                    {
                                        nearestOverlapStart = intersectsFL;
                                        nearestOverlapEnd = intersectsFF;
                                    }
                                    else
                                    {//Other truck would be overlapping the agent diagonally
                                        nearestOverlapStart = intersectsFF;
                                        nearestOverlapEnd = intersectsFL;
                                    }
                                }
                            }
                            else if (intersectsFR)
                            {
                                if (intersectsFB)
                                {//FF FR FB
                                    if (isFromLeft)
                                    {
                                        nearestOverlapStart = intersectsFB;
                                        nearestOverlapEnd = intersectsFF;
                                    }
                                    else
                                    {//For this I think the other truck would be resting sideways exactly across the top or bottom (or through) the agent truck!
                                        nearestOverlapStart = intersectsFF;
                                        nearestOverlapEnd = intersectsFB;
                                    }                                    
                                }
                                else
                                {//FF FR
                                    if (isFromLeft)
                                    {
                                        nearestOverlapStart = intersectsFR;
                                        nearestOverlapEnd = intersectsFF;
                                    }
                                    else
                                    {//Other truck would be overlapping the agent diagonally
                                        nearestOverlapStart = intersectsFF;
                                        nearestOverlapEnd = intersectsFR;
                                    }                                    
                                }
                            }
                            else if (intersectsFB)
                            {//FF FB
                                if (fromLeft)
                                {
                                    nearestOverlapStart = intersectsFB;
                                    nearestOverlapEnd = intersectsFF;
                                }
                                else
                                {
                                    nearestOverlapStart = intersectsFF;
                                    nearestOverlapEnd = intersectsFB;
                                }
                            }
                            else
                            {//FF only - Need to check other truck's orientation to determine whether it overlaps LHS or RHS of agent's front
                                if (fromLeft)
                                {
                                    nearestOverlapStart = agentRect0; // Note of course the obstacle extends beyond agentRect0
                                    nearestOverlapEnd = intersectsFF;
                                }
                                else
                                {
                                    nearestOverlapStart = agentRect1;
                                    nearestOverlapEnd = intersectsFF;
                                }
                            }
                        }
                        else if (intersectsFL)
                        {
                            if (intersectsFR)
                            {
                                if (intersectsFB)
                                {//FL FB FR
                                    
                                }
                                else
                                {//FL FR - left and right may be reversed if head on
                                    
                                }
                            }
                            else if (intersectsFB)
                            {//FL FB
                                
                            }
                        }
                        else if (intersectsFR)
                        {
                            
                        }
                        else if (intersectsFB)
                        {
                            //FB only - Need to check other truck's orientation to determine whether it overlaps LHS or RHS of agent's front
                            if (fromLeft)
                            {
                                nearestOverlapStart = agentRect0; // Note of course the obstacle extends beyond agentRect0
                                nearestOverlapEnd = intersectsFF;
                            }
                            else
                            {
                                nearestOverlapStart = agentRect1;
                                nearestOverlapEnd = intersectsFF;
                            }
                        }
                        */
#endif                        
                        //!!! TEMP TEST Just test budging the waypoint over a bit
                        if (collisionOccurs)//gotIntersection)
                        {
                            //TODO we need to be absolutely sure we're correcting course for a vehicle IN FRONT - I've noticed a vehicle in front seems to kind of move in unison as the rear one moves over!
                            //RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, 
                            //    "Intersection Vehicle: " + TOSTRING(beam->trucknum) + "  Other: " + TOSTRING(otherTruck->trucknum) + " cur. pos (" + 
                            //    TOSTRING(mAgentAbsPosition.x) + ", " + TOSTRING(mAgentAbsPosition.y) + ", " + TOSTRING(mAgentAbsPosition.z) + ");", "note.png");
#if 0
                            //Recalculate the heading and steering force from new waypoint - cosmic vole February 28 2017
                            //TEMP TEST - Just budge it off course a bit!
                            TargetPosition = current_waypoint;
                            if (fabsf(current_waypoint.x - mAgentAbsPosition.x) > fabsf(current_waypoint.z - mAgentAbsPosition.z))
                            {
                                // TargetPosition.z += 35.0f;
                            }
                            else
                            {
                              //  TargetPosition.x += 35.0f; 
                            }
                            TargetPosition.y = 0; //Vector3 > Vector2
                            TargetOrientation = Quaternion::ZERO;
                            //!!! TEMP TEST - just budge it off course just a few degrees! cosmic vole
                            //TargetOrientation = Quaternion(Degree(-35.0f), Vector3::NEGATIVE_UNIT_Y) * TargetOrientation;
                            //Adjust speed and power TODO we need to look at how tight we have to turn as well as current speed and turns coming up to judge what adjustment if any is needed here - cosmic vole
                            float speed = waypoint_speed[current_waypoint_id];
                            //if (fabsf((mAgentAbsPosition - otherPosition).length()) < 4.0f && agentVelocity.length() > otherVelocity.length())// && agentVelocity.length())
                            //{
                            //    if (speed)
                            //        maxspeed = speed * 0.6f;
                            //    float power = waypoint_power[current_waypoint_id];
                            //    if (power)
                            //        acc_power = power * 0.6f;
                            //}
                            
                            mAgentPosition.y = 0; //Vector3 > Vector2
                            agentRotation = beam->getHeadingDirectionAngle();
                            
                            //!!! TEMP TEST - just budge it off course just a few degrees! cosmic vole
                            //agentRotation += Degree(60.0f).valueRadians();
                            
                            Quaternion mAgentOrientation2 = Quaternion(Radian(agentRotation), Vector3::NEGATIVE_UNIT_Y);
                            mAgentOrientation2.normalise();

                            mVectorToTarget = TargetPosition - mAgentPosition; // A-B = B->A
                            mAgentPosition.normalise();

                            Vector3 mAgentHeading2 = mAgentOrientation2 * mAgentPosition;
                            mTargetHeading = TargetOrientation * TargetPosition;
                            mAgentHeading2.normalise();
                            mTargetHeading.normalise();

                            // Compute new torque scalar (-1.0 to 1.0) based on heading vector to target.
                            mSteeringForce = mAgentOrientation2.Inverse() * mVectorToTarget; 
                            //mSteeringForce.x += 2.0f;//TEMP TEST
                            break;//TEMP TEST
#endif                            
                        }
                    }
                }
                //break;
            }
        }
                                
    }
    
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

    //cosmic vole added collision avoidance steering March 8 2017

    if (collision_avoid_time > 0.0f)
    {
        time_since_last_collision_avoid = 0.0f;
        //TODO make sure we haven't driven past collision_avoid_target yet, or project it further down the track
        Vector3 collisionAvoidSteeringForce = mAgentOrientation.Inverse() * (collision_avoid_target - mAgentAbsPosition); 
        if (collisionAvoidSteeringForce.x > 0.0f)//(collision_avoid_target.x > 0.0f)
            mYaw = 1;
        else
        if (collisionAvoidSteeringForce.x < 0.0f)//(collision_avoid_target.x < 0.0f)
            mYaw = -1;
        else
            mYaw = beam->hydrodirstate;
        collision_avoid_time -= dt;
        if (collision_avoid_time < 0.0f)
            collision_avoid_time = 0.0f;
        usePIDControl = false;
        beam->hydroSpeedCoupling = true;// re-enable digital steering input as we don't want to instantaneously jump to full lock
        //if (usePIDControl)
        //{
        //    steeringForceRaw = collision_avoid_target.x * 100.0f;//2000.0f;//30.0f;
        //}
    }
    else
    {
        time_since_last_collision_avoid += dt;
        
        if (time_since_last_collision_avoid < 3.0f)//4.0f)//2.3f)//0.25f)//0.5f 0.7f)
        {
            //Gradually straighten the steering after a collision avoidance maneuvre (ignoring waypoints) - TODO not sure if this is making things worse!
            //if (mAgentAbsVelocity.length() > 12.0f)
            //{
            //    
            //}
            mYaw = beam->hydrodirstate * 0.65f + mYaw * 0.35f;//0.25f + mYaw * 0.45f;//beam->hydrodirstate * 0.99f;//0.0f;//beam->hydrodirstate * 0.1f;
            usePIDControl = false;
            //if (steering_delay < 0.1f)//0.2f) Note when steering delay was higher it was just making the cars see-saw / fishtail all the time!
            //{
            //    steering_delay = 0.1f;//0.2f;
            //}
            //TODO we don't actually need to roll off the power if we just braked in a straight line rather than swerving for the collision avoidance
            if (acc_power > 0.45f)
            {
                acc_power = 0.45f;
            }
        }
        
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
    if (fabsf(rollAngle0) > 5.0f)
    {
        //if (rollMsgCount == 0)
        //    RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, "Roll detected. angle0: " + TOSTRING(rollAngle0) + " angle: " + TOSTRING(rollAngle) + " angle2: " + TOSTRING(rollAngle2) + " \n", "note.png");
        //rollMsgCount++;
        //if (rollMsgCount > 5)
        //{
        //    rollMsgCount = 0;
        //}
        if (rollAngle0 > /*12.0f*/14.0f && rollAngle0 < 180.0f)
        {
            //If angle is anticlockwise, looking from rear of vehicle, it has its right side raised, so we need to steer left
            mYaw = -1.0f;
            beam->hydroSpeedCoupling = true;// re-enable digital steering input as we don't want to instantaneously jump to full lock
            usePIDControl = false;
        }
        else if (rollAngle0 < /*-12.0f*/-15.0f && rollAngle0 > -180.0f)
        {
            //If angle is clockwise, looking from rear of vehicle, it has its left side raised, so we need to steer right
            mYaw = 1.0f;
            beam->hydroSpeedCoupling = true;// re-enable digital steering input as we don't want to instantaneously jump to full lock
            usePIDControl = false;
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
    float maxsteermult = 11.0f;//was 11

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
        
      
        //Sanity check for collision avoidance - TODO remove this if we can get PID to handle it normally
        if (collision_avoid_time > 0.0f && mYaw != 0.0f && /*abs(mYaw) < 0.25f &&*/ !hasCarOnLeft && !hasCarOnRight)
        {
            RoR::App::GetConsole()->putMessage(RoR::Console::CONSOLE_MSGTYPE_SCRIPT, RoR::Console::CONSOLE_SYSTEM_NOTICE, "PID made collision avoidance too small. steeringForceRaw: " + TOSTRING(steeringForceRaw) + " mYaw: " + TOSTRING(mYaw) + "\n", "note.png");
            if (collision_avoid_target.x > 0.0f)
                mYaw = 1.0f;
            else
            if (collision_avoid_target.x < 0.0f)
                mYaw = -1.0f;
            //Make sure steering is quick enough to complete the avoidance manoeuvre
            maxsteermult = 40.0f;
      
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

        if (abs(mYaw) < 0.5f)
        {
            if (kmh_wheel_speed < maxspeed - 1)
            {
                beam->brake = 0;
                beam->engine->autoSetAcc(acc_power);
            }
            else if (kmh_wheel_speed > maxspeed + 1)
            {
                beam->brake = beam->brakeforce / 1.5;// / 3;
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
            if (kmh_wheel_speed < maxspeed - 1)
            {
                beam->brake = 0;
                beam->engine->autoSetAcc(acc_power / 3);
            }
            else if (kmh_wheel_speed > maxspeed + 1)
            {
                //cosmic vole - don't apply brakes when skidding March 8 2017
                if (skidding || (skid_time > 0.0f && skid_time < 0.8f))
                {
                    beam->brake = /*0.0f;*/beam->brakeforce / 6.0f;//8;//0.0f;
                }
                else
                {
                    beam->brake = beam->brakeforce / 3.25;// / 3,  / 2;
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

Vector3 getClosetPointOnLine(Vector3& A, Vector3& B, Vector3& p, bool clampToBounds)
{
    Vector3 AP = p - A;
    Vector3 AB = B - A;
    float ab2 = AB.x*AB.x + AB.y*AB.y;
    float ap_ab = AP.x*AB.x + AP.y*AB.y;
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

    //Sometimes better to just sum the last errors
    float averageAmount = 20.0f;//40.0f;//20.0f;

    sumErrorsAcrossTrack = sumErrorsAcrossTrack + ((errorAcrossTrack - sumErrorsAcrossTrack) / averageAmount);

    factor += PID_I * sumErrorsAcrossTrack;

    //D
    float d_dt_error = (errorAcrossTrack - lastErrorAcrossTrack) / dt;

    factor += PID_D * d_dt_error;

    lastErrorAcrossTrack = errorAcrossTrack;

    return factor;    
}

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

#endif // USE_ANGELSCRIPT
