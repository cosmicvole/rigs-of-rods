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

#include "Beam.h"
#include "VehicleAI.h"
#include "RaceResult.h"
#include "Race.h"
#include "ChampionshipManager.h"

const std::map<int, int>& Race::GenerateGridPositions(int numAICompetitors)
{
    //We start off with the drivers sorted by their ability and apply a sort of flattened probability distribution to that
    //Once a driver's position has been calculated, we don't let another driver go in that position BUT
    //to avoid the chance of such collisions we can generate a decimal random position and compare those to tie break.
    ChampionshipManager& cm = ChampionshipManager::getSingleton();
    const std::vector<RaceCompetitor>& competitors = cm.getCompetitors();
    int numCompetitors = numAICompetitors;
    if (numCompetitors > competitors.size())
    {
        numCompetitors = competitors.size();
    }
    //Add one on for the player
    //numCompetitors++;
    //gridPositions.reserve(numCompetitors);
    gridPositions.clear();
    
    for (int i = 0; i < numCompetitors; i++)
    {
        //Number zero is the player
        int position = rand() % numCompetitors;
        while (gridPositions.find(position) != gridPositions.end()) //TODO fix this condition
        {
            position = rand() % numCompetitors;
        }
        gridPositions[position] = i;
        RaceCompetitor competitor = competitors[i];
        //We save the current grid position into the competitor object too for easy reverse lookup
        competitor.SetGridPosition(position);
        cm.updateCompetitorByIndex(competitor, i);
    }
    return gridPositions;
}

void Race::SetValueAtWaypoint(int index, int value_id, float value)
{
    //int waypointid = waypoint_ids[id];
    //if (waypointid)
    {
        switch (value_id)
        {
        case AI_SPEED:
            waypoint_speed.emplace(index, value);
            //adjusted_waypoint_speed.emplace(waypointid, value);
            break;
        case AI_POWER:
            waypoint_power.emplace(index, value);
            break;
        default:
            break;
        }
    }
}

void Race::AddWaypointsToVehicle(VehicleAI *vehicleAI)
{
    int laps = numberOfLaps;
    if (numberOfLaps < 1)
    {
        laps = 1;
    }
    for (int l = 0; l <= laps; l++)
    {
        for (int i=0; i<waypoints.size(); i++)
        {
            Ogre::Vector3 waypoint = waypoints[i];
            int wayptNum = i + 1;
            Ogre::String waypointID = Ogre::String("waypoint") + TOSTRING(wayptNum + l * waypoints.size());
            vehicleAI->AddWaypoint(waypointID, waypoint);
            float speed = waypoint_speed[i];
            float power = waypoint_power[i];
            if (l >= speedLimitStartLap && l <= speedLimitEndLap && speedLimitStartLap >= 0 &&
                wayptNum >= speedLimitStartWaypoint && wayptNum <= speedLimitEndWaypoint &&
                speedLimitStartWaypoint > 0 && speedLimit >= 0.0f)
            {
                if (speed > speedLimit)
                {
                    speed = speedLimit;
                }
            }
            if (speed > 0.0f)
            {
                int type = AI_SPEED;
                vehicleAI->SetValueAtWaypoint(waypointID, type, speed);
            }
            if (power > 0.0f)
            {
                int type = AI_POWER;
                vehicleAI->SetValueAtWaypoint(waypointID, type, power);
            }
        }
    }
    //As we've added a preliminary warm-up/out lap, we need to set the first waypoint to one lap after that
    Ogre::String startWaypointID = Ogre::String("waypoint") + TOSTRING(waypoints.size()+1);
    vehicleAI->SetCurrentWaypoint(startWaypointID);
}

Race *RaceFactory()
{ 
    return new Race();
}

#endif