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

/// @file   Race.h
/// @brief  Race AI route, logic and results
/// @author cosmic vole
/// @author Waypoint code by AnotherFoxGuy
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

class RaceResult;

class RaceCompetitor //: public ZeroedMemoryAllocator
{
public:
    RaceCompetitor()
    {
        this->name = "";
        this->model = "";
        this->skin = "";
        this->driverScale = 0.85f; //TODO set to 1.0 and load from championship file!
        this->truckNum = -1;
        this->beam = nullptr;
        this->gridPosition = -1;
        this->points = -1;
        this->bestPosition = -1;
        this->podiums = -1;
        this->torqueMultiplier = 1.0f;
        this->brakingMultiplier = 1.0f;
        this->gripMultiplier = 1.0f;
        this->aggression = 1.0f;
    }
    static RaceCompetitor NONE;
    Ogre::UTFString GetName() const
    {
        return name;
    }
    Ogre::UTFString GetModel() const
    {
        return model;
    }
    Ogre::UTFString GetSkin() const
    {
        return skin;
    }
    inline int GetTruckNum() const
    {
        return truckNum;
    }
    inline float GetDriverScale() const
    {
        return driverScale;
    }
    inline int GetGridPosition() const
    {
        return gridPosition;
    }
    inline int GetPoints() const
    {
        return points;
    }
    inline int GetBestPosition() const
    {
        return bestPosition;
    }
    inline int GetPodiums() const
    {
        return podiums;
    }
    inline void SetName(const Ogre::UTFString& name)
    {
        this->name = name;
    }
    inline void SetModel(const Ogre::UTFString& model)
    {
        this->model = model;
    }
    inline void SetSkin(const Ogre::UTFString& skin)
    {
        this->skin = skin;
    }
    inline void SetTruckNum(int truckNum)
    {
        this->truckNum = truckNum;
    }
    inline void SetDriverScale(float driverScale)
    {
        this->driverScale = driverScale;
    }
    inline void SetGridPosition(int gridPosition)
    {
        this->gridPosition = gridPosition;
    }
    inline void SetPoints(int points)
    {
        this->points = points;
    }
    inline void SetBestPosition(int bestPosition)
    {
        this->bestPosition = bestPosition;
    }
    inline void SetPodiums(int podiums)
    {
        this->podiums = podiums;
    }
    RaceCompetitor & operator = (const RaceCompetitor &other)
    {
        // Don't do anything if it is the same reference
        if (&other == this)
            return *this;
        this->name = other.name;
        this->model = other.model;
        this->skin = other.skin;
        this->driverScale = other.driverScale;
        this->truckNum = other.truckNum;
        this->beam = other.beam;
        this->gridPosition = other.gridPosition;
        this->points = other.points;
        this->bestPosition = other.bestPosition;
        this->podiums = other.podiums;
        this->torqueMultiplier = other.torqueMultiplier;
        this->brakingMultiplier = other.brakingMultiplier;
        this->gripMultiplier = other.gripMultiplier;
        this->aggression = other.aggression;
        return *this;        
    }
    /*
    RaceCompetitor & operator == (const RaceCompetitor &other)
    {
        if (&other == this)
            return true;
        if (this == &RaceCompetitor::NONE || &other == &RaceCompetitor::NONE)
            return false;
        return !(*this < other || other < *this);
    }
    */
    
    friend bool operator <(const RaceCompetitor& a, const RaceCompetitor& b)
    {
        if (a.GetPoints() > b.GetPoints())
        {
            return true;
        }
        if (a.GetPoints() == b.GetPoints())
        {
            if (a.GetPodiums() > b.GetPodiums())
            {
                return true;
            }
            else if (a.GetPodiums() == b.GetPodiums())
            {
                if (a.GetBestPosition() < b.GetBestPosition() && a.GetBestPosition() > 0)
                {
                    return true;
                }
                //TODO Ideally need a further tie break, like total time across all races
            }
        }
        return false;
    }
 
    inline void SetAggression(float aggression) {this->aggression = aggression;}
    inline void SetBrakingMultiplier(float brakingMultiplier) {this->brakingMultiplier = brakingMultiplier;}
    inline void SetGripMultiplier(float gripMultiplier) {this->gripMultiplier = gripMultiplier;}
    inline void SetTorqueMultiplier(float torqueMultiplier) {this->torqueMultiplier = torqueMultiplier;}
    inline float GetAggression() const {return aggression;}
    inline float GetBrakingMultiplier() const {return brakingMultiplier;}
    inline float GetGripMultiplier() const {return gripMultiplier;}
    inline float GetTorqueMultiplier() const {return torqueMultiplier;}

private:
    Ogre::String name;
    Ogre::String model;
    Ogre::String skin;
    float driverScale;
    int truckNum;
    Beam *beam;
    int gridPosition;
    int points;
    int bestPosition;
    int podiums;
    float torqueMultiplier;
    float brakingMultiplier;
    float gripMultiplier;
    float aggression;
};

/* NOT USED YET May 31 2017. If we do end up using this it should have a master copy of the waypoints, edge points (and maybe checkpoints)
 * initialized in the angelscript and adding vehicles to the race should probably automatically assign those waypoints to them and possibly
 * even handle grid positions.
 */
//cosmic vole May 28 2017 - TODO move into separate file. Possibly move coll. avoidance grid data inside this class also.
class Race : public ZeroedMemoryAllocator
{
public:
    Race() { disabled = false; raceStartTime = 0.0f; raceElapsedTime = 0.0f; }
    Race(int raceID) { this->raceID = raceID; disabled = false; }
    void Start(float timeNow) { raceStartTime = timeNow; isFinished = false; isStarted = true; }
    void Stop() { isFinished = true; }
    inline void Update(float dt) { if (IsRunning()) raceElapsedTime += dt; }
    double GetRaceStartTime()
    { 
        LOG("GetRaceStartTime() for raceID:" + TOSTRING(raceID) + " Time: " + TOSTRING(raceStartTime) + " Name: " + trackName + ".");
        return raceStartTime;
    }
    inline float GetElapsedRaceTime() { return raceElapsedTime; }
    inline bool IsDisabled() { return disabled; }
    inline void Disable() { disabled = true; }
    inline void Enable() { disabled = false; }
    inline bool IsStarted() { return isStarted; }
    inline bool IsFinished() { return isFinished; }
    inline bool IsRunning() { return isStarted && !isFinished; }
    inline bool IsPaused() { return isPaused; }
    inline int GetID() const { return raceID; }
    inline Ogre::String GetTrackName() const { return trackName; }
    inline Ogre::String GetTerrain() const { return terrain; }
    inline Ogre::String GetTerrainFileName() const { return terrainFileName; }
    inline RaceResult GetRaceResult(int truckNum) { return results[truckNum]; }
    std::vector<RaceResult> GetSortedRaceResults()
    {
        std::vector<RaceResult> sorted;
        for (std::map<int, RaceResult>::iterator it = results.begin(); it != results.end(); ++it)
        {
            sorted.push_back(it->second);
        }
        std::sort(sorted.begin(), sorted.end());
        return sorted;
    }
    inline Ogre::Vector3 GetGridPosition1() { return gridPosition1; }
    inline Ogre::Vector3 GetGridPosition2() { return gridPosition2; }
    inline Ogre::Vector3 GetGridStep() { return gridStep; }
    inline Ogre::Vector3 GetVehicleGridRotation() const { return vehicleGridRotation; }
    inline void SetID(int raceID) { this->raceID = raceID; }
    inline void SetTerrain(Ogre::String terrain) { this->terrain = terrain; }
    inline void SetGridPosition(Ogre::Vector3 gridPosition1, Ogre::Vector3 gridPosition2, Ogre::Vector3 gridStep)
    {
        this->gridPosition1 = gridPosition1;
        this->gridPosition2 = gridPosition2;
        this->gridStep = gridStep;
    }
    inline void SetVehicleGridRotation(Ogre::Vector3 rotationDegrees) { vehicleGridRotation = rotationDegrees; }
    inline void SetTrackName(Ogre::String trackName) { this->trackName = trackName; };
    inline void SetRaceResult(RaceResult& result) { results[result.truckNum] = result; }
    inline void SetTerrainFileName(Ogre::String terrainFileName) { this->terrainFileName = terrainFileName; };
    void SetRaceResult(int truckNum, Ogre::Vector3 position, int lastWaypointID)
    {
        RaceResult& r = results[truckNum];
        //TODO!
    }
    inline void SetLapRecord(float lapRecord) { this->lapRecord = lapRecord; }
    inline float GetLapRecord() const { return lapRecord; }
    inline void SetFastestLapTime(float fastestLapTime) {this->fastestLapTime = fastestLapTime;}
    inline float GetFastestLapTime() const {return fastestLapTime;}
    void ClearWaypoints()
    {
        waypoints.clear();
    }
    void AddWaypoint(Ogre::Vector3 waypoint)
    {
        waypoints.emplace(waypoints.size(), waypoint);
        //TODO we'll have to add these to the VehicleAIs and generate our collision avoidance grid at the right time too!
    }
    void AddRoadEdgePointLeft(Ogre::Vector3 edgePoint)
    {
        road_edge_points_left.push_back(edgePoint);
    }
    void AddRoadEdgePointRight(Ogre::Vector3 edgePoint)
    {
        road_edge_points_right.push_back(edgePoint);
    }
    void SetValueAtWaypoint(int index, int value_id, float value);
    const std::map<int, int>& GenerateGridPositions(int numCompetitors);
    void AddWaypointsToVehicle(VehicleAI *vehicleAI);
    inline void SetNumberOfLaps(int numberOfLaps) {this->numberOfLaps = numberOfLaps;}
    inline int GetNumberOfLaps() const {return numberOfLaps;}
    /*Performance setting getters / setters:*/
    inline void SetMaxNormalizedPower(float maxNormalizedPower) {this->maxNormalizedPower = maxNormalizedPower;}
    inline void SetMaxSpeed(float maxSpeed) {this->maxSpeed = maxSpeed;}
    inline void SetMinNormalizedPower(float minNormalizedPower) {this->minNormalizedPower = minNormalizedPower;}
    inline void SetMinSpeed(float minSpeed) {this->minSpeed = minSpeed;}
    inline void SetNormalizedPowerDeadzone(float normalizedPowerDeadzone) {this->normalizedPowerDeadzone = normalizedPowerDeadzone;}
    inline void SetNormalizedPowerSaturation(float normalizedPowerSaturation) {this->normalizedPowerSaturation = normalizedPowerSaturation;}
    inline void SetPowerValueOffset(float powerValueOffset) {this->powerValueOffset = powerValueOffset;}
    inline void SetPowerValueScale(float powerValueScale) {this->powerValueScale = powerValueScale;}
    inline void SetSpeedDeadzone(float speedDeadzone) {this->speedDeadzone = speedDeadzone;}
    inline void SetSpeedLimit(float speedLimit) {this->speedLimit = speedLimit;}
    inline void SetSpeedLimitEndLap(int speedLimitEndLap) {this->speedLimitEndLap = speedLimitEndLap;}
    inline void SetSpeedLimitEndWaypoint(int speedLimitEndWaypoint) {this->speedLimitEndWaypoint = speedLimitEndWaypoint;}
    inline void SetSpeedLimitStartLap(int speedLimitStartLap) {this->speedLimitStartLap = speedLimitStartLap;}
    inline void SetSpeedLimitStartWaypoint(int speedLimitStartWaypoint) {this->speedLimitStartWaypoint = speedLimitStartWaypoint;}
    inline void SetSpeedSaturation(float speedSaturation) {this->speedSaturation = speedSaturation;}
    inline float GetMaxNormalizedPower() const {return maxNormalizedPower;}
    inline float GetMaxSpeed() const {return maxSpeed;}
    inline float GetMinNormalizedPower() const {return minNormalizedPower;}
    inline float GetMinSpeed() const {return minSpeed;}
    inline float GetNormalizedPowerDeadzone() const {return normalizedPowerDeadzone;}
    inline float GetNormalizedPowerSaturation() const {return normalizedPowerSaturation;}
    inline float GetPowerValueOffset() const {return powerValueOffset;}
    inline float GetPowerValueScale() const {return powerValueScale;}
    inline float GetSpeedDeadzone() const {return speedDeadzone;}
    inline float GetSpeedLimit() const {return speedLimit;}
    inline int GetSpeedLimitEndLap() const {return speedLimitEndLap;}
    inline int GetSpeedLimitEndWaypoint() const {return speedLimitEndWaypoint;}
    inline int GetSpeedLimitStartLap() const {return speedLimitStartLap;}
    inline int GetSpeedLimitStartWaypoint() const {return speedLimitStartWaypoint;}
    inline float GetSpeedSaturation() const {return speedSaturation;}
#ifdef USE_ANGELSCRIPT
    // we have to add this to be able to use the class as reference inside scripts
    void addRef()
    {
    };

    void release()
    {
    };
#endif
    Race & operator = (const Race &other)
    {
        // Don't do anything if it is the same reference
        if (&other == this)
            return *this;
        this->raceID = other.raceID;
        this->raceStartTime = other.raceStartTime;
        this->raceElapsedTime = other.raceElapsedTime;
        this->disabled = other.disabled;
        this->isStarted = other.isStarted;
        this->isFinished = other.isFinished;
        this->isPaused = other.isPaused;
        this->season = other.season;
        this->terrain = other.terrain;
        this->trackName = other.trackName;
        this->track = other.track;
        this->numberOfLaps = other.numberOfLaps;
        this->fastestLapTime = other.fastestLapTime;
        this->lapRecord = other.lapRecord;
        this->gridPosition1 = other.gridPosition1;
        this->gridPosition2 = other.gridPosition2;
        this->gridStep = other.gridStep;
        this->vehicleGridRotation = other.vehicleGridRotation;
        this->results = other.results;
        this->gridPositions = other.gridPositions;
    
        //Waypoint stuff
        this->waypoints = other.waypoints;
        this->waypoint_ids = other.waypoint_ids;
        this->waypoint_names = other.waypoint_names;
        this->waypoint_events = other.waypoint_events;
        this->waypoint_speed = other.waypoint_speed;
        this->waypoint_power = other.waypoint_power;
        this->waypoint_wait_time = other.waypoint_wait_time;
        this->road_edge_points_left = other.road_edge_points_left;
        this->road_edge_points_right = other.road_edge_points_right;
        this->hard_edge_points_left = other.hard_edge_points_left;
        this->hard_edge_points_right = other.hard_edge_points_right;
        this->pit_lane_waypoints = other.pit_lane_waypoints;
        this->pit_lane_waypoint_ids = other.pit_lane_waypoint_ids;
        this->pit_lane_waypoint_names = other.pit_lane_waypoint_names;
        
        /*Performance stuff:*/
        this->speedLimit = other.speedLimit;
        this->speedLimitStartLap = other.speedLimitStartLap;
        this->speedLimitEndLap = other.speedLimitEndLap;
        this->speedLimitStartWaypoint = other.speedLimitStartWaypoint;
        this->speedLimitEndWaypoint = other.speedLimitEndWaypoint;
        
        this->powerValueOffset = other.powerValueOffset;
        this->powerValueScale = other.powerValueScale;
        this->minNormalizedPower = other.minNormalizedPower;
        this->maxNormalizedPower = other.maxNormalizedPower;
        this->normalizedPowerSaturation = other.normalizedPowerSaturation;
        this->normalizedPowerDeadzone = other.normalizedPowerDeadzone;
        this->minSpeed = other.minSpeed;
        this->maxSpeed = other.maxSpeed;
        this->speedSaturation = other.speedSaturation;
        this->speedDeadzone = other.speedDeadzone;
    }
private:
    int raceID;
    //The raceID used in races.as is more temporary as it cannot be known until the terrain is loaded and the script is running (it's an array index in Angelscript).
    //We still need to keep it for backwards compatibility so that Angelscript-only races will still work without the ChampionshipManager and GUI.
    //Crucially, it is also the ID number used to identify the race checkpoints which in turn determine race positions and lap times.
//    int scriptRaceID;
    double raceStartTime;
    double raceElapsedTime;
    bool disabled;
    bool isStarted;
    bool isFinished;
    bool isPaused;
    int season;
    Ogre::String terrain;
    Ogre::String trackName;
    Ogre::String terrainFileName;
    int track;
    int numberOfLaps;
    double fastestLapTime;
    double lapRecord;
    Ogre::Vector3 gridPosition1;
    Ogre::Vector3 gridPosition2;
    Ogre::Vector3 gridStep;
    Ogre::Vector3 vehicleGridRotation;
    std::map<int, RaceResult> results;
    std::map<int, int> gridPositions;
    
    /*Waypoint stuff:*/
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
    
    /*Performance stuff:*/
    float speedLimit;
    int speedLimitStartLap;
    int speedLimitEndLap;
    int speedLimitStartWaypoint;
    int speedLimitEndWaypoint;
    
    float powerValueOffset;
    float powerValueScale;
    float minNormalizedPower;
    float maxNormalizedPower;
    float normalizedPowerSaturation;
    float normalizedPowerDeadzone;
    float minSpeed;
    float maxSpeed;
    float speedSaturation;
    float speedDeadzone;
};

Race *RaceFactory();

#endif //USE_ANGELSCRIPT
