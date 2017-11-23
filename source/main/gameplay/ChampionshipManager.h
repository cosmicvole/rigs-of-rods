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

/// @file   ChampionshipManager.h
/// @brief  Racing championship management
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
#include "ConfigFile.h"
#include "Singleton.h"
#include "BeamData.h"
#include "RaceResult.h"
#include "Race.h"

class Race;
class RaceResult;
class RaceCompetitor;
class CacheEntry;

class DifficultyLevel
{
public:
    void SetAIBaseBrakingMultiplier(float AIBaseBrakingMultiplier) {this->AIBaseBrakingMultiplier = AIBaseBrakingMultiplier;}
    void SetAIBaseTorqueMultiplier(float AIBaseTorqueMultiplier) {this->AIBaseTorqueMultiplier = AIBaseTorqueMultiplier;}
    void SetAIGripMultiplier(float AIGripMultiplier) {this->AIGripMultiplier = AIGripMultiplier;}
    void SetAIInertiaMultiplier(float AIInertiaMultiplier) {this->AIInertiaMultiplier = AIInertiaMultiplier;}
    void SetLevel(int level) {this->level = level;}
    void SetName(const Ogre::String& name) {this->name = name;}
    void SetPlayerBrakingMultiplier(float playerBrakingMultiplier) {this->playerBrakingMultiplier = playerBrakingMultiplier;}
    void SetPlayerGripMultiplier(float playerGripMultiplier) {this->playerGripMultiplier = playerGripMultiplier;}
    void SetPlayerInertiaMultiplier(float playerInertiaMultiplier) {this->playerInertiaMultiplier = playerInertiaMultiplier;}
    void SetPlayerTorqueMultiplier(float playerTorqueMultiplier) {this->playerTorqueMultiplier = playerTorqueMultiplier;}
    float GetAIBaseBrakingMultiplier() const {return AIBaseBrakingMultiplier;}
    float GetAIBaseTorqueMultiplier() const {return AIBaseTorqueMultiplier;}
    float GetAIBaseGripMultiplier() const {return AIGripMultiplier;}
    float GetAIBaseInertiaMultiplier() const {return AIInertiaMultiplier;}
    int GetLevel() const {return level;}
    const Ogre::String& GetName() const {return name;}
    float GetPlayerBrakingMultiplier() const {return playerBrakingMultiplier;}
    float GetPlayerGripMultiplier() const {return playerGripMultiplier;}
    float GetPlayerInertiaMultiplier() const {return playerInertiaMultiplier;}
    float GetPlayerTorqueMultiplier() const {return playerTorqueMultiplier;}
    DifficultyLevel & operator = (const DifficultyLevel &other)
    {
        // Don't do anything if it is the same reference
        if (&other == this)
            return *this;
        this->name = other.name;
        this->level = other.level;
        this->playerTorqueMultiplier = other.playerTorqueMultiplier;
        this->playerBrakingMultiplier = other.playerBrakingMultiplier;
        this->playerInertiaMultiplier = other.playerInertiaMultiplier;
        this->playerGripMultiplier = other.playerGripMultiplier;
        this->AIBaseTorqueMultiplier = other.AIBaseTorqueMultiplier;
        this->AIBaseBrakingMultiplier = other.AIBaseBrakingMultiplier;
        this->AIInertiaMultiplier = other.AIInertiaMultiplier;
        return *this;        
    }
private:
Ogre::String name;
int level;
float playerTorqueMultiplier;
float playerBrakingMultiplier;
float playerInertiaMultiplier;
float playerGripMultiplier;
float AIBaseTorqueMultiplier;
float AIBaseBrakingMultiplier;
float AIInertiaMultiplier;
float AIGripMultiplier;  
};

enum ChampionshipState
{
    CS_None,
    CS_RacePending,
    CS_RaceScheduled,
    CS_RaceRunning
};

/* cosmic vole September 15 2017 */
class ChampionshipManager : public RoRSingleton<ChampionshipManager>//, public ZeroedMemoryAllocator
{
public:
    ChampionshipManager();
    void loadChampionship(Ogre::DataStreamPtr& ds);
    inline Ogre::String getGUID() { return guid; }
    inline Ogre::String getName() { return championship_name; }
    inline const std::vector<authorinfo_t>& getAuthors() { return authors; }
    inline int getVersion() { return version; }
    inline int getCategoryID() { return categoryID; }
    inline Race *getCurrentRace() { if (current_race < 0 || current_race >= number_of_races) return nullptr; return &races[current_race]; }
    inline int getCurrentRaceIndex() { return current_race; }
    inline bool hasCurrentRace() { return current_race >= 0 && current_race < number_of_races; }
    inline Race *getRace(int raceID) { for (int i = 0; i < number_of_races; i++) { if (races[i].GetID() == raceID) return &races[i]; } return nullptr; }
    //inline int getCurrentRaceID() { Race *race = getCurrentRace(); if (race == nullptr) return -1; return race->GetID(); }
    int allocateRaceID(Ogre::String terrain, Ogre::String trackName)
    {
        Ogre::String terrainLwr = terrain;
        Ogre::String trackNameLwr = trackName;
        Ogre::StringUtil::toLowerCase(terrainLwr);
        Ogre::StringUtil::toLowerCase(trackNameLwr);
        for (std::vector<Race>::iterator it = races.begin(); it != races.end(); ++it)
        {
            Ogre::String trackName2 = it->GetTrackName();
            Ogre::String terrain2 = it->GetTerrain();
            Ogre::StringUtil::toLowerCase(trackName2);
            Ogre::StringUtil::toLowerCase(terrain2);
            if (trackNameLwr == trackName2 && terrainLwr == terrain2)
            {
                LOG("Returned championship raceID: " + TOSTRING(it->GetID()) + " for race at terrain: " + terrain + " track / race name: " + trackName + ".");
                return it->GetID();
            }
        }
        //If we got here, this race is not part of the current championship.
        //This normally means it's defined in the Angelscript for the current terrain but not in the current championship config file.
        //We still need to create an placeholder Race object to ensure the race IDs are unique.
        for (std::vector<Race>::iterator it = inactiveRaces.begin(); it != inactiveRaces.end(); ++it)
        {
            Ogre::String trackName2 = it->GetTrackName();
            Ogre::String terrain2 = it->GetTerrain();
            Ogre::StringUtil::toLowerCase(trackName2);
            Ogre::StringUtil::toLowerCase(terrain2);
            if (trackNameLwr == trackName2 && terrainLwr == terrain2)
            {
                LOG("Retured existing non championship raceID: " + TOSTRING(it->GetID()) + " for script generated race at terrain: " + terrain + " track / race name: " + trackName + ".");
                return it->GetID();
            }
        }
        //Generate a new ID and create an empty race to reserve the ID with that terrain and track name.
        int newID = races.size() + inactiveRaces.size() + 1;
        Race newRace;
        newRace.SetID(newID);
        newRace.SetTerrain(terrain);
        newRace.SetTrackName(trackName);
        newRace.Disable();
        inactiveRaces.push_back(newRace);
        LOG("Allocated new raceID: " + TOSTRING(newID) + " for script generated race at terrain: " + terrain + " track / race name: " + trackName + ".");
        return newID;
    }
    inline int getSelectedRaceIndex() { return selected_race; }
    inline RaceCompetitor *getCompetitorByTruckNum(int truckNum)
    { 
        std::map<int, RaceCompetitor *>::iterator it = competitorsByTruckNum.find(truckNum);
        if (it == competitorsByTruckNum.end())
        {
            return nullptr;
        }
        return &(*it->second);
    }
    inline void setSelectedRaceIndex(int index) { if (current_race < 0 || current_race >= number_of_races) return; selected_race = index; }
    inline void setCurrentRaceIndex(int index) { current_race = index; }
    inline const std::vector<Race>& getRaces() { return races; }
    const std::vector<RaceCompetitor>& getCompetitors() const { return competitors; }
    void updateCompetitorByIndex(RaceCompetitor competitor, int index)
    {
        if (index >= competitors.size()) return;
        RaceCompetitor &old = competitors[index];
        int oldTruckNum = old.GetTruckNum();
        competitorsByTruckNum.erase(oldTruckNum);
        competitors[index] = competitor;
        int truckNum = competitor.GetTruckNum();
        competitorsByTruckNum[truckNum] = &competitors[index];
    }
    void updateCompetitorByTruckNum(RaceCompetitor competitor, int oldTruckNum = -1)
    {
        int truckNum = competitor.GetTruckNum();
        if (oldTruckNum >= 0)
        {
            competitorsByTruckNum.erase(oldTruckNum);
            int i = 0;
            for (std::vector<RaceCompetitor>::iterator it = competitors.begin(); it != competitors.end(); it++)
            {
                if (it->GetTruckNum() == oldTruckNum)
                {
                    *it = competitor;
                    competitorsByTruckNum[truckNum] = &competitors[i];//&(*it);
                    return;
                }
                i++;
            }
        }
        RaceCompetitor *pCompetitor = competitorsByTruckNum[truckNum];
        if (pCompetitor == nullptr)
        {
            competitors.push_back(competitor);
            pCompetitor = &competitors[competitors.size()-1];
            competitorsByTruckNum[truckNum] = pCompetitor;
        }
        else
        {
            pCompetitor->SetName(competitor.GetName());
            pCompetitor->SetModel(competitor.GetModel());
            pCompetitor->SetSkin(competitor.GetSkin());
            pCompetitor->SetTruckNum(truckNum);
        }
    }
    inline void setCacheEntry(CacheEntry *entry) { cache_entry = entry; }
    inline CacheEntry *getCacheEntry() { return cache_entry; }
    inline ChampionshipState getState() { return state; }
    inline void setState(ChampionshipState state) { this->state = state; }
    std::vector<RaceCompetitor> GetSortedStandings()
    {
        std::vector<RaceCompetitor> sorted;
        int numCompetitors = competitors.size();
        for (std::vector<RaceCompetitor>::iterator itc = competitors.begin(); itc != competitors.end(); ++itc)
        {
            int truckNum = itc->GetTruckNum();
            if (truckNum < 0)
            {
                continue;
            }
            itc->SetPoints(0);
            itc->SetPodiums(0);
            itc->SetBestPosition(-1);

            for (std::vector<Race>::iterator it = races.begin(); it != races.end(); ++it)
            {
                int points = itc->GetPoints();
                int podiums = itc->GetPodiums();
                const RaceResult &result = it->GetRaceResult(truckNum);
                int position = result.GetPosition();
                switch (position)
                {
                    case 1:
                    itc->SetPoints(points + 10);
                    itc->SetPodiums(podiums + 1);
                    break;
                    case 2:
                    itc->SetPoints(points + 8);
                    itc->SetPodiums(podiums + 1);
                    break;
                    case 3:
                    itc->SetPoints(points + 6);
                    itc->SetPodiums(podiums + 1);
                    break;
                    case 4:
                    itc->SetPoints(points + 4);
                    break;
                    case 5:
                    itc->SetPoints(points + 2);
                    break;
                    case 6:
                    itc->SetPoints(points + 1);
                    break;
                }
                if ((position < itc->GetBestPosition() || itc->GetBestPosition() < 0) && position > 0)
                {
                    itc->SetBestPosition(position);
                }
            }
            sorted.push_back(*itc);
        }
        std::sort(sorted.begin(), sorted.end());
        return sorted;
    }
    
    const std::vector<DifficultyLevel>& GetDifficultyLevels() const { return difficultyLevels; }
    
    const DifficultyLevel *GetDifficultyLevel(int level)
    {
        const DifficultyLevel *prevLevel = nullptr;
        for (std::vector<DifficultyLevel>::const_iterator it = difficultyLevels.begin(); it != difficultyLevels.end(); ++it)
        {
            if (it->GetLevel() > level)
            {
                if (prevLevel == nullptr)
                {
                    return &(*it);
                }
                else
                {
                    return prevLevel;
                }
            }
            prevLevel = &(*it);
        }
        return prevLevel;
    }
    
    void AddDifficultyLevel(const DifficultyLevel &difficultyLevel)
    {
        //The difficulty levels will be stored in ascending order
        if (difficultyLevels.size() < 1)
        {
            difficultyLevels.push_back(difficultyLevel);
            return;
        }
        int level = difficultyLevel.GetLevel();
        //Brute force is fine here - there's only gonna be, like, 4 of 'em!
        for (std::vector<DifficultyLevel>::iterator it = difficultyLevels.begin(); it != difficultyLevels.end(); ++it)
        {
            if (it->GetLevel() > level)
            {
                difficultyLevels.insert(it, difficultyLevel);
                return;
            }
        }
        difficultyLevels.push_back(difficultyLevel);
    }
    
    const DifficultyLevel *GetCurrentDifficultyLevel()
    {
        return GetDifficultyLevel(current_difficulty_level);
    }
    
    void SetCurrentDifficultyLevel(int level)
    {
        current_difficulty_level = level;
    }
    
private:
    Ogre::String file_hash;
    Ogre::String guid;
    Ogre::String championship_name;
    int version;
    int categoryID;
    std::vector<authorinfo_t> authors;
    int races_per_track;
    int number_of_races;
    int current_race;
    int current_season;
    int selected_race;
    int number_of_competitors;
    ChampionshipState state;
    RoR::ConfigFile m_championship_config;
    CacheEntry *cache_entry;
    std::map<int, Ogre::String> difficulties;
    std::vector<Race> races;
    //This is just used to keep track of race IDs in the races.as script for tracks that aren't part of the championship
    std::vector<Race> inactiveRaces;
    std::vector<RaceCompetitor> competitors;
    std::map<int, RaceCompetitor*> competitorsByTruckNum;
    std::vector<DifficultyLevel> difficultyLevels;
    int current_difficulty_level;
};


#endif //USE_ANGELSCRIPT