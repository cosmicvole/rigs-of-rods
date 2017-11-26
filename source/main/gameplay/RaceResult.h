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

/// @file   RaceResult.h
/// @brief  Positions, times and stats for one vehicle and race
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

class RaceResult : public ZeroedMemoryAllocator
{
public:
	RaceResult()
	{
		truckNum = -1;
	}
    void SetBestLapTime(float bestLapTime) {this->bestLapTime = bestLapTime;}
    void SetBestLapTimeOfRace(bool bestLapTimeOfRace) {this->bestLapTimeOfRace = bestLapTimeOfRace;}
    void SetCurrentLapTime(float currentLapTime) {this->currentLapTime = currentLapTime;}
    void SetLapTimeRecord(bool lapTimeRecord) {this->lapTimeRecord = lapTimeRecord;}
    void SetLapsStarted(int lapsStarted) {this->lapsStarted = lapsStarted;}
    void SetLastLapTime(float lastLapTime) {this->lastLapTime = lastLapTime;}
    void SetNumWaypointsInLap(int numWaypointsInLap) {this->numWaypointsInLap = numWaypointsInLap;}
    void SetPersonalBestLapTime(float personalBestLapTime) {this->personalBestLapTime = personalBestLapTime;}
    void SetPosition(int position) {this->position = position;}
    void SetRaceID(int raceID) {this->raceID = raceID;}
    void SetTotalDistance(float totalDistance) {this->totalDistance = totalDistance;}
    void SetTotalTime(float totalTime) {this->totalTime = totalTime;}
    void SetTruckNum(int truckNum) {this->truckNum = truckNum;}
    void SetWaypointsPassed(float waypointsPassed) {this->waypointsPassed = waypointsPassed;}
    void SetDisqualified(bool disqualified) {this->disqualified = disqualified;}
    void SetRetired(bool retired) {this->retired = retired;}
    float GetBestLapTime() const {return bestLapTime;}
    bool IsBestLapTimeOfRace() const {return bestLapTimeOfRace;}
    float GetCurrentLapTime() const {return currentLapTime;}
    bool IsLapTimeRecord() const {return lapTimeRecord;}
    int GetLapsStarted() const {return lapsStarted;}
    float GetLastLapTime() const {return lastLapTime;}
    int GetNumWaypointsInLap() const {return numWaypointsInLap;}
    float GetPersonalBestLapTime() const {return personalBestLapTime;}
    int GetPosition() const {return position;}
    int GetRaceID() const {return raceID;}
    float GetTotalDistance() const {return totalDistance;}
    float GetTotalTime() const {return totalTime;}
    int GetTruckNum() const {return truckNum;}
    float GetWaypointsPassed() const {return waypointsPassed;}
    bool WasDisqualified() const {return disqualified;}
    bool Retired() const {return retired;}
    Ogre::UTFString FormatLapTime(float time)
    {
//        Ogre::UTFString str;
//        /*wchar_t*/char txt[10];
//        /*swprintf*/sprintf(txt, /*10, L*/"%.2i'", ((int)(time)) / 60);
//        str += txt;
//        str += ":";//L":";
//        /*swprintf*/sprintf(txt, /*10, L*/"%.2i", ((int)(time)) % 60);
//        str += txt;
//        str += ":";//L".";
//        /*swprintf*/sprintf(txt, /*10, L*/"%.2i", ((int)(time * 100.0)) % 100);
//        str += txt;
        wchar_t txt[32];
        swprintf(txt, 32, L"%.2i:%.2i.%.2i'", ((int)(time)) / 60, ((int)(time)) % 60, ((int)(time * 100.0)) % 100);
        return Ogre::UTFString(txt);
//        str.append(txt);//str += txt;
//        str.append(L":");//str += L":";
//        swprintf/*sprintf*/(txt, 10, L"%.2i", ((int)(time)) % 60);
//        str += txt;
//        str += L":";//L".";
//        swprintf(txt, 10, L"%.2i", ((int)(time * 100.0)) % 100);
//        str += txt;
    }
    friend bool operator <(const RaceResult& a, const RaceResult& b)
    {
		if (a.truckNum == -1)
		{
			return false;
		}
        if (a.lapsStarted > b.lapsStarted)
        {
            return true;
        }
        if (a.lapsStarted == b.lapsStarted)
        {
            //TODO This won't work if both trucks DNF midway through the same lap
            //or if a race is canceled midway through a lap. We need to compare the waypoints then.
            if (a.totalTime < b.totalTime)
            {
                return true;
            }
            if (a.totalTime == b.totalTime)
            {
                if (a.bestLapTime < b.bestLapTime && a.bestLapTime > 0.0f)
                {
                    return true;
                }
                if (a.bestLapTime == b.bestLapTime)
                {
                    return a.truckNum < b.truckNum;
                }
            }
        }
        return false;
    }
    RaceResult & operator = (const RaceResult &other)
    {
        // Don't do anything if it is the same reference
        if (&other == this)
            return *this;
        this->raceID = other.raceID;
        this->truckNum = other.truckNum;
		this->currentLapTime = other.currentLapTime;
		this->lastLapTime = other.lastLapTime;
		this->bestLapTime = other.bestLapTime;
		this->totalTime = other.totalTime;
		this->lapsStarted = other.lapsStarted;
		this->position = other.position;
		this->waypointsPassed = other.waypointsPassed;
		this->numWaypointsInLap = other.numWaypointsInLap;
		this->totalDistance = other.totalDistance;
		this->personalBestLapTime = other.personalBestLapTime;
		this->bestLapTimeOfRace = other.bestLapTimeOfRace;
		this->lapTimeRecord = other.lapTimeRecord;
		this->disqualified = other.disqualified;
		this->retired = other.retired;
	}
private:
    int raceID;
    int truckNum;
    float currentLapTime;
    float lastLapTime;
    float bestLapTime;
    float totalTime;
    int lapsStarted;
    int position;
    float waypointsPassed;
    int numWaypointsInLap;
    float totalDistance;
    float personalBestLapTime;
    bool bestLapTimeOfRace;
    bool lapTimeRecord;
    bool disqualified;
    bool retired;
    friend class Race;
};


 
#endif //USE_ANGELSCRIPT