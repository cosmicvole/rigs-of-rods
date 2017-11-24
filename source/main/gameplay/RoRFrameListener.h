/*
    This source file is part of Rigs of Rods
    Copyright 2005-2012 Pierre-Michel Ricordel
    Copyright 2007-2012 Thomas Fischer

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

#pragma once

#include "BeamFactory.h"
#include "CharacterFactory.h"
#include "EnvironmentMap.h"
#include "ForceFeedback.h"
#include "RoRPrerequisites.h"


#include <Ogre.h>

namespace RoR { struct Terrn2Telepoint; } // Forward decl.

class RoRFrameListener: public Ogre::FrameListener, public Ogre::WindowEventListener, public ZeroedMemoryAllocator
{
public:

    RoRFrameListener(RoR::ForceFeedback* ff, RoR::SkidmarkConfig* skid_conf);
    virtual ~RoRFrameListener();

    // Ogre::FrameListener public interface
    bool   frameStarted          (const Ogre::FrameEvent& evt);
    bool   frameEnded            (const Ogre::FrameEvent& evt);

    // Scripting interface
    double getTime               () { return m_time; }
    void   UpdateDirectionArrow  (char* text, Ogre::Vector3 position);
    void   ShowLoaderGUI         (int type, const Ogre::String& instance, const Ogre::String& box);
    //cosmic vole added support for racing against multiple bots January 13 2017
    void SetRaceTimer(int truckNum, float time, bool raceIsInProgress);
    bool   LoadTerrain           (); ///< Reads GVar 'sim_terrain_pending'

    void StartRaceTimer(int truckNum = -1);
    float StopRaceTimer(int truckNum = -1);
    
    void ScheduleRaceStart(int raceID, double secondsDelay); // cosmic vole August 23 2017
	
	    // GUI interface
    void   ReloadCurrentTruck    (); // Top menubar
    void   TeleportPlayer        (RoR::Terrn2Telepoint* telepoint); // Teleport UI
    void   TeleportPlayerXZ      (float x, float y); // Teleport UI

    // BeamFactory callback
    void   ChangedCurrentVehicle (Beam* previous_vehicle, Beam* current_vehicle);

    void UpdateRacingGui(int truckNum = -1); // TODO This is private Upstream November 23 2017 See if we can make it private - cosmic vole

    bool IsRaceInProgress(int truckNum = -1) { return (truckNum >= 0) ? (m_races_in_progress[truckNum]) : m_race_in_progress; }
    
    bool IsRacePending(int truckNum); // cosmic vole August 23 2017
    
    bool IsRaceFinished(int truckNum); // cosmic vole August 23 2017 TODO ideally we could do with an enum for race state
    
    //void SetReloadPos(Ogre::Vector3 position) { m_reload_pos = position; } TODO This was deleted Upstream November 23 2017 Do we need it? cosmic vole

    /// @return True if everything was prepared OK and simulation may start.
    bool   SetupGameplayLoop     ();
    void   EnterGameplayLoop     ();

    RoR::BeamFactory*           GetBeamFactory  () { return &m_beam_factory; }
    RoR::SkidmarkConfig*        GetSkidmarkConf () { return m_skidmark_conf; }

protected:

    // Ogre::WindowEventListener interface
    void   windowMoved             (Ogre::RenderWindow* rw);
    void   windowClosed            (Ogre::RenderWindow* rw);
    void   windowFocusChange       (Ogre::RenderWindow* rw);
    void   windowResized           (Ogre::RenderWindow* rw);

    void   UpdateForceFeedback     (float dt);
    bool   UpdateInputEvents       (float dt);
    void   FinalizeTruckSpawning   (Beam* local_truck, Beam* previous_truck);
    void   HideGUI                 (bool hidden);
    void   CleanupAfterSimulation  (); /// Unloads all data

    RoR::BeamFactory         m_beam_factory;
    RoR::CharacterFactory    m_character_factory;
    RoR::GfxEnvmap           m_gfx_envmap;
    HeatHaze*                m_heathaze;
    RoR::SkidmarkConfig*     m_skidmark_conf;
    Ogre::Real               m_time_until_next_toggle; ///< just to stop toggles flipping too fast
    float                    m_last_simulation_speed;  ///< previously used time ratio between real time (evt.timeSinceLastFrame) and physics time ('dt' used in calcPhysics)
    bool                     m_is_pace_reset_pressed;
    int                      m_stats_on;
    float                    m_netcheck_gui_timer;
    collision_box_t*         m_reload_box;
    double                   m_time;
    RoR::ForceFeedback*      m_force_feedback;
    bool                     m_hide_gui;
    bool                     m_was_app_window_closed;
    bool                     m_truck_info_on;
    bool                     m_pressure_pressed;

    CacheEntry*              m_last_cache_selection;
    RoR::SkinDef*            m_last_skin_selection;
    std::vector<std::string> m_last_vehicle_configs;

    bool                     m_is_dir_arrow_visible;
    Ogre::Vector3            m_dir_arrow_pointed;

    int                      m_last_screenshot_id;
    Ogre::String             m_last_screenshot_date;

    double m_race_start_time; //cosmic vole changed from long to double August 23 2017
    bool                     m_race_in_progress;
    double m_race_bestlap_time; //TODO shouldn't this be renamed m_race_LASTlap_time ?? cosmic vole January 13 2017

    
    //Arrays of race times for racing against bots - cosmic vole January 13 2016
    double* m_race_start_times; //TODO name this the same as what is currently called m_race_bestlap_time ?? cosmic vole January 13 2017
    bool* m_races_in_progress;
    double* m_race_lastlap_times;
    bool                     m_advanced_truck_repair;
    float                    m_advanced_truck_repair_timer;

    Ogre::Vector3            m_reload_pos;
    Ogre::Quaternion         m_reload_dir;
};
