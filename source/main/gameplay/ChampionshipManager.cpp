#include "Application.h"
#include "Utils.h"
#include "CacheSystem.h"
#include "ConfigFile.h"
#include "ErrorUtils.h"
#include "Language.h"
#include "GUIManager.h"
#include "GUI_LoadingWindow.h"
#include "VehicleAI.h"
#include "RaceResult.h"
#include "Race.h"
#include "ChampionshipManager.h"

using namespace Ogre;

// some shortcut to remove ugly code - taken from TerrainManager.cpp
#ifdef USE_MYGUI
#   define PROGRESS_WINDOW(x, y) { LOG(Ogre::String("  ## ") + y); RoR::App::GetGuiManager()->GetLoadingWindow()->setProgress(x, y); }
#else
#   define PROGRESS_WINDOW(x, y) { LOG(Ogre::String("  ## ") + y) }
#endif //USE_MYGUI

ChampionshipManager::ChampionshipManager() : m_sim_controller(nullptr)
{
    file_hash = "";
    guid = "";
    championship_name = "";
    version = -1;
    categoryID = -1;
    races_per_track = 1;
    number_of_races = 0;
    current_race = 0;
    current_season = 2017;
    selected_race = -1;
    number_of_competitors = 0;
    current_difficulty_level = 0;    
}
 
/* cosmic vole September 15 2017 */
void ChampionshipManager::loadChampionship(Ogre::DataStreamPtr& ds)//(Ogre::String filename)
{
    /*
    DataStreamPtr ds;

    try
    {
        String group = ResourceGroupManager::getSingleton().findGroupContainingResource(filename);
        ds = ResourceGroupManager::getSingleton().openResource(filename, group);
    }
    catch (...)
    {
        LOG("Championship not found: " + String(filename));
        ErrorUtils::ShowError(_L("Championship loading error"), _L("Championship not found: ") + filename);
        exit(125);
    }
    */

	//No point in putting this up - it is behind the loading window!
    //PROGRESS_WINDOW(10, _L("Loading Championship Configuration"));

    LOG(" ===== LOADING CHAMPIONSHIP " + ds->getName());

    // now generate the hash of it
    generateHashFromDataStream(ds, file_hash);

    // Load the config from the resource data stream, enabling multiline values (useful for waypoints etc)
    m_championship_config.load(ds, "\t:=", true, true);

    // read in the settings
    championship_name = m_championship_config.GetStringEx("Name", "General");
    if (championship_name.empty())
    {
        ErrorUtils::ShowError(_L("Championship loading error"), _L("the championship name cannot be empty"));
        exit(125);
    }

    /*
    Ogre::String truck_filename = m_championship_config.GetStringEx("Model", competitorSection);
    // otc = ogre terrain config
    if (ogre_terrain_config_filename.find(".otc") == String::npos)
    {
        ErrorUtils::ShowError(_L("Terrain loading error"), _L("the new terrain mode only supports .otc configurations"));
        exit(125);
    }
    */

    //category_id = m_championship_config.GetInt("CategoryID", "General", 129);
    guid = m_championship_config.GetStringEx("GUID", "General");
    //start_position = StringConverter::parseVector3(m_championship_config.GetStringEx("StartPosition", "General"), Vector3(512.0f, 0.0f, 512.0f));
    version = m_championship_config.GetInt("Version", "General", 1);
    //gravity = m_championship_config.GetFloat("Gravity", "General", -9.81);

    // parse author info
    Ogre::ConfigFile::SettingsIterator it = m_championship_config.getSettingsIterator("Authors");

    authors.clear();

    while (it.hasMoreElements())
    {
        String type = RoR::Utils::SanitizeUtf8String(it.peekNextKey()); // e.g. terrain
        String name = RoR::Utils::SanitizeUtf8String(it.peekNextValue()); // e.g. john doe

        if (!name.empty())
        {
            authorinfo_t author;

            author.type = type;
            author.name = name;

            authors.push_back(author);
        }

        it.moveNext();
    }
    
    int numDifficultyLevels = m_championship_config.GetInt("NumberOfDifficulties", "Difficulties");
    if (numDifficultyLevels > 50)
        numDifficultyLevels = 50;
    if (numDifficultyLevels > 0)
    {
        for (int i = 1; i <= numDifficultyLevels; i++)
        {
            DifficultyLevel difficultyLevel;
            Ogre::String diffSection = Ogre::String("Difficulty");
            diffSection = diffSection + TOSTRING(i);
            Ogre::String defaultName = Ogre::String("Level ");
            defaultName = defaultName + TOSTRING(i);
            Ogre::String name = m_championship_config.GetStringEx("Name", diffSection, defaultName);//Ogre::String("Number ") + TOSTRING(i));
            int level = m_championship_config.GetInt("Level", diffSection, i);
            float playerTorqueMultiplier = m_championship_config.GetFloat("PlayerTorqueMultiplier", diffSection, 1.0f);
            float playerBrakingMultiplier = m_championship_config.GetFloat("PlayerBrakingMultiplier", diffSection, 1.0f);
            float playerInertiaMultiplier = m_championship_config.GetFloat("PlayerInertiaMultiplier", diffSection, 1.0f);
            float playerGripMultiplier = m_championship_config.GetFloat("PlayerGripMultiplier", diffSection, 1.0f);
            //If no AI difficulty levels are specified, we still mildly tune the AI vehicles by default
            //otherwise they will be no match for a human brain!
            float AIBaseTorqueMultiplier = m_championship_config.GetFloat("AIBaseTorqueMultiplier", diffSection, 1.1f);
            float AIBaseBrakingMultiplier = m_championship_config.GetFloat("AIBaseBrakingMultiplier", diffSection, 1.5f);
            float AIBaseInertiaMultiplier = m_championship_config.GetFloat("AIBaseInertiaMultiplier", diffSection, 0.8f);
            float AIBaseGripMultiplier = m_championship_config.GetFloat("AIBaseGripMultiplier", diffSection, 1.3f);
            
            LOG("AI settings for difficulty " + diffSection + " Torque: " + TOSTRING(AIBaseTorqueMultiplier) + " Braking: " + TOSTRING(AIBaseBrakingMultiplier) + " Inertia: " + TOSTRING(AIBaseInertiaMultiplier) + " Grip: " + TOSTRING(AIBaseGripMultiplier));
            
            difficultyLevel.SetName(name);
            difficultyLevel.SetLevel(level);
            difficultyLevel.SetPlayerTorqueMultiplier(playerTorqueMultiplier);
            difficultyLevel.SetPlayerBrakingMultiplier(playerBrakingMultiplier);
            difficultyLevel.SetPlayerInertiaMultiplier(playerInertiaMultiplier);
            difficultyLevel.SetPlayerGripMultiplier(playerGripMultiplier);
            difficultyLevel.SetAIBaseTorqueMultiplier(AIBaseTorqueMultiplier);
            difficultyLevel.SetAIBaseBrakingMultiplier(AIBaseBrakingMultiplier);
            difficultyLevel.SetAIInertiaMultiplier(AIBaseInertiaMultiplier);
            difficultyLevel.SetAIGripMultiplier(AIBaseGripMultiplier);            
            AddDifficultyLevel(difficultyLevel);
        }
        
        if (difficultyLevels.size() > 1)
        {
            SetCurrentDifficultyLevel(difficultyLevels[1].GetLevel());
        }
        else
        {
            SetCurrentDifficultyLevel(difficultyLevels[0].GetLevel());
        }
    }
    
    number_of_races = m_championship_config.GetInt("NumberOfRaces", "Races");
    if (number_of_races > 500)
        number_of_races = 500;
    //The raceID is a bit of a hack unfortunately because when we create races here, we don't know what integer raceID will be assigned to them
    //by the races.as Angelscript and we can't easily replace that ID because it's an array index and would break backwards compatibility with other scripts.
    //The current solution is to number ChampionshipManager generated raceIDs starting from 1000.
    //Once races.as generates its own raceID (also used to label the checkpoints), it can invoke ChampionshipManager to update its ID if >= 1000, to that.
    //We probably need a separate GUID as well so the race stays unique in saved files of stats across multiple seasons.
    int raceID = 1000; //TODO we need to load this from the saved player based on season and races completed
    for (int i = 1; i <= number_of_races; i++)
    {
        Race race;
        race.SetID(raceID);
        Ogre::String raceSection = Ogre::String("Race") + TOSTRING(i);
        Ogre::String trackName = m_championship_config.GetStringEx("Name", raceSection, Ogre::String("Race ") + TOSTRING(i));
        Ogre::String terrainFilename = m_championship_config.GetStringEx("TerrainFilename", raceSection);
        if (terrainFilename.empty())
        {
            LOG("Error in Championship File: TerrainFilename not specified for race '" + raceSection + "'.");
        }
        terrainFilename = RoR::Utils::SanitizeUtf8String(terrainFilename);
        Ogre::String terrainName = m_championship_config.GetStringEx("TerrainName", raceSection);
        race.SetTrackName(RoR::Utils::SanitizeUtf8String(trackName));
        race.SetTerrainFileName(terrainFilename);
        if (terrainName.empty())
        {
            //If it's not found in the cache, the name will just be returned blank
            CacheEntry entry = RoR::App::GetCacheSystem()->getResourceInfo(terrainFilename);
            terrainName = ANSI_TO_UTF(entry.dname);
        }
        race.SetTerrain(RoR::Utils::SanitizeUtf8String(terrainName));
        Vector3 gridPosition1 = StringConverter::parseVector3(m_championship_config.GetStringEx("GridPosition1", raceSection), Vector3(512.0f, 0.0f, 512.0f));
        Vector3 gridPosition2 = StringConverter::parseVector3(m_championship_config.GetStringEx("GridPosition2", raceSection), Vector3(517.0f, 0.0f, 512.0f));
        float gridStepX = m_championship_config.GetFloat("GridStepX", raceSection, 5.0f);
        float gridStepY = m_championship_config.GetFloat("GridStepY", raceSection, 0.0f);
        float gridStepZ = m_championship_config.GetFloat("GridStepZ", raceSection, 10.0f);
        Vector3 vehicleGridRotation = StringConverter::parseVector3(m_championship_config.GetStringEx("VehicleGridRotation", raceSection), Vector3(0.0f, 0.0f, 0.0f));
        race.SetGridPosition(gridPosition1, gridPosition2, Vector3(gridStepX, gridStepY, gridStepZ));
        race.SetVehicleGridRotation(vehicleGridRotation);
        int numberOfLaps = m_championship_config.GetInt("NumberOfLaps", raceSection, 1);
        race.SetNumberOfLaps(numberOfLaps);
        //Read waypoint performance settings
        float speedLimit = m_championship_config.GetFloat("SpeedLimit", raceSection, -1.0f);
        int speedLimitStartLap = m_championship_config.GetInt("SpeedLimitStartLap", raceSection, -1);
        int speedLimitEndLap = m_championship_config.GetInt("SpeedLimitEndLap", raceSection, -1);
        int speedLimitStartWpt = m_championship_config.GetInt("SpeedLimitStartWaypoint", raceSection, -1);
        int speedLimitEndWpt = m_championship_config.GetInt("SpeedLimitEndWaypoint", raceSection, -1);
        
        float speedMultiplier = m_championship_config.GetFloat("SpeedMultiplier", raceSection, 1.0f);
        float speedOffset = m_championship_config.GetFloat("SpeedOffset", raceSection, 0.0f);
        
        race.SetSpeedLimit(speedLimit);
        race.SetSpeedLimitStartLap(speedLimitStartLap);
        race.SetSpeedLimitEndLap(speedLimitEndLap);
        race.SetSpeedLimitStartWaypoint(speedLimitStartWpt);
        race.SetSpeedLimitEndWaypoint(speedLimitEndWpt);
        
        float powerValueOffset = m_championship_config.GetFloat("PowerValueOffset", raceSection, 0.0f);
        //TODO We *could* automatically calculate a default power value scale by finding the min, max and range of the waypoint power values
        float powerValueScale = m_championship_config.GetFloat("PowerValueScale", raceSection, 1.0f);
        float minNormalizedPower = m_championship_config.GetFloat("MinNormalizedPower", raceSection, -1.0f);
        float maxNormalizedPower = m_championship_config.GetFloat("MaxNormalizedPower", raceSection, -1.0f);
        float normalizedPowerSaturation = m_championship_config.GetFloat("NormalizedPowerSaturation", raceSection, -1.0f);
        float normalizedPowerDeadzone = m_championship_config.GetFloat("NormalizedPowerDeadzone", raceSection, 0.0f);
        float maxSpeed = m_championship_config.GetFloat("MaxSpeed", raceSection, -1.0f);
        float speedSaturation = m_championship_config.GetFloat("SpeedSaturation", raceSection, -1.0f);
        if (maxNormalizedPower >= 0.0f && normalizedPowerSaturation < 0.0f)
        {
            normalizedPowerSaturation = maxNormalizedPower;
        }
        if (maxSpeed >= 0.0f && speedSaturation < 0.0f)
        {
            speedSaturation = maxSpeed;
        }
        
        //Read race waypoints
        Ogre::ConfigFile::SettingsIterator it = m_championship_config.getSettingsIterator(raceSection);

        race.ClearWaypoints();
        int waypointIndex = 0;

        while (it.hasMoreElements())
        {
            String key = RoR::Utils::SanitizeUtf8String(it.peekNextKey());
            String value = RoR::Utils::SanitizeUtf8String(it.peekNextValue());

            if (key == "Waypoints")
            {
                //Need to break up the string into a list of waypoints
                StringVector tokens = StringUtil::tokenise(value, "{}\n\r");//TODO we need our own function to do this: tokenise(value, ", \t\n", "{}"); because curly brackets don't work as double delims
                for (int j = 0; j < tokens.size(); j++)
                {
                    Vector3 nextWaypoint = Vector3::ZERO;
                    float speed = -1.0f;
                    float power = -1.0f;
                    
                    String s = tokens[j];
                    LOG("Parsing waypoint from string: '" + s + "'.");
                    s = StringUtil::replaceAll(s, "{}", "");
                    //Split on commas
                    Ogre::vector<Ogre::String>::type vals = Ogre::StringUtil::split(s, ",", 5);

                    if (vals.size() >= 3)
                    {
                        nextWaypoint = Vector3(Ogre::StringConverter::parseReal(vals[0], 0.0),
                            Ogre::StringConverter::parseReal(vals[1], 0.0),
                            Ogre::StringConverter::parseReal(vals[2], 0.0));
                        if (vals.size() >= 4)
                        {
                            speed = Ogre::StringConverter::parseReal(vals[3], 0.0);
                            LOG("Parsed speed as: " + TOSTRING(speed));
                            if (vals.size() >= 5)
                            {
                                power = Ogre::StringConverter::parseReal(vals[4], 0.0);
                                LOG("Parsed power as: " + TOSTRING(power));
                            }
                        }
                    }
                    else
                    {
                        LOG("Waypoint was invalid. Skipping.");
                        continue;
                    }
                    
                    //nextWaypoint = StringConverter::parseVector3(s);
                    LOG("Waypoint was parsed as: " + TOSTRING(nextWaypoint) + ".");
                    if (nextWaypoint != Vector3::ZERO)
                    {
                        race.AddWaypoint(nextWaypoint);
                        if (speed > 0.0f)
                        {
                            if (speedSaturation >= 0.0f && (speed >= speedSaturation || speed >= maxSpeed) && maxSpeed >= 0.0f)
                            {
                                speed = maxSpeed;
                            }
                            if (speedMultiplier > 0.0f)
                            {
                                speed *= speedMultiplier;
                            }
                            speed += speedOffset;
                            LOG("Waypoint speed tweaked to: " + TOSTRING(speed));
                            race.SetValueAtWaypoint(waypointIndex, AI_SPEED, speed);
                        }
                        if (power > 0.0f)
                        {
                            power += powerValueOffset;
                            power *= powerValueScale;
                            if ((power <= normalizedPowerDeadzone || power < minNormalizedPower) && minNormalizedPower >= 0.0f)
                            {
                                power = minNormalizedPower;
                            }
                            if (normalizedPowerSaturation >= 0.0f && maxNormalizedPower >= 0.0f && (power >= normalizedPowerSaturation || power >= maxNormalizedPower))
                            {
                                power = maxNormalizedPower;
                            }
                            LOG("Waypoint power tweaked to: " + TOSTRING(power));
                            race.SetValueAtWaypoint(waypointIndex, AI_POWER, power);
                        }
                        waypointIndex++;
                    }
                }
            }
            else if (key == "EdgePointsLeft")
            {
                Vector3 nextWaypoint;
                StringVector tokens = StringUtil::tokenise(value, "{}\n\r");// TODO need our own tokenise() method to handle curly brackets tokenise(value, ", \t\n", "{}");
                for (int j = 0; j < tokens.size(); j++)
                {
                    String s = tokens[j];
                    LOG("Parsing left edge point from string: '" + s + "'.");
                    s = StringUtil::replaceAll(s, "{}", "");
                    nextWaypoint = StringConverter::parseVector3(s);
                    LOG("Edge point was parsed as: " + TOSTRING(nextWaypoint) + ".");
                    if (nextWaypoint != Vector3::ZERO)
                    {
                        race.AddRoadEdgePointLeft(nextWaypoint);
                    }
                }                    
            }
            else if (key == "EdgePointsRight")
            {
                Vector3 nextWaypoint;
                StringVector tokens = StringUtil::tokenise(value, "{}\n\r");// TODO need our own tokenise() method to handle curly brackets tokenise(value, ", \t\n", "{}");
                for (int j = 0; j < tokens.size(); j++)
                {
                    String s = tokens[j];
                    LOG("Parsing right edge point from string: '" + s + "'.");
                    s = StringUtil::replaceAll(s, "{}", "");
                    nextWaypoint = StringConverter::parseVector3(s);
                    LOG("Edge point was parsed as: " + TOSTRING(nextWaypoint) + ".");
                    if (nextWaypoint != Vector3::ZERO)
                    {
                        race.AddRoadEdgePointRight(nextWaypoint);
                    }
                }                    
            }

            it.moveNext();
        }
        races.push_back(race);
        raceID++;
    }
    
    number_of_competitors = m_championship_config.GetInt("NumberOfCompetitors", "Competitors");
    //Add one on to represent the player. Note, I'm not too keen on this logic. cosmic vole October 24 2017
    number_of_competitors++;
    RaceCompetitor playerCompetitor;
    playerCompetitor.SetName(Ogre::String("Player"));
    playerCompetitor.SetModel(Ogre::String("FordTaunusSedan.truck")); //TODO Where to select player model?
    playerCompetitor.SetSkin(Ogre::String(""));
    //These won't be used for the player (the difficulty settings hold the actual values used)
    playerCompetitor.SetTorqueMultiplier(1.0);
    playerCompetitor.SetBrakingMultiplier(1.0);
    playerCompetitor.SetGripMultiplier(1.0);
    Ogre::String checkName = playerCompetitor.GetName();
    Ogre::String checkModel = playerCompetitor.GetModel();
    Ogre::String checkSkin = playerCompetitor.GetSkin();
    LOG(checkName);
    LOG(checkModel);
    LOG(checkSkin);
    competitors.push_back(playerCompetitor);
    
    
    if (number_of_competitors > 50)
        number_of_competitors = 50;
    if (number_of_competitors > 1)
    {
        //competitors.reserve(number_of_competitors);
        for (int i = 1; i < number_of_competitors; i++)
        {
            RaceCompetitor competitor;
            Ogre::String compSection = Ogre::String("Competitor");
            compSection = compSection + TOSTRING(i);
            Ogre::String defaultName = Ogre::String("Number");
            defaultName = defaultName + TOSTRING(i);
            Ogre::String name = m_championship_config.GetStringEx("Name", compSection, defaultName);//Ogre::String("Number ") + TOSTRING(i));
            Ogre::String model = m_championship_config.GetStringEx("Model", compSection);
            Ogre::String skin = m_championship_config.GetStringEx("Skin", compSection);
            competitor.SetName(name);
            competitor.SetModel(model);
            competitor.SetSkin(skin);
            float torqueMultiplier = m_championship_config.GetFloat("TorqueMultiplier", compSection, 1.0f);
            float brakingMultiplier = m_championship_config.GetFloat("BrakingMultiplier", compSection, 1.0f);
            float gripMultiplier = m_championship_config.GetFloat("GripMultiplier", compSection, 1.0f);
            competitor.SetTorqueMultiplier(torqueMultiplier);
            competitor.SetBrakingMultiplier(brakingMultiplier);
            competitor.SetGripMultiplier(gripMultiplier);
            Ogre::String checkName = competitor.GetName();
            Ogre::String checkModel = competitor.GetModel();
            Ogre::String checkSkin = competitor.GetSkin();
            LOG(checkName);
            LOG(checkModel);
            LOG(checkSkin);
            competitors.push_back(competitor);
            //WHAT - how can it have a truckNum at this point?!
            //competitorsByTruckNum[competitor.GetTruckNum()] = &competitor;
        }    
    }

    LOG(" ===== CHAMPIONSHIP LOADING DONE " + ds->getName());
}    
