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

/** 
    @file   ConfigFile.cpp
    @date   06/2014
    @author Petr Ohlidal
*/

#include "ConfigFile.h"
#include "Utils.h"

#include <OgreConfigFile.h>
#include <OgreString.h>
#include <OgreStringConverter.h>

using namespace RoR;

float ConfigFile::GetFloat(Ogre::String const& key, Ogre::String const& section, float defaultValue)
{
    return Ogre::StringConverter::parseReal(Ogre::ConfigFile::getSetting(key, section), defaultValue);
}

Ogre::ColourValue ConfigFile::GetColourValue(Ogre::String const& key, Ogre::String const& section, Ogre::ColourValue const& defaultValue)
{
    return Ogre::StringConverter::parseColourValue(Ogre::ConfigFile::getSetting(key, section), defaultValue);
}

int ConfigFile::GetInt(Ogre::String const& key, Ogre::String const& section, int defaultValue)
{
    return Ogre::StringConverter::parseInt(Ogre::ConfigFile::getSetting(key, section), defaultValue);
}

bool ConfigFile::GetBool(Ogre::String const& key, Ogre::String const& section, bool defaultValue)
{
    return Ogre::StringConverter::parseBool(Ogre::ConfigFile::getSetting(key, section), defaultValue);
}

Ogre::String ConfigFile::GetStringEx(Ogre::String const& key, Ogre::String const& section, Ogre::String const& defaultValue)
{
    auto setting = Ogre::ConfigFile::getSetting(key, section);
    if (setting.empty())
    {
        return defaultValue;
    }
    return RoR::Utils::SanitizeUtf8String(setting);
}

void ConfigFile::SetString(Ogre::String key, Ogre::String value, Ogre::String section /* = Ogre::StringUtil::BLANK */)
{
    SettingsMultiMap* set = mSettings[section];
    if (!set)
    {
        // new section
        set = new SettingsMultiMap();
        mSettings[section] = set;
    }
    if (set->count(key))
    {
        // known key, delete old first
        set->erase(key);
    }
    // add key
    set->insert(std::multimap<Ogre::String, Ogre::String>::value_type(key, value));
}

/// load from a data stream - overloaded to allow multiline values - cosmic vole October 22 2017
/// Original code for this function was copied from the
/// OGRE 1.9 ConfigFile source: Copyright (c) 2000-2014 Torus Knot Software Ltd
void ConfigFile::load(const Ogre::DataStreamPtr& stream, const Ogre::String& separators, bool trimWhitespace, bool multiline)
{
    /* Clear current settings map */
    clear();

    Ogre::String currentSection = Ogre::StringUtil::BLANK;
    Ogre::ConfigFile::SettingsMultiMap* currentSettings = OGRE_NEW_T(Ogre::ConfigFile::SettingsMultiMap, Ogre::MEMCATEGORY_GENERAL)();
    mSettings[currentSection] = currentSettings;

    /* Process the file line for line */
    Ogre::String line, optName, optVal, lastLine;
    while (!stream->eof())
    {
        line = stream->getLine();
        /* Ignore comments & blanks */
        if (line.length() > 0 && line.at(0) != '#' && line.at(0) != '@')
        {
            if (line.at(0) == '[' && line.at(line.length()-1) == ']')
            {
                // Section
                currentSection = line.substr(1, line.length() - 2);
                Ogre::ConfigFile::SettingsBySection::const_iterator seci = mSettings.find(currentSection);
                
                //Store any previous multiline setting - cosmic vole October 22 2017
                if (multiline && optName.length() > 0)
                {
                    //We store it under the previous section before we update currentSettings
                    currentSettings->insert(Ogre::ConfigFile::SettingsMultiMap::value_type(optName, optVal));
                    //Reset multiline state ready for next key / value pair
                    optName = optVal = "";
                }
                
                if (seci == mSettings.end())
                {
                    currentSettings = OGRE_NEW_T(Ogre::ConfigFile::SettingsMultiMap, Ogre::MEMCATEGORY_GENERAL)();
                    mSettings[currentSection] = currentSettings;
                }
                else
                {
                    currentSettings = seci->second;
                }
            }
            else
            {
                /* Find the first separator character and split the string there */
                Ogre::String::size_type separator_pos = line.find_first_of(separators, 0);
                if (separator_pos != Ogre::String::npos)
                {
                    //If multiline enabled, store any previous key and value - cosmic vole October 22 2017
                    if (multiline && optName.length() > 0)
                    {
                        currentSettings->insert(Ogre::ConfigFile::SettingsMultiMap::value_type(optName, optVal));
                        //Reset multiline state ready for next key / value pair
                        optName = optVal = "";                        
                    }
                    optName = line.substr(0, separator_pos);
                    /* Find the first non-separator character following the name */
                    Ogre::String::size_type nonseparator_pos = line.find_first_not_of(separators, separator_pos);
                    /* ... and extract the value */
                    /* Make sure we don't crash on an empty setting (it might be a valid value) */
                    optVal = (nonseparator_pos == Ogre::String::npos) ? "" : line.substr(nonseparator_pos);
                    if (trimWhitespace)
                    {
                        Ogre::StringUtil::trim(optVal);
                        Ogre::StringUtil::trim(optName);
                    }
                    //Only save the setting straight away if multiline is disabled - cosmic vole October 22 2017
                    if (!multiline)
                    {
                        currentSettings->insert(Ogre::ConfigFile::SettingsMultiMap::value_type(optName, optVal));
                    }
                }
                else if (multiline && optName.length() > 0) //If multiline enabled, keep appending lines to current value - cosmic vole October 22 2017
                {
                    if (trimWhitespace)
                    {
                        Ogre::StringUtil::trim(line);
                    }
                    optVal += line;
                }
            }
        }
    }
    
    //We reached EOF so store any final multiline setting - cosmic vole October 22 2017
    if (multiline && optName.length() > 0)
    {
        currentSettings->insert(Ogre::ConfigFile::SettingsMultiMap::value_type(optName, optVal));
    }
}

