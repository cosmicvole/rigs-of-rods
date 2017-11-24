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

#pragma once

#include "RoRPrerequisites.h"

#include "Character.h"
#include "Network.h"

#include <memory>
#include <vector>

namespace RoR {

class CharacterFactory
{
public:
    CharacterFactory(RoRFrameListener* sim): m_sim_controller(sim) {}
    Character* createLocal(int playerColour);
    void DeleteAllRemoteCharacters();
    //cosmic vole added separate collection of the local AI characters November 21 2016
    Character *createAIInstance(int id, int aiColour);
    void removeAIInstance(int id);

    void update(float dt);
#ifdef USE_SOCKETW
    void handleStreamData(std::vector<RoR::Networking::recv_packet_t> packet);
#endif // USE_SOCKETW

private:

    RoRFrameListener*                       m_sim_controller;
    //cosmic vole added separate collection of the local AI characters November 21 2016
    std::vector<std::unique_ptr<Character>> m_AIcharacters;
    std::vector<std::unique_ptr<Character>> m_remote_characters;

    void createRemoteInstance(int sourceid, int streamid);
    void removeStreamSource(int sourceid);
};

} // namespace RoR
