#pragma once

// ----------------------------------------------------------------------------
// Generated by MyGUI's LayoutEditor using RoR's code templates.
// Find the templates at [repository]/tools/MyGUI_LayoutEditor/
//
// IMPORTANT:
// Do not modify this code directly. Create a derived class instead.
// ----------------------------------------------------------------------------

#include "ForwardDeclarations.h"
#include "BaseLayout.h"

namespace RoR {
namespace GUI {

ATTRIBUTE_CLASS_LAYOUT(GameRacingMenuLayout, "RacingMenu.layout");
class GameRacingMenuLayout : public wraps::BaseLayout
{

public:

    GameRacingMenuLayout(MyGUI::Widget* _parent = nullptr);
    virtual ~GameRacingMenuLayout();

protected:

    //%LE Widget_Declaration list start
    ATTRIBUTE_FIELD_WIDGET_NAME(GameRacingMenuLayout, m_load_player, "load_player");
    MyGUI::Button* m_load_player;

    ATTRIBUTE_FIELD_WIDGET_NAME(GameRacingMenuLayout, m_save_player, "save_player");
    MyGUI::Button* m_save_player;      
                                    
    ATTRIBUTE_FIELD_WIDGET_NAME(GameRacingMenuLayout, m_single_race, "single_race");
    MyGUI::Button* m_single_race;         
                                    
    ATTRIBUTE_FIELD_WIDGET_NAME(GameRacingMenuLayout, m_championship, "championship");
    MyGUI::Button* m_championship;          
                                    
    ATTRIBUTE_FIELD_WIDGET_NAME(GameRacingMenuLayout, m_main_menu, "main_menu");
    MyGUI::Button* m_main_menu;    
                                    
    //%LE Widget_Declaration list end
};

} // namespace GUI
} // namespace RoR
