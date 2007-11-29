/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: FLTK main menu
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id:$
 */

#include "Global.hh"
#include "MainMenu.hh"

using namespace gazebo;

MainMenu::MainMenu(int x, int y, int w, int h, char *name)
  : Fl_Menu_Bar(x,y,w,h,name)
{
  const Fl_Menu_Item menuitems[] = {
    { "File", 0, 0, 0, FL_SUBMENU,  FL_NORMAL_LABEL, 0, 14, 0 },
    { "Quit", 0, &gazebo::MainMenu::QuitCB, 0, 0, FL_NORMAL_LABEL,0, 14,0 },
    { 0 },
  
    { 0 }
  };


  this->copy(menuitems);
}

void MainMenu::QuitCB(Fl_Widget * /*w*/, void * /*data*/)
{
  Global::SetUserQuit(true);
}
