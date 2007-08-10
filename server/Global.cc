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
/* Desc: Generic Typedefs, macros, functions, etc
 * Author: Nate Koenig
 * Date: 21 July 2007
 * CVS: $Id:$
 */

#include "Global.hh"
#include "Gui.hh"

using namespace gazebo;

bool Global::userQuit = false;
bool Global::userPause = false;
bool Global::userStep = false;
bool Global::userStepInc = false;
unsigned long Global::iterations = 0;
Gui *Global::gui = NULL;
Pose3d Global::poseOffset;

Global::Global()
{
}

Global::~Global()
{
}
