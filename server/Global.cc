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
 * CVS: $Id$
 */

#include "Global.hh"
#include "Gui.hh"

using namespace gazebo;

bool Global::userQuit = false;
bool Global::userPause = false;
bool Global::userStep = false;
bool Global::userStepInc = false;
bool Global::showBoundingBoxes = false;
bool Global::showJoints = false;
bool Global::wireframe = false;
unsigned long Global::iterations = 0;
Gui *Global::gui = NULL;
Pose3d Global::poseOffset;


std::list<std::string> Global::gazeboPaths;
std::list<std::string> Global::ogrePaths;

////////////////////////////////////////////////////////////////////////////////
Global::Global()
{
}

////////////////////////////////////////////////////////////////////////////////
Global::~Global()
{
}

////////////////////////////////////////////////////////////////////////////////
bool Global::GetUserQuit()
{
  return userQuit;
}

////////////////////////////////////////////////////////////////////////////////
void Global::SetUserQuit(bool quit)
{
  userQuit = quit;
}


////////////////////////////////////////////////////////////////////////////////
bool Global::GetUserPause()
{
  return userPause;
}

////////////////////////////////////////////////////////////////////////////////
void Global::SetUserPause(bool pause)
{
  userPause = pause;
}

////////////////////////////////////////////////////////////////////////////////
bool Global::GetUserStep()
{
  return userStep;
}

////////////////////////////////////////////////////////////////////////////////
void Global::SetUserStep( bool step )
{
  userStep = step;
}

////////////////////////////////////////////////////////////////////////////////
bool Global::GetUserStepInc()
{
  return userStepInc;
}

////////////////////////////////////////////////////////////////////////////////
void Global::SetUserStepInc(bool step)
{
  userStepInc = step;
}

////////////////////////////////////////////////////////////////////////////////
bool Global::GetShowBoundingBoxes()
{
  return showBoundingBoxes;
}

////////////////////////////////////////////////////////////////////////////////
void Global::SetShowBoundingBoxes(bool show)
{
  showBoundingBoxes = show;
}

////////////////////////////////////////////////////////////////////////////////
unsigned long Global::GetIterations()
{
  return iterations;
}

////////////////////////////////////////////////////////////////////////////////
void Global::SetIterations(unsigned long count)
{
  iterations = count;
}

////////////////////////////////////////////////////////////////////////////////
void Global::IncIterations()
{
  iterations++;
}

////////////////////////////////////////////////////////////////////////////////
/// Get wheter to show the joints
bool Global::GetShowJoints()
{
  return showJoints;
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether to show the joints
void Global::SetShowJoints(bool show)
{
  showJoints = show;
}

////////////////////////////////////////////////////////////////////////////////
/// Set to view as wireframe
void Global::SetWireframe( bool wire )
{
  wireframe = wire;
}

////////////////////////////////////////////////////////////////////////////////
/// Get whether to view as wireframe
bool Global::GetWireframe()
{
  return wireframe;
}


