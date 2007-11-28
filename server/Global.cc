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
unsigned long Global::iterations = 0;
Gui *Global::gui = NULL;
Pose3d Global::poseOffset;

boost::recursive_mutex Global::mutex;

std::list<std::string> Global::gazeboPaths;
std::list<std::string> Global::ogrePaths;

Global::Global()
{
}

Global::~Global()
{
}

bool Global::GetUserQuit()
{
  //boost::recursive_mutex::scoped_lock lock(gazebo::Global::mutex);
  return userQuit;
}

void Global::SetUserQuit(bool quit)
{
  //boost::recursive_mutex::scoped_lock lock(gazebo::Global::mutex);
  userQuit = quit;
}


bool Global::GetUserPause()
{
  //boost::recursive_mutex::scoped_lock lock(gazebo::Global::mutex);
  return userPause;
}

void Global::SetUserPause(bool pause)
{
  //boost::recursive_mutex::scoped_lock lock(gazebo::Global::mutex);
  userPause = pause;
}

bool Global::GetUserStep()
{
  //boost::recursive_mutex::scoped_lock lock(gazebo::Global::mutex);
  return userStep;
}

void Global::SetUserStep( bool step )
{
  //boost::recursive_mutex::scoped_lock lock(gazebo::Global::mutex);
  userStep = step;
}

bool Global::GetUserStepInc()
{
  //boost::recursive_mutex::scoped_lock lock(gazebo::Global::mutex);
  return userStepInc;
}

void Global::SetUserStepInc(bool step)
{
  //boost::recursive_mutex::scoped_lock lock(gazebo::Global::mutex);
  userStepInc = step;
}

bool Global::GetShowBoundingBoxes()
{
  //boost::recursive_mutex::scoped_lock lock(gazebo::Global::mutex);
  return showBoundingBoxes;
}

void Global::SetShowBoundingBoxes(bool show)
{
  //boost::recursive_mutex::scoped_lock lock(gazebo::Global::mutex);
  showBoundingBoxes = show;
}

unsigned long Global::GetIterations()
{
  //boost::recursive_mutex::scoped_lock lock(gazebo::Global::mutex);
  return iterations;
}

void Global::SetIterations(unsigned long count)
{
  //boost::recursive_mutex::scoped_lock lock(gazebo::Global::mutex);
  iterations = count;
}

void Global::IncIterations()
{
  //boost::recursive_mutex::scoped_lock lock(gazebo::Global::mutex);
  iterations++;
}
