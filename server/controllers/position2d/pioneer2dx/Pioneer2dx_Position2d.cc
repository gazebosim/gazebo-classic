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
/*
 * Desc: Position2d controller for a Pioneer2dx.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN info: $Id$
 */

#include "Pioneer2dx_Position2d.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Pioneer2dx_Position2d::Pioneer2dx_Position2d()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Pioneer2dx_Position2d::~Pioneer2dx_Position2d()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
int Pioneer2dx_Position2d::LoadChild(XMLConfigNode *node)
{
  printf ("Loading\n");
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
int Pioneer2dx_Position2d::InitChild()
{
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
int Pioneer2dx_Position2d::UpdateChild()
{
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
int Pioneer2dx_Position2d::FiniChild()
{
  return 0;
}
