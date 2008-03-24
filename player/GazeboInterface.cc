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
/* Desc: Generic Gazebo Device Inteface for Player
 * Author: Nate Koenig
 * Date: 2 March 2006
 * CVS: $Id$
 */


#include "GazeboInterface.hh"
#include "GazeboDriver.hh"

using namespace gazebo;

GazeboInterface::GazeboInterface(player_devaddr_t addr, GazeboDriver *driver,
                                 ConfigFile * /*cf*/, int /*section*/)
{
  this->device_addr = addr;
  this->driver = driver;
}

GazeboInterface::~GazeboInterface()
{
}

