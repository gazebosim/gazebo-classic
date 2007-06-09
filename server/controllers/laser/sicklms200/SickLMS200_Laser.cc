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
 * Desc: SickLMS200 Laser controller.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN info: $Id: SickLMS200_Laser.cc 28 2007-05-31 00:53:17Z natepak $
 */

#include "Sensor.hh"
#include "Global.hh"
#include "XMLConfig.hh"
#include "HingeJoint.hh"
#include "World.hh"
#include "gazebo.h"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "SickLMS200_Laser.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("sicklms200_laser", SickLMS200_Laser);

////////////////////////////////////////////////////////////////////////////////
// Constructor
SickLMS200_Laser::SickLMS200_Laser(Iface *iface, Entity *parent)
  : Controller(iface, parent)
{
  this->myIface = dynamic_cast<LaserIface*>(this->iface);
  this->myParent = dynamic_cast<Sensor*>(this->parent);

  if (!this->myIface)
    gzthrow("SickLMS200_Laser controller requires a LaserIface");

  if (!this->myParent)
    gzthrow("SickLMS200_Laser controller requires a Sensor as its parent");
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
SickLMS200_Laser::~SickLMS200_Laser()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
int SickLMS200_Laser::LoadChild(XMLConfigNode *node)
{
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
int SickLMS200_Laser::InitChild()
{
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
int SickLMS200_Laser::UpdateChild(UpdateParams &params)
{
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
int SickLMS200_Laser::FiniChild()
{
  return 0;
}
