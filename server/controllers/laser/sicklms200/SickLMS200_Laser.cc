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

#include <algorithm>
#include <assert.h>

#include "Sensor.hh"
#include "Global.hh"
#include "XMLConfig.hh"
#include "HingeJoint.hh"
#include "World.hh"
#include "gazebo.h"
#include "GazeboError.hh"
#include "ControllerFactory.hh"
#include "RaySensor.hh"
#include "SickLMS200_Laser.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_CONTROLLER("sicklms200_laser", SickLMS200_Laser);

////////////////////////////////////////////////////////////////////////////////
// Constructor
SickLMS200_Laser::SickLMS200_Laser(Iface *laserIface, Entity *parent)
  : Controller(laserIface, parent)
{
  this->laserIface = dynamic_cast<LaserIface*>(this->iface);
  this->myParent = dynamic_cast<RaySensor*>(this->parent);

  if (!this->laserIface)
    gzthrow("SickLMS200_Laser controller requires a LaserIface");

  if (!this->myParent)
    gzthrow("SickLMS200_Laser controller requires a Ray Sensor as its parent");
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
SickLMS200_Laser::~SickLMS200_Laser()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void SickLMS200_Laser::LoadChild(XMLConfigNode *node)
{
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void SickLMS200_Laser::InitChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void SickLMS200_Laser::UpdateChild(UpdateParams &params)
{
  this->PutLaserData();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void SickLMS200_Laser::FiniChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void SickLMS200_Laser::PutLaserData()
{
  int i, ja, jb;
  double ra, rb, r, b;
  int v;

  double maxAngle = this->myParent->GetMaxAngle();
  double minAngle = this->myParent->GetMinAngle();

  double maxRange = this->myParent->GetMaxRange();
  double minRange = this->myParent->GetMinRange();
  int rayCount = this->myParent->GetRayCount();
  int rangeCount = this->myParent->GetRangeCount();
 
  this->laserIface->Lock(1);

  // Data timestamp
  this->laserIface->data->time = World::Instance()->GetSimTime();
    
  // Read out the laser range data
  this->laserIface->data->min_angle = minAngle;
  this->laserIface->data->max_angle = maxAngle;
  this->laserIface->data->res_angle = (maxAngle - minAngle) / (rangeCount - 1);
  this->laserIface->data->max_range = maxRange;
  this->laserIface->data->range_count = rangeCount;

  assert(this->laserIface->data->range_count < GZ_LASER_MAX_RANGES );

  // Interpolate the range readings from the rays
  for (i = 0; i<rangeCount; i++)
  {
    b = (double) i * (rayCount - 1) / (rangeCount - 1);
    ja = (int) floor(b);
    jb = std::min(ja + 1, rayCount - 1);    
    b = b - floor(b);

    assert(ja >= 0 && ja < rayCount);
    assert(jb >= 0 && jb < rayCount);

    ra = std::min(this->myParent->GetRange(ja) , maxRange);
    rb = std::min(this->myParent->GetRange(jb) , maxRange);

    // Range is linear interpolation if values are close,
    // and min if they are very different
    if (fabs(ra - rb) < 0.10)
      r = (1 - b) * ra + b * rb;
    else r = std::min(ra, rb);

    // Intensity is either-or
    //v = (int) this->myParent->GetRetro(ja) || (int) this->myParent->GetRetro(jb);

    this->laserIface->data->ranges[rangeCount-i-1] =  r + minRange;
    //this->laserIface->data->intensity[i] = v;
  }
  this->laserIface->Unlock();

  // New data is available
  this->laserIface->Post();
}

