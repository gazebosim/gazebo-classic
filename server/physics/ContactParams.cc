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
/* Desc: Parameters for contact joints
 * Author: Nate Koenig
 * Date: 30 July 2003
 * CVS: $Id$
 */

#include <ode/ode.h>

#include "XMLConfig.hh"
#include "ContactParams.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Default constructor
ContactParams::ContactParams()
{
  this->kp = 10000.0; //dInfinity;
  this->kd = 0;
  this->bounce = 0.1;
  this->bounceVel = 0.1;
  this->softCfm = 0.01;

  this->mu1 = dInfinity;
  this->mu2 = dInfinity;
  this->slip1 = 0.01;
  this->slip2 = 0.01;
}

//////////////////////////////////////////////////////////////////////////////
/// Load the contact params
void ContactParams::Load(XMLConfigNode *node)
{
  this->kp = node->GetDouble("kp",this->kp);
  this->kd = node->GetDouble("kd",this->kd);
  this->bounce = node->GetDouble("bounce",this->bounce);
  this->bounceVel = node->GetDouble("bounceVel",this->bounceVel);

  this->mu1 = node->GetDouble("mu1",this->mu1);
  this->mu2 = node->GetDouble("mu2",this->mu2);
  this->slip1 = node->GetDouble("slip1",this->slip1);
  this->slip2 = node->GetDouble("slip2",this->slip2);

  this->softCfm = node->GetDouble("softCFM",this->softCfm);
}
