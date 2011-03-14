/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/* Desc: Parameters for contact joints
 * Author: Nate Koenig
 * Date: 30 July 2003
 * CVS: $Id$
 */

#include <float.h>
#include "common/XMLConfig.hh"
#include "SurfaceParams.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////////////////////////////////
// Default constructor
SurfaceParams::SurfaceParams()
{
  this->kp = 100000000.0;

  // Damping constraint
  this->kd = 1.0;

  // Bounce param
  this->bounce = 0.0;

  // Minumum velocity before bouce is applied
  this->bounceVel = 10.0;

  this->softCfm = 0.01;

  this->mu1 = FLT_MAX;
  this->mu2 = FLT_MAX;
  this->slip1 = 0.01;
  this->slip2 = 0.01;

  this->enableFriction = true;

  //this->useGlobalERP = false;
  //this->useGlobalCFM = false;
}

//////////////////////////////////////////////////////////////////////////////
/// Load the contact params
void SurfaceParams::Load(XMLConfigNode *node)
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
