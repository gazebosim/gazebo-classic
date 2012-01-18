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
/* Desc: common::Parameters for contact joints
 * Author: Nate Koenig
 * Date: 30 July 2003
 */

#include <float.h>
#include "physics/ode/ODESurfaceParams.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODESurfaceParams::ODESurfaceParams()
{
  this->kp = 1000000.0;
  this->kd = 10000.0;

  this->bounce = 0.0;
  this->bounceThreshold = 10.0;

  this->softCFM = 0.0;
  this->softERP = 0.2;

  this->mu1 = 0.0;
  this->mu2 = 0.0;
  this->slip1 = 0.0;
  this->slip2 = 0.0;
  this->fdir1 = math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
ODESurfaceParams::~ODESurfaceParams()
{
}

//////////////////////////////////////////////////
/// Load the contact params
void ODESurfaceParams::Load(sdf::ElementPtr _sdf)
{
  if (_sdf->HasElement("friction"))
  {
    sdf::ElementPtr friction = _sdf->GetElement("friction")->GetElement("ode");
    this->mu1 = friction->GetValueDouble("mu");
    this->mu2 = friction->GetValueDouble("mu2");
    this->slip1 = friction->GetValueDouble("slip1");
    this->slip2 = friction->GetValueDouble("slip2");
    this->fdir1 = friction->GetValueVector3("fdir1");
  }

  if (_sdf->HasElement("bounce"))
  {
    sdf::ElementPtr bounceElem = _sdf->GetElement("bounce");
    this->bounce = bounceElem->GetValueDouble("restitution_coefficient");
    this->bounceThreshold = bounceElem->GetValueDouble("threshold");
  }

  if (_sdf->HasElement("contact"))
  {
    sdf::ElementPtr contact = _sdf->GetElement("contact")->GetElement("ode");
    this->softCFM = contact->GetValueDouble("soft_cfm");
    this->softERP = contact->GetValueDouble("soft_erp");
    this->kp = contact->GetValueDouble("kp");
    this->kd = contact->GetValueDouble("kd");
    this->maxVel = contact->GetValueDouble("max_vel");
    this->minDepth = contact->GetValueDouble("min_depth");
  }
}





