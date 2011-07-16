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
#include "physics/SurfaceParams.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////////////////////////////////
// Default constructor
SurfaceParams::SurfaceParams()
{
  // Bounce param
  this->bounce = 0.0;

  // Minumum velocity before bounce is applied
  this->bounceThreshold = 10.0;

  this->kp = 1000000.0;
  this->kd = 10000.0;
  this->cfm = 0;
  this->erp = 0.2; // hm, not defined in sdf?
  this->mu1 = 1.0;
  this->mu2 = 1.0;
  this->max_vel = -1.0;
  this->min_depth = 0.0;
  this->slip1 = 0.0;
  this->slip2 = 0.0;
  this->enableFriction = true;
  this->fdir1 = math::Vector3(0,0,0);
}

//////////////////////////////////////////////////////////////////////////////
/// Load the contact params
void SurfaceParams::Load( sdf::ElementPtr &_sdf )
{
  sdf::ElementPtr bounceElem = _sdf->GetElement("bounce");
  if (bounceElem)
  {
    this->bounce = bounceElem->GetValueDouble("restitution_coefficient");
    this->bounceThreshold = bounceElem->GetValueDouble("threshold");
  }

  // TODO: read mu1, mu2, fdir1, slip1, slip2 from sdf
  sdf::ElementPtr frictionElem = _sdf->GetElement("friction");
  sdf::ElementPtr frictionOdeElem = frictionElem->GetElement("ode");
  if (frictionOdeElem)
  {
    this->mu1 = frictionOdeElem->GetValueDouble("mu");
    this->mu2 = frictionOdeElem->GetValueDouble("mu2");
    this->slip1 = frictionOdeElem->GetValueDouble("slip1");
    this->slip2 = frictionOdeElem->GetValueDouble("slip2");
    this->fdir1 = frictionOdeElem->GetValueVector3("fdir1");
  }


  // TODO: read kp, kd, soft_cfm, max_vel, min_depth from sdf
  sdf::ElementPtr contactElem = _sdf->GetElement("contact");
  sdf::ElementPtr contactOdeElem = contactElem->GetElement("ode");
  if (contactOdeElem)
  {
    this->kp = contactOdeElem->GetValueDouble("kp");
    this->kd = contactOdeElem->GetValueDouble("kd");
    this->cfm = contactOdeElem->GetValueDouble("soft_cfm");
    this->max_vel = contactOdeElem->GetValueDouble("max_vel");
    this->min_depth = contactOdeElem->GetValueDouble("min_depth");
  }
}
