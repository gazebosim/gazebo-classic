/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include <boost/bind.hpp>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/dart/DARTSliderJoint.hh"

#include "dart/kinematics/Dof.h"
#include "dart/kinematics/Joint.h"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
DARTSliderJoint::DARTSliderJoint(BasePtr _parent)
    : SliderJoint<DARTJoint>(_parent)
{
}

//////////////////////////////////////////////////
DARTSliderJoint::~DARTSliderJoint()
{
}

//////////////////////////////////////////////////
void DARTSliderJoint::Load(sdf::ElementPtr _sdf)
{
  //
  SliderJoint<DARTJoint>::Load(_sdf);

  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
math::Vector3 DARTSliderJoint::GetGlobalAxis(int /*_index*/) const
{
  gzwarn << "Not implemented!\n";

  return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
math::Angle DARTSliderJoint::GetAngleImpl(int /*_index*/) const
{
   math::Angle result;

   gzwarn << "Not implemented!\n";

   return result;
}

//////////////////////////////////////////////////
double DARTSliderJoint::GetVelocity(int /*index*/) const
{
  gzwarn << "Not implemented!\n";

  return 0;
}

//////////////////////////////////////////////////
void DARTSliderJoint::SetVelocity(int /*index*/, double /*_angle*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTSliderJoint::SetAxis(int /*index*/, const math::Vector3 &_axis)
{
  // TODO: check whether below code is needed.
   if (this->childLink)
     this->childLink->SetEnabled(true);
   if (this->parentLink)
     this->parentLink->SetEnabled(true);

  // Slider joint has only one degree of freedom.
  // _axis must have a value of (1, 0, 0), (0, 1, 0), and (0, 0, 1)
  if (_axis == math::Vector3(1, 0, 0))
  {
    // When dart's 'Joint' is destroied, it deletes all 'Transform's.
    // When 'Transform' is destroied, it deletes all 'Dof's.
    kinematics::Dof* dofs = new kinematics::Dof;
    kinematics::TrfmTranslateX* trans = new kinematics::TrfmTranslateX(dofs);
    this->dartJoint->addTransform(trans);
  }
  else if (_axis == math::Vector3(1, 0, 0))
  {
    // When dart's 'Joint' is destroied, it deletes all 'Transform's.
    // When 'Transform' is destroied, it deletes all 'Dof's.
    kinematics::Dof* dofs = new kinematics::Dof;
    kinematics::TrfmTranslateY* trans = new kinematics::TrfmTranslateY(dofs);
    this->dartJoint->addTransform(trans);
  }
  else if (_axis == math::Vector3(1, 0, 0))
  {
    // When dart's 'Joint' is destroied, it deletes all 'Transform's.
    // When 'Transform' is destroied, it deletes all 'Dof's.
    kinematics::Dof* dofs = new kinematics::Dof;
    kinematics::TrfmTranslateZ* trans = new kinematics::TrfmTranslateZ(dofs);
    this->dartJoint->addTransform(trans);
  }
  else
  {
    // We assume that the axis has the value among these:
    // (1, 0, 0), (0, 1, 0), (0, 0, 1)
    gzthrow("Axis must be one of these: (1, 0, 0), (0, 1, 0), (0, 0, 1)\n");
  }
}

//////////////////////////////////////////////////
void DARTSliderJoint::SetDamping(int /*index*/, double /*_damping*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTSliderJoint::ApplyDamping()
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTSliderJoint::SetForce(int /*index*/, double /*_force*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTSliderJoint::SetMaxForce(int /*_index*/, double /*_t*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
double DARTSliderJoint::GetMaxForce(int /*_index*/)
{
  gzwarn << "Not implemented!\n";

  return 0;
}





