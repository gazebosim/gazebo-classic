/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include <string>

#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyPhysics.hh"
#include "gazebo/physics/simbody/SimbodyTypes.hh"
#include "gazebo/physics/simbody/SimbodyScrewJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyScrewJoint::SimbodyScrewJoint(SimTK::MultibodySystem * /*_world*/,
                                     BasePtr _parent)
    : ScrewJoint<SimbodyJoint>(_parent)
{
  this->physicsInitialized = false;
}

//////////////////////////////////////////////////
SimbodyScrewJoint::~SimbodyScrewJoint()
{
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::Load(sdf::ElementPtr _sdf)
{
  ScrewJoint<SimbodyJoint>::Load(_sdf);
  this->SetThreadPitch(0, this->threadPitch);
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::Init()
{
  gzerr << "Not implemented in simbody\n";
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::GetVelocity(unsigned int /*_index*/) const
{
  gzerr << "Not implemented in simbody\n";
  return 0;
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetVelocity(unsigned int /*_index*/, double /*_angle*/)
{
  gzerr << "Not implemented in simbody\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetAxis(unsigned int /*_index*/,
    const math::Vector3 &/*_axis*/)
{
  gzerr << "Not implemented in simbody\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetThreadPitch(unsigned int /*_index*/,
    double /*_threadPitch*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetForceImpl(unsigned int /*_index*/, double /*_force*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetMaxForce(unsigned int /*_index*/, double /*_force*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::GetMaxForce(unsigned int /*index*/)
{
  gzerr << "Not implemented\n";
  return 0;
}

//////////////////////////////////////////////////
math::Vector3 SimbodyScrewJoint::GetGlobalAxis(unsigned int /*_index*/) const
{
  gzerr << "SimbodyScrewJoint::GetGlobalAxis not implemented\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
math::Angle SimbodyScrewJoint::GetAngleImpl(unsigned int /*_index*/) const
{
  gzerr << "SimbodyScrewJoint::GetAngleImpl not implemented\n";
  return math::Angle();
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::GetThreadPitch(unsigned int /*_index*/)
{
  gzerr << "Not implemented in Simbody\n";
  return 0;
}

//////////////////////////////////////////////////
void SimbodyScrewJoint::SetAttribute(const std::string &_key,
  unsigned int _index,
  const boost::any &_value)
{
  this->SetParam(_key, _index, _value);
}

//////////////////////////////////////////////////
bool SimbodyScrewJoint::SetParam(const std::string &_key,
  unsigned int _index,
  const boost::any &_value)
{
  if (_key  == "thread_pitch")
  {
    try
    {
      this->threadPitch = boost::any_cast<double>(_value);
    }
    catch(const boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
      return false;
    }
  }
  else
    return SimbodyJoint::SetParam(_key, _index, _value);

  return true;
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::GetAttribute(const std::string &_key,
  unsigned int _index)
{
  return this->GetParam(_key, _index);
}

//////////////////////////////////////////////////
double SimbodyScrewJoint::GetParam(const std::string &_key,
  unsigned int _index)
{
  if (_key  == "thread_pitch")
    return this->threadPitch;
  else
    return SimbodyJoint::GetParam(_key, _index);
}
