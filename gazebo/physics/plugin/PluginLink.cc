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
/* Desc: Link class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#include <math.h>
#include <sstream>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/plugin/PluginCollision.hh"
#include "gazebo/physics/plugin/PluginSurfaceParams.hh"
#include "gazebo/physics/plugin/PluginPhysics.hh"
#include "gazebo/physics/plugin/PluginLink.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
PluginLink::PluginLink(EntityPtr _parent)
    : Link(_parent)
{
  this->linkId = NULL;
}

//////////////////////////////////////////////////
PluginLink::~PluginLink()
{
  this->linkId = NULL;
}

//////////////////////////////////////////////////
void PluginLink::Load(sdf::ElementPtr _sdf)
{
  this->pluginPhysics = boost::dynamic_pointer_cast<PluginPhysics>(
      this->GetWorld()->GetPhysicsEngine());

  if (this->pluginPhysics == NULL)
    gzthrow("Not using the plugin physics engine");

  Link::Load(_sdf);
}

//////////////////////////////////////////////////
void PluginLink::Init()
{
  if (!this->IsStatic())
  {

    // Only use auto disable if no joints and no sensors are present
    if (this->GetModel()->GetAutoDisable() &&
        this->GetModel()->GetJointCount() == 0 &&
        this->GetSensorCount() == 0)
    {
    }
    else
    {
    }
  }

  GZ_ASSERT(this->sdf != NULL, "Unable to initialize link, SDF is NULL");
  this->SetKinematic(this->sdf->Get<bool>("kinematic"));
  this->SetGravityMode(this->sdf->Get<bool>("gravity"));

  this->SetLinearDamping(this->GetLinearDamping());
  this->SetAngularDamping(this->GetAngularDamping());

  Link::Init();
}

//////////////////////////////////////////////////
void PluginLink::Fini()
{
  Link::Fini();

  this->pluginPhysics.reset();
}

//////////////////////////////////////////////////
void PluginLink::OnPoseChange()
{
  Link::OnPoseChange();
}

//////////////////////////////////////////////////
void PluginLink::SetEnabled(bool _enable) const
{
}

/////////////////////////////////////////////////////////////////////
bool PluginLink::GetEnabled() const
{
  bool result = true;
  return result;
}

/////////////////////////////////////////////////////////////////////
void PluginLink::UpdateSurface()
{
}
/////////////////////////////////////////////////////////////////////
void PluginLink::UpdateMass()
{
}

//////////////////////////////////////////////////
void PluginLink::SetLinearVel(const math::Vector3 &_vel)
{
}

//////////////////////////////////////////////////
math::Vector3 PluginLink::GetWorldLinearVel(const math::Vector3 &_offset) const
{
  math::Vector3 vel;
  return vel;
}

//////////////////////////////////////////////////
math::Vector3 PluginLink::GetWorldLinearVel(const math::Vector3 &_offset,
                                         const math::Quaternion &_q) const
{
  math::Vector3 vel;
  return vel;
}

//////////////////////////////////////////////////
math::Vector3 PluginLink::GetWorldCoGLinearVel() const
{
  math::Vector3 vel;
  return vel;
}

//////////////////////////////////////////////////
void PluginLink::SetAngularVel(const math::Vector3 &_vel)
{
}

//////////////////////////////////////////////////
math::Vector3 PluginLink::GetWorldAngularVel() const
{
  math::Vector3 vel;
  return vel;
}

//////////////////////////////////////////////////
void PluginLink::SetForce(const math::Vector3 &_force)
{
}

//////////////////////////////////////////////////
void PluginLink::SetTorque(const math::Vector3 &_torque)
{
}

//////////////////////////////////////////////////
void PluginLink::AddForce(const math::Vector3 &_force)
{
}

/////////////////////////////////////////////////
void PluginLink::AddRelativeForce(const math::Vector3 &_force)
{
}

/////////////////////////////////////////////////
void PluginLink::AddForceAtRelativePosition(const math::Vector3 &_force,
                               const math::Vector3 &_relpos)
{
}

/////////////////////////////////////////////////
void PluginLink::AddForceAtWorldPosition(const math::Vector3 &_force,
                                      const math::Vector3 &_pos)
{
}

/////////////////////////////////////////////////
void PluginLink::AddTorque(const math::Vector3 &_torque)
{
}

/////////////////////////////////////////////////
void PluginLink::AddRelativeTorque(const math::Vector3 &_torque)
{
}

/////////////////////////////////////////////////
math::Vector3 PluginLink::GetWorldForce() const
{
  math::Vector3 force;
  return force;
}

//////////////////////////////////////////////////
math::Vector3 PluginLink::GetWorldTorque() const
{
  math::Vector3 torque;
  return torque;
}

//////////////////////////////////////////////////
void PluginLink::SetLinearDamping(double _damping)
{
}

//////////////////////////////////////////////////
void PluginLink::SetAngularDamping(double _damping)
{
}

//////////////////////////////////////////////////
void PluginLink::SetKinematic(const bool &_state)
{
}

//////////////////////////////////////////////////
bool PluginLink::GetKinematic() const
{
  bool result = false;
  return result;
}

//////////////////////////////////////////////////
void PluginLink::SetAutoDisable(bool _disable)
{
}

//////////////////////////////////////////////////
void PluginLink::SetLinkStatic(bool /*_static*/)
{
}
