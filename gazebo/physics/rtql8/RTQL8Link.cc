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

#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/World.hh"

#include "gazebo/physics/rtql8/rtql8_inc.h"
//#include "physics/bullet/BulletCollision.hh"
//#include "physics/bullet/BulletMotionState.hh"
#include "gazebo/physics/rtql8/RTQL8Physics.hh"
#include "gazebo/physics/rtql8/RTQL8Model.hh"
#include "gazebo/physics/rtql8/RTQL8Link.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
RTQL8Link::RTQL8Link(EntityPtr _parent)
  : Link(_parent), rtql8BodyNode(NULL)
{

}

//////////////////////////////////////////////////
RTQL8Link::~RTQL8Link()
{
  if (rtql8BodyNode)
    delete rtql8BodyNode;
}

void RTQL8Link::Load(sdf::ElementPtr _sdf)
{
  this->rtql8Physics = boost::shared_dynamic_cast<RTQL8Physics>(
      this->GetWorld()->GetPhysicsEngine());

  if (this->rtql8Physics == NULL)
    gzthrow("Not using the rtql8 physics engine");

  Link::Load(_sdf);

  // TODO:
  this->rtql8BodyNode = new rtql8::kinematics::BodyNode();

  RTQL8ModelPtr rtql8Model
      = boost::shared_dynamic_cast<RTQL8Model>(this->GetModel());
  rtql8Model->GetSkeletonDynamics()->addNode(rtql8BodyNode);
}

//////////////////////////////////////////////////
void RTQL8Link::Init()
{
  Link::Init();

  // TODO:
  math::Pose pose = this->GetWorldPose();

  // Set pose
  rtql8BodyNode;
}

//////////////////////////////////////////////////
void RTQL8Link::Fini()
{

  Link::Fini();
}

//////////////////////////////////////////////////
void RTQL8Link::Update()
{
  Link::Update();
}

//////////////////////////////////////////////////
void RTQL8Link::OnPoseChange()
{
  Link::OnPoseChange();

//   if (!this->linkId)
//     return;
// 
//   this->SetEnabled(true);
// 
//   const math::Pose myPose = this->GetWorldPose();
// 
//   math::Vector3 cog = myPose.rot.RotateVector(this->inertial->GetCoG());
// 
//   // adding cog location for ode pose
//   dBodySetPosition(this->linkId,
//       myPose.pos.x + cog.x,
//       myPose.pos.y + cog.y,
//       myPose.pos.z + cog.z);
// 
//   dQuaternion q;
//   q[0] = myPose.rot.w;
//   q[1] = myPose.rot.x;
//   q[2] = myPose.rot.y;
//   q[3] = myPose.rot.z;
// 
//   // Set the rotation of the ODE link
//   dBodySetQuaternion(this->linkId, q);
}

//////////////////////////////////////////////////
void RTQL8Link::SetEnabled(bool /*_enable*/) const
{
//   if (!this->linkId)
//     return;

//   if (_enable)
//     dBodyEnable(this->linkId);
//   else
//     dBodyDisable(this->linkId);
}

/////////////////////////////////////////////////////////////////////
bool RTQL8Link::GetEnabled() const
{
  bool result = true;

//   if (this->linkId)
//     result = dBodyIsEnabled(this->linkId);

  return result;
}

/////////////////////////////////////////////////////////////////////
void RTQL8Link::UpdateMass()
{
//   if (!this->linkId)
//     return;
// 
//   dMass odeMass;
//   dMassSetZero(&odeMass);
// 
//   // The CoG must always be (0, 0, 0)
//   math::Vector3 cog(0, 0, 0);
// 
//   math::Vector3 principals = this->inertial->GetPrincipalMoments();
//   math::Vector3 products = this->inertial->GetProductsofInertia();
// 
//   dMassSetParameters(&odeMass, this->inertial->GetMass(),
//       cog.x, cog.y, cog.z,
//       principals.x, principals.y, principals.z,
//       products.x, products.y, products.z);
// 
//   if (this->inertial->GetMass() > 0)
//     dBodySetMass(this->linkId, &odeMass);
//   else
//     gzthrow("Setting custom link " + this->GetName() + "mass to zero!");
}

/////////////////////////////////////////////////////////////////////
void RTQL8Link::UpdateSurface()
{
//   Base_V::iterator iter;
//   Base_V::iterator iter_end = this->children.end();
//   for (iter = this->children.begin(); iter != iter_end; ++iter)
//   {
//     if ((*iter)->HasType(Base::COLLISION))
//     {
//       ODECollisionPtr g = boost::shared_static_cast<ODECollision>(*iter);
//       if (g->IsPlaceable() && g->GetCollisionId())
//       {
//         // Set surface properties max_vel and min_depth
//         dBodySetMaxVel(this->linkId, g->GetSurface()->maxVel);
//         dBodySetMinDepth(this->linkId, g->GetSurface()->minDepth);
//       }
//     }
//   }
}

//////////////////////////////////////////////////
void RTQL8Link::SetLinearVel(const math::Vector3 & /*_vel*/)
{
//   if (this->linkId)
//   {
//     dBodySetLinearVel(this->linkId, _vel.x, _vel.y, _vel.z);
//   }
}

//////////////////////////////////////////////////
void RTQL8Link::SetAngularVel(const math::Vector3 & /*_vel*/)
{
//   if (this->linkId)
//   {
//     dBodySetAngularVel(this->linkId, _vel.x, _vel.y, _vel.z);
//   }
}

//////////////////////////////////////////////////
void RTQL8Link::SetForce(const math::Vector3 & /*_force*/)
{
//   if (this->linkId)
//   {
//     this->SetEnabled(true);
//     dBodySetForce(this->linkId, _force.x, _force.y, _force.z);
//   }
}

//////////////////////////////////////////////////
void RTQL8Link::SetTorque(const math::Vector3 & /*_torque*/)
{
//   if (this->linkId)
//   {
//     this->SetEnabled(true);
//     dBodySetTorque(this->linkId, _torque.x, _torque.y, _torque.z);
//   }
}

//////////////////////////////////////////////////
void RTQL8Link::AddForce(const math::Vector3 & /*_force*/)
{
//   if (this->linkId)
//   {
//     this->SetEnabled(true);
//     dBodyAddForce(this->linkId, _force.x, _force.y, _force.z);
//   }
}

/////////////////////////////////////////////////
void RTQL8Link::AddRelativeForce(const math::Vector3 & /*_force*/)
{
//   if (this->linkId)
//   {
//     this->SetEnabled(true);
//     dBodyAddRelForce(this->linkId, _force.x, _force.y, _force.z);
//   }
}

/////////////////////////////////////////////////
void RTQL8Link::AddForceAtWorldPosition(const math::Vector3 & /*_force*/,
                                      const math::Vector3 & /*_pos*/)
{
//   if (this->linkId)
//   {
//     this->SetEnabled(true);
//     dBodyAddForceAtPos(this->linkId, _force.x, _force.y, _force.z,
//                           _pos.x, _pos.y, _pos.z);
//   }
}
/////////////////////////////////////////////////
void RTQL8Link::AddForceAtRelativePosition(const math::Vector3 & /*_force*/,
                               const math::Vector3 & /*_relpos*/)
{
//   if (this->linkId)
//   {
//     this->SetEnabled(true);
//     dBodyAddForceAtRelPos(this->linkId, _force.x, _force.y, _force.z,
//                           _relpos.x, _relpos.y, _relpos.z);
//   }
}

/////////////////////////////////////////////////
void RTQL8Link::AddTorque(const math::Vector3 & /*_torque*/)
{
//   if (this->linkId)
//   {
//     this->SetEnabled(true);
//     dBodyAddTorque(this->linkId, _torque.x, _torque.y, _torque.z);
//   }
}

/////////////////////////////////////////////////
void RTQL8Link::AddRelativeTorque(const math::Vector3 & /*_torque*/)
{
//   if (this->linkId)
//   {
//     this->SetEnabled(true);
//     dBodyAddRelTorque(this->linkId, _torque.x, _torque.y, _torque.z);
//   }
}

//////////////////////////////////////////////////
math::Vector3 RTQL8Link::GetWorldLinearVel() const
{
  math::Vector3 vel;

//   if (this->linkId)
//   {
//     const dReal *dvel;
//     dvel = dBodyGetLinearVel(this->linkId);
//     vel.Set(dvel[0], dvel[1], dvel[2]);
//   }

  return vel;
}

//////////////////////////////////////////////////
math::Vector3 RTQL8Link::GetWorldAngularVel() const
{
  math::Vector3 vel;

//   if (this->linkId)
//   {
//     const dReal *dvel;
// 
//     dvel = dBodyGetAngularVel(this->linkId);
// 
//     vel.Set(dvel[0], dvel[1], dvel[2]);
//   }

  return vel;
}

/////////////////////////////////////////////////
math::Vector3 RTQL8Link::GetWorldForce() const
{
  math::Vector3 force;

//   if (this->linkId)
//   {
//     const dReal *dforce;
// 
//     dforce = dBodyGetForce(this->linkId);
// 
//     force.x = dforce[0];
//     force.y = dforce[1];
//     force.z = dforce[2];
//   }

  return force;
}

//////////////////////////////////////////////////
math::Vector3 RTQL8Link::GetWorldTorque() const
{
  math::Vector3 torque;

//   if (this->linkId)
//   {
//     const dReal *dtorque;
// 
//     dtorque = dBodyGetTorque(this->linkId);
// 
//     torque.x = dtorque[0];
//     torque.y = dtorque[1];
//     torque.z = dtorque[2];
//   }

  return torque;
}

//////////////////////////////////////////////////
void RTQL8Link::SetGravityMode(bool _mode)
{
  this->sdf->GetElement("gravity")->Set(_mode);
//   if (this->linkId)
//   {
//     dBodySetGravityMode(this->linkId, _mode ? 1: 0);
//   }
}

//////////////////////////////////////////////////
bool RTQL8Link::GetGravityMode()
{
  int mode = 0;
//   if (this->linkId)
//   {
//     mode = dBodyGetGravityMode(this->linkId);
//   }

  return mode;
}

//////////////////////////////////////////////////
void RTQL8Link::SetSelfCollide(bool /*_collide*/)
{
//   this->sdf->GetElement("self_collide")->Set(_collide);
//   if (_collide)
//     this->spaceId = dSimpleSpaceCreate(this->odePhysics->GetSpaceId());
}

//////////////////////////////////////////////////
void RTQL8Link::SetLinearDamping(double /*_damping*/)
{
//   if (this->GetODEId())
//     dBodySetLinearDamping(this->GetODEId(), _damping);
}

//////////////////////////////////////////////////
void RTQL8Link::SetAngularDamping(double /*_damping*/)
{
//   if (this->GetODEId())
//     dBodySetAngularDamping(this->GetODEId(), _damping);
}

//////////////////////////////////////////////////
void RTQL8Link::SetKinematic(const bool &_state)
{
  this->sdf->GetElement("kinematic")->Set(_state);
//   if (this->linkId)
//   {
//     if (_state)
//       dBodySetKinematic(this->linkId);
//     else
//       dBodySetDynamic(this->linkId);
//   }
}

//////////////////////////////////////////////////
bool RTQL8Link::GetKinematic() const
{
  bool result = false;

//   if (this->linkId)
//     result = dBodyIsKinematic(this->linkId);

  return result;
}

//////////////////////////////////////////////////
void RTQL8Link::SetAutoDisable(bool /*_disable*/)
{
//   if (this->GetModel()->GetJointCount() == 0 && this->linkId)
//   {
//     dBodySetAutoDisableFlag(this->linkId, _disable);
//   }
}

















