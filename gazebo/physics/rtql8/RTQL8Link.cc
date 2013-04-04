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
#include "gazebo/physics/rtql8/RTQL8Physics.hh"
#include "gazebo/physics/rtql8/RTQL8Model.hh"
#include "gazebo/physics/rtql8/RTQL8Link.hh"
#include "gazebo/physics/rtql8/RTQL8Joint.hh"
#include "gazebo/physics/rtql8/RTQL8Utils.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
RTQL8Link::RTQL8Link(EntityPtr _parent)
  : Link(_parent),
    rtql8BodyNode(NULL)
{

}

//////////////////////////////////////////////////
RTQL8Link::~RTQL8Link()
{
  // We don't need to delete rtql8BodyNode because skeletone will delete
  // rtql8BodyNode if this is registered to the skeletone.
}

void RTQL8Link::Load(sdf::ElementPtr _sdf)
{
  if (this->GetRTQL8Physics() == NULL)
    gzthrow("Not using the rtql8 physics engine");

  Link::Load(_sdf);

  // Create rtql8's body node according to gazebo's link.
  rtql8::kinematics::BodyNode* bodyNode
      = this->GetRTQL8Model()->GetSkeletonDynamics()->createBodyNode();
  rtql8::dynamics::BodyNodeDynamics* bodyNodeDyn
      = static_cast<rtql8::dynamics::BodyNodeDynamics*>(bodyNode);

  this->rtql8BodyNode = bodyNodeDyn;

  // Add rtql8's body node to rtql8's skeleton.
  GetRTQL8Model()->GetSkeletonDynamics()->addNode(rtql8BodyNode, false);
}

//////////////////////////////////////////////////
void RTQL8Link::Init()
{
  //----------------------------------------------------------------------------
  Link::Init();

  //////////////////////////////////////////////////////////////////////////////
  // TODO: At some point, RTQL8 should support collision shape and ...
  //
  //////////////////////////////////////////////////////////////////////////////
  if (this->sdf->HasElement("visual"))
  {
    sdf::ElementPtr visualElem = this->sdf->GetElement("visual");
    sdf::ElementPtr geometryElem = visualElem->GetElement("geometry");
    std::string geomType = geometryElem->GetFirstElement()->GetName();

    if (geomType == "sphere")
    {
      double radius = geometryElem->GetFirstElement()->GetValueDouble("radius");
      Eigen::Vector3d eigenSize(radius, radius, radius);
      rtql8::kinematics::ShapeEllipsoid* shape
          = new rtql8::kinematics::ShapeEllipsoid(eigenSize, 1);
      this->rtql8BodyNode->setVisualShape(shape);
    }
    else if (geomType == "plane")
    {
      // TODO: rtql8 does not support plane!!!
      //      math::Vector3 normal
      //          = geometryElem->GetFirstElement()->GetValueVector3("normal");
      math::Vector2d size
          = geometryElem->GetFirstElement()->GetValueVector2d("size");
      Eigen::Vector3d eigenSize(size.x, size.y, 0.001);
      rtql8::kinematics::ShapeCube* shape
          = new rtql8::kinematics::ShapeCube(eigenSize, 1);
      this->rtql8BodyNode->setVisualShape(shape);
    }

    else if (geomType == "box")
    {
      math::Vector3 mathSize
          = geometryElem->GetFirstElement()->GetValueVector3("size");
      Eigen::Vector3d eigenSize(mathSize.x, mathSize.y, mathSize.z);
      rtql8::kinematics::ShapeCube* shape
          = new rtql8::kinematics::ShapeCube(eigenSize, 1);
      this->rtql8BodyNode->setVisualShape(shape);
    }
    else if (geomType == "cylinder")
    {
      double radius = geometryElem->GetFirstElement()->GetValueDouble("radius");
      double length = geometryElem->GetFirstElement()->GetValueDouble("length");
      Eigen::Vector3d eigenSize(radius, length, 0.0);
      //      rtql8::kinematics::ShapeCylinder* shape
      //              = new rtql8::kinematics::ShapeCylinder(radius, length);
      rtql8::kinematics::ShapeCylinder* shape
          = new rtql8::kinematics::ShapeCylinder(eigenSize, 1);
      this->rtql8BodyNode->setVisualShape(shape);
    }
    else if (geomType == "multiray")
      gzerr << "Not implemented yet...";
    else if (geomType == "mesh" || geomType == "trimesh")
      gzerr << "Not implemented yet...";
    else if (geomType == "heightmap")
      gzerr << "Not implemented yet...";
    else if (geomType == "map" || geomType == "image")
      gzerr << "Not implemented yet...";
    else if (geomType == "ray")
      gzerr << "Not implemented yet...";
    else
      gzerr << "Unknown visual type[" << geomType << "]\n";

  }
  //////////////////////////////////////////////////////////////////////////////

  //----------------------------------------------------------------------------
  math::Vector3 cog = this->inertial->GetCoG();
  math::Pose poseJointToChildLink;
  if (this->rtql8ParentJoint)
  {
    poseJointToChildLink = this->rtql8ParentJoint->GetPose_JointToChildLink();
  }
  else
  {
    //poseJointToChildLink = this->GetWorldPose();
    poseJointToChildLink = math::Pose::Zero;
  }

  Eigen::Vector3d rtql8COMLocal(poseJointToChildLink.pos.x + cog.x,
                                poseJointToChildLink.pos.y + cog.y,
                                poseJointToChildLink.pos.z + cog.z);

  this->rtql8BodyNode->setLocalCOM(rtql8COMLocal);

  //----------------------------------------------------------------------------
  // Gazebo's Link Pose -->> RTQL8's BodyNode Transform
  // Set rtql8's body node transformation from gazebo's link pose.
  Eigen::Matrix4d newTrfm;
  RTQL8Utils::ConvPoseToMat(&newTrfm, this->GetWorldPose());
  this->rtql8BodyNode->setWorldTransform(newTrfm);
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

  const math::Pose myPose = this->GetWorldPose();
  Eigen::Matrix4d trfm;
  RTQL8Utils::ConvPoseToMat(&trfm, myPose);
  this->rtql8BodyNode->setWorldTransform(trfm);
}

//////////////////////////////////////////////////
void RTQL8Link::SetEnabled(bool /*_enable*/) const
{
  // TODO: RTQL8 does not support this function yet.
}

//////////////////////////////////////////////////
bool RTQL8Link::GetEnabled() const
{
  bool result = true;

  // TODO: RTQL8 does not support this function yet.

  return result;
}

//////////////////////////////////////////////////
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
  gzerr << "Not implemented...\n";
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
  gzerr << "Not implemented...\n";
}

//////////////////////////////////////////////////
void RTQL8Link::SetLinearVel(const math::Vector3& /*_vel*/)
{
  //   if (this->linkId)
  //   {
  //     dBodySetLinearVel(this->linkId, _vel.x, _vel.y, _vel.z);
  //   }
  gzerr << "Not implemented...\n";
}

//////////////////////////////////////////////////
void RTQL8Link::SetAngularVel(const math::Vector3& /*_vel*/)
{
  //   if (this->linkId)
  //   {
  //     dBodySetAngularVel(this->linkId, _vel.x, _vel.y, _vel.z);
  //   }
  gzerr << "Not implemented...\n";
}

//////////////////////////////////////////////////
void RTQL8Link::SetForce(const math::Vector3& /*_force*/)
{
  //   if (this->linkId)
  //   {
  //     this->SetEnabled(true);
  //     dBodySetForce(this->linkId, _force.x, _force.y, _force.z);
  //   }
  gzerr << "Not implemented...\n";
}

//////////////////////////////////////////////////
void RTQL8Link::SetTorque(const math::Vector3& /*_torque*/)
{
  //   if (this->linkId)
  //   {
  //     this->SetEnabled(true);
  //     dBodySetTorque(this->linkId, _torque.x, _torque.y, _torque.z);
  //   }
  gzerr << "Not implemented...\n";
}

//////////////////////////////////////////////////
void RTQL8Link::AddForce(const math::Vector3& /*_force*/)
{
  //   if (this->linkId)
  //   {
  //     this->SetEnabled(true);
  //     dBodyAddForce(this->linkId, _force.x, _force.y, _force.z);
  //   }
  gzerr << "Not implemented...\n";
}

/////////////////////////////////////////////////
void RTQL8Link::AddRelativeForce(const math::Vector3& /*_force*/)
{
  //   if (this->linkId)
  //   {
  //     this->SetEnabled(true);
  //     dBodyAddRelForce(this->linkId, _force.x, _force.y, _force.z);
  //   }
  gzerr << "Not implemented...\n";
}

/////////////////////////////////////////////////
void RTQL8Link::AddForceAtWorldPosition(const math::Vector3& /*_force*/,
                                        const math::Vector3& /*_pos*/)
{
  //   if (this->linkId)
  //   {
  //     this->SetEnabled(true);
  //     dBodyAddForceAtPos(this->linkId, _force.x, _force.y, _force.z,
  //                           _pos.x, _pos.y, _pos.z);
  //   }
  gzerr << "Not implemented...\n";
}

/////////////////////////////////////////////////
void RTQL8Link::AddForceAtRelativePosition(const math::Vector3& /*_force*/,
                                           const math::Vector3& /*_relpos*/)
{
  //   if (this->linkId)
  //   {
  //     this->SetEnabled(true);
  //     dBodyAddForceAtRelPos(this->linkId, _force.x, _force.y, _force.z,
  //                           _relpos.x, _relpos.y, _relpos.z);
  //   }
  gzerr << "Not implemented...\n";
}

/////////////////////////////////////////////////
void RTQL8Link::AddTorque(const math::Vector3& /*_torque*/)
{
  //   if (this->linkId)
  //   {
  //     this->SetEnabled(true);
  //     dBodyAddTorque(this->linkId, _torque.x, _torque.y, _torque.z);
  //   }
}

/////////////////////////////////////////////////
void RTQL8Link::AddRelativeTorque(const math::Vector3& /*_torque*/)
{
  //   if (this->linkId)
  //   {
  //     this->SetEnabled(true);
  //     dBodyAddRelTorque(this->linkId, _torque.x, _torque.y, _torque.z);
  //   }
}

//////////////////////////////////////////////////
math::Vector3 RTQL8Link::GetWorldLinearVel(
    const math::Vector3& /*_offset*/) const
{
  math::Vector3 vel;

  // TODO: This is the velocity in body frame not world frame.
  const Eigen::Vector3d& linVel = this->rtql8BodyNode->mVel;

  vel.x = linVel[0];
  vel.y = linVel[1];
  vel.z = linVel[2];

  //   if (this->linkId)
  //   {
  //     const dReal *dvel;
  //     dvel = dBodyGetLinearVel(this->linkId);
  //     vel.Set(dvel[0], dvel[1], dvel[2]);
  //   }

  return vel;
}

//////////////////////////////////////////////////
math::Vector3 RTQL8Link::GetWorldLinearVel(
    const gazebo::math::Vector3& /*_offset*/,
    const gazebo::math::Quaternion& /*_q*/) const
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

math::Vector3 RTQL8Link::GetWorldCoGLinearVel() const
{
  math::Vector3 vel;

  gzerr << "Not implemented yet...";

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

  gzerr << "Not implemented...\n";
  return torque;
}

//////////////////////////////////////////////////
void RTQL8Link::SetGravityMode(bool _mode)
{
  this->sdf->GetElement("gravity")->Set(_mode);

  this->rtql8BodyNode->setGravityMode(_mode);
}

//////////////////////////////////////////////////
bool RTQL8Link::GetGravityMode() const
{
  int mode = 0;

  this->rtql8BodyNode->getGravityMode();

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
void RTQL8Link::SetKinematic(const bool& _state)
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

void RTQL8Link::updateDirtyPoseFromRTQL8Transformation()
{
  //-- Step 1: get rtql8 body's transformation
  Eigen::Matrix4d tran = this->rtql8BodyNode->getWorldTransform();

  //-- Step 2: set gazebo link's pose using the transformation
  math::Pose newPose;
  RTQL8Utils::ConvMatToPose(&newPose, tran);

  // Set the new pose to this link
  this->dirtyPose = newPose;

  // Set the new pose to the world
  // (Below method can be changed in gazebo code)
  this->world->dirtyPoses.push_back(this);
}


//////////////////////////////////////////////////
RTQL8PhysicsPtr RTQL8Link::GetRTQL8Physics(void) const {
  return boost::shared_dynamic_cast<RTQL8Physics>(this->GetWorld()->GetPhysicsEngine());
}

//////////////////////////////////////////////////
rtql8::simulation::World* RTQL8Link::GetRTQL8World(void) const
{
  return GetRTQL8Physics()->GetRTQL8World();
}

//////////////////////////////////////////////////
RTQL8ModelPtr RTQL8Link::GetRTQL8Model() const
{
  return boost::shared_dynamic_cast<RTQL8Model>(this->GetModel());
}

//////////////////////////////////////////////////
void RTQL8Link::SetRTQL8ParentJoint(RTQL8JointPtr _rtql8ParentJoint)
{
  rtql8ParentJoint = _rtql8ParentJoint;
}

//////////////////////////////////////////////////
void RTQL8Link::AddRTQL8ChildJoint(RTQL8JointPtr _rtql8ChildJoint)
{
  rtql8ChildJoints.push_back(_rtql8ChildJoint);
}






