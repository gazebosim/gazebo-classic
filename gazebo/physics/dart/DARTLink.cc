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

#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTPhysics.hh"
#include "gazebo/physics/dart/DARTModel.hh"
#include "gazebo/physics/dart/DARTLink.hh"
#include "gazebo/physics/dart/DARTJoint.hh"
#include "gazebo/physics/dart/DARTUtils.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTLink::DARTLink(EntityPtr _parent)
  : Link(_parent),
    dartBodyNode(NULL)
{

}

//////////////////////////////////////////////////
DARTLink::~DARTLink()
{
  // We don't need to delete dartBodyNode because skeletone will delete
  // dartBodyNode if this is registered to the skeletone.
}

void DARTLink::Load(sdf::ElementPtr _sdf)
{
  if (this->GetDARTPhysics() == NULL)
    gzthrow("Not using the dart physics engine");

  Link::Load(_sdf);

  // Create dart's body node according to gazebo's link.
  kinematics::BodyNode* bodyNode
      = this->GetDARTModel()->GetSkeletonDynamics()->createBodyNode();
  dynamics::BodyNodeDynamics* bodyNodeDyn
      = static_cast<dynamics::BodyNodeDynamics*>(bodyNode);

  this->dartBodyNode = bodyNodeDyn;

  // Add dart's body node to dart's skeleton.
  GetDARTModel()->GetSkeletonDynamics()->addNode(dartBodyNode, false);
}

//////////////////////////////////////////////////
void DARTLink::Init()
{
  //----------------------------------------------------------------------------
  Link::Init();

  // Mass
  double mass = this->inertial->GetMass();
  this->dartBodyNode->setMass(mass);

  // Inertia
  double Ixx = this->inertial->GetIXX();
  double Iyy = this->inertial->GetIYY();
  double Izz = this->inertial->GetIZZ();
  double Ixy = this->inertial->GetIXY();
  double Ixz = this->inertial->GetIXZ();
  double Iyz = this->inertial->GetIYZ();
  this->dartBodyNode->setLocalInertia(Ixx, Iyy, Izz, Ixy, Ixz, Iyz);

  // Name
  std::string name = this->GetName();
  this->dartBodyNode->setName(name.c_str());

  this->visuals;

  //////////////////////////////////////////////////////////////////////////////
  // TODO: At some point, DART should support collision shape and ...
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
      Eigen::Vector3d eigenSize(radius*2, radius*2, radius*2);
      kinematics::ShapeEllipsoid* shape
          = new kinematics::ShapeEllipsoid(eigenSize);
      this->dartBodyNode->setVisualizationShape(shape);
    }
    else if (geomType == "plane")
    {
      // TODO: dart does not support plane!!!
      //      math::Vector3 normal
      //          = geometryElem->GetFirstElement()->GetValueVector3("normal");
      math::Vector2d size
          = geometryElem->GetFirstElement()->GetValueVector2d("size");
      //Eigen::Vector3d eigenSize(size.x, size.y, 0.001);
      Eigen::Vector3d eigenSize(100, 100, 0.001);
            kinematics::ShapeBox* shape
          = new kinematics::ShapeBox(eigenSize);
      this->dartBodyNode->setVisualizationShape(shape);
    }

    else if (geomType == "box")
    {
      math::Vector3 mathSize
          = geometryElem->GetFirstElement()->GetValueVector3("size");
      Eigen::Vector3d eigenSize(mathSize.x, mathSize.y, mathSize.z);
      kinematics::ShapeBox* shape
          = new kinematics::ShapeBox(eigenSize);
      this->dartBodyNode->setVisualizationShape(shape);
    }
    else if (geomType == "cylinder")
    {
      double radius = geometryElem->GetFirstElement()->GetValueDouble("radius");
      double length = geometryElem->GetFirstElement()->GetValueDouble("length");
//      Eigen::Vector3d eigenSize(radius, length, 0.0);
            kinematics::ShapeCylinder* shape
                    = new kinematics::ShapeCylinder(radius, length);
//      kinematics::ShapeCylinder* shape
//          = new kinematics::ShapeCylinder(eigenSize);
      this->dartBodyNode->setVisualizationShape(shape);
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
  else
  {
      Eigen::Vector3d eigenSize(0.1, 0.1, 0.1);
      kinematics::ShapeEllipsoid* shape
          = new kinematics::ShapeEllipsoid(eigenSize);
      this->dartBodyNode->setVisualizationShape(shape);
  }
  //////////////////////////////////////////////////////////////////////////////

  //----------------------------------------------------------------------------
  math::Vector3 cog = this->inertial->GetCoG();
  math::Pose poseJointToChildLink;
  if (this->dartParentJoint)
  {
    poseJointToChildLink = this->dartParentJoint->GetPose_JointToChildLink();
  }
  else
  {
    //poseJointToChildLink = this->GetWorldPose();
    poseJointToChildLink = math::Pose::Zero;
  }

  Eigen::Vector3d dartCOMLocal(poseJointToChildLink.pos.x + cog.x,
                                poseJointToChildLink.pos.y + cog.y,
                                poseJointToChildLink.pos.z + cog.z);

  this->dartBodyNode->setLocalCOM(dartCOMLocal);

  //----------------------------------------------------------------------------
  // Gazebo's Link Pose -->> DART's BodyNode Transform
  // Set dart's body node transformation from gazebo's link pose.
  Eigen::Matrix4d newTrfm;
  DARTUtils::ConvPoseToMat(&newTrfm, this->GetWorldPose());
  this->dartBodyNode->setWorldTransform(newTrfm);
}

//////////////////////////////////////////////////
void DARTLink::Fini()
{

  Link::Fini();
}

//////////////////////////////////////////////////
void DARTLink::Update()
{
  Link::Update();
}

//////////////////////////////////////////////////
void DARTLink::OnPoseChange()
{
  Link::OnPoseChange();

  const math::Pose myPose = this->GetWorldPose();
  Eigen::Matrix4d trfm;
  DARTUtils::ConvPoseToMat(&trfm, myPose);
  this->dartBodyNode->setWorldTransform(trfm);
}

//////////////////////////////////////////////////
void DARTLink::SetEnabled(bool /*_enable*/) const
{
  // TODO: DART does not support this function yet.
}

//////////////////////////////////////////////////
bool DARTLink::GetEnabled() const
{
  bool result = true;

  // TODO: DART does not support this function yet.

  return result;
}

//////////////////////////////////////////////////
void DARTLink::UpdateMass()
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
  gzwarn << "Not implemented!\n";
}

/////////////////////////////////////////////////////////////////////
void DARTLink::UpdateSurface()
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
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::SetLinearVel(const math::Vector3& /*_vel*/)
{
  //   if (this->linkId)
  //   {
  //     dBodySetLinearVel(this->linkId, _vel.x, _vel.y, _vel.z);
  //   }
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::SetAngularVel(const math::Vector3& /*_vel*/)
{
  //   if (this->linkId)
  //   {
  //     dBodySetAngularVel(this->linkId, _vel.x, _vel.y, _vel.z);
  //   }
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::SetForce(const math::Vector3& /*_force*/)
{
  //   if (this->linkId)
  //   {
  //     this->SetEnabled(true);
  //     dBodySetForce(this->linkId, _force.x, _force.y, _force.z);
  //   }
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::SetTorque(const math::Vector3& /*_torque*/)
{
  //   if (this->linkId)
  //   {
  //     this->SetEnabled(true);
  //     dBodySetTorque(this->linkId, _torque.x, _torque.y, _torque.z);
  //   }
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::AddForce(const math::Vector3& /*_force*/)
{
  //   if (this->linkId)
  //   {
  //     this->SetEnabled(true);
  //     dBodyAddForce(this->linkId, _force.x, _force.y, _force.z);
  //   }
  gzwarn << "Not implemented!\n";
}

/////////////////////////////////////////////////
void DARTLink::AddRelativeForce(const math::Vector3& /*_force*/)
{
  //   if (this->linkId)
  //   {
  //     this->SetEnabled(true);
  //     dBodyAddRelForce(this->linkId, _force.x, _force.y, _force.z);
  //   }
  gzwarn << "Not implemented!\n";
}

/////////////////////////////////////////////////
void DARTLink::AddForceAtWorldPosition(const math::Vector3& /*_force*/,
                                        const math::Vector3& /*_pos*/)
{
  //   if (this->linkId)
  //   {
  //     this->SetEnabled(true);
  //     dBodyAddForceAtPos(this->linkId, _force.x, _force.y, _force.z,
  //                           _pos.x, _pos.y, _pos.z);
  //   }
  gzwarn << "Not implemented!\n";
}

/////////////////////////////////////////////////
void DARTLink::AddForceAtRelativePosition(const math::Vector3& /*_force*/,
                                           const math::Vector3& /*_relpos*/)
{
  //   if (this->linkId)
  //   {
  //     this->SetEnabled(true);
  //     dBodyAddForceAtRelPos(this->linkId, _force.x, _force.y, _force.z,
  //                           _relpos.x, _relpos.y, _relpos.z);
  //   }
  gzwarn << "Not implemented!\n";
}

/////////////////////////////////////////////////
void DARTLink::AddTorque(const math::Vector3& /*_torque*/)
{
  //   if (this->linkId)
  //   {
  //     this->SetEnabled(true);
  //     dBodyAddTorque(this->linkId, _torque.x, _torque.y, _torque.z);
  //   }
  gzwarn << "Not implemented!\n";
}

/////////////////////////////////////////////////
void DARTLink::AddRelativeTorque(const math::Vector3& /*_torque*/)
{
  //   if (this->linkId)
  //   {
  //     this->SetEnabled(true);
  //     dBodyAddRelTorque(this->linkId, _torque.x, _torque.y, _torque.z);
  //   }
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
math::Vector3 DARTLink::GetWorldLinearVel(
    const math::Vector3& /*_offset*/) const
{
  math::Vector3 vel;

  //const Eigen::Vector3d& linVel = this->dartBodyNode->getWorldLinearVel();
  // TODO:
  const Eigen::Vector3d& linVel =  this->dartBodyNode->mVel;

  vel.x = linVel[0];
  vel.y = linVel[1];
  vel.z = linVel[2];

  return vel;
}

//////////////////////////////////////////////////
math::Vector3 DARTLink::GetWorldLinearVel(
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
  gzwarn << "Not implemented!\n";

  return vel;
}

math::Vector3 DARTLink::GetWorldCoGLinearVel() const
{
  math::Vector3 vel;

  gzwarn << "Not implemented!\n";

  return vel;
}

//////////////////////////////////////////////////
math::Vector3 DARTLink::GetWorldAngularVel() const
{
  math::Vector3 vel;

  const Eigen::Vector3d& AngularVel =  this->dartBodyNode->mOmega;

  vel.x = AngularVel[0];
  vel.y = AngularVel[1];
  vel.z = AngularVel[2];

  return vel;
}

/////////////////////////////////////////////////
math::Vector3 DARTLink::GetWorldForce() const
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

  gzwarn << "Not implemented!\n";

  return force;
}

//////////////////////////////////////////////////
math::Vector3 DARTLink::GetWorldTorque() const
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

  gzwarn << "Not implemented!\n";

  return torque;
}

//////////////////////////////////////////////////
void DARTLink::SetGravityMode(bool _mode)
{
  this->sdf->GetElement("gravity")->Set(_mode);

  this->dartBodyNode->setGravityMode(_mode);
}

//////////////////////////////////////////////////
bool DARTLink::GetGravityMode() const
{
  int mode = 0;

  this->dartBodyNode->getGravityMode();

  return mode;
}

//////////////////////////////////////////////////
void DARTLink::SetSelfCollide(bool /*_collide*/)
{
  //   this->sdf->GetElement("self_collide")->Set(_collide);
  //   if (_collide)
  //     this->spaceId = dSimpleSpaceCreate(this->odePhysics->GetSpaceId());
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::SetLinearDamping(double /*_damping*/)
{
  //   if (this->GetODEId())
  //     dBodySetLinearDamping(this->GetODEId(), _damping);
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::SetAngularDamping(double /*_damping*/)
{
  //   if (this->GetODEId())
  //     dBodySetAngularDamping(this->GetODEId(), _damping);
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::SetKinematic(const bool& _state)
{
  this->sdf->GetElement("kinematic")->Set(_state);
  //   if (this->linkId)
  //   {
  //     if (_state)
  //       dBodySetKinematic(this->linkId);
  //     else
  //       dBodySetDynamic(this->linkId);
  //   }
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
bool DARTLink::GetKinematic() const
{
  bool result = false;

  //   if (this->linkId)
  //     result = dBodyIsKinematic(this->linkId);
  gzwarn << "Not implemented!\n";

  return result;
}

//////////////////////////////////////////////////
void DARTLink::SetAutoDisable(bool /*_disable*/)
{
  //   if (this->GetModel()->GetJointCount() == 0 && this->linkId)
  //   {
  //     dBodySetAutoDisableFlag(this->linkId, _disable);
  //   }
  gzwarn << "Not implemented!\n";
}

void DARTLink::updateDirtyPoseFromDARTTransformation()
{
  //-- Step 1: get dart body's transformation
  Eigen::Matrix4d tran = this->dartBodyNode->getWorldTransform();

  //-- Step 2: set gazebo link's pose using the transformation
  math::Pose newPose;
  DARTUtils::ConvMatToPose(&newPose, tran);

  // Set the new pose to this link
  this->dirtyPose = newPose;

  // Set the new pose to the world
  // (Below method can be changed in gazebo code)
  this->world->dirtyPoses.push_back(this);
}


//////////////////////////////////////////////////
DARTPhysicsPtr DARTLink::GetDARTPhysics(void) const {
  return boost::shared_dynamic_cast<DARTPhysics>(this->GetWorld()->GetPhysicsEngine());
}

//////////////////////////////////////////////////
simulation::World* DARTLink::GetDARTWorld(void) const
{
  return GetDARTPhysics()->GetDARTWorld();
}

//////////////////////////////////////////////////
DARTModelPtr DARTLink::GetDARTModel() const
{
  return boost::shared_dynamic_cast<DARTModel>(this->GetModel());
}

//////////////////////////////////////////////////
void DARTLink::SetDARTParentJoint(DARTJointPtr _dartParentJoint)
{
  dartParentJoint = _dartParentJoint;
}

//////////////////////////////////////////////////
void DARTLink::AddDARTChildJoint(DARTJointPtr _dartChildJoint)
{
  dartChildJoints.push_back(_dartChildJoint);
}

