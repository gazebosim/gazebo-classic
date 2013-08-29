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
      this->dartBodyNode->addVisualizationShape(shape);
    }
    else if (geomType == "plane")
    {
      // TODO: dart does not support plane!!!
      //      math::Vector3 normal
      //          = geometryElem->GetFirstElement()->GetValueVector3("normal");
      math::Vector2d size
          = geometryElem->GetFirstElement()->Get<math::Vector2d>("size");
      //Eigen::Vector3d eigenSize(size.x, size.y, 0.001);
      Eigen::Vector3d eigenSize(2100, 2100, 0.001);
            kinematics::ShapeBox* shape
          = new kinematics::ShapeBox(eigenSize);
      this->dartBodyNode->addVisualizationShape(shape);
    }

    else if (geomType == "box")
    {
      math::Vector3 mathSize
          = geometryElem->GetFirstElement()->Get<math::Vector3>("size");
      Eigen::Vector3d eigenSize(mathSize.x, mathSize.y, mathSize.z);
      kinematics::ShapeBox* shape
          = new kinematics::ShapeBox(eigenSize);
      this->dartBodyNode->addVisualizationShape(shape);
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
      this->dartBodyNode->addVisualizationShape(shape);
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
      this->dartBodyNode->addVisualizationShape(shape);
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

//  Eigen::Vector3d dartCOMLocal(cog.x, cog.y, cog.z);

  this->dartBodyNode->setLocalCOM(dartCOMLocal);

  //----------------------------------------------------------------------------
  // Gazebo's Link Pose -->> DART's BodyNode Transform
  // Set dart's body node transformation from gazebo's link pose.
  Eigen::Matrix4d newTrfm;
  DARTUtils::ConvPoseToMat(&newTrfm, this->GetWorldPose());
  this->dartBodyNode->setWorldTransform(newTrfm);










  if (dartParentJoint == 0)
  {
    // TODO: need to access to Model::canonicalLink
    //       the member is private for now. this should be protected.
    //LinkPtr this = this;

    kinematics::BodyNode* parentBodyNode = NULL;
    kinematics::BodyNode* childBodyNode
        = /*boost::shared_dynamic_cast<DARTLink>*/(this)->GetBodyNode();

    kinematics::Joint* newJoint
        = new kinematics::Joint(parentBodyNode, childBodyNode);
    GetDARTModel()->GetSkeletonDynamics()->addJoint(newJoint);

    //---- Step 1. Transformation from rotated joint frame to child link frame.
    math::Pose canonicalLinkPose = this->GetWorldPose();
//    DARTUtils::addTransformToDARTJoint(this->dartCanonicalJoint,
//                                         canonicalLinkPose);

    //---- Step 2. Transformation by the rotate axis.
    kinematics::Dof* tranX = new kinematics::Dof(0, -10000, 10000);
    kinematics::Dof* tranY = new kinematics::Dof(0, -10000, 10000);
    kinematics::Dof* tranZ = new kinematics::Dof(0, -10000, 10000);
//    kinematics::Dof* rotX = new kinematics::Dof(0, -6.1416, 6.1416);
//    kinematics::Dof* rotY = new kinematics::Dof(0, -6.1416, 6.1416);
//    kinematics::Dof* rotZ = new kinematics::Dof(0, -6.1416, 6.1416);
    kinematics::Dof* rotX = new kinematics::Dof(0, -60.1416, 60.1416);
    kinematics::Dof* rotY = new kinematics::Dof(0, -60.1416, 60.1416);
    kinematics::Dof* rotZ = new kinematics::Dof(0, -60.1416, 60.1416);

    kinematics::TrfmTranslate* trfmTranslateCanonical
        = new kinematics::TrfmTranslate(tranX, tranY, tranZ);
    kinematics::TrfmRotateExpMap* trfmRotateCanonical
        = new kinematics::TrfmRotateExpMap(rotX, rotY, rotZ);

    // Set the initial pose (transformation) of bodies.
    tranX->setValue(canonicalLinkPose.pos.x);
    tranY->setValue(canonicalLinkPose.pos.y);
    tranZ->setValue(canonicalLinkPose.pos.z);

    Eigen::Quaterniond eigenQuat(canonicalLinkPose.rot.w,
                                canonicalLinkPose.rot.x,
                                canonicalLinkPose.rot.y,
                                canonicalLinkPose.rot.z);

    //Eigen::Quaterniond expToQuat(Eigen::Vector3d& v);
    Eigen::Vector3d eigenVec3 = dart_math::quatToExp(eigenQuat);

    rotX->setValue(eigenVec3(0));
    rotY->setValue(eigenVec3(1));
    rotZ->setValue(eigenVec3(2));

    // Get the model associated with
    // Add the transform to the skeletone in the model.
    // add to model because it's variable
    newJoint->addTransform(trfmTranslateCanonical, true);
    newJoint->addTransform(trfmRotateCanonical, true);

    GetDARTModel()->GetSkeletonDynamics()->addTransform(trfmTranslateCanonical);
    GetDARTModel()->GetSkeletonDynamics()->addTransform(trfmRotateCanonical);
  }

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
  gzwarn << "Not implemented!\n";
}

/////////////////////////////////////////////////////////////////////
void DARTLink::UpdateSurface()
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::SetLinearVel(const math::Vector3& /*_vel*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::SetAngularVel(const math::Vector3& /*_vel*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::SetForce(const math::Vector3& /*_force*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::SetTorque(const math::Vector3& /*_torque*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::AddForce(const math::Vector3& /*_force*/)
{
  gzwarn << "Not implemented!\n";
}

/////////////////////////////////////////////////
void DARTLink::AddRelativeForce(const math::Vector3& /*_force*/)
{
  gzwarn << "Not implemented!\n";
}

/////////////////////////////////////////////////
void DARTLink::AddForceAtWorldPosition(const math::Vector3& /*_force*/,
                                        const math::Vector3& /*_pos*/)
{
  gzwarn << "Not implemented!\n";
}

/////////////////////////////////////////////////
void DARTLink::AddForceAtRelativePosition(const math::Vector3& /*_force*/,
                                           const math::Vector3& /*_relpos*/)
{
  gzwarn << "Not implemented!\n";
}

/////////////////////////////////////////////////
void DARTLink::AddTorque(const math::Vector3& /*_torque*/)
{
  gzwarn << "Not implemented!\n";
}

/////////////////////////////////////////////////
void DARTLink::AddRelativeTorque(const math::Vector3& /*_torque*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
gazebo::math::Vector3 DARTLink::GetWorldLinearVel(
    const math::Vector3& /*_offset*/) const
{
  math::Vector3 vel;

  // TODO:
  //const Eigen::Vector3d& linVel = this->dartBodyNode->getWorldLinearVel();
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

  gzwarn << "Not implemented!\n";

  return vel;
}

math::Vector3 DARTLink::GetWorldCoGLinearVel() const
{
  math::Vector3 vel;

  const Eigen::Vector3d& linVel =  this->dartBodyNode->mVel;

  vel.x = linVel[0];
  vel.y = linVel[1];
  vel.z = linVel[2];

  return vel;

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

  gzwarn << "Not implemented!\n";

  return force;
}

//////////////////////////////////////////////////
math::Vector3 DARTLink::GetWorldTorque() const
{
  math::Vector3 torque;

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
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::SetLinearDamping(double /*_damping*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::SetAngularDamping(double /*_damping*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTLink::SetKinematic(const bool& _state)
{
  this->sdf->GetElement("kinematic")->Set(_state);

  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
bool DARTLink::GetKinematic() const
{
  bool result = false;

  gzwarn << "Not implemented!\n";

  return result;
}

//////////////////////////////////////////////////
void DARTLink::SetAutoDisable(bool /*_disable*/)
{
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

