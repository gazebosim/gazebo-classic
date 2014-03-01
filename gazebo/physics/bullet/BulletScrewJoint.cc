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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/bullet/BulletPhysics.hh"
#include "gazebo/physics/bullet/BulletTypes.hh"
#include "gazebo/physics/bullet/BulletScrewJoint.hh"

namespace gazebo
{
  namespace physics
  {
    class btScrewConstraint : public btSliderConstraint
    {
      public: btScrewConstraint(btRigidBody &_rbA, btRigidBody &_rbB,
          const btTransform &_frameInA, const btTransform &_frameInB,
          bool _useLinearReferenceFrameA)
          : btSliderConstraint(_rbA, _rbB, _frameInA, _frameInB,
              _useLinearReferenceFrameA) {};

      public: btScrewConstraint(btRigidBody &_rbB,
          const btTransform &_frameInB, bool _useLinearReferenceFrameA)
          : btSliderConstraint(_rbB, _frameInB, _useLinearReferenceFrameA) {};

      public: virtual void getInfo2(btConstraintInfo2 *_info)
      {
        this->_getInfo2NonVirtual(
            _info,
            m_rbA.getCenterOfMassTransform(),
            m_rbB.getCenterOfMassTransform(),
            m_rbA.getLinearVelocity(),
            m_rbB.getLinearVelocity(),
            m_rbA.getInvMass(),
            m_rbB.getInvMass());
      }

      public: void _getInfo2NonVirtual(
          btConstraintInfo2* info,
          const btTransform& transA,
          const btTransform& transB,
          const btVector3& linVelA,
          const btVector3& linVelB,
          btScalar rbAinvMass,btScalar rbBinvMass);

      public: virtual void setThreadPitch(double _threadPitch)
      {
        this->threadPitch = _threadPitch;
      }

      public: virtual double getThreadPitch() const
      {
        return this->threadPitch;
      }

      private: double threadPitch;
    };
  }
}

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
BulletScrewJoint::BulletScrewJoint(btDynamicsWorld *_world, BasePtr _parent)
    : ScrewJoint<BulletJoint>(_parent), bulletScrew(NULL)
{
  GZ_ASSERT(_world, "bullet world pointer is NULL");
  this->bulletWorld = _world;
}

//////////////////////////////////////////////////
BulletScrewJoint::~BulletScrewJoint()
{
}

//////////////////////////////////////////////////
void BulletScrewJoint::Load(sdf::ElementPtr _sdf)
{
  ScrewJoint<BulletJoint>::Load(_sdf);
  this->SetThreadPitch(0, this->threadPitch);
}

//////////////////////////////////////////////////
void BulletScrewJoint::Init()
{
  ScrewJoint<BulletJoint>::Init();

  gzwarn << "Screw joint constraints are currently not enforced" << "\n";

  BulletLinkPtr bulletChildLink =
    boost::static_pointer_cast<BulletLink>(this->childLink);
  BulletLinkPtr bulletParentLink =
    boost::static_pointer_cast<BulletLink>(this->parentLink);

  // Get axis unit vector (expressed in world frame).
  math::Vector3 axis = this->initialWorldAxis;
  if (axis == math::Vector3::Zero)
  {
    gzerr << "axis must have non-zero length, resetting to 0 0 1\n";
    axis.Set(0, 0, 1);
  }

  // Local variables used to compute pivots and axes in body-fixed frames
  // for the parent and child links.
  math::Vector3 pivotParent, pivotChild, axisParent, axisChild;
  math::Pose pose;
  btTransform frameParent, frameChild;
  btVector3 axis2, axis3;

  // Initialize pivots to anchorPos, which is expressed in the
  // world coordinate frame.
  pivotParent = this->anchorPos;
  pivotChild = this->anchorPos;

  // Check if parentLink exists. If not, the parent will be the world.
  if (this->parentLink)
  {
    // Compute relative pose between joint anchor and CoG of parent link.
    pose = this->parentLink->GetWorldCoGPose();
    // Subtract CoG position from anchor position, both in world frame.
    pivotParent -= pose.pos;
    // Rotate pivot offset and axis into body-fixed frame of parent.
    pivotParent = pose.rot.RotateVectorReverse(pivotParent);
    frameParent.setOrigin(BulletTypes::ConvertVector3(pivotParent));
    axisParent = pose.rot.RotateVectorReverse(axis);
    axisParent = axisParent.Normalize();
    // The following math is based on btHingeConstraint.cpp:95-115
    btPlaneSpace1(BulletTypes::ConvertVector3(axisParent), axis2, axis3);
    frameParent.getBasis().setValue(
      axisParent.x, axis2.x(), axis3.x(),
      axisParent.y, axis2.y(), axis3.y(),
      axisParent.z, axis2.z(), axis3.z());
  }
  // Check if childLink exists. If not, the child will be the world.
  if (this->childLink)
  {
    // Compute relative pose between joint anchor and CoG of child link.
    pose = this->childLink->GetWorldCoGPose();
    // Subtract CoG position from anchor position, both in world frame.
    pivotChild -= pose.pos;
    // Rotate pivot offset and axis into body-fixed frame of child.
    pivotChild = pose.rot.RotateVectorReverse(pivotChild);
    frameChild.setOrigin(BulletTypes::ConvertVector3(pivotChild));
    axisChild = pose.rot.RotateVectorReverse(axis);
    axisChild = axisChild.Normalize();
    // The following math is based on btHingeConstraint.cpp:95-115
    btPlaneSpace1(BulletTypes::ConvertVector3(axisChild), axis2, axis3);
    frameChild.getBasis().setValue(
      axisChild.x, axis2.x(), axis3.x(),
      axisChild.y, axis2.y(), axis3.y(),
      axisChild.z, axis2.z(), axis3.z());
  }

  // If both links exist, then create a joint between the two links.
  if (bulletChildLink && bulletParentLink)
  {
    this->bulletScrew = new btScrewConstraint(
        *bulletParentLink->GetBulletLink(),
        *bulletChildLink->GetBulletLink(),
        frameParent, frameChild, true);
  }
  // If only the child exists, then create a joint between the child
  // and the world.
  else if (bulletChildLink)
  {
    this->bulletScrew = new btScrewConstraint(
        *bulletChildLink->GetBulletLink(), frameChild, true);
  }
  // If only the parent exists, then create a joint between the parent
  // and the world.
  else if (bulletParentLink)
  {
    this->bulletScrew = new btScrewConstraint(
        *bulletParentLink->GetBulletLink(), frameParent, true);
  }
  // Throw an error if no links are given.
  else
  {
    gzthrow("joint without links\n");
  }

  if (!this->bulletScrew)
    gzthrow("unable to create bullet screw joint\n");

  // Apply joint translation limits here.
  // TODO: velocity and effort limits.
  GZ_ASSERT(this->sdf != NULL, "Joint sdf member is NULL");
  sdf::ElementPtr limitElem;
  limitElem = this->sdf->GetElement("axis")->GetElement("limit");
  this->bulletScrew->setLowerLinLimit(limitElem->Get<double>("lower"));
  this->bulletScrew->setUpperLinLimit(limitElem->Get<double>("upper"));

  this->constraint = this->bulletScrew;

  // Add the joint to the world
  GZ_ASSERT(this->bulletWorld, "bullet world pointer is NULL");
  this->bulletWorld->addConstraint(this->constraint);

  // Allows access to impulse
  this->constraint->enableFeedback(true);

  // Setup Joint force and torque feedback
  this->SetupJointFeedback();
}

//////////////////////////////////////////////////
double BulletScrewJoint::GetVelocity(unsigned int /*_index*/) const
{
  double result = 0;
  math::Vector3 globalAxis = this->GetGlobalAxis(0);
  if (this->childLink)
    result += globalAxis.Dot(this->childLink->GetWorldLinearVel());
  if (this->parentLink)
    result -= globalAxis.Dot(this->parentLink->GetWorldLinearVel());
  return result;
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetVelocity(unsigned int _index, double _vel)
{
  math::Vector3 desiredVel;
  if (this->parentLink)
    desiredVel = this->parentLink->GetWorldLinearVel();
  desiredVel += _vel * this->GetGlobalAxis(_index);
  if (this->childLink)
    this->childLink->SetLinearVel(desiredVel);
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetAxis(unsigned int /*_index*/,
    const math::Vector3 &_axis)
{
  // Note that _axis is given in a world frame,
  // but bullet uses a body-fixed frame
  if (!this->bulletScrew)
  {
    // this hasn't been initialized yet, store axis in initialWorldAxis
    math::Quaternion axisFrame = this->GetAxisFrame(0);
    this->initialWorldAxis = axisFrame.RotateVector(_axis);
  }
  else
  {
    gzerr << "SetAxis for existing joint is not implemented\n";
  }
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetThreadPitch(unsigned int /*_index*/,
    double _threadPitch)
{
  if (this->bulletScrew)
    this->bulletScrew->setThreadPitch(_threadPitch);
}

//////////////////////////////////////////////////
double BulletScrewJoint::GetThreadPitch(unsigned int /*_index*/)
{
  double result = this->threadPitch;
  if (this->bulletScrew)
    result = this->bulletScrew->getThreadPitch();
  return result;
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetForceImpl(unsigned int /*_index*/, double /*_force*/)
{
  gzlog << "Not implemented\n";
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetHighStop(unsigned int /*_index*/,
    const math::Angle &_angle)
{
  if (this->bulletScrew)
    this->bulletScrew->setUpperLinLimit(_angle.Radian());
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetLowStop(unsigned int /*_index*/,
    const math::Angle &_angle)
{
  if (this->bulletScrew)
    this->bulletScrew->setLowerLinLimit(_angle.Radian());
}

//////////////////////////////////////////////////
math::Angle BulletScrewJoint::GetHighStop(unsigned int /*_index*/)
{
  math::Angle result;
  if (this->bulletScrew)
    result = this->bulletScrew->getUpperLinLimit();
  return result;
}

//////////////////////////////////////////////////
math::Angle BulletScrewJoint::GetLowStop(unsigned int /*_index*/)
{
  math::Angle result;
  if (this->bulletScrew)
    result = this->bulletScrew->getLowerLinLimit();
  return result;
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetMaxForce(unsigned int /*_index*/, double _force)
{
  if (this->bulletScrew)
    this->bulletScrew->setMaxLinMotorForce(_force);
}

//////////////////////////////////////////////////
double BulletScrewJoint::GetMaxForce(unsigned int /*index*/)
{
  double result = 0;
  if (this->bulletScrew)
    result = this->bulletScrew->getMaxLinMotorForce();
  return result;
}

//////////////////////////////////////////////////
math::Vector3 BulletScrewJoint::GetGlobalAxis(unsigned int /*_index*/) const
{
  math::Vector3 result = this->initialWorldAxis;
  if (this->bulletScrew)
  {
    // I have not verified the following math, though I based it on internal
    // bullet code at line 250 of btHingeConstraint.cpp
    btVector3 vec =
      this->bulletScrew->getRigidBodyA().getCenterOfMassTransform().getBasis() *
      this->bulletScrew->getFrameOffsetA().getBasis().getColumn(2);
    result = BulletTypes::ConvertVector3(vec);
  }
  else
    gzwarn << "bulletHinge does not exist, returning fake axis\n";
  return result;
}

//////////////////////////////////////////////////
math::Angle BulletScrewJoint::GetAngleImpl(unsigned int /*_index*/) const
{
  math::Angle result;
  if (this->bulletScrew)
    result = this->bulletScrew->getLinearPos();
  return result;
}

//////////////////////////////////////////////////
void btScrewConstraint::_getInfo2NonVirtual(
    btConstraintInfo2* info,
    const btTransform& transA,
    const btTransform& transB,
    const btVector3& linVelA,
    const btVector3& linVelB,
    btScalar rbAinvMass,btScalar rbBinvMass)
{
  ///TODO set up screw constraints;
}
