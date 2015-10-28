/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include <algorithm>
#include <limits>
#include <string>

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
              _useLinearReferenceFrameA), threadPitch(1.0) {}

      public: btScrewConstraint(btRigidBody &_rbB,
          const btTransform &_frameInB, bool _useLinearReferenceFrameA)
          : btSliderConstraint(_rbB, _frameInB, _useLinearReferenceFrameA),
          threadPitch(1.0) {}

      public: virtual void getInfo1(btConstraintInfo1 *_info)
      {
        this->_getInfo1NonVirtual(_info);
      }

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

      public: void _getInfo1NonVirtual(btConstraintInfo1* info);
      public: void _getInfo2NonVirtual(
          btConstraintInfo2* info,
          const btTransform& transA,
          const btTransform& transB,
          const btVector3& linVelA,
          const btVector3& linVelB,
          btScalar rbAinvMass, btScalar rbBinvMass);

      public: btScalar getLinearPosition();
      public: btScalar getAngularPosition();

      // needed non-const version for SetForce
      public: btRigidBody& getRigidBodyA();
      public: btRigidBody& getRigidBodyB();

      public: virtual void setThreadPitch(double _threadPitch)
      {
        this->threadPitch = -_threadPitch;
      }

      public: virtual double getThreadPitch() const
      {
        return -this->threadPitch;
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
math::Vector3 BulletScrewJoint::GetAnchor(unsigned int /*index*/) const
{
  gzerr << "BulletScrewJoint::GetAnchor not implemented, return 0 vector.\n";
  return math::Vector3();
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetAnchor(unsigned int /*index*/,
    const math::Vector3 &/*_anchor*/)
{
  gzerr << "BulletScrewJoint::SetAnchor not implemented.\n";
}

//////////////////////////////////////////////////
void BulletScrewJoint::Init()
{
  ScrewJoint<BulletJoint>::Init();

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
    gzerr << "Unable to create a joint without links\n";
    return;
  }

  if (!this->bulletScrew)
  {
    gzerr << "unable to create bullet screw joint\n";
    return;
  }

  // Apply joint translation limits here.
  // TODO: velocity and effort limits.
  GZ_ASSERT(this->sdf != NULL, "Joint sdf member is NULL");
  sdf::ElementPtr limitElem;
  limitElem = this->sdf->GetElement("axis")->GetElement("limit");
  // joint limit is set on the revolute dof in sdf,
  double upper = limitElem->Get<double>("upper");
  double lower = limitElem->Get<double>("lower");
  this->bulletScrew->setLowerAngLimit(lower);
  this->bulletScrew->setUpperAngLimit(upper);
  // enforce linear dof in bullet.
  double tp = this->threadPitch;
  if (math::equal(tp, 0.0))
  {
    gzerr << "thread pitch should not be zero (joint is a slider?)"
          << " using thread pitch = 1.0e6\n";
    tp = 1.0e6;
  }
  if (tp > 0)
  {
    this->bulletScrew->setLowerLinLimit(lower/tp);
    this->bulletScrew->setUpperLinLimit(upper/tp);
  }
  else
  {
    this->bulletScrew->setLowerLinLimit(upper/tp);
    this->bulletScrew->setUpperLinLimit(lower/tp);
  }
  this->bulletScrew->setThreadPitch(tp);

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
  this->SetThreadPitch(_threadPitch);
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetThreadPitch(double _threadPitch)
{
  this->threadPitch = _threadPitch;
}

//////////////////////////////////////////////////
double BulletScrewJoint::GetThreadPitch()
{
  double result = this->threadPitch;
  if (this->bulletScrew)
    result = this->bulletScrew->getThreadPitch();
  else
    gzwarn << "bulletScrew not created yet, returning cached threadPitch.\n";
  return result;
}

//////////////////////////////////////////////////
double BulletScrewJoint::GetThreadPitch(unsigned int /*_index*/)
{
  return this->GetThreadPitch();
}

//////////////////////////////////////////////////
void BulletScrewJoint::SetForceImpl(unsigned int _index, double _force)
{
  if (this->bulletScrew)
  {
    // x-axis of constraint frame
    btVector3 hingeAxisLocalA =
      this->bulletScrew->getFrameOffsetA().getBasis().getColumn(0);
    btVector3 hingeAxisLocalB =
      this->bulletScrew->getFrameOffsetB().getBasis().getColumn(0);

    btVector3 hingeAxisWorldA =
      this->bulletScrew->getRigidBodyA().getWorldTransform().getBasis() *
      hingeAxisLocalA;
    btVector3 hingeAxisWorldB =
      this->bulletScrew->getRigidBodyB().getWorldTransform().getBasis() *
      hingeAxisLocalB;

    btVector3 hingeEffortA = _force * hingeAxisWorldA;
    btVector3 hingeEffortB = _force * hingeAxisWorldB;

    if (_index == 0)
    {
      this->bulletScrew->getRigidBodyA().applyTorque(-hingeEffortA);
      this->bulletScrew->getRigidBodyB().applyTorque(hingeEffortB);
    }
    else if (_index == 1)
    {
      if (this->constraint)
      {
        // TODO: switch to applyForce and specify body-fixed offset
        this->constraint->getRigidBodyA().applyCentralForce(-hingeEffortA);
        this->constraint->getRigidBodyB().applyCentralForce(hingeEffortB);
      }
      else
        gzerr << "BulletScrewJoint::constraint not created yet.\n";
    }
    else
      gzerr << "Invalid index [" << _index << "]\n";
  }
  else
    gzerr << "bulletScrew not created yet.\n";
}

//////////////////////////////////////////////////
bool BulletScrewJoint::SetHighStop(unsigned int _index,
                      const math::Angle &_angle)
{
  Joint::SetHighStop(0, _angle);

  // bulletScrew axial rotation is backward
  if (!this->bulletScrew)
  {
    gzerr << "bulletScrew not created yet.\n";
    return false;
  }

  if (_index == 0)
  {
    // _index = 0: angular constraint
    double upperAng = this->bulletScrew->getUpperAngLimit();
    this->bulletScrew->setUpperAngLimit(std::max(upperAng, _angle.Radian()));

    // set corresponding linear constraints
    double tp = this->threadPitch;
    if (math::equal(tp, 0.0))
    {
      gzwarn << "thread pitch should not be zero (joint is a slider?)"
             << " using thread pitch = 1.0e6\n";
      tp = 1.0e6;
    }
    // linear is angular / threadPitch
    if (tp > 0)
    {
      double lowerLin = this->bulletScrew->getLowerLinLimit();
      this->bulletScrew->setUpperLinLimit(std::max(lowerLin,
        _angle.Radian()/tp));
    }
    else
    {
      // flip upper lower because thread pitch is negative
      double upperLin = this->bulletScrew->getUpperLinLimit();
      this->bulletScrew->setLowerLinLimit(std::min(upperLin,
        _angle.Radian()/tp));
    }
    return true;
  }
  else if (_index == 1)
  {
    // _index = 1: linear constraint
    double lowerLin = this->bulletScrew->getLowerLinLimit();
    this->bulletScrew->setUpperLinLimit(std::max(lowerLin, _angle.Radian()));

    // set corresponding angular constraints
    double tp = this->threadPitch;
    // angular is linear * threadPitch
    if (tp > 0)
    {
      double lowerAng = this->bulletScrew->getLowerAngLimit();
      this->bulletScrew->setUpperAngLimit(std::max(lowerAng,
        _angle.Radian()*tp));
    }
    else
    {
      // flip upper lower because thread pitch is negative
      double upperAng = this->bulletScrew->getUpperAngLimit();
      this->bulletScrew->setLowerAngLimit(std::min(upperAng,
        _angle.Radian()*tp));
    }
    return true;
  }
  else
  {
    gzerr << "Invalid index [" << _index << "]\n";
    return false;
  }
}

//////////////////////////////////////////////////
bool BulletScrewJoint::SetLowStop(unsigned int _index,
                     const math::Angle &_angle)
{
  Joint::SetLowStop(0, _angle);

  // bulletScrew axial rotation is backward
  if (!this->bulletScrew)
  {
    gzerr << "bulletScrew not created yet.\n";
    return false;
  }

  // bulletScrew axial rotation is backward
  if (_index == 0)
  {
    // _index = 0: angular constraint
    double upperAng = this->bulletScrew->getUpperAngLimit();
    this->bulletScrew->setLowerAngLimit(std::min(upperAng, _angle.Radian()));

    // set corresponding linear constraints
    double tp = this->threadPitch;
    if (math::equal(tp, 0.0))
    {
      gzerr << "thread pitch should not be zero (joint is a slider?)"
            << " using thread pitch = 1.0e6\n";
      tp = 1.0e6;
    }
    // linear is angular / threadPitch
    if (tp > 0)
    {
      double upperLin = this->bulletScrew->getUpperLinLimit();
      this->bulletScrew->setLowerLinLimit(std::min(upperLin,
        _angle.Radian()/tp));
    }
    else
    {
        // flip upper lower because thread pitch is negative
      double lowerLin = this->bulletScrew->getLowerLinLimit();
      this->bulletScrew->setUpperLinLimit(std::max(lowerLin,
        _angle.Radian()/tp));
    }
    return true;
  }
  else if (_index == 1)
  {
    // _index = 1: linear constraint
    double upperLin = this->bulletScrew->getUpperLinLimit();
    this->bulletScrew->setLowerLinLimit(std::min(upperLin, _angle.Radian()));

    // set corresponding angular constraints
    double tp = this->threadPitch;
    // angular is linear * threadPitch
    if (tp > 0)
    {
      double upperAng = this->bulletScrew->getUpperAngLimit();
      this->bulletScrew->setLowerAngLimit(std::min(upperAng,
        _angle.Radian()*tp));
    }
    else
    {
        // flip upper lower because thread pitch is negative
      double lowerAng = this->bulletScrew->getLowerAngLimit();
      this->bulletScrew->setUpperAngLimit(std::max(lowerAng,
        _angle.Radian()*tp));
    }
    return true;
  }
  else
  {
    gzerr << "Invalid index [" << _index << "]\n";
    return false;
  }
}

//////////////////////////////////////////////////
math::Vector3 BulletScrewJoint::GetGlobalAxis(unsigned int /*_index*/) const
{
  math::Vector3 result = this->initialWorldAxis;
  if (this->bulletScrew)
  {
    // bullet uses x-axis for slider
    btVector3 vec =
      this->bulletScrew->getRigidBodyA().getCenterOfMassTransform().getBasis()
      * this->bulletScrew->getFrameOffsetA().getBasis().getColumn(0);
    result = BulletTypes::ConvertVector3(vec);
  }
  else
    gzwarn << "bulletScrew does not exist, returning fake axis\n";
  return result;
}

//////////////////////////////////////////////////
math::Angle BulletScrewJoint::GetAngleImpl(unsigned int _index) const
{
  math::Angle result;
  if (this->bulletScrew)
  {
    if (_index == 0)
    {
      // angular position
      result = this->bulletScrew->getAngularPosition();
    }
    else if (_index == 1)
    {
      // linear position
      result = this->bulletScrew->getLinearPosition();
    }
    else
      gzerr << "Invalid index [" << _index << "]\n";
  }
  else
    gzerr << "bulletScrew not created yet\n";
  return result;
}

//////////////////////////////////////////////////
double BulletScrewJoint::GetParam(
  const std::string &_key, unsigned int _index)
{
  if (_key  == "thread_pitch")
    return this->threadPitch;
  else
    return BulletJoint::GetParam(_key, _index);
}

//////////////////////////////////////////////////
btScalar btScrewConstraint::getAngularPosition()
{
  this->calculateTransforms(
    m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform());
  const btVector3 axisA0 = m_calculatedTransformA.getBasis().getColumn(1);
  const btVector3 axisA1 = m_calculatedTransformA.getBasis().getColumn(2);
  const btVector3 axisB0 = m_calculatedTransformB.getBasis().getColumn(1);
  return btAtan2(axisB0.dot(axisA1), axisB0.dot(axisA0));
}

//////////////////////////////////////////////////
btScalar btScrewConstraint::getLinearPosition()
{
  this->calculateTransforms(
    m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform());
  return this->m_depth[0];
}

//////////////////////////////////////////////////
btRigidBody& btScrewConstraint::getRigidBodyA()
{
  return this->m_rbA;
}

//////////////////////////////////////////////////
btRigidBody& btScrewConstraint::getRigidBodyB()
{
  return this->m_rbB;
}

//////////////////////////////////////////////////
void btScrewConstraint::_getInfo2NonVirtual(
    btConstraintInfo2* info,
    const btTransform& transA,
    const btTransform& transB,
    const btVector3& linVelA,
    const btVector3& linVelB,
    btScalar rbAinvMass, btScalar rbBinvMass)
{
  /// This is a copy of btSliderConstraint::getInfo2NonVirtual(...)
  /// with minor changes to the ax1 direction constraint.
  /// Mainly, the axial limit constraint is always on and is
  /// changed to a screw constraint.

  /// First, always turn on
  const btTransform& trA = getCalculatedTransformA();
  const btTransform& trB = getCalculatedTransformB();

  btAssert(!m_useSolveConstraintObsolete);
  int i, s = info->rowskip;

  btScalar signFact = m_useLinearReferenceFrameA ?
    btScalar(1.0f) : btScalar(-1.0f);

  // difference between frames in WCS
  btVector3 ofs = trB.getOrigin() - trA.getOrigin();
  // now get weight factors depending on masses
  btScalar miA = rbAinvMass;
  btScalar miB = rbBinvMass;
  bool hasStaticBody = (miA < SIMD_EPSILON) || (miB < SIMD_EPSILON);
  btScalar miS = miA + miB;
  btScalar factA, factB;
  if (miS > btScalar(0.f))
  {
    factA = miB / miS;
  }
  else
  {
    factA = btScalar(0.5f);
  }
  factB = btScalar(1.0f) - factA;
  btVector3 ax1, p, q;
  btVector3 ax1A = trA.getBasis().getColumn(0);
  btVector3 ax1B = trB.getBasis().getColumn(0);
  if (m_useOffsetForConstraintFrame)
  {
    // get the desired direction of slider axis
    // as weighted sum of X-orthos of frameA and frameB in WCS
    ax1 = ax1A * factA + ax1B * factB;
    ax1.normalize();
    // construct two orthos to slider axis
    btPlaneSpace1(ax1, p, q);
  }
  else
  { // old way - use frameA
    ax1 = trA.getBasis().getColumn(0);
    // get 2 orthos to slider axis (Y, Z)
    p = trA.getBasis().getColumn(1);
    q = trA.getBasis().getColumn(2);
  }
  // make rotations around these orthos equal
  // the slider axis should be the only unconstrained
  // rotational axis, the angular velocity of the two bodies perpendicular to
  // the slider axis should be equal. thus the constraint equations are
  //    p*w1 - p*w2 = 0
  //    q*w1 - q*w2 = 0
  // where p and q are unit vectors normal to the slider axis, and w1 and w2
  // are the angular velocity vectors of the two bodies.
  info->m_J1angularAxis[0] = p[0];
  info->m_J1angularAxis[1] = p[1];
  info->m_J1angularAxis[2] = p[2];
  info->m_J1angularAxis[s+0] = q[0];
  info->m_J1angularAxis[s+1] = q[1];
  info->m_J1angularAxis[s+2] = q[2];

  info->m_J2angularAxis[0] = -p[0];
  info->m_J2angularAxis[1] = -p[1];
  info->m_J2angularAxis[2] = -p[2];
  info->m_J2angularAxis[s+0] = -q[0];
  info->m_J2angularAxis[s+1] = -q[1];
  info->m_J2angularAxis[s+2] = -q[2];
  // compute the right hand side of the constraint equation. set relative
  // body velocities along p and q to bring the slider back into alignment.
  // if ax1A,ax1B are the unit length slider axes as computed from bodyA and
  // bodyB, we need to rotate both bodies along the axis u = (ax1 x ax2).
  // if "theta" is the angle between ax1 and ax2, we need an angular velocity
  // along u to cover angle erp*theta in one step :
  //   |angular_velocity| = angle/time = erp*theta / stepsize
  //                      = (erp*fps) * theta
  //    angular_velocity  = |angular_velocity| * (ax1 x ax2) / |ax1 x ax2|
  //                      = (erp*fps) * theta * (ax1 x ax2) / sin(theta)
  // ...as ax1 and ax2 are unit length. if theta is smallish,
  // theta ~= sin(theta), so
  //    angular_velocity  = (erp*fps) * (ax1 x ax2)
  // ax1 x ax2 is in the plane space of ax1, so we project the angular
  // velocity to p and q to find the right hand side.
//  btScalar k = info->fps * info->erp * getSoftnessOrthoAng();
  btScalar currERP = (m_flags & BT_SLIDER_FLAGS_ERP_ORTANG) ?
    m_softnessOrthoAng : m_softnessOrthoAng * info->erp;
  btScalar k = info->fps * currERP;

  btVector3 u = ax1A.cross(ax1B);
  info->m_constraintError[0] = k * u.dot(p);
  info->m_constraintError[s] = k * u.dot(q);
  if (m_flags & BT_SLIDER_FLAGS_CFM_ORTANG)
  {
    info->cfm[0] = m_cfmOrthoAng;
    info->cfm[s] = m_cfmOrthoAng;
  }

  // last filled row
  int nrow = 1;
  int srow;
  btScalar limit_err;
  int limit;
  int powered;

  // next two rows.
  // we want: velA + wA x relA == velB + wB x relB ... but this would
  // result in three equations, so we project along two orthos to the
  // slider axis

  btTransform bodyA_trans = transA;
  btTransform bodyB_trans = transB;
  nrow++;
  int s2 = nrow * s;
  nrow++;
  int s3 = nrow * s;
  btVector3 tmpA(0, 0, 0), tmpB(0, 0, 0), relA(0, 0, 0),
    relB(0, 0, 0), c(0, 0, 0);
  if (m_useOffsetForConstraintFrame)
  {
    // get vector from bodyB to frameB in WCS
    relB = trB.getOrigin() - bodyB_trans.getOrigin();
    // get its projection to slider axis
    btVector3 projB = ax1 * relB.dot(ax1);
    // get vector directed from bodyB to slider axis (and orthogonal to it)
    btVector3 orthoB = relB - projB;
    // same for bodyA
    relA = trA.getOrigin() - bodyA_trans.getOrigin();
    btVector3 projA = ax1 * relA.dot(ax1);
    btVector3 orthoA = relA - projA;
    // get desired offset between frames A and B along slider axis
    btScalar sliderOffs = m_linPos - m_depth[0];
    // desired vector from projection of center of bodyA to projection of
    // center of bodyB to slider axis
    btVector3 totalDist = projA + ax1 * sliderOffs - projB;
    // get offset vectors relA and relB
    relA = orthoA + totalDist * factA;
    relB = orthoB - totalDist * factB;
    // now choose average ortho to slider axis
    p = orthoB * factA + orthoA * factB;
    btScalar len2 = p.length2();
    if (len2 > SIMD_EPSILON)
    {
      p /= btSqrt(len2);
    }
    else
    {
      p = trA.getBasis().getColumn(1);
    }
    // make one more ortho
    q = ax1.cross(p);
    // fill two rows
    tmpA = relA.cross(p);
    tmpB = relB.cross(p);
    for (i = 0; i < 3; ++i)
    {
      info->m_J1angularAxis[s2+i] = tmpA[i];
      info->m_J2angularAxis[s2+i] = -tmpB[i];
    }
    tmpA = relA.cross(q);
    tmpB = relB.cross(q);
    if (hasStaticBody && getSolveAngLimit())
    { // to make constraint between static and dynamic objects more rigid
      // remove wA (or wB) from equation if angular limit is hit
      tmpB *= factB;
      tmpA *= factA;
    }
    for (i = 0; i < 3; ++i)
    {
      info->m_J1angularAxis[s3+i] = tmpA[i];
      info->m_J2angularAxis[s3+i] = -tmpB[i];
      info->m_J1linearAxis[s2+i] = p[i];
      info->m_J1linearAxis[s3+i] = q[i];
      info->m_J2linearAxis[s2+i] = -p[i];
      info->m_J2linearAxis[s3+i] = -q[i];
    }
  }
  else
  {
    // old way - maybe incorrect if bodies are not on the slider axis
    // see discussion "Bug in slider constraint"
    // http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=4024&start=0
    c = bodyB_trans.getOrigin() - bodyA_trans.getOrigin();
    btVector3 tmp = c.cross(p);
    for (i = 0; i < 3; ++i)
    {
     info->m_J1angularAxis[s2+i] = factA*tmp[i];
     info->m_J2angularAxis[s2+i] = factB*tmp[i];
    }
    tmp = c.cross(q);
    for (i = 0; i < 3; ++i)
    {
      info->m_J1angularAxis[s3+i] = factA*tmp[i];
      info->m_J2angularAxis[s3+i] = factB*tmp[i];
      info->m_J1linearAxis[s2+i] = p[i];
      info->m_J1linearAxis[s3+i] = q[i];
      info->m_J2linearAxis[s2+i] = -p[i];
      info->m_J2linearAxis[s3+i] = -q[i];
    }
  }
  // compute two elements of right hand side

  //  k = info->fps * info->erp * getSoftnessOrthoLin();
  currERP = (m_flags & BT_SLIDER_FLAGS_ERP_ORTLIN) ?
    m_softnessOrthoLin : m_softnessOrthoLin * info->erp;
  k = info->fps * currERP;

  btScalar rhs = k * p.dot(ofs);
  info->m_constraintError[s2] = rhs;
  rhs = k * q.dot(ofs);
  info->m_constraintError[s3] = rhs;
  if (m_flags & BT_SLIDER_FLAGS_CFM_ORTLIN)
  {
    info->cfm[s2] = m_cfmOrthoLin;
    info->cfm[s3] = m_cfmOrthoLin;
  }

  // Screw Constraint (coupled linear and angular motion)
  {
    nrow++;
    srow = nrow * info->rowskip;
    info->m_J1linearAxis[srow+0] = -ax1[0] * this->threadPitch;
    info->m_J1linearAxis[srow+1] = -ax1[1] * this->threadPitch;
    info->m_J1linearAxis[srow+2] = -ax1[2] * this->threadPitch;

    info->m_J1angularAxis[srow+0] = ax1[0];
    info->m_J1angularAxis[srow+1] = ax1[1];
    info->m_J1angularAxis[srow+2] = ax1[2];

    info->m_J2linearAxis[srow+0] = ax1[0] * this->threadPitch;
    info->m_J2linearAxis[srow+1] = ax1[1] * this->threadPitch;
    info->m_J2linearAxis[srow+2] = ax1[2] * this->threadPitch;

    info->m_J2angularAxis[srow+0] = -ax1[0];
    info->m_J2angularAxis[srow+1] = -ax1[1];
    info->m_J2angularAxis[srow+2] = -ax1[2];

    // correction
    // rhs = k * ax1.dot(ofs);  // from hinge constraint
    btScalar lin_disp = ax1.dot(ofs);
    btScalar ang_pos = this->getAngularPosition();
    info->m_constraintError[srow] =
      -k * (lin_disp * this->threadPitch - ang_pos);
    info->cfm[srow] = -m_cfmOrthoLin;

    // debug, set cfm to 0
    // info->cfm[srow] = 0;
    // debug, set error correction to 0
    // info->m_constraintError[srow] = 0.0;
  }

  // notes: below enforces
  // joint limit
  // powered joint
  // rotation along slider axis

  // check linear limits
  limit_err = btScalar(0.0);
  limit = 0;
  if (getSolveLinLimit())
  {
    limit_err = getLinDepth() *  signFact;
    limit = (limit_err > btScalar(0.0)) ? 2 : 1;
  }
  powered = 0;
  if (getPoweredLinMotor())
  {
    powered = 1;
  }
  // if the slider has joint limits or motor, add in the extra row
  if (limit || powered)
  {
    nrow++;
    srow = nrow * info->rowskip;
    info->m_J1linearAxis[srow+0] = ax1[0];
    info->m_J1linearAxis[srow+1] = ax1[1];
    info->m_J1linearAxis[srow+2] = ax1[2];
    info->m_J2linearAxis[srow+0] = -ax1[0];
    info->m_J2linearAxis[srow+1] = -ax1[1];
    info->m_J2linearAxis[srow+2] = -ax1[2];
    // linear torque decoupling step:
    //
    // we have to be careful that the linear constraint forces (+/- ax1)
    // applied to the two bodies
    // do not create a torque couple. in other words, the points that the
    // constraint force is applied at must lie along the same ax1 axis.
    // a torque couple will result in limited slider-jointed free
    // bodies from gaining angular momentum.
    if (m_useOffsetForConstraintFrame)
    {
      // this is needed only when bodyA and bodyB are both dynamic.
      if (!hasStaticBody)
      {
        tmpA = relA.cross(ax1);
        tmpB = relB.cross(ax1);
        info->m_J1angularAxis[srow+0] = tmpA[0];
        info->m_J1angularAxis[srow+1] = tmpA[1];
        info->m_J1angularAxis[srow+2] = tmpA[2];
        info->m_J2angularAxis[srow+0] = -tmpB[0];
        info->m_J2angularAxis[srow+1] = -tmpB[1];
        info->m_J2angularAxis[srow+2] = -tmpB[2];
      }
    }
    else
    { // The old way. May be incorrect if bodies are not on the slider axis
      // Linear Torque Decoupling vector (a torque)
      btVector3 ltd;
      ltd = c.cross(ax1);
      info->m_J1angularAxis[srow+0] = factA*ltd[0];
      info->m_J1angularAxis[srow+1] = factA*ltd[1];
      info->m_J1angularAxis[srow+2] = factA*ltd[2];
      info->m_J2angularAxis[srow+0] = factB*ltd[0];
      info->m_J2angularAxis[srow+1] = factB*ltd[1];
      info->m_J2angularAxis[srow+2] = factB*ltd[2];
    }
    // right-hand part
    btScalar lostop = getLowerLinLimit();
    btScalar histop = getUpperLinLimit();

    // issue #1104:
    // if (limit && (lostop == histop)) raises warnings, using
    // a warning-less implementation.
    if (limit &&
      math::equal(lostop, histop,
      static_cast<btScalar>(std::numeric_limits<double>::epsilon())))
    {
      // the joint motor is ineffective
      powered = 0;
    }
    info->m_constraintError[srow] = 0.;
    info->m_lowerLimit[srow] = 0.;
    info->m_upperLimit[srow] = 0.;
    currERP = (m_flags & BT_SLIDER_FLAGS_ERP_LIMLIN) ?
      m_softnessLimLin : info->erp;
    if (powered)
    {
      if (m_flags & BT_SLIDER_FLAGS_CFM_DIRLIN)
      {
        info->cfm[srow] = m_cfmDirLin;
      }
      btScalar tag_vel = getTargetLinMotorVelocity();
      btScalar mot_fact = getMotorFactor(m_linPos, m_lowerLinLimit,
        m_upperLinLimit, tag_vel, info->fps * currERP);
      info->m_constraintError[srow] -=
        signFact * mot_fact * getTargetLinMotorVelocity();
      info->m_lowerLimit[srow] += -getMaxLinMotorForce() * info->fps;
      info->m_upperLimit[srow] += getMaxLinMotorForce() * info->fps;
    }
    if (limit)
    {
      k = info->fps * currERP;
      info->m_constraintError[srow] += k * limit_err;
      if (m_flags & BT_SLIDER_FLAGS_CFM_LIMLIN)
      {
        info->cfm[srow] = m_cfmLimLin;
      }
      // issue #1104:
      // if (lostop == histop) raises warnings, using
      // a warning-less implementation.
      if (math::equal(lostop, histop,
          static_cast<btScalar>(std::numeric_limits<double>::epsilon())))
      {
        // limited low and high simultaneously
        info->m_lowerLimit[srow] = -SIMD_INFINITY;
        info->m_upperLimit[srow] = SIMD_INFINITY;
      }
      else if (limit == 1)
      {
         // low limit
        info->m_lowerLimit[srow] = -SIMD_INFINITY;
        info->m_upperLimit[srow] = 0;
      }
      else
      { // high limit
        info->m_lowerLimit[srow] = 0;
        info->m_upperLimit[srow] = SIMD_INFINITY;
      }
      // bounce (we'll use slider parameter abs(1.0 - m_dampingLimLin)
      //   for that)
      btScalar bounce = btFabs(btScalar(1.0) - getDampingLimLin());
      if (bounce > btScalar(0.0))
      {
        btScalar vel = linVelA.dot(ax1);
        vel -= linVelB.dot(ax1);
        vel *= signFact;
        // only apply bounce if the velocity is incoming, and if the
        // resulting c[] exceeds what we already have.
        if (limit == 1)
        {  // low limit
          if (vel < 0)
          {
            btScalar newc = -bounce * vel;
            if (newc > info->m_constraintError[srow])
            {
              info->m_constraintError[srow] = newc;
            }
          }
        }
        else
        { // high limit - all those computations are reversed
          if (vel > 0)
          {
            btScalar newc = -bounce * vel;
            if (newc < info->m_constraintError[srow])
            {
              info->m_constraintError[srow] = newc;
            }
          }
        }
      }
      info->m_constraintError[srow] *= getSoftnessLimLin();
    }
    // line above is the end of if (limit)
  }
  // line above is the end of if linear limit

  // printf("tp: %f\n", this->threadPitch);

  // check angular limits
  limit_err = btScalar(0.0);
  limit = 0;
  if (getSolveAngLimit())
  {
    limit_err = getAngDepth();
    limit = (limit_err > btScalar(0.0)) ? 1 : 2;
  }
  // if the slider has joint limits, add in the extra row
  powered = 0;
  if (getPoweredAngMotor())
  {
    powered = 1;
  }
  if (limit || powered)
  {
    nrow++;
    srow = nrow * info->rowskip;
    info->m_J1angularAxis[srow+0] = ax1[0];
    info->m_J1angularAxis[srow+1] = ax1[1];
    info->m_J1angularAxis[srow+2] = ax1[2];

    info->m_J2angularAxis[srow+0] = -ax1[0];
    info->m_J2angularAxis[srow+1] = -ax1[1];
    info->m_J2angularAxis[srow+2] = -ax1[2];

    btScalar lostop = getLowerAngLimit();
    btScalar histop = getUpperAngLimit();
    // issue #1104:
    // if (limit && (lostop == histop)) raises warnings, using
    // a warning-less implementation.
    if (limit &&
      math::equal(lostop, histop,
      static_cast<btScalar>(std::numeric_limits<double>::epsilon())))
    {  // the joint motor is ineffective
      powered = 0;
    }
    currERP = (m_flags & BT_SLIDER_FLAGS_ERP_LIMANG) ?
      m_softnessLimAng : info->erp;
    if (powered)
    {
      if (m_flags & BT_SLIDER_FLAGS_CFM_DIRANG)
      {
        info->cfm[srow] = m_cfmDirAng;
      }
      btScalar mot_fact = getMotorFactor(m_angPos, m_lowerAngLimit,
        m_upperAngLimit, getTargetAngMotorVelocity(), info->fps * currERP);
      info->m_constraintError[srow] = mot_fact * getTargetAngMotorVelocity();
      info->m_lowerLimit[srow] = -getMaxAngMotorForce() * info->fps;
      info->m_upperLimit[srow] = getMaxAngMotorForce() * info->fps;
    }
    if (limit)
    {
      k = info->fps * currERP;
      info->m_constraintError[srow] += k * limit_err;
      if (m_flags & BT_SLIDER_FLAGS_CFM_LIMANG)
      {
        info->cfm[srow] = m_cfmLimAng;
      }
      // issue #1104:
      // if (lostop == histop) raises warnings, using
      // a warning-less implementation.
      if (math::equal(lostop, histop,
          static_cast<btScalar>(std::numeric_limits<double>::epsilon())))
      {
        // limited low and high simultaneously
        info->m_lowerLimit[srow] = -SIMD_INFINITY;
        info->m_upperLimit[srow] = SIMD_INFINITY;
      }
      else if (limit == 1)
      { // low limit
        info->m_lowerLimit[srow] = 0;
        info->m_upperLimit[srow] = SIMD_INFINITY;
      }
      else
      { // high limit
        info->m_lowerLimit[srow] = -SIMD_INFINITY;
        info->m_upperLimit[srow] = 0;
      }
      // bounce (we'll use slider parameter abs(1.0 - m_dampingLimAng)
      // for that)
      btScalar bounce = btFabs(btScalar(1.0) - getDampingLimAng());
      if (bounce > btScalar(0.0))
      {
        btScalar vel = m_rbA.getAngularVelocity().dot(ax1);
        vel -= m_rbB.getAngularVelocity().dot(ax1);
        // only apply bounce if the velocity is incoming, and if the
        // resulting c[] exceeds what we already have.
        if (limit == 1)
        {  // low limit
          if (vel < 0)
          {
            btScalar newc = -bounce * vel;
            if (newc > info->m_constraintError[srow])
            {
              info->m_constraintError[srow] = newc;
            }
          }
        }
        else
        {  // high limit - all those computations are reversed
          if (vel > 0)
          {
            btScalar newc = -bounce * vel;
            if (newc < info->m_constraintError[srow])
            {
              info->m_constraintError[srow] = newc;
            }
          }
        }
      }
      info->m_constraintError[srow] *= getSoftnessLimAng();
    }
    // line above is the end of if (limit)
  }
  // line above is the end of if angular limit or powered
}

//////////////////////////////////////////////////
void btScrewConstraint::_getInfo1NonVirtual(btConstraintInfo1* info)
{
  /// this is a modified version of
  /// void btSliderConstraint::getInfo1(btConstraintInfo1* info)
  /// with the rotational limits always turned on.
  /// The rotational limit constraint is modified to be a screw constraint
  /// in btScrewConstraint::_getInfo2NonVirtual
  /// which is a copy of btSliderConstraint::getInfo2NonVirtual.

  // info->m_numConstraintRows = 6;
  // Fixed 2 linear + 2 angular + 1 limit (even if not used)
  // info->nub = 0;

  if (m_useSolveConstraintObsolete)
  {
    info->m_numConstraintRows = 0;
    info->nub = 0;
  }
  else
  {
    // Fixed 2 linear + 2 angular
    info->m_numConstraintRows = 4;
    info->nub = 2;

    // Add constraint for screw motion, coupling linear and angular motion.
    info->m_numConstraintRows++;
    info->nub++;

    // prepare constraint
    calculateTransforms(
      m_rbA.getCenterOfMassTransform(), m_rbB.getCenterOfMassTransform());
    testLinLimits();
    if (getSolveLinLimit() || getPoweredLinMotor())
    {
      // limit 3rd linear as well
      info->m_numConstraintRows++;
      info->nub--;
    }
    testAngLimits();
    if (getSolveAngLimit() || getPoweredAngMotor())
    {
      // limit 3rd angular as well
      info->m_numConstraintRows++;
      info->nub--;
    }
  }
  // printf("m: %d\n", info->m_numConstraintRows);
}
