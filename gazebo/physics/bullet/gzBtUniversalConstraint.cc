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
#include <iostream>
#include "gazebo/physics/bullet/gzBtUniversalConstraint.hh"

#define UNIV_EPS btScalar(0.01f)

/////////////////////////////////////////////////
gzBtUniversalConstraint::gzBtUniversalConstraint(btRigidBody &_rbA,
    btRigidBody &_rbB, const btVector3 &_anchor, const btVector3 &_axis1,
    const btVector3 &_axis2)
: btGeneric6DofConstraint(_rbA, _rbB, btTransform::getIdentity(),
    btTransform::getIdentity(), true),
  m_anchor(_anchor),
  m_axis1(_axis1),
  m_axis2(_axis2)
{
  this->maxMotorImpulse[0] = 0;
  this->maxMotorImpulse[1] = 0;

  // build frame basis
  // 6DOF constraint uses Euler angles and to define limits
  // it is assumed that rotational order is :
  // Z - first, allowed limits are (-PI,PI);
  // new position of Y - second (allowed limits are
  // (-PI/2 + epsilon, PI/2 - epsilon), where epsilon is a small positive number
  // used to prevent constraint from instability on poles;
  // new position of X, allowed limits are (-PI,PI);
  // So to simulate ODE Universal joint we should use parent
  // axis as Z, child axis as Y and limit all other DOFs
  // Build the frame in world coordinate system first
  btVector3 zAxis = m_axis1.normalize();
  btVector3 yAxis = m_axis2.normalize();

  // we want right coordinate system
  btVector3 xAxis = yAxis.cross(zAxis);
  btTransform frameInW;
  frameInW.setIdentity();
  frameInW.getBasis().setValue(xAxis[0], yAxis[0], zAxis[0],
      xAxis[1], yAxis[1], zAxis[1],
      xAxis[2], yAxis[2], zAxis[2]);
  frameInW.setOrigin(_anchor);

  // now get constraint frame in local coordinate systems
  m_frameInA = _rbA.getCenterOfMassTransform().inverse() * frameInW;
  m_frameInB = _rbB.getCenterOfMassTransform().inverse() * frameInW;

  // set limits
  setLinearLowerLimit(btVector3(0., 0., 0.));
  setLinearUpperLimit(btVector3(0., 0., 0.));
  this->setAngularLowerLimit(btVector3(0.f,
        -SIMD_HALF_PI + UNIV_EPS, -SIMD_PI + UNIV_EPS));
  this->setAngularUpperLimit(btVector3(0.f,
        SIMD_HALF_PI - UNIV_EPS,  SIMD_PI - UNIV_EPS));
}

/////////////////////////////////////////////////
gzBtUniversalConstraint::gzBtUniversalConstraint(btRigidBody &_rbB,
    const btVector3 &_anchor, const btVector3 &_axis1, const btVector3 &_axis2)
: btGeneric6DofConstraint(_rbB, btTransform::getIdentity(), true),
  m_anchor(_anchor),
  m_axis1(_axis1),
  m_axis2(_axis2)
{
  this->maxMotorImpulse[0] = 0;
  this->maxMotorImpulse[1] = 0;

  // build frame basis
  // 6DOF constraint uses Euler angles and to define limits
  // it is assumed that rotational order is :
  // Z - first, allowed limits are (-PI,PI);
  // new position of Y - second (allowed limits are
  // (-PI/2 + epsilon, PI/2 - epsilon),
  // where epsilon is a small positive number
  // used to prevent constraint from instability on poles;
  // new position of X, allowed limits are (-PI,PI);
  // So to simulate ODE Universal joint we should use parent axis as Z,
  // child axis as Y and limit all other DOFs
  // Build the frame in world coordinate system first
  btVector3 zAxis = m_axis1.normalize();
  btVector3 yAxis = m_axis2.normalize();

  // we want right coordinate system
  btVector3 xAxis = yAxis.cross(zAxis);

  btTransform frameInW;
  frameInW.setIdentity();
  frameInW.getBasis().setValue(xAxis[0], yAxis[0], zAxis[0],
                               xAxis[1], yAxis[1], zAxis[1],
                               xAxis[2], yAxis[2], zAxis[2]);
  frameInW.setOrigin(_anchor);

  // now get constraint frame in local coordinate systems
  m_frameInA = frameInW;
  m_frameInB = _rbB.getCenterOfMassTransform().inverse() * frameInW;

  // set limits
  setLinearLowerLimit(btVector3(0., 0., 0.));
  setLinearUpperLimit(btVector3(0., 0., 0.));
  setAngularLowerLimit(btVector3(0.f,
        -SIMD_HALF_PI + UNIV_EPS, -SIMD_PI + UNIV_EPS));
  setAngularUpperLimit(btVector3(0.f,
        SIMD_HALF_PI - UNIV_EPS,  SIMD_PI - UNIV_EPS));
}

/////////////////////////////////////////////////
void gzBtUniversalConstraint::setAxis(const btVector3 &_axis1,
    const btVector3 &_axis2)
{
  m_axis1 = _axis1;
  m_axis2 = _axis2;

  btVector3 zAxis = _axis1.normalized();
  btVector3 yAxis = _axis2.normalized();

  // we want right coordinate system
  btVector3 xAxis = yAxis.cross(zAxis);

  btTransform frameInW;
  frameInW.setIdentity();
  frameInW.getBasis().setValue(xAxis[0], yAxis[0], zAxis[0],
                               xAxis[1], yAxis[1], zAxis[1],
                               xAxis[2], yAxis[2], zAxis[2]);
  frameInW.setOrigin(m_anchor);

  // now get constraint frame in local coordinate systems
  m_frameInA = m_rbA.getCenterOfMassTransform().inverse() * frameInW;
  m_frameInB = m_rbB.getCenterOfMassTransform().inverse() * frameInW;

  this->calculateTransforms();
}

/////////////////////////////////////////////////
const btVector3 &gzBtUniversalConstraint::getAnchor()
{
  return m_calculatedTransformA.getOrigin();
}

/////////////////////////////////////////////////
const btVector3 &gzBtUniversalConstraint::getAnchor2()
{
  return m_calculatedTransformB.getOrigin();
}

/////////////////////////////////////////////////
const btVector3 &gzBtUniversalConstraint::getAxis1()
{
  return m_axis1;
}

/////////////////////////////////////////////////
const btVector3 &gzBtUniversalConstraint::getAxis2()
{
  return m_axis2;
}

/////////////////////////////////////////////////
btScalar gzBtUniversalConstraint::getAngle1()
{
  this->calculateTransforms();
  return this->getAngle(1);
}

/////////////////////////////////////////////////
btScalar gzBtUniversalConstraint::getAngle2()
{
  this->calculateTransforms();
  return this->getAngle(2);
}

/////////////////////////////////////////////////
void gzBtUniversalConstraint::getUpperLimit(
    btScalar &_ang1max, btScalar &_ang2max)
{
  btVector3 angles;
  this->getAngularUpperLimit(angles);
  _ang1max = angles.getY();
  _ang2max = angles.getZ();
}

/////////////////////////////////////////////////
void gzBtUniversalConstraint::getLowerLimit(
    btScalar &_ang1min, btScalar &_ang2min)
{
  btVector3 angles;
  this->getAngularLowerLimit(angles);
  _ang1min = angles.getY();
  _ang2min = angles.getZ();
}

/////////////////////////////////////////////////
void gzBtUniversalConstraint::setUpperLimit(
    btScalar _ang1max, btScalar _ang2max)
{
  this->setAngularUpperLimit(btVector3(0.f, _ang1max, _ang2max));
}

/////////////////////////////////////////////////
void gzBtUniversalConstraint::setLowerLimit(
    btScalar _ang1min, btScalar _ang2min)
{
  this->setAngularLowerLimit(btVector3(0.f, _ang1min, _ang2min));
}

/////////////////////////////////////////////////
btScalar gzBtUniversalConstraint::getMaxMotorImpulse1() const
{
  return this->maxMotorImpulse[0];
}

/////////////////////////////////////////////////
btScalar gzBtUniversalConstraint::getMaxMotorImpulse2() const
{
  return this->maxMotorImpulse[1];
}

/////////////////////////////////////////////////
void gzBtUniversalConstraint::setMaxMotorImpulse1(btScalar _i)
{
  this->maxMotorImpulse[0] = _i;
}

/////////////////////////////////////////////////
void gzBtUniversalConstraint::setMaxMotorImpulse2(btScalar _i)
{
  this->maxMotorImpulse[1] = _i;
}
