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
#ifndef _GAZEBO_BULLET_UNIVERSAL_CONSTRAINT_
#define _GAZEBO_BULLET_UNIVERSAL_CONSTRAINT_

#include "gazebo/physics/bullet/bullet_inc.h"

ATTRIBUTE_ALIGNED16(class) gzBtUniversalConstraint
: public btGeneric6DofConstraint
{
  public: BT_DECLARE_ALIGNED_ALLOCATOR();

  /// \brief Constructor
  /// \param[in] _rbA Rigid body A, also known as the parent link.
  /// \param[in] _rbB Rigid body B, also known as the child link.
  /// \param[in] _anchor The point in world space that constrains the two
  /// rigid bodies.
  /// \param[in] _axis1 First axis of rotation in world frame.
  /// \param[in] _axis2 Second axis of rotation in world frame.
  public: gzBtUniversalConstraint(
              btRigidBody &_rbA,
              btRigidBody &_rbB,
              const btVector3 &_anchor,
              const btVector3 &_axis1,
              const btVector3 &_axis2);

  /// \brief Constructor
  /// \param[in] _rbB Rigid body B, also known as the child link.
  /// \param[in] _anchor The point in world space that constrains the child
  /// link to the world.
  /// \param[in] _axis1 First axis of rotation in world frame.
  /// \param[in] _axis2 Second axis of rotation in world frame.
  public: gzBtUniversalConstraint(
              btRigidBody &_rbA,
              const btVector3 &_anchor,
              const btVector3 &_axis1,
              const btVector3 &_axis2);

  /// \brief Get the anchor point in link A reference frame.
  /// \return Anchor point in link A's reference frame.
  public: const btVector3 &getAnchor();

  /// \brief Get the anchor point in link B reference frame.
  /// \return Anchor point in link B's reference frame.
  public: const btVector3 &getAnchor2();

  /// \brief Get the first axis of rotation.
  /// \return The first axis of rotation as a vector
  public: const btVector3 &getAxis1();

  /// \brief Get the second axis of rotation.
  /// \return The second axis of rotation as a vector
  public: const btVector3 &getAxis2();

  /// \brief Get the value of angle 1 in radians.
  /// \return Angle value of the first axis in radians.
  public: btScalar getAngle1();

  /// \brief Get the value of angle 2 in radians.
  /// \return Angle value of the second axis in radians.
  public: btScalar getAngle2();

  /// \brief Get upper limits.
  /// \param[out] _ang1max Maximum value for angle 1
  /// \param[out] _ang2max Maximum value for angle 2
  public: void getUpperLimit(btScalar &_ang1max, btScalar &_ang2max);

  /// \brief Get lower limits.
  /// \param[out] _ang1min Minimum value for angle 1
  /// \param[out] _ang2min Minimum value for angle 2
  public: void getLowerLimit(btScalar &_ang1min, btScalar &_ang2min);

  /// \brief Set upper limits.
  /// \param[in] _ang1max Maximum value for angle 1
  /// \param[in] _ang2max Maximum value for angle 2
  public: void setUpperLimit(btScalar _ang1max, btScalar _ang2max);

  /// \brief Set lower limits.
  /// \param[in] _ang1min Minimum value for angle 1
  /// \param[in] _ang2min Minimum value for angle 2
  public: void setLowerLimit(btScalar _ang1min, btScalar _ang2min);

  /// \brief Set the axis of rotation.
  /// \param[in] _axis1 First axis of rotation.
  /// \param[in] _axis2 Second axis of rotation.
  public: void setAxis(const btVector3 &_axis1, const btVector3 &_axis2);

  /// \brief Get the maximum allowed motor impluse for the first axis of
  /// rotation.
  /// \return Maximum motor impulse for axis 1.
  public: btScalar getMaxMotorImpulse1() const;

  /// \brief Get the maximum allowed motor impluse for the second axis of
  /// rotation.
  /// \return Maximum motor impulse for axis 2.
  public: btScalar getMaxMotorImpulse2() const;

  /// \brief Set the maximum allowed motor impluse for the first axis of
  /// rotation.
  /// \param[in] _i Maximum motor impulse for axis 1.
  public: void setMaxMotorImpulse1(btScalar _i);

  /// \brief Set the maximum allowed motor impluse for the second axis of
  /// rotation.
  /// \param[in] _i Maximum motor impulse for axis 2.
  public: void setMaxMotorImpulse2(btScalar _i);

  /// \brief Anchor point in world coordinate frame.
  protected: btVector3 m_anchor;

  /// \brief First axis of rotation.
  protected: btVector3 m_axis1;

  /// \brief Second axis of rotation.
  protected: btVector3 m_axis2;

  /// \brief Maximum motor impulses.
  private: btScalar maxMotorImpulse[2];
};

#endif
