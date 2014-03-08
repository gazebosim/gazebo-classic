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
#ifndef _GAZEBO_BULLET_UNIVERSAL_CONSTRAINT_
#define _GAZEBO_BULLET_UNIVERSAL_CONSTRAINT_

#include "gazebo/physics/bullet/bullet_inc.h"

ATTRIBUTE_ALIGNED16(class) gzBtUniversalConstraint :
public btGeneric6DofConstraint
{
  public: BT_DECLARE_ALIGNED_ALLOCATOR();
  public: gzBtUniversalConstraint(
              btRigidBody& rbA,
              btRigidBody& rbB,
              const btVector3& anchor,
              const btVector3& axis1,
              const btVector3& axis2);

  public: gzBtUniversalConstraint(
              btRigidBody &_rbA,
              const btVector3 &_anchor,
              const btVector3 &_axis1,
              const btVector3 &_axis2);

  const btVector3& getAnchor() { return m_calculatedTransformA.getOrigin(); }
  const btVector3& getAnchor2() { return m_calculatedTransformB.getOrigin(); }
  const btVector3& getAxis1() { return m_axis1; }
  const btVector3& getAxis2() { return m_axis2; }
  btScalar getAngle1() { return getAngle(2); }
  btScalar getAngle2() { return getAngle(1); }

  // Set upper limit.
  public: void setUpperLimit(btScalar ang1max, btScalar ang2max)
          {
            this->setAngularUpperLimit(btVector3(0.f, ang1max, ang2max));
          }

  // Set lower limit.
  public: void setLowerLimit(btScalar ang1min, btScalar ang2min)
          {
            this->setAngularLowerLimit(btVector3(0.f, ang1min, ang2min));
          }

  void setAxis( const btVector3& axis1, const btVector3& axis2);

  public: inline btScalar getMaxMotorImpulse1() const
          {
            return m_maxMotorImpulse[0];
          }

  public: inline btScalar getMaxMotorImpulse2() const
          {
            return m_maxMotorImpulse[1];
          }

  public: inline void setMaxMotorImpulse1(btScalar _i)
          {
            m_maxMotorImpulse[0] = _i;
          }

  public: inline void setMaxMotorImpulse2(btScalar _i)
          {
            m_maxMotorImpulse[1] = _i;
          }

  protected: btVector3 m_anchor;
  protected: btVector3 m_axis1;
  protected: btVector3 m_axis2;

  private: btScalar m_maxMotorImpulse[2];
};

#endif
