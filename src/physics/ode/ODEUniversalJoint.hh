/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: A universal joint
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
*/

#ifndef ODEUNIVERSALJOINT_HH
#define ODEUNIVERSALJOINT_HH

#include "physics/UniversalJoint.hh"
#include "physics/ode/ODEJoint.hh"

namespace gazebo
{
	namespace physics
  {
    /// \addtogroup gazebo_physics_joints
    /// \{
    /** \defgroup gazebo_universal_joint Universal Joint
     
      \brief A universal joint
    
      \par Attributes
      - body1 (string)
        - Name of the first body to attach to the joint
      - body2 (string)
        - Name of the second body to attach to the joint
      - anchor (string)
        - Name of the body which will act as the anchor to the joint
      - axis1 (float, tuple)
        - Defines the axis of rotation for the first degree of freedom
        - Default: 0 0 1
      - axis2 (float, tuple)
        - Defines the axis of rotation for the second degree of freedom
        - Default: 0 0 1
      - lowStop1 (float, degrees)
        - The low stop angle for the first degree of freedom
        - Default: infinity
      - highStop1 (float, degrees)
        - The high stop angle for the first degree of freedom
        - Default: infinity
      - lowStop2 (float, degrees)
        - The low stop angle for the second degree of freedom
        - Default: infinity
      - highStop2 (float, degrees)
        - The high stop angle for the second degree of freedom
        - Default: infinity
      - erp (double)
        - Error reduction parameter. 
        - Default = 0.4
      - cfm (double)
        - Constraint force mixing. 
        - Default = 0.8
    
    
      \par Example
      \verbatim
      <joint:universal name="universal_joint>
        <body1>body1_name</body1>
        <body2>body2_name</body2>
        <anchor>anchor_body</anchor>
        <axis1>0 0 1</axis1>
        <axis2>0 1 0</axis2>
        <lowStop1>0</lowStop1>
        <highStop1>30</highStop1>
        <lowStop2>0</lowStop2>
        <highStop2>30</highStop2>
      </joint:universal>
      \endverbatim
    */
    /// \}
    
    /// \defgroup gazebo_universal_joint Universal Joint
    /// \{
    
    /// \brief A universal joint
    class ODEUniversalJoint : public UniversalJoint<ODEJoint>
    {
      /// \brief Constructor
      public: ODEUniversalJoint(dWorldID worldId);
    
      /// \brief Destuctor
      public: virtual ~ODEUniversalJoint();
    
      /// \brief Get the anchor point
      public: virtual common::Vector3 GetAnchor(int index) const;
  
      /// \brief Set the anchor point
      public: virtual void SetAnchor( int index, const common::Vector3 &anchor );
   
      /// \brief Set joint damping, not yet implemented
      public: virtual void SetDamping(int index, const double damping);
  
      /// \brief Get the first axis of rotation
      public: virtual common::Vector3 GetAxis(int index ) const;
  
      /// \brief Set the first axis of rotation
      public: virtual void SetAxis( int index, const common::Vector3 &axis );
    
      /// \brief Get the angle of axis 
      public: virtual common::Angle GetAngle(int index) const;
  
      /// \brief Get the angular rate of an axis
      public: virtual double GetVelocity(int index) const;
   
      /// \brief Set the velocity of an axis(index).
      public: virtual void SetVelocity(int index, double angle);
  
      /// \brief Set the torque of a joint.
      public: virtual void SetForce(int index, double torque);
  
      /// \brief Set the max allowed force of an axis(index).
      public: virtual void SetMaxForce(int index, double t);
  
      /// \brief Get the max allowed force of an axis(index).
      public: virtual double GetMaxForce(int index);
  
  
      /// \brief Set the parameter to value
      public: virtual void SetParam( int parameter, double value );
    
    };
    
    /// \}
  }
}
#endif
