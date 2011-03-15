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
/* Desc: A body that has a box shape
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef ODEHINGEJOINT_HH
#define ODEHINGEJOINT_HH

#include "common/Angle.hh"
#include "common/Vector3.hh"

#include "physics/HingeJoint.hh"
#include "physics/ode/ODEJoint.hh"

namespace gazebo
{
	namespace physics
  {
    /// \addtogroup gazebo_physics_joints
    /// \{
    /** \defgroup gazebo_hinge_joint Hinge Joint
      
      \brief A single-axis hinge joint in ODE.
    
      \par Attributes
      - body1 (string)
        - Name of the first body to attach to the joint
      - body2 (string)
        - Name of the second body to attach to the joint
      - anchor (string)
        - Name of the body which will act as the anchor to the joint
      - axis (float, tuple)
        - Defines the axis of rotation for the first degree of freedom
        - Default: 0 0 1
      - lowStop (float, degrees)
        - The low stop angle for the first degree of freedom
        - Default: infinity
      - highStop (float, degrees)
        - The high stop angle for the first degree of freedom
        - Default: infinity
      - erp (double)
        - Error reduction parameter. 
        - Default = 0.4
      - cfm (double)
        - Constraint force mixing. 
        - Default = 0.8
    
      \par Example
      \verbatim
      <joint:hinge name="hinge_joint>
        <body1>body1_name</body1>
        <body2>body2_name</body2>
        <anchor>anchor_body</anchor>
        <axis>0 0 1</axis>
        <lowStop>0</lowStop>
        <highStop>30</highStop>
      </joint:hinge>
      \endverbatim
    */
    /// \}
    
    /// \addtogroup gazebo_hinge_joint
    /// \{
    
    ///\brief A single axis hinge joint
    class ODEHingeJoint : public HingeJoint<ODEJoint>
    {
      ///  Constructor
      public: ODEHingeJoint(dWorldID worldId);
    
      /// Destructor
      public: virtual ~ODEHingeJoint();
    
      /// \brief Load joint
      protected: virtual void Load(common::XMLConfigNode *node);
  
      /// Get the anchor point
      public: virtual common::Vector3 GetAnchor(int index) const;
  
      /// Set the anchor point
      public: virtual void SetAnchor(int index, const common::Vector3 &anchor);
  
      /// Get the axis of rotation
      public: virtual common::Vector3 GetAxis(int index) const;
  
      /// Set the axis of rotation
      public: virtual void SetAxis(int index, const common::Vector3 &axis);
   
      /// \brief Set the joint damping
      public: virtual void SetDamping( int index, const double damping );
  
      /// \brief callback to apply damping force to joint
      public: void ApplyDamping();
  
      /// Get the angle of rotation
      public: virtual common::Angle GetAngle(int index) const;
   
      /// \brief Set the velocity of an axis(index).
      public: virtual void SetVelocity(int index, double angle);
  
      /// \brief Get the rotation rate of an axis(index)
      public: virtual double GetVelocity(int index) const;
   
      /// \brief Set the max allowed force of an axis(index).
      public: virtual void SetMaxForce(int index, double t);
  
      /// \brief Get the max allowed force of an axis(index).
      public: virtual double GetMaxForce(int index);
  
      /// \brief Set the torque of a joint.
      public: virtual void SetForce(int index, double torque);
  
      /// Get the specified parameter
      public: virtual double GetParam( int parameter ) const;
    
      /// Set the parameter to value
      public: virtual void SetParam( int parameter, double value);
  
      private: event::ConnectionPtr jointUpdateConnection;
    };
  
    /// \}
    
  }
}
#endif

