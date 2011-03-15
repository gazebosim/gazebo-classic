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
/* Desc: A ball joint
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id$
 */

#ifndef BALLJOINT_HH
#define BALLJOINT_HH

#include "Joint.hh"

namespace gazebo
{
	namespace physics
  {
  
    /// \addtogroup gazebo_physics_joints
    /// \{
    /** \defgroup gazebo_ball_joint Ball Joint
    
      \brief A ball joint
    
      \par Attributes
      - body1 (string)
        - Name of the first body to attach to the joint
      - body2 (string)
        - Name of the second body to attach to the joint
      - anchor (string)
        - Name of the body which will act as the anchor to the joint
      - erp (double)
        - Error reduction parameter. Default = 0.4
      - cfm (double)
        - Constraint force mixing. Default = 0.8
    
      \par Example
      \verbatim
      <joint:ball name="ball_joint>
        <body1>body1_name</body1>
        <body2>body2_name</body2>
        <anchor>anchor_body</anchor>
      </joint:ball>
      \endverbatim
    */
    /// \}
    /// \addtogroup gazebo_ball_joint
    /// \{
    
    /// \brief A ball joint
    template< class T>
    class BallJoint : public T
    {
      /// \brief Constructor
      public: BallJoint() : T()
              {
                this->AddType(BALL_JOINT);
              }
    
      /// \brief Destructor
      public: virtual ~BallJoint()
              {
              }
    
      /// \brief Load the joint
      protected: void Load(common::XMLConfigNode *node)
                 {
                   T::Load(node);
                 }
    
      /// \brief Save a joint to a stream in XML format
      protected: void SaveJoint(std::string &prefix, std::ostream &stream)
                 {
                   T::SaveJoint(prefix,stream);
                 }
  
      /// \brief Set the axis of rotation
      public: virtual void SetAxis(int index, const common::Vector3 &axis) {}
   
      /// \brief Set the high stop of an axis(index).
      public: virtual void SetHighStop(int index, common::Angle angle) {}
  
      /// \brief Set the low stop of an axis(index).
      public: virtual void SetLowStop(int index, common::Angle angle) {}
   
      /// \brief Get the high stop of an axis(index).
      public: virtual common::Angle GetHighStop(int index) {return common::Angle();}
  
      /// \brief Get the low stop of an axis(index).
      public: virtual common::Angle GetLowStop(int index) { return common::Angle();}
  
    };
    
    /// \}
  }

}
#endif
