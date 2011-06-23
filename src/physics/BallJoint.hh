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
 */

#ifndef BALLJOINT_HH
#define BALLJOINT_HH

#include "Joint.hh"

namespace gazebo
{
	namespace physics
  {
     /// \brief A ball joint
    template< class T>
    class BallJoint : public T
    {
      /// \brief Constructor
      public: BallJoint() : T()
              {
                this->AddType(Base::BALL_JOINT);
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
      public: virtual void SetAxis(int /*_index*/, 
                                   const math::Vector3 &/*_axis*/) 
              {}
   
      /// \brief Set the high stop of an axis(index).
      public: virtual void SetHighStop(int /*_index*/, 
                                       math::Angle /*_angle*/) {}
  
      /// \brief Set the low stop of an axis(index).
      public: virtual void SetLowStop(int /*_index*/, math::Angle /*_angle*/) {}
   
      /// \brief Get the high stop of an axis(index).
      public: virtual math::Angle GetHighStop(int /*_index*/) 
              {return math::Angle();}
  
      /// \brief Get the low stop of an axis(index).
      public: virtual math::Angle GetLowStop(int /*_index*/) 
              { return math::Angle();}
  
    };
  }
}
#endif
