/*
 * Copyright 2011 Nate Koenig
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
/* Desc: A slider or primastic joint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef SLIDERJOINT_HH
#define SLIDERJOINT_HH

#include <float.h>
#include "physics/Joint.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief A slider joint
    template<class T>
    class SliderJoint : public T
    {
      /// \brief Constructor
      public: SliderJoint(BasePtr _parent) : T(_parent)
              { this->AddType(Base::SLIDER_JOINT); }

      /// \brief Destructor
      public: virtual ~SliderJoint()
              {}

      /// \brief Load a SliderJoint
      protected: virtual void Load(sdf::ElementPtr _sdf)
                 { T::Load(_sdf); }

      /// \brief Set the anchor
      public: virtual void SetAnchor(int /*_index*/,
                                      const math::Vector3 &_anchor)
              {this->fakeAnchor = _anchor;}

      /// \brief Get the anchor
      public: virtual math::Vector3 GetAnchor(int /*_index*/) const
              {return this->fakeAnchor;}

      protected: math::Vector3 fakeAnchor;
    };
    /// \}
  }
}
#endif


