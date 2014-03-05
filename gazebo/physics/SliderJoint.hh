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
/* Desc: A slider or primastic joint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#ifndef _SLIDERJOINT_HH_
#define _SLIDERJOINT_HH_

#include "gazebo/physics/Joint.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class SliderJoint SliderJoint.hh physics/physics.hh
    /// \brief A slider joint
    template<class T>
    class SliderJoint : public T
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent of the joint.
      public: explicit SliderJoint(BasePtr _parent) : T(_parent)
              {this->AddType(Base::SLIDER_JOINT);}

      /// \brief Destructor.
      public: virtual ~SliderJoint()
              {}

      /// \brief Load a SliderJoint.
      /// \param[in] _sdf SDF values to load from
      public: virtual void Load(sdf::ElementPtr _sdf)
              {T::Load(_sdf);}

      /// \interal
      public: virtual unsigned int GetAngleCount() const
              {return 1;}

      /// \brief Set the anchor.
      /// \param[in] _index Index of the axis. Not used.
      /// \param[in] _anchor Anchor for the axis.
      public: virtual void SetAnchor(int _index, const math::Vector3 &_anchor);

      /// \brief Get the anchor.
      /// \param[in] _index Index of the axis. Not used.
      /// \return Anchor for the joint.
      public: virtual math::Vector3 GetAnchor(int _index) const;

      /// \brief The anchor value is not used internally.
      protected: math::Vector3 fakeAnchor;
    };
    /// \}

    template<class T>
    void SliderJoint<T>::SetAnchor(int /*_index*/, const math::Vector3 &_anchor)
    {this->fakeAnchor = _anchor;}

    template<class T>
    math::Vector3 SliderJoint<T>::GetAnchor(int /*_index*/) const
    {return this->fakeAnchor;}
  }
}
#endif
