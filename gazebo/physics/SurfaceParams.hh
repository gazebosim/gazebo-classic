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

#ifndef _SURFACEPARAMS_HH_
#define _SURFACEPARAMS_HH_

#include <sdf/sdf.hh>

#include "gazebo/msgs/msgs.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class SurfaceParams SurfaceParams.hh physics/physics.hh
    /// \brief SurfaceParams defines various Surface contact parameters.
    /// These parameters defines the properties of a
    /// physics::Contact constraint.
    class SurfaceParams
    {
      /// \brief Constructor.
      public: SurfaceParams();

      /// \brief Destructor.
      public: virtual ~SurfaceParams();

      /// \brief Load the contact params.
      /// \param[in] _sdf SDF values to load from.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Fill in a surface message.
      /// \param[in] _msg Message to fill with this object's values.
      public: virtual void FillMsg(msgs::Surface &_msg);

      /// \brief Process a surface message.
      /// \param[in] _msg Message to read values from.
      public: virtual void ProcessMsg(const msgs::Surface &_msg);

      /// \brief Allow collision checking without generating a contact joint.
      public: bool collideWithoutContact;

      /// \brief Custom collision filtering used when collideWithoutContact is
      /// true.
      public: unsigned int collideWithoutContactBitmask;
    };
    /// \}
  }
}
#endif
