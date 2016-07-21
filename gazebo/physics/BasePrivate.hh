/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef GAZEBO_PHYSICS_BASE_PRIVATE_HH_
#define GAZEBO_PHYSICS_BASE_PRIVATE_HH_
#include <string>

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/physics/PhysicsTypes.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Base private data.
    class BasePrivate
    {
      /// \brief Compute the scoped name of this object based on its
      /// parents.
      /// \sa Base::GetScopedName
      public: void ComputeScopedName();

      /// \brief The SDF values for this object.
      public: sdf::ElementPtr sdf;

      /// \brief Parent of this entity.
      public: BasePtr parent;

      /// \brief Children of this entity.
      public: Base_V children;

      /// \brief Pointer to the world.
      public: WorldPtr world;

      /// \brief Set to true if the object should be saved.
      public: bool saveable;

      /// \brief This entities ID.
      public: uint32_t id;

      /// \brief The type of this object.
      public: unsigned int type;

      /// \brief The string representation of the type of this object.
      public: std::string typeStr;

      /// \brief True if selected.
      public: bool selected;

      /// \brief Local copy of the sdf name.
      public: std::string name;

      /// \brief Local copy of the scoped name.
      public: std::string scopedName;

      /// \brief The string representation of the type of this object.
      public: std::string typeStr;
    };
  }
}
#endif
