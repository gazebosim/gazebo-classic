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
#ifndef _GAZEBO_PHYSICS_BASE_PRIVATE_HH_
#define _GAZEBO_PHYSICS_BASE_PRIVATE_HH_

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/physics/PhysicsTypes.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Base protected data.
    class BaseProtected
    {
      /// \brief Compute the scoped name of this object based on its
      /// parents.
      /// \sa Base::GetScopedName
      protected: void ComputeScopedName();

      /// \brief The SDF values for this object.
      protected: sdf::ElementPtr sdf;

      /// \brief Parent of this entity.
      protected: BasePtr parent;

      /// \brief Children of this entity.
      protected: Base_V children;

      /// \brief Pointer to the world.
      protected: WorldPtr world;
    };

    /// \internal
    /// \brief Base private data.
    class BasePrivate : public BaseProtected
    {
      /// \brief Set to true if the object should be saved.
      private: bool saveable;

      /// \brief This entities ID.
      private: uint32_t id;

      /// \brief The type of this object.
      private: unsigned int type;

      /// \brief True if selected.
      private: bool selected;

      /// \brief Local copy of the sdf name.
      private: std::string name;

      /// \brief Local copy of the scoped name.
      private: std::string scopedName;
    };
  }
}
#endif
