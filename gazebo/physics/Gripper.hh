/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifndef GAZEBO_PHYSICS_GRIPPER_HH_
#define GAZEBO_PHYSICS_GRIPPER_HH_

#include <string>

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    // Forward declare private data class
    class GripperPrivate;

    /// \addtogroup gazebo_physics
    /// \{

    /// \class Gripper Gripper.hh physics/physics.hh
    /// \brief A gripper abstraction
    ///
    /// A gripper is a collection of links that act as a gripper. This class
    /// will intelligently generate fixed joints between the gripper and an
    /// object within the gripper. This allows the object to be manipulated
    /// without falling or behaving poorly.
    class GZ_PHYSICS_VISIBLE Gripper
    {
      /// \brief Constructor
      /// \param[in] _model The model which contains the Gripper.
      public: explicit Gripper(ModelPtr _model);

      /// \brief Destructor.
      public: virtual ~Gripper();

      /// \brief Load the gripper.
      /// \param[in] _sdf Shared point to an sdf element that contains the list
      /// of links in the gripper.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize.
      public: virtual void Init();

      /// \brief Return the name of the gripper.
      /// \return Name of the gripper
      public: std::string Name() const;

      /// \brief True if the gripper is attached to another model.
      /// \return True if the gripper is active and a joint has been
      /// created between the gripper and another model.
      public: bool IsAttached() const;

      /// \internal
      /// \brief Private data pointer
      private: std::unique_ptr<GripperPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
