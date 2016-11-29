/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_DARTMODEL_HH_
#define _GAZEBO_DARTMODEL_HH_

#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTTypes.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// Forward declare private data class
    class DARTModelPrivate;

    /// \ingroup gazebo_physics
    /// \addtogroup gazebo_physics_dart DART Physics
    /// \brief dart physics engine wrapper
    /// \{

    /// \class DARTModel
    /// \brief DART model class
    class GZ_PHYSICS_VISIBLE DARTModel : public Model
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent object.
      public: explicit DARTModel(BasePtr _parent);

      /// \brief Destructor.
      public: virtual ~DARTModel();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Init();

      // Documentation inherited.
      public: virtual void Update();

      // Documentation inherited.
      public: virtual void Fini();

      /// \brief
      public: void BackupState();

      /// \brief
      public: void RestoreState();

      /// \brief Get pointer to DART Skeleton.
      /// \return The pointer to DART Skeleton.
      /// \deprecated See dart::dynamics::SkeletonPtr DARTSkeleton()
      public: dart::dynamics::Skeleton *GetDARTSkeleton()
              GAZEBO_DEPRECATED(8.0);

      /// \brief Get pointer to DART Skeleton.
      /// \return The pointer to DART Skeleton.
      public: dart::dynamics::SkeletonPtr DARTSkeleton();

      /// \brief Get pointer to DART Physics.
      /// \return The pointer to DART Physics.
      public: DARTPhysicsPtr GetDARTPhysics(void) const;

      /// \brief Get pointer to DART World
      /// \return The pointer to DART World.
      /// \deprecated See dart::simulation::WorldPtr DARTWorld(void) const
      public: dart::simulation::World *GetDARTWorld(void) const
              GAZEBO_DEPRECATED(8.0);

      /// \brief Get pointer to DART World
      /// \return The pointer to DART World.
      public: dart::simulation::WorldPtr DARTWorld(void) const;

      /// \internal
      /// \brief Pointer to private data
      private: DARTModelPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
