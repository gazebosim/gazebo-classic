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

#ifndef _GAZEBO_DARTMODEL_PRIVATE_HH_
#define _GAZEBO_DARTMODEL_PRIVATE_HH_

#include "gazebo/physics/dart/dart_inc.h"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data class for DARTModel
    class DARTModelPrivate
    {
      /// \brief Constructor
      public: DARTModelPrivate()
        : dtSkeleton(NULL),
          dtConfig(),
          dtVelocity()
      {
      }

      /// \brief Default destructor
      public: ~DARTModelPrivate()
      {
        // We don't need to delete dtSkeleton because world will delete
        // dtSkeleton if it is registered to the world.
      }

      /// \brief Pointer to DART Skeleton
      public: dart::dynamics::Skeleton *dtSkeleton;

      /// \brief Generalized positions
      public: Eigen::VectorXd dtConfig;

      /// \brief Generalized velocities
      public: Eigen::VectorXd dtVelocity;
    };
  }
}
#endif
