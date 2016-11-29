/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_DARTPLANESHAPE_PRIVATE_HH_
#define _GAZEBO_DARTPLANESHAPE_PRIVATE_HH_

#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTTypes.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data class for DARTPlaneShape
    class DARTPlaneShapePrivate
    {
      /// \brief Constructor
      public: DARTPlaneShapePrivate()
        : dtBoxShape(new dart::dynamics::BoxShape(
                       Eigen::Vector3d(2100, 2100, 2100)))
      {
        this->dtBoxShape->setOffset(Eigen::Vector3d(0.0, 0.0, -2100*0.5));
      }

      /// \brief Default destructor
      public: ~DARTPlaneShapePrivate() = default;

      /// \brief DART box shape
      public: std::shared_ptr<dart::dynamics::BoxShape> dtBoxShape;
      // We use BoxShape untile PlaneShape is completely supported in DART.
      // Please see: https://github.com/dartsim/dart/issues/114
    };
  }
}
#endif
