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

#ifndef _GAZEBO_DARTMESHSHAPE_PRIVATE_HH_
#define _GAZEBO_DARTMESHSHAPE_PRIVATE_HH_

#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTMesh.hh"
#include "gazebo/physics/dart/DARTTypes.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data class for DARTMeshShape
    class DARTMeshShapePrivate
    {
      /// \brief Constructor
      public: DARTMeshShapePrivate()
        : dartMesh(new DARTMesh())
      {
      }

      /// \brief Default destructor
      public: ~DARTMeshShapePrivate()
      {
        delete this->dartMesh;
      }

      /// \brief Disabled copy constructor
      public: DARTMeshShapePrivate(const DARTMeshShapePrivate&) = delete;

      /// \brief DART collision mesh helper class
      public: DARTMesh *dartMesh;
    };
  }
}
#endif
