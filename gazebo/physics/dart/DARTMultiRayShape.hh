/*
 * Copyright 2012 Open Source Robotics Foundation
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

#ifndef _DARTMULTIRAYSHAPE_HH_
#define _DARTMULTIRAYSHAPE_HH_

#include "gazebo/physics/MultiRayShape.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief DART specific version of MultiRayShape
    class DARTMultiRayShape : public MultiRayShape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent Collision.
      public: explicit DARTMultiRayShape(CollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~DARTMultiRayShape();

      // Documentation inherited.
      public: virtual void UpdateRays();
    };
  }
}
#endif
