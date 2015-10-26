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
#ifndef _GAZEBO_ENTITYMAKER_PRIVATE_HH_
#define _GAZEBO_ENTITYMAKER_PRIVATE_HH_

#include <ignition/math/Vector3.hh>

#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/gui/EntityMaker.hh"

namespace gazebo
{
  namespace gui
  {
    /// \brief Private data for the EntityMaker class
    class EntityMakerPrivate
    {
      /// \brief Node to publish messages and spawn the entity. Each derived
      /// class must create its own publisher to the appropriate topic.
      public: transport::NodePtr node;
    };
  }
}
#endif

