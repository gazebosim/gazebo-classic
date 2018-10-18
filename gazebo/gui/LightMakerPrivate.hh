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

#ifndef GAZEBO_GUI_LIGHTMAKER_PRIVATE_HH_
#define GAZEBO_GUI_LIGHTMAKER_PRIVATE_HH_

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace gui
  {
    /// \brief Privata data for LightMaker class
    class LightMakerPrivate
    {
      /// \brief Node to publish messages and spawn the entity.
      public: transport::NodePtr node;

      /// \brief Publisher used to spawn a new light.
      public: transport::PublisherPtr lightPub;

      /// \brief Pointer to the light being spawned.
      public: rendering::LightPtr light;
    };
  }
}
#endif
