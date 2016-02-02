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
#ifndef _GAZEBO_RENDERING_LIGHT_PRIVATE_HH_
#define _GAZEBO_RENDERING_LIGHT_PRIVATE_HH_

#include <sdf/sdf.hh>

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/common/CommonTypes.hh"

namespace Ogre
{
  class Light;
}

namespace gazebo
{
  namespace rendering
  {
    class DynamicLines;

    /// \internal
    /// \brief Private data for the Light class
    class LightPrivate
    {
      /// \brief The ogre light source
      public: Ogre::Light *light;

      /// \brief The visual used to visualize the light.
      public: VisualPtr visual;

      /// \brief The lines used to visualize the light.
      public: DynamicLines *line;

      /// \brief SDF element data for the light.
      public: sdf::ElementPtr sdf;

      /// \brief Event connection to toggle visualization on/off.
      public: event::ConnectionPtr showLightsConnection;

      /// \brief Pointer to the scene.
      public: ScenePtr scene;

      /// \brief Counter used to generate unique light names.
      public: static unsigned int lightCounter;
    };
  }
}
#endif
