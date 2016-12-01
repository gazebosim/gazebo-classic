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

#ifndef _GAZEBO_RENDERING_RENDERENGINE_PRIVATE_HH_
#define _GAZEBO_RENDERING_RENDERENGINE_PRIVATE_HH_

#include <vector>
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/RenderEngine.hh"

namespace Ogre
{
  class Root;
  class LogManager;
  class OverlaySystem;
}

namespace gazebo
{
  namespace rendering
  {
    /// \internal
    /// \brief Private data for the RenderEngine class
    class RenderEnginePrivate
    {
      /// \brief Pointer to ogre root.
      public: Ogre::Root *root;

      /// \brief All of the scenes
      public: std::vector<ScenePtr> scenes;

      /// \brief Pointer the log manager
      public: Ogre::LogManager *logManager;

      /// \brief True if the GUI is enabled.
      public: bool headless;

      /// \brief True if initialized.
      public: bool initialized;

      /// \brief All the event connections.
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief The type of render path used.
      public: RenderEngine::RenderPathType renderPathType;

      /// \brief Pointer to the window manager.
      public: WindowManagerPtr windowManager;

      /// \brief A list of supported fsaa levels
      public: std::vector<unsigned int> fsaaLevels;

#if OGRE_VERSION_MAJOR > 1 || OGRE_VERSION_MINOR >= 9
      /// \brief Ogre overlay system needed for initialization of Ogre
      public: Ogre::OverlaySystem *overlaySystem;
#endif
    };
  }
}
#endif
