/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#ifndef _GUI_OVERLAY_PRIVATE_HH_
#define _GUI_OVERLAY_PRIVATE_HH_

#include <string>
#include <vector>

#include "gazebo/common/Events.hh"
#include "gazebo/rendering/cegui.h"

namespace Ogre
{
  class RenderTarget;
}

#ifdef HAVE_CEGUI
namespace CEGUI
{
  class OgreRenderer;
  class Window;
}
#endif

namespace gazebo
{
  namespace rendering
  {
    /// \brief Private data for the GUIOverlay class
    class GUIOverlayPrivate
    {
#ifdef HAVE_CEGUI
      /// \brief Pointer to the CEGUI ogre renderer
      public: CEGUI::OgreRenderer *guiRenderer;
#endif

      /// \brief All the connections
      public: std::vector<event::ConnectionPtr> connections;

      /// \brief The layout file used to create gui elements
      public: std::string layoutFilename;

      /// \brief Used in the AttachCameraToImage function to create unique
      /// names
      public: unsigned int rttImageSetCount;

      /// \brief True if initialized
      public: bool initialized;
    };
    /// \}
  }
}
#endif
