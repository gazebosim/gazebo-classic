/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_RENDERING_SELECTION_BUFFER_SELECTIONBUFFER_HH_
#define _GAZEBO_RENDERING_SELECTION_BUFFER_SELECTIONBUFFER_HH_

#include <memory>
#include <string>
#include "gazebo/util/system.hh"

namespace Ogre
{
  class Entity;
  class RenderTarget;
  class SceneManager;
}

namespace gazebo
{
  namespace rendering
  {
    struct SelectionBufferPrivate;

    class GZ_RENDERING_VISIBLE SelectionBuffer
    {
      /// \brief Constructor
      /// \param[in] _camera Name of the camera to generate a selection
      /// buffer for.
      /// \param[in] _mgr Pointer to the scene manager.
      /// \param[in] _renderTarget Pointer to the render target.
      public: SelectionBuffer(const std::string &_cameraName,
                  Ogre::SceneManager *_mgr, Ogre::RenderTarget *_renderTarget);

      /// \brief Destructor
      public: ~SelectionBuffer();

      /// \brief Handle on mouse click
      /// \param[in] _x X coordinate in pixels.
      /// \param[in] _y Y coordinate in pixels.
      /// \return Returns the Ogre entity at the coordinate.
      public: Ogre::Entity *OnSelectionClick(int _x, int _y);

      /// \brief Debug show overlay
      /// \param[in] _show True to show the selection buffer in an overlay.
      public: void ShowOverlay(bool _show);

      /// \brief Call this to update the selection buffer contents
      public: void Update();

      /// \brief Delete the render texture
      private: void DeleteRTTBuffer();

      /// \brief Create the render texture
      private: void CreateRTTBuffer();

      /// \brief Create the selection buffer offscreen render texture.
      private: void CreateRTTOverlays();

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<SelectionBufferPrivate> dataPtr;
    };
  }
}
#endif
