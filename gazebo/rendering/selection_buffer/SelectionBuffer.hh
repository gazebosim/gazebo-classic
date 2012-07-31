/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#ifndef _SELECTIONBUFFER_HH_
#define _SELECTIONBUFFER_HH_

namespace Ogre
{
  class SceneManager;
  class Camera;
  class RenderTarget;
  class RenderTexture;
  class PixelBox;
  class Overlay;
}

namespace gazebo
{
  namespace rendering
  {
    class MaterialSwitcher;
    class SelectionRenderListener;
    class Scene;
    
    class SelectionBuffer
    {
      /// \brief Constructor
      public: SelectionBuffer(const std::string _cameraName,
                              Scene *_scene, Ogre::RenderTarget *_renderTarget);

      /// \brief Destructor
      public: ~SelectionBuffer();
      
      /// \brief Handle on mouse click
      public: Ogre::Entity *OnSelectionClick(int _x, int _y);

      /// \brief Debug show overlay
      public: void ShowOverlay(bool _show);

      /// \brief Call this to update the selection buffer contents
      public: void Update();

      private: void CreateRTTOverlays();
      private: void UpdateBufferSize();
      
      // This is the material listener - Note: it is controlled by a separate
      // RenderTargetListener, not applied globally to all targets
      private: MaterialSwitcher *materialSwitchListener;
      
      private: SelectionRenderListener *selectionTargetListener;

      private: Ogre::SceneManager *sceneMgr;
      private: Ogre::Camera *camera;
      private: Ogre::RenderTarget *renderTarget;
      private: Ogre::TexturePtr texture;
      private: Ogre::RenderTexture *renderTexture;
      private: uint8_t *buffer;
      private: Ogre::PixelBox *pixelBox;
      private: Ogre::Overlay *selectionDebugOverlay;
      
    };
  }
}

#endif
