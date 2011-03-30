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
/* Desc: Camera for viewing the world
 * Author: Nate Koenig
 * Date: 19 Jun 2008
 * SVN: $Id$
 */

#ifndef USERCAMERA_HH
#define USERCAMERA_HH

#include "common/XMLConfig.hh"
#include "rendering/Camera.hh"

namespace Ogre
{
  class RenderWindow;
  class AnimationState;
}

namespace gazebo
{
  namespace common
  {
    class XMLConfigNode;
    class MouseEvent;
  }

	namespace rendering
  {
    class Visual;
  
    class UserCamera : public Camera
    {
      /// \brief Constructor
      public: UserCamera( const std::string &name, Scene *scene);
  
      /// \brief Destructor
      public: virtual ~UserCamera();
  
      /// \brief Load child
      public: void Load( common::XMLConfigNode *node );
  
      /// \brief Initialize
      public: void Init();
  
      /// \brief Render the camera
      public: virtual void Render();
  
      /// \brief Post render
      public: virtual void PostRender();
  
      /// \brief Finialize
      public: void Fini();
  
      /// \brief Hande a mouse event
      public: void HandleMouseEvent(const common::MouseEvent &evt);
  
      /// \brief Set view controller
      public: void SetViewController( const std::string type );
  
      /// \brief Resize the camera
      //public: void Resize(unsigned int w, unsigned int h);
  
      /// \brief Set the dimensions of the viewport
      public: void SetViewportDimensions(float x, float y, float w, float h);
  
      /// \brief Get the average frames per second
      public: float GetAvgFPS() const;

      /// \brief Get the triangle count 
      public: float GetTriangleCount() const;


      /// \brief Move the camera to focus on a scene node
      public: void MoveToVisual(Visual *visual);
  
      /// \brief Set the camera to track a scene node
      public: void TrackVisual( Visual *visual );
   
      /// \brief Toggle whether to show the visual
      private: void ToggleShowVisual();
  
      /// \brief Set whether to show the visual
      private: void ShowVisual(bool s);
  
      private: std::string name;
      private: static int count;
  
      private: Visual *visual;
  
      private: ViewController *viewController;
  
      private: std::vector<event::ConnectionPtr> connections;
      private: Ogre::AnimationState *animState;
    };
  }

}
#endif
