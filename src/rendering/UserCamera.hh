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
 */

#ifndef USERCAMERA_HH
#define USERCAMERA_HH

#include "rendering/Camera.hh"
#include "rendering/RenderTypes.hh"
#include "common/CommonTypes.hh"

namespace Ogre
{
  class AnimationState;
}

namespace gazebo
{
	namespace rendering
  {
    class OrbitViewController;
    class FPSViewController;
    class Visual;
    class GUIOverlay;

    /// \addtogroup gazebo_rendering
    /// \{

    /// \brief A camera used for user visualization of a scene
    class UserCamera : public Camera
    {
      /// \brief Constructor
      public: UserCamera( const std::string &name, Scene *scene);
  
      /// \brief Destructor
      public: virtual ~UserCamera();
  
      /// \brief Load the user camera
      public: void Load( sdf::ElementPtr _sdf );
      public: void Load( );
  
      /// \brief Initialize
      public: void Init();
  
      /// \brief Render the camera
      public: virtual void Update();
  
      /// \brief Post render
      public: virtual void PostRender();
  
      /// \brief Finialize
      public: void Fini();
  
      /// \brief Hande a mouse event
      public: void HandleMouseEvent(const common::MouseEvent &evt);
  
      /// \brief Set view controller
      public: void SetViewController( const std::string &type );

       /// \brief Set view controller
      public: void SetViewController( const std::string &type,
                                      const math::Vector3 &_pos );
  
      /// \brief Resize the camera
      public: void Resize(unsigned int w, unsigned int h);
  
      /// \brief Set the dimensions of the viewport
      public: void SetViewportDimensions(float x, float y, float w, float h);
  
      /// \brief Get the average frames per second
      public: float GetAvgFPS() const;

      /// \brief Get the triangle count 
      public: float GetTriangleCount() const;

      /// \brief Move the camera to focus on a scene node
      public: void MoveToVisual(VisualPtr _visual);
      public: void MoveToVisual(const std::string &_visualName);

      /// \brief Set the camera to be attached to a scene node
      protected: virtual bool AttachToVisualImpl( VisualPtr _visual,
                    double _minDist=0, double _maxDist=0 );

      /// \brief Set the camera to track a scene node
      protected: virtual bool TrackVisualImpl( VisualPtr _visual );

      /// \brief Set to true to enable rendering
      public: virtual void SetRenderTarget( Ogre::RenderTarget *_target );

      /// \brief Get the GUI overlay 
      public: GUIOverlay *GetGUIOverlay();

      /// \brief Toggle whether to show the visual
      private: void ToggleShowVisual();
  
      /// \brief Set whether to show the visual
      private: void ShowVisual(bool s);
  
      private: std::string name;
      private: static int count;
  
      private: Visual *visual;
  
      private: ViewController *viewController;
      private: OrbitViewController *orbitViewController;
      private: FPSViewController *fpsViewController;
  
      private: Ogre::AnimationState *animState;

      private: GUIOverlay *gui;
    };
    /// \}
  }

}
#endif
