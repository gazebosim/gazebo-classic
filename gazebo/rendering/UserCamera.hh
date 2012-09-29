/*
 * Copyright 2011 Nate Koenig
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

#ifndef _USERCAMERA_HH_
#define _USERCAMERA_HH_

#include <string>
#include <vector>

#include "rendering/Camera.hh"
#include "rendering/RenderTypes.hh"
#include "common/CommonTypes.hh"

namespace gazebo
{
  namespace rendering
  {
    class OrbitViewController;
    class FPSViewController;
    class Visual;
    class GUIOverlay;
    class SelectionBuffer;

    /// \addtogroup gazebo_rendering
    /// \{

    /// \brief A camera used for user visualization of a scene
    class UserCamera : public Camera
    {
      /// \brief Constructor
      public: UserCamera(const std::string &_name, Scene *_scene);

      /// \brief Destructor
      public: virtual ~UserCamera();

      /// \brief Load the user camera
      public: void Load(sdf::ElementPtr _sdf);
      public: void Load();

      /// \brief Initialize
      public: void Init();

      /// \brief Render the camera
      public: virtual void Update();

      /// \brief Post render
      public: virtual void PostRender();

      /// \brief Finialize
      public: void Fini();

      public: virtual void SetWorldPose(const math::Pose &_pose);

      /// \brief Hande a mouse event
      public: void HandleMouseEvent(const common::MouseEvent &_evt);
      public: void HandleKeyPressEvent(const std::string &_key);
      public: void HandleKeyReleaseEvent(const std::string &_key);

      /// \brief Set view controller
      public: void SetViewController(const std::string &_type);

       /// \brief Set view controller
      public: void SetViewController(const std::string &_type,
                                     const math::Vector3 &_pos);

      /// \brief Resize the camera
      public: void Resize(unsigned int _w, unsigned int _h);

      /// \brief Set the dimensions of the viewport
      public: void SetViewportDimensions(float _x, float _y,
                                         float _w, float _h);

      /// \brief Get the average frames per second
      public: float GetAvgFPS() const;

      /// \brief Get the triangle count
      public: float GetTriangleCount() const;

      /// \brief Move the camera to focus on a scene node
      public: void MoveToVisual(VisualPtr _visual);
      public: void MoveToVisual(const std::string &_visualName);

      /// \brief Set the camera to be attached to a scene node
      protected: virtual bool AttachToVisualImpl(VisualPtr _visual,
                     bool _inheritOrientation, double _minDist = 0,
                     double _maxDist = 0);

      /// \brief Set the camera to track a scene node
      protected: virtual bool TrackVisualImpl(VisualPtr _visual);

      /// \brief Set to true to enable rendering
      public: virtual void SetRenderTarget(Ogre::RenderTarget *_target);

      /// \brief Get the GUI overlay
      public: GUIOverlay *GetGUIOverlay();

      /// \brief Set whether the view controller is enabled
      public: void EnableViewController(bool _value) const;

      /// \brief Get an entity at a pixel location using a camera. Used for
      ///        mouse picking.
      /// \param mousePos The position of the mouse in screen coordinates
      /// \param _mod Used for object manipulation
      /// \return The selected entity, or NULL
      public: VisualPtr GetVisual(math::Vector2i mousePos, std::string &mod);

      /// \brief Get a visual at a mouse position
      public: VisualPtr GetVisual(math::Vector2i _mousePos);

      /// \brief Toggle whether to show the visual
      private: void ToggleShowVisual();

      /// \brief Set whether to show the visual
      private: void ShowVisual(bool _s);

      private: void OnMoveToVisualComplete();

      private: static int count;

      private: Visual *visual;

      private: ViewController *viewController;
      private: OrbitViewController *orbitViewController;
      private: FPSViewController *fpsViewController;

      private: GUIOverlay *gui;

      private: Ogre::SceneNode *axisNode;
      private: SelectionBuffer *selectionBuffer;
    };
    /// \}
  }
}
#endif
