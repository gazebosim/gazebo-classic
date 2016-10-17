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
#ifndef GAZEBO_RENDERING_ORTHOVIEWCONTROLLER_HH_
#define GAZEBO_RENDERING_ORTHOVIEWCONTROLLER_HH_

#include <string>

#include "gazebo/rendering/ViewController.hh"
#include "gazebo/math/Vector2i.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    // Forward declare private data pointer.
    class OrthoViewControllerPrivate;

    /// \addtogroup gazebo_rendering
    /// \{

    /// \class OrthoViewController OrbitVeiwController.hh rendering/rendering.hh
    /// \brief Orthographic view controller
    class GAZEBO_VISIBLE OrthoViewController : public OrbitViewController
    {
      /// \brief Constructor.
      /// \param[in] _camera Pointer to the camera to control.
      public: explicit OrthoViewController(UserCameraPtr _camera);

      /// \brief Destructor.
      public: virtual ~OrthoViewController();

      /// \brief Initialize the controller.
      public: virtual void Init();

      // Documentation inherited
      public: virtual void Init(const math::Vector3 &_focalPoint,
                  const double _yaw = 0, const double _pitch = 0);

      /// \brief Get the type name of this view controller.
      /// \return The view controller name: "ortho".
      public: static std::string GetTypeString();

      /// \brief Handle a mouse event.
      /// \param[in] _event The mouse event.
      public: virtual void HandleMouseEvent(const common::MouseEvent &_event);

      /// \brief Zoom the camera.
      /// \param[in] _amount Zoom quantity.
      /// \param[in] _point Position on screen to zoom to
      private: void Zoom(const float _amount,
                         const math::Vector2i &_point = math::Vector2i(0, 0));

      // Documentation inherited
      public: virtual void Resize(const unsigned int _width,
                                  const unsigned int _height);

      /// \brief Build a custom scaled orthographic projection matrix.
      /// \param[in] _left Left position
      /// \param[in] _right Right position
      /// \param[in] _bottom Bottom position
      /// \param[in] _top Top position
      /// \param[in] _near Near clip distance
      /// \param[in] _far Far clip distance
      private: Ogre::Matrix4 BuildScaledOrthoMatrix(
                   const float _left, const float _right,
                   const float _bottom, const float _top,
                   const float _near, const float _far) const;


      /// \brief Private data pointer
      private: OrthoViewControllerPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
