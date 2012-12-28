/*
 * Copyright 2012 Open Source Robotics Foundation
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
#ifndef _ORBITVIEWCONTROLLER_HH_
#define _ORBITVIEWCONTROLLER_HH_

#include <string>

#include "rendering/DynamicLines.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/ViewController.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Vector2i.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class OrbitViewController OrbitVeiwController.hh rendering/rendering.hh
    /// \brief Orbit view controller
    class OrbitViewController : public ViewController
    {
      /// \brief Constructor.
      /// \param[in] _camera Pointer to the camera to control.
      public: OrbitViewController(UserCameraPtr _camera);

      /// \brief Destructor.
      public: virtual ~OrbitViewController();

      /// \brief Initialize the controller.
      public: virtual void Init();

      /// \brief Initialze the controller with a focal point.
      /// \param[in] _focalPoint Point to look at.
      public: virtual void Init(const math::Vector3 &_focalPoint);

      /// \brief Update.
      public: virtual void Update();

      /// \brief Handle a mouse event.
      /// \param[in] _event The mouse event.
      public: virtual void HandleMouseEvent(const common::MouseEvent &_event);

      /// \brief Get the type name of this view controller.
      /// \return The view controller name: "orbit".
      public: static std::string GetTypeString();

      /// \brief Set the distance to the focal point
      /// \param[in] _d The distance from the focal point.
      public: void SetDistance(float _d);

      /// \brief Set the focal point
      /// \param[in] _fp The focal point
      public: void SetFocalPoint(const math::Vector3 &_fp);

      /// \brief Get the focal point
      /// \return The focal point
      public: math::Vector3 GetFocalPoint() const;

      // Documentation inherited from parent
      public: void HandleKeyReleaseEvent(const std::string &_key);

      // Documentation inherited from parent
      public: virtual void HandleKeyPressEvent(const std::string &_key);

      /// \brief Translate the focal point in the local coordinate frame.
      /// \param[in] _vec Direction and amount to translate the camera.
      private: void TranslateLocal(const math::Vector3 &_vec);

      /// \brief Translate the focal point in the global coordinate frame.
      /// \param[in] _vec Direction and amount to translate the camera.
      private: void TranslateGlobal(const math::Vector3 &_vec);

      /// \brief Zoom the camera.
      /// \paramp[in] _amount Zoom quatity.
      private: void Zoom(float _amount);

      /// \brief Normalize yaw value.
      /// \paramp[in] _v Normalize a yaw value.
      private: void NormalizeYaw(float &_v);

      /// \brief Normalize pitch value.
      /// \paramp[in] _v Normalize a pitch value.
      private: void NormalizePitch(float &_v);

      /// \brief Update the reference visual.
      private: void UpdateRefVisual();

      /// \brief Update the camera's pose based on a rotation update.
      private: void Orbit();

      private: float yaw, pitch;
      private: float distance;
      private: math::Vector3 focalPoint;

      private: VisualPtr refVisual;

      private: float dy, dp;

      private: std::string key;
    };
    /// \}
  }
}
#endif
