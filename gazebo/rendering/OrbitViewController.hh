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
#ifndef _ORBITVIEWCONTROLLER_HH_
#define _ORBITVIEWCONTROLLER_HH_

#include <string>

#include <ignition/math/Vector3.hh>

#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/ViewController.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class OrbitViewController OrbitVeiwController.hh rendering/rendering.hh
    /// \brief Orbit view controller
    class GZ_RENDERING_VISIBLE OrbitViewController : public ViewController
    {
      /// \brief Constructor.
      /// \param[in] _camera Pointer to the camera to control.
      /// \param[in] _name Name of the view controller. A subclass of
      /// OrbitViewController should use a name other than
      /// "OrbitViewController".
      public: OrbitViewController(UserCameraPtr _camera,
                  const std::string &_name = "OrbitViewController");

      /// \brief Destructor.
      public: virtual ~OrbitViewController();

      /// \brief Initialize the controller.
      public: virtual void Init();

      // Documentation inherited
      public: virtual void Init(const math::Vector3 &_focalPoint,
                  const double _yaw = 0, const double _pitch = 0);

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

      public: double Pitch() const;

      public: double Yaw() const;

      /// \brief Translate the focal point in the local coordinate frame.
      /// \param[in] _vec Direction and amount to translate the camera.
      protected: void TranslateLocal(const math::Vector3 &_vec);

      /// \brief Translate the focal point in the global coordinate frame.
      /// \param[in] _vec Direction and amount to translate the camera.
      protected: void TranslateGlobal(const math::Vector3 &_vec);

      /// \brief Zoom the camera.
      /// \paramp[in] _amount Zoom quatity.
      protected: void Zoom(float _amount);

      /// \brief Normalize yaw value.
      /// \paramp[in] _v Normalize a yaw value.
      /// \return The normalized value.
      protected: double NormalizeYaw(double _v);

      /// \brief Normalize pitch value.
      /// \paramp[in] _v Normalize a pitch value.
      /// \return The normalized value.
      protected: double NormalizePitch(double _v);

      /// \brief Update the reference visual.
      protected: void UpdateRefVisual();

      /// \brief Update the camera's pose based on a rotation update.
      /// \param[in] _dy Delta yaw movement.
      /// \param[in] _dp Delta pitch movement.
      protected: void Orbit(double _dy, double _dp);

      /// \brief Yaw value.
      protected: float yaw;

      /// \brief Pitch value.
      protected: float pitch;

      /// \brief Distance to the focal point.
      protected: float distance;

      /// \brief A reference visual.
      protected: VisualPtr refVisual;

      /// \brief Key that is currently pressed.
      protected: std::string key;

      /// \brief A flag used to inidicate that the view controller has just
      /// been initialized.
      protected: bool init;

      /// \brief The focal point.
      protected: ignition::math::Vector3d focalPoint;
    };
    /// \}
  }
}
#endif
