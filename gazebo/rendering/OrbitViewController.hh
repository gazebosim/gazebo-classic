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
#ifndef _ORBITVIEWCONTROLLER_HH_
#define _ORBITVIEWCONTROLLER_HH_

#include <string>

#include "rendering/ViewController.hh"
#include "math/Vector3.hh"
#include "math/Vector2i.hh"

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

      /// \brief Set the min and max distance from the focal point.
      /// \param[in] _minDist Min distance to the focal point.
      /// \param[in] _maxDist Max distance from the focal point.
      public: void SetDistanceRange(double _minDist, double _maxDist);

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

      /// \brief Set the yaw angle of the camera.
      /// \param[in] Yaw angle in radians
      public: void SetYaw(double _yaw);

      /// \brief Set the pitch angle of the camera.
      /// \param[in] Pitch angle in radians.
      public: void SetPitch(double _pitch);

      /// \brief Translate the focal point.
      /// \param[in] _vec Direction and amount to translate the camera.
      private: void Translate(math::Vector3 _vec);

      /// \brief Zoom the camera.
      /// \paramp[in] _amount Zoom quatity.
      private: void Zoom(float _amount);

      /// \brief Normalize yaw value.
      /// \paramp[in] _v Normalize a yaw value.
      private: void NormalizeYaw(float &_v);

      /// \brief Normalize pitch value.
      /// \paramp[in] _v Normalize a pitch value.
      private: void NormalizePitch(float &_v);

      /// \brief Update the camera's pose.
      private: void UpdatePose();

      private: float yaw, pitch;
      private: float distance;
      private: float minDist, maxDist;
      private: math::Vector3 focalPoint;

      private: VisualPtr refVisual;
      private: math::Vector2i posCache;
      private: math::Vector3 worldFocal;
    };
    /// \}
  }
}
#endif
