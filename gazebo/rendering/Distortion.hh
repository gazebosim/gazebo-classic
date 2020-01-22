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
#ifndef GAZEBO_RENDERING_DISTORTION_HH_
#define GAZEBO_RENDERING_DISTORTION_HH_

#include <memory>
#include <ignition/math/Vector2.hh>
#include <sdf/Element.hh>

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \ingroup gazebo_rendering
  /// \brief Rendering namespace
  namespace rendering
  {
    class DistortionPrivate;

    /// \addtogroup gazebo_rendering Rendering
    /// \{

    /// \class Distortion Distortion.hh rendering/rendering.hh
    /// \brief Camera distortion based on the Brown-Conrady model.
    class GZ_RENDERING_VISIBLE Distortion
    {
      /// \brief Constructor
      public: Distortion();

      /// \brief Destructor
      public: virtual ~Distortion();

      /// \brief Load the camera with a set of parmeters
      /// \param[in] _sdf The SDF camera info
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Set the camera which distortion will be applied to.
      /// \param[in] _camera Camera to be distorted
      public: void SetCamera(CameraPtr _camera);

      /// \brief Set whether to crop the black border around the distorted
      /// image points.
      /// \param[in] _crop True to crop the black border
      /// \sa Crop
      public: void SetCrop(const bool _crop);

      /// \brief Get whether or not the camera is being cropped to
      /// account for black borders created by barrel distortion.
      /// \return True if the black border is cropped
      /// \sa SetCrop
      public: bool Crop() const;

      /// \brief Get the radial distortion coefficient k1.
      /// \return Distortion coefficient k1.
      public: double K1() const;

      /// \brief Get the radial distortion coefficient k2.
      /// \return Distortion coefficient k2.
      public: double K2() const;

      /// \brief Get the radial distortion coefficient k3.
      /// \return Distortion coefficient k3.
      public: double K3() const;

      /// \brief Get the tangential distortion coefficient p1.
      /// \return Distortion coefficient p1.
      public: double P1() const;

      /// \brief Get the tangential distortion coefficient p2.
      /// \return Distortion coefficient p2.
      public: double P2() const;

      /// \brief Get the distortion center.
      /// \return Distortion center.
      public: ignition::math::Vector2d Center() const;

      /// \brief Apply distortion model
      /// \param[in] _in Input uv coordinate.
      /// \param[in] _center Normalized distortion center.
      /// \param[in] _k1 Radial distortion coefficient k1.
      /// \param[in] _k2 Radial distortion coefficient k2.
      /// \param[in] _k3 Radial distortion coefficient k3.
      /// \param[in] _p1 Tangential distortion coefficient p1.
      /// \param[in] _p2 Tangential distortion coefficient p2.
      /// \return Distorted coordinate.
      public: static ignition::math::Vector2d Distort(
                  const ignition::math::Vector2d &_in,
                  const ignition::math::Vector2d &_center,
                  double _k1, double _k2, double _k3,
                  double _p1, double _p2);

      /// \brief get the distortion map value.
      /// \return the distortion map value at the specified index,
      /// or (-1, -1) if the index
      /// is out of bounds.
      protected: ignition::math::Vector2d
        DistortionMapValueClamped(const int x, const int y) const;

      /// \brief calculate the correct scale factor to "zoom" the render,
      /// cutting off black borders caused by distortion (only if the crop
      /// flag has been set).
      protected: void CalculateAndApplyDistortionScale();

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<DistortionPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
