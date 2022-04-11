/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifndef _GAZEBO_RENDERING_GPULASER_HH_
#define _GAZEBO_RENDERING_GPULASER_HH_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>

#include <sdf/sdf.hh>

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/GpuLaserCubeFace.hh"
#include "gazebo/rendering/GpuLaserDataIterator.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

namespace Ogre
{
  class Renderable;
  class Pass;
  class AutoParamDataSource;
}

namespace gazebo
{
  namespace rendering
  {
    // Forward declare private data.
    class GpuLaserPrivate;

    /// \addtogroup gazebo_rendering Rendering
    /// \{

    /// \class GpuLaser GpuLaser.hh rendering/rendering.hh
    /// \brief GPU based laser distance sensor
    class GZ_RENDERING_VISIBLE GpuLaser
      : public Camera, public Ogre::RenderObjectListener
    {
      /// \brief Constructor
      /// \param[in] _namePrefix Unique prefix name for the camera.
      /// \param[in] _scene Scene that will contain the camera
      /// \param[in] _autoRender Almost everyone should leave this as true.
      public: GpuLaser(const std::string &_namePrefix,
          ScenePtr _scene, const bool _autoRender = true);

      /// \brief Destructor
      public: ~GpuLaser() override;

      // Documentation inherited
      public: void Load(sdf::ElementPtr _sdf) override;

      // Documentation inherited
      public: void Load() override;

      // Documentation inherited
      public: void Init() override;

      // Documentation inherited
      public: void Fini() override;

      /// \brief Create the texture which is used to render laser data.
      /// \param[in] _textureName Name of the new texture.
      public: void CreateLaserTexture(const std::string &_textureName);

      // Documentation inherited
      public: void PostRender() override;

      /// \brief Constant iterator to access laser data
      public: typedef GpuLaserDataIterator<GpuLaser> DataIter;

      /// \brief Return an iterator to the beginning of the laser data
      public: DataIter LaserDataBegin() const;

      /// \brief Return an iterator to one past the end of the laser data
      public: DataIter LaserDataEnd() const;

      /// \brief Connect to a laser frame signal
      /// \param[in] _subscriber Callback that is called when a new image is
      /// generated
      /// \return A pointer to the connection. This must be kept in scope.
      public: event::ConnectionPtr ConnectNewLaserFrame(
                  std::function<void (const float *_frame, unsigned int _width,
                  unsigned int _height, unsigned int _depth,
                  const std::string &_format)> _subscriber);

      /// \brief Set the number of samples in width and height.
      /// \param[in] _w Number of samples in the horizontal sweep
      /// \param[in] _h Number of samples in the vertical sweep
      public: void SetRangeCount(const unsigned int _w,
          const unsigned int _h = 1);

      /// \internal
      /// \brief Implementation of Ogre::RenderObjectListener
      public: void notifyRenderSingleObject(Ogre::Renderable *_rend,
              const Ogre::Pass *_p, const Ogre::AutoParamDataSource *_s,
              const Ogre::LightList *_ll, bool _supp) override;

      /// \brief Get (horizontal_max_angle + horizontal_min_angle) * 0.5
      /// \return (horizontal_max_angle + horizontal_min_angle) * 0.5
      public: double HorzHalfAngle() const;

      /// \brief Get (vertical_max_angle + vertical_min_angle) * 0.5
      /// \return (vertical_max_angle + vertical_min_angle) * 0.5
      public: double VertHalfAngle() const;

      /// \brief Set the horizontal half angle
      /// \param[in] _angle horizontal half angle
      public: void SetHorzHalfAngle(const double _angle);

      /// \brief Set the vertical half angle
      /// \param[in] _angle vertical half angle
      public: void SetVertHalfAngle(const double _angle);

      /// \brief Set sensor horizontal or vertical
      /// \param[in] _horizontal True if horizontal, false if not
      public: void SetIsHorizontal(const bool _horizontal);

      /// \brief Gets if sensor is horizontal
      /// \return True if horizontal, false if not
      public: bool IsHorizontal() const;

      /// \brief Get the horizontal field of view of the laser sensor.
      /// \return The horizontal field of view of the laser sensor.
      public: double HorzFOV() const;

      /// \brief Get Cos Horz field-of-view
      /// \return 2 * atan(tan(this->hfov/2) / cos(this->vfov/2))
      public: double CosHorzFOV() const;

      /// \brief Set the Cos Horz FOV
      /// \param[in] _chfov Cos Horz FOV
      public: void SetCosHorzFOV(const double _chfov);

      /// \brief Get the vertical field-of-view.
      /// \return The vertical field of view of the laser sensor.
      public: double VertFOV() const;

      /// \brief Get Cos Vert field-of-view
      /// \return 2 * atan(tan(this->vfov/2) / cos(this->hfov/2))
      public: double CosVertFOV() const;

      /// \brief Set the Cos Horz FOV
      /// \param[in] _cvfov Cos Horz FOV
      public: void SetCosVertFOV(const double _cvfov);

      /// \brief Get near clip
      /// \return near clip distance
      public: double NearClip() const;

      /// \brief Get far clip
      /// \return far clip distance
      public: double FarClip() const;

      /// \brief Set the near clip distance
      /// \param[in] _near near clip distance
      public: void SetNearClip(const double _near);

      /// \brief Set the far clip distance
      /// \param[in] _far far clip distance
      public: void SetFarClip(const double _far);

      /// \brief Set the horizontal fov
      /// \param[in] _hfov horizontal fov
      public: void SetHorzFOV(const double _hfov);

      /// \brief Set the vertical fov
      /// \param[in] _vfov vertical fov
      public: void SetVertFOV(const double _vfov);

      /// \brief Get the number of cameras required
      /// \return Number of cameras needed to generate the rays
      public: unsigned int CameraCount() const;

      /// \brief Set the number of cameras required. Has no effect for this
      /// implementation since the number of cameras is calculated based on the
      /// rays.
      /// \deprecated The camera count cannot be set from here anymore since it
      /// is determined automatically in GpuLaser::InitMapping.
      /// \param[in] _cameraCount The number of cameras required to generate
      /// the rays
      public: void SetCameraCount(const unsigned int _cameraCount) GAZEBO_DEPRECATED(11.10);

      /// \brief Get the ray count ratio (equivalent to aspect ratio)
      /// \return The ray count ratio (equivalent to aspect ratio)
      public: double RayCountRatio() const;

      /// \brief Sets the ray count ratio (equivalent to aspect ratio)
      /// \param[in] _rayCountRatio ray count ratio (equivalent to aspect ratio)
      public: void SetRayCountRatio(const double _rayCountRatio);

      /// \brief Initializes the mapping of ray angles to cube map coordinates.
      /// Each combination of values (azimuth, elevation) corresponds to one
      /// laser ray.
      /// \param[in] _azimuth_values Set of azimuth angles (radians). The order matters!
      /// \param[in] _elevation_values Set of elevation angles (radians). The order matters!
      public: void InitMapping(const std::set<double> &_azimuth_values, const std::set<double> &_elevation_values);

      /// \brief Finds the corresponding cube map face and the coordinates of
      /// intersection of the view ray.
      /// \note The azimuth must be specified relative to the minimum azimuth value!
      /// \param[in] _azimuth Horizontal angle (radians) relative to minimum azimuth angle
      /// \param[in] _elevation Vertical angle (radians) where zero is orthogonal to the spin axis
      /// \returns Mapping for the given ray
      public: static GpuLaserCubeMappingPoint FindCubeFaceMapping(const double _azimuth, const double _elevation);

      /// \brief Finds the corresponding face of the cube map for a pair of
      /// azimuth and elevation angles.
      /// \note The azimuth must be specified relative to the minimum azimuth value!
      /// \param[in] _azimuth Horizontal angle (radians) relative to minimum azimuth angle
      /// \param[in] _elevation Vertical angle (radians) where zero is orthogonal to the spin axis
      /// \returns Identifier for the corresponding face.
      public: static GpuLaserCubeFaceId FindCubeFace(const double _azimuth, const double _elevation);

      /// \brief Calculates a vector in the direction of the ray.
      /// \note The azimuth must be specified relative to the minimum azimuth value!
      /// \param[in] _azimuth Horizontal angle (radians) relative to minimum azimuth angle
      /// \param[in] _elevation Vertical angle (radians) where zero is orthogonal to the spin axis
      /// \returns Viewing ray vector
      public: static ignition::math::Vector3d ViewingRay(const double _azimuth, const double _elevation);

      // Documentation inherited.
      private: void RenderImpl() override;

      /// \brief Update a render target.
      /// \param[in, out] _cube_face Cube face for which to update the render
      /// target.
      private: void UpdateRenderTarget(GpuLaserCubeFace &_cube_face);

      /// \brief Setup the render target for the specified cube face.
      /// \param[in] cube_face The cube face.
      private: virtual void SetUpRenderTarget(GpuLaserCubeFace &_cube_face);

      /// \brief Applies the camera orientation offset by rotation in roll and yaw.
      /// \param[in] _setting The camera orientation offset to apply.
      private: void ApplyCameraSetting(const GpuLaserCameraOrientationOffset &_setting);

      /// \brief Inverse of GpuLaser::ApplyCameraSetting(): Reverts the given
      /// camera orientation offset.
      /// \param[in] _setting The camera orientation offset to revert.
      private: void RevertCameraSetting(const GpuLaserCameraOrientationOffset &_setting);

      /// \brief Horizontal half angle.
      protected: double horzHalfAngle;

      /// \brief Vertical half angle.
      protected: double vertHalfAngle;

      /// \brief Ray count ratio.
      protected: double rayCountRatio;

      /// \brief Horizontal field-of-view.
      protected: double hfov;

      /// \brief Vertical field-of-view.
      protected: double vfov;

      /// \brief Cos horizontal field-of-view.
      protected: double chfov;

      /// \brief Cos vertical field-of-view.
      protected: double cvfov;

      /// \brief Near clip plane.
      protected: double nearClip;

      /// \brief Far clip plane.
      protected: double farClip;

      /// \brief True if the sensor is horizontal only.
      protected: bool isHorizontal;

      /// \brief Number of cameras needed to generate the rays.
      protected: unsigned int cameraCount;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<GpuLaserPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
