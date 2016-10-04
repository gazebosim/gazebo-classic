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
#ifndef _GAZEBO_RENDERING_GPULASER_HH_
#define _GAZEBO_RENDERING_GPULASER_HH_

#include <memory>
#include <string>

#include <sdf/sdf.hh>

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

namespace Ogre
{
  class Material;
  class Renderable;
  class Pass;
  class AutoParamDataSource;
  class Matrix4;
  class MovableObject;
}

namespace gazebo
{
  namespace common
  {
    class Mesh;
  }

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
      public: virtual ~GpuLaser();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual void Load();

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      public: virtual void Fini();

      /// \brief Create the texture which is used to render laser data.
      /// \param[in] _textureName Name of the new texture.
      public: void CreateLaserTexture(const std::string &_textureName);

      // Documentation inherited
      public: virtual void PostRender();

      /// \brief All things needed to get back z buffer for laser data.
      /// \return Array of laser data.
      public: const float *LaserData() const;

      /// \brief Connect to a laser frame signal
      /// \param[in] _subscriber Callback that is called when a new image is
      /// generated
      /// \return A pointer to the connection. This must be kept in scope.
      public: event::ConnectionPtr ConnectNewLaserFrame(
                  std::function<void (const float *_frame, unsigned int _width,
                  unsigned int _height, unsigned int _depth,
                  const std::string &_format)> _subscriber);

      /// \brief Disconnect from a laser frame signal
      /// \param[in] _c The connection to disconnect
      /// \deprecated Use event::~Connection to disconnect
      public: void DisconnectNewLaserFrame(event::ConnectionPtr &_c)
              GAZEBO_DEPRECATED(8.0);

      /// \brief Set the number of laser samples in the width and height
      /// \param[in] _w Number of samples in the horizontal sweep
      /// \param[in] _h Number of samples in the vertical sweep
      public: void SetRangeCount(const unsigned int _w,
          const unsigned int _h = 1);

      /// \internal
      /// \brief Implementation of Ogre::RenderObjectListener
      public: virtual void notifyRenderSingleObject(Ogre::Renderable *_rend,
              const Ogre::Pass *_p, const Ogre::AutoParamDataSource *_s,
              const Ogre::LightList *_ll, bool _supp);

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

      /// \brief Set the number of cameras required
      /// \param[in] _cameraCount The number of cameras required to generate
      /// the rays
      public: void SetCameraCount(const unsigned int _cameraCount);

      /// \brief Get the ray count ratio (equivalent to aspect ratio)
      /// \return The ray count ratio (equivalent to aspect ratio)
      public: double RayCountRatio() const;

      /// \brief Sets the ray count ratio (equivalen to aspect ratio)
      /// \param[in] _rayCountRatio ray count ratio (equivalent to aspect ratio)
      public: void SetRayCountRatio(const double _rayCountRatio);

      // Documentation inherited.
      private: virtual void RenderImpl();

      /// \brief Update a render target.
      /// \param[in, out] _target Render target to update (render).
      /// \param[in, out] _material Material used during render.
      /// \param[in] _cam Camerat to render from.
      /// \param[in] _updateTex True to update the textures in the material
      private: void UpdateRenderTarget(Ogre::RenderTarget *_target,
                                       Ogre::Material *_material,
                                       Ogre::Camera *_cam,
                                       const bool _updateTex = false);

      /// \brief Create an ortho camera.
      private: void CreateOrthoCam();

      /// \brief Create a mesh.
      private: void CreateMesh();

      /// \brief Create a canvas.
      private: void CreateCanvas();

      /// \brief Builds scaled Orthogonal Matrix from parameters.
      /// \param[in] _left Left clip.
      /// \param[in] _right Right clip.
      /// \param[in] _bottom Bottom clip.
      /// \param[in] _top Top clip.
      /// \param[in] _near Near clip.
      /// \param[in] _far Far clip.
      /// \return The Scaled orthogonal Ogre::Matrix4
      private: Ogre::Matrix4 BuildScaledOrthoMatrix(const float _left,
          const float _right, const float _bottom, const float _top,
          const float _near, const float _far);

      /// \brief Sets first pass target.
      /// \param[in] _target Render target for the first pass.
      /// \param[in] _index Index of the texture.
      private: virtual void Set1stPassTarget(Ogre::RenderTarget *_target,
                                             const unsigned int _index);

      /// \brief Sets second pass target.
      /// \param[in] _target Render target for the second pass.
      private: virtual void Set2ndPassTarget(Ogre::RenderTarget *_target);

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
