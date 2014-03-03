/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
/* Desc: A laser sensor using OpenGL
 * Author: Mihai Emanuel Dolha
 * Date: 29 March 2012
 */

#ifndef _GPULASER_HH_
#define _GPULASER_HH_

#include <string>
#include <vector>

#include <sdf/sdf.hh>

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/RenderTypes.hh"

#include "gazebo/common/Event.hh"
#include "gazebo/common/Time.hh"

#include "gazebo/math/Angle.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/math/Vector2i.hh"

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
    class Scene;

    /// \addtogroup gazebo_rendering Rendering
    /// \{

    /// \class GpuLaser GpuLaser.hh rendering/rendering.hh
    /// \brief GPU based laser distance sensor
    class GpuLaser : public Camera, public Ogre::RenderObjectListener
    {
      /// \brief Constructor
      /// \param[in] _namePrefix Unique prefix name for the camera.
      /// \param[in] _scene Scene that will contain the camera
      /// \param[in] _autoRender Almost everyone should leave this as true.
      public: GpuLaser(const std::string &_namePrefix,
                          ScenePtr _scene, bool _autoRender = true);

      /// \brief Destructor
      public: virtual ~GpuLaser();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr &_sdf);

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
      public: const float *GetLaserData();

      /// \brief Connect to a laser frame signal
      /// \param[in] _subscriber Callback that is called when a new image is
      /// generated
      /// \return A pointer to the connection. This must be kept in scope.
      public: template<typename T>
              event::ConnectionPtr ConnectNewLaserFrame(T _subscriber)
              { return newLaserFrame.Connect(_subscriber); }

      /// \brief Disconnect from a laser frame signal
      /// \param[in] _c The connection to disconnect
      public: void DisconnectNewLaserFrame(event::ConnectionPtr &_c)
              { newLaserFrame.Disconnect(_c); }

      /// \brief Set the number of laser samples in the width and height
      /// \param[in] _w Number of samples in the horizontal sweep
      /// \param[in] _h Number of samples in the vertical sweep
      public: void SetRangeCount(unsigned int _w, unsigned int _h = 1);

      /// \internal
      /// \brief Implementation of Ogre::RenderObjectListener
      public: virtual void notifyRenderSingleObject(Ogre::Renderable *_rend,
              const Ogre::Pass *_p, const Ogre::AutoParamDataSource *_s,
              const Ogre::LightList *_ll, bool _supp);

      /// \brief Get (horizontal_max_angle + horizontal_min_angle) * 0.5
      /// \return (horizontal_max_angle + horizontal_min_angle) * 0.5
      public: double GetHorzHalfAngle() const;

      /// \brief Get (vertical_max_angle + vertical_min_angle) * 0.5
      /// \return (vertical_max_angle + vertical_min_angle) * 0.5
      public: double GetVertHalfAngle() const;

      /// \brief Set the horizontal half angle
      /// \param[in] _angle horizontal half angle
      public: void SetHorzHalfAngle(double _angle);

      /// \brief Set the vertical half angle
      /// \param[in] _angle vertical half angle
      public: void SetVertHalfAngle(double _angle);

      /// \brief Set sensor horizontal or vertical
      /// \param[in] _horizontal True if horizontal, false if not
      public: void SetIsHorizontal(bool _horizontal);

      /// \brief Gets if sensor is horizontal
      /// \return True if horizontal, false if not
      public: bool IsHorizontal() const;

      /// \brief Get the horizontal field of view of the laser sensor.
      /// \return The horizontal field of view of the laser sensor.
      public: double GetHorzFOV() const;

      /// \brief Get Cos Horz field-of-view
      /// \return 2 * atan(tan(this->hfov/2) / cos(this->vfov/2))
      public: double GetCosHorzFOV() const;

      /// \brief Set the Cos Horz FOV
      /// \param[in] _chfov Cos Horz FOV
      public: void SetCosHorzFOV(double _chfov);

      /// \brief Get the vertical field-of-view.
      /// \return The vertical field of view of the laser sensor.
      public: double GetVertFOV() const;

      /// \brief Get Cos Vert field-of-view
      /// \return 2 * atan(tan(this->vfov/2) / cos(this->hfov/2))
      public: double GetCosVertFOV() const;

      /// \brief Set the Cos Horz FOV
      /// \param[in] _cvfov Cos Horz FOV
      public: void SetCosVertFOV(double _cvfov);

      /// \brief Get near clip
      /// \return near clip distance
      public: double GetNearClip() const;

      /// \brief Get far clip
      /// \return far clip distance
      public: double GetFarClip() const;

      /// \brief Set the near clip distance
      /// \param[in] _near near clip distance
      public: void SetNearClip(double _near);

      /// \brief Set the far clip distance
      /// \param[in] _far far clip distance
      public: void SetFarClip(double _far);

      /// \brief Set the horizontal fov
      /// \param[in] _hfov horizontal fov
      public: void SetHorzFOV(double _hfov);

      /// \brief Set the vertical fov
      /// \param[in] _vfov vertical fov
      public: void SetVertFOV(double _vfov);

      /// \brief Get the number of cameras required
      /// \return Number of cameras needed to generate the rays
      public: double GetCameraCount() const;

      /// \brief Set the number of cameras required
      /// \param[in] _cameraCount The number of cameras required to generate
      /// the rays
      public: void SetCameraCount(double _cameraCount);

      /// \brief Get the ray count ratio (equivalent to aspect ratio)
      /// \return The ray count ratio (equivalent to aspect ratio)
      public: double GetRayCountRatio() const;

      /// \brief Sets the ray count ratio (equivalen to aspect ratio)
      /// \param[in] _rayCountRatio ray count ratio (equivalent to aspect ratio)
      public: void SetRayCountRatio(double _rayCountRatio);

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
                                       bool _updateTex = false);

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
      private: Ogre::Matrix4 BuildScaledOrthoMatrix(float _left, float _right,
               float _bottom, float _top, float _near, float _far);

      /// \brief Sets first pass target.
      /// \param[in] _target Render target for the first pass.
      /// \param[in] _index Index of the texture.
      private: virtual void Set1stPassTarget(Ogre::RenderTarget *_target,
                                             unsigned int _index);

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
      protected: double near;

      /// \brief Far clip plane.
      protected: double far;

      /// \brief True if the sensor is horizontal only.
      protected: bool isHorizontal;

      /// \brief Number of cameras needed to generate the rays.
      protected: unsigned int cameraCount;

      /// \brief Event triggered when new laser range data are available.
      /// \param[in] _frame New frame containing raw laser data.
      /// \param[in] _width Width of frame.
      /// \param[in] _height Height of frame.
      /// \param[in] _depth Depth of frame.
      /// \param[in] _format Format of frame.
      private: event::EventT<void(const float *_frame, unsigned int _width,
                   unsigned int _height, unsigned int _depth,
                   const std::string &_format)> newLaserFrame;

      /// \brief Raw buffer of laser data.
      private: float *laserBuffer;

      /// \brief Outgoing laser data, used by newLaserFrame event.
      private: float *laserScan;

      /// \brief Pointer to Ogre material for the first rendering pass.
      private: Ogre::Material *matFirstPass;

      /// \brief Pointer to Ogre material for the sencod rendering pass.
      private: Ogre::Material *matSecondPass;

      /// \brief An array of first pass textures.
      private: Ogre::Texture *firstPassTextures[3];

      /// \brief Second pass texture.
      private: Ogre::Texture *secondPassTexture;

      /// \brief First pass render targets.
      private: Ogre::RenderTarget *firstPassTargets[3];

      /// \brief Second pass render target.
      private: Ogre::RenderTarget *secondPassTarget;

      /// \brief First pass viewports.
      private: Ogre::Viewport *firstPassViewports[3];

      /// \brief Second pass viewport
      private: Ogre::Viewport *secondPassViewport;

      /// \brief Number of first pass textures.
      private: unsigned int textureCount;

      /// \brief A list of camera angles for first pass rendering.
      private: double cameraYaws[4];

      /// \brief Temporary pointer to the current render target.
      private: Ogre::RenderTarget *currentTarget;

      /// \brief Temporary pointer to the current material.
      private: Ogre::Material *currentMat;

      /// \brief Ogre orthorgraphic camera used in the second pass for
      /// undistortion.
      private: Ogre::Camera *orthoCam;

      /// \brief Ogre scenenode where the orthorgraphic camera is attached to.
      private: Ogre::SceneNode *pitchNodeOrtho;

      /// \brief Ogre mesh used to create a canvas for undistorting range values
      /// in the second rendering pass.
      private: common::Mesh *undistMesh;

      /// \brief Ogre movable object created from the canvas mesh.
      private: Ogre::MovableObject *object;

      /// \brief Pointer to visual that holds the canvas.
      private: VisualPtr visual;

      /// \brief Image width.
      private: unsigned int w2nd;

      /// \brief Image height.
      private: unsigned int h2nd;

      /// \brief Time taken to complete the two rendering passes.
      private: double lastRenderDuration;

      /// \brief List of texture unit indices used during the second
      /// rendering pass.
      private: std::vector<int> texIdx;

      /// Number of second pass texture units created.
      private: static int texCount;
    };
    /// \}
  }
}
#endif
