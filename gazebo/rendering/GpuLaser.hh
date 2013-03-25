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
#include "gazebo/sensors/SensorTypes.hh"

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
                          Scene *_scene, bool _autoRender = true);

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

      /// \brief Set the parent sensor
      /// \param[in] _parent Pointer to a sensors::GpuRaySensor
      public: void SetParentSensor(sensors::GpuRaySensor *_parent);

      /// \internal
      /// \brief Implementation of Ogre::RenderObjectListener
      public: virtual void notifyRenderSingleObject(Ogre::Renderable *_rend,
              const Ogre::Pass *_p, const Ogre::AutoParamDataSource *_s,
              const Ogre::LightList *_ll, bool _supp);

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

      private: event::EventT<void(const float *, unsigned int, unsigned int,
                   unsigned int, const std::string &)> newLaserFrame;

      private: float *laserBuffer;
      private: float *laserScan;
      private: Ogre::Material *mat_1st_pass;
      private: Ogre::Material *mat_2nd_pass;

      private: Ogre::Texture *_1stPassTextures[3];
      private: Ogre::Texture *_2ndPassTexture;
      private: Ogre::RenderTarget *_1stPassTargets[3];
      private: Ogre::RenderTarget *_2ndPassTarget;
      private: Ogre::Viewport *_1stPassViewports[3];
      private: Ogre::Viewport *_2ndPassViewport;

      private: unsigned int _textureCount;
      private: double cameraYaws[4];

      private: Ogre::RenderTarget *current_target;
      private: Ogre::Material *current_mat;

      private: Ogre::Camera *orthoCam;

      private: Ogre::SceneNode *origParentNode_ortho;
      private: Ogre::SceneNode *pitchNode_ortho;

      private: common::Mesh *undist_mesh;

      private: Ogre::MovableObject *object;

      private: VisualPtr visual;

      private: unsigned int w2nd;
      private: unsigned int h2nd;

      private: sensors::GpuRaySensor *parent_sensor;
      private: double lastRenderDuration;

      private: std::vector<int> texIdx;
      private: static int texCount;
    };
    /// \}
  }
}
#endif
