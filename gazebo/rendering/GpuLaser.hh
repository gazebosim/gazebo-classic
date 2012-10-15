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
/* Desc: A laser sensor using OpenGL
 * Author: Mihai Emanuel Dolha
 * Date: 29 March 2012
 */

#ifndef _GPULASER_HH_
#define _GPULASER_HH_

#include <string>
#include <vector>

#include "rendering/Camera.hh"
#include "OGRE/OgreRenderObjectListener.h"
#include "sensors/SensorTypes.hh"

#include "common/Event.hh"
#include "common/Time.hh"

#include "math/Angle.hh"
#include "math/Pose.hh"
#include "math/Vector2i.hh"

#include "sdf/sdf.hh"

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
    ///
    /// This is the base class for all cameras.
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

      /// \brief Load the camera with a set of parmeters
      /// \param[in] _sdf The SDF camera info
      public: void Load(sdf::ElementPtr &_sdf);

       /// \brief Load the camera with default parmeters
      public: void Load();

      /// \brief Initialize the camera
      public: void Init();

      /// \brief Finalize the camera
      public: void Fini();

      /// \brief Create the texture which is used to render laser data
      /// \param[in] _textureName Name of the new texture
      public: void CreateLaserTexture(const std::string &_textureName);

      /// \brief Render the camera
      public: virtual void PostRender();

      /// \brief All things needed to get back z buffer for laser data
      /// \return Array of laser data
      public: virtual const float *GetLaserData();

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

      /// \brief Implementation of the render function
      private: virtual void RenderImpl();

      private: void UpdateRenderTarget(Ogre::RenderTarget *target,
                                       Ogre::Material *material,
                                       Ogre::Camera *cam,
                                       bool updateTex = false);

      private: void CreateOrthoCam();

      private: void CreateMesh();

      private: void CreateCanvas();

      private: Ogre::Matrix4 BuildScaledOrthoMatrix(float left, float right,
               float bottom, float top, float near, float far);

      private: virtual void Set1stPassTarget(Ogre::RenderTarget *target,
                  unsigned int index);

      private: virtual void Set2ndPassTarget(Ogre::RenderTarget *target);

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
