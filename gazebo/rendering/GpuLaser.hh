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

#ifndef RENDERING_GPULASER_HH
#define RENDERING_GPULASER_HH
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

  /// \ingroup gazebo_rendering
  /// \brief Rendering namespace
  namespace rendering
  {
    class MouseEvent;
    class ViewController;
    class Scene;

    /// \addtogroup gazebo_rendering Rendering
    /// \{

    /// \brief GPU based laser distance sensor
    ///
    /// This is the base class for all cameras.
    class GpuLaser : public Camera, public Ogre::RenderObjectListener
    {
      /// \brief Constructor
      public: GpuLaser(const std::string &_namePrefix,
                          Scene *_scene, bool _autoRender = true);

      /// \brief Destructor
      public: virtual ~GpuLaser();

      /// \brief Load the camera with a set of parmeters
      /// \param _sdf The SDF camera info
      public: void Load(sdf::ElementPtr &_sdf);

       /// \brief Load the camera with default parmeters
      public: void Load();

      /// \brief Initialize the camera
      public: void Init();

      /// Finalize the camera
      public: void Fini();

      public: void CreateLaserTexture(const std::string &_textureName);

      /// \brief Render the camera
      public: virtual void PostRender();

      // All things needed to get back z buffer for laser data
      public: virtual const float* GetLaserData();

      /// \brief Connect a to the add entity signal
      public: template<typename T>
              event::ConnectionPtr ConnectNewLaserFrame(T subscriber)
              { return newLaserFrame.Connect(subscriber); }
      public: void DisconnectNewLaserFrame(event::ConnectionPtr &c)
              { newLaserFrame.Disconnect(c); }

      public: void SetRangeCount(unsigned int _w, unsigned int _h = 1);

      public: void SetParentSensor(sensors::GpuRaySensor *parent);

      public: virtual void notifyRenderSingleObject(Ogre::Renderable *rend,
              const Ogre::Pass* /*p*/, const Ogre::AutoParamDataSource* /*s*/,
              const Ogre::LightList* /*ll*/, bool /*supp*/);

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

      private: float *laserBuffer;
      private: float *laserScan;
      private: Ogre::Material *mat_1st_pass;
      private: Ogre::Material *mat_2nd_pass;

      private: event::EventT<void(const float *, unsigned int, unsigned int,
                   unsigned int, const std::string &)> newLaserFrame;

      protected: Ogre::Texture *_1stPassTextures[3];
      protected: Ogre::Texture *_2ndPassTexture;
      protected: Ogre::RenderTarget *_1stPassTargets[3];
      protected: Ogre::RenderTarget *_2ndPassTarget;
      protected: Ogre::Viewport *_1stPassViewports[3];
      protected: Ogre::Viewport *_2ndPassViewport;

      protected: unsigned int _textureCount;
      protected: double cameraYaws[4];

      protected: virtual void Set1stPassTarget(Ogre::RenderTarget *target,
                  unsigned int index);

      protected: virtual void Set2ndPassTarget(Ogre::RenderTarget *target);

      protected: Ogre::RenderTarget *current_target;
      private: Ogre::Material *current_mat;

      protected: Ogre::Camera *orthoCam;

      protected: Ogre::SceneNode *origParentNode_ortho;
      protected: Ogre::SceneNode *pitchNode_ortho;

      protected: common::Mesh *undist_mesh;

      protected: Ogre::MovableObject *object;

      protected: VisualPtr visual;

      protected: unsigned int w2nd;
      protected: unsigned int h2nd;

      protected: sensors::GpuRaySensor *parent_sensor;
      protected: double lastRenderDuration;

      protected: std::vector<int> texIdx;
      protected: static int texCount;
    };

    /// \}
  }
}
#endif



