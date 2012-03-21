/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: A persepective OGRE Camera with Depth Sensor
 * Author: Nate Koenig
 * Date: 15 July 2003
 */

#ifndef RENDERING_VISUALLASER_HH
#define RENDERING_VISUALLASER_HH
#include <string>

#include "rendering/Camera.hh"
#include "OGRE/OgreRenderObjectListener.h"
#include "sensors/SensorTypes.hh"

#include "common/Event.hh"
#include "common/Time.hh"

#include "math/Angle.hh"
#include "math/Pose.hh"
#include "math/Vector2i.hh"

#include "sdf/sdf.h"

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
    /// \brief A set of rendering related class, functions, and definitions
    /// \{
    /// \brief Basic camera sensor
    ///
    /// This is the base class for all cameras.
    class VisualLaser : public Camera, public Ogre::RenderObjectListener
    {
      /// \brief Constructor
      public: VisualLaser(const std::string &_namePrefix,
                          Scene *_scene, bool _autoRender = true);

      /// \brief Destructor
      public: virtual ~VisualLaser();

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

      public: void SetParentSensor(sensors::VisualLaserSensor *parent);

      public: virtual void notifyRenderSingleObject (Ogre::Renderable *rend, const Ogre::Pass* /*pass*/, 
              const Ogre::AutoParamDataSource* /*source*/, const Ogre::LightList* /*lights*/, bool /*supp*/);

      private: virtual void RenderImpl();

      private: void UpdateRenderTarget(Ogre::RenderTarget *target, Ogre::Material *material, 
              Ogre::Camera *cam);

      private: void CreateOrthoCam();

      private: void CreateMesh();

      private: void CreateCanvas();

      private: Ogre::Matrix4 BuildScaledOrthoMatrix(float left, float right, float bottom, float top, float near, float far);

      private: float *laserBuffer;
      private: float *laserScan;
      private: unsigned char *imageBuffer;
      private: unsigned char *_1stBuffer;

      private: Ogre::Material *mat_1st_pass;
      private: Ogre::Material *mat_2nd_pass;

      private: event::EventT<void(const float *, unsigned int, unsigned int,
                   unsigned int, const std::string &)> newLaserFrame;

      private: void Publish1stTexture();
      private: void Publish2ndTexture();
      public: template<typename T>
              event::ConnectionPtr ConnectNewImage2Frame(T subscriber)
              { return newImage2Frame.Connect(subscriber); }
      public: void DisconnectNewImage2Frame(event::ConnectionPtr &c)
              { newImage2Frame.Disconnect(c); }

      private: event::EventT<void(const unsigned char *, unsigned int, unsigned int,
                   unsigned int, const std::string &)> newImage2Frame;

      protected: Ogre::Texture *_1stTexture;
      protected: Ogre::Texture *_2ndTexture;
      protected: Ogre::RenderTarget *_1stTarget;
      protected: Ogre::RenderTarget *_2ndTarget;
      protected: Ogre::Viewport *_1stViewport;
      protected: Ogre::Viewport *_2ndViewport;
      protected: Ogre::Texture *_1stTexture_dbg;
      protected: Ogre::Texture *_2ndTexture_dbg;
      protected: Ogre::RenderTarget *_1stTarget_dbg;
      protected: Ogre::RenderTarget *_2ndTarget_dbg;
      protected: Ogre::Viewport *_1stViewport_dbg;
      protected: Ogre::Viewport *_2ndViewport_dbg;
      private: Ogre::Material *mat_1st_pass_dbg;
      private: Ogre::Material *mat_2nd_pass_dbg;
      protected: virtual void Set1stTarget(Ogre::RenderTarget *target);
      protected: virtual void Set2ndTarget(Ogre::RenderTarget *target);

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

      protected: sensors::VisualLaserSensor *parent_sensor;
    };

    /// \}
  }
}
#endif



