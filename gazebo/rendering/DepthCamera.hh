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

#ifndef RENDERING_DEPTHCAMERA_HH
#define RENDERING_DEPTHCAMERA_HH
#include <string>

#include "rendering/Camera.hh"

#include "common/Event.hh"
#include "common/Time.hh"

#include "math/Angle.hh"
#include "math/Pose.hh"
#include "math/Vector2i.hh"

#include "sdf/sdf.hh"

namespace Ogre
{
  class Material;
  class RenderObjectListener;
  class Renderable;
  class Pass;
  class AutoParamDataSource;
}

namespace gazebo
{
  /// \ingroup gazebo_rendering
  /// \brief Rendering namespace
  namespace rendering
  {
    class MouseEvent;
    class ViewController;
    class Scene;

    /// \addtogroup gazebo_rendering Rendering
    /// \{

    /// \brief depth camera sensor, depends on rendering::Camera class
    class DepthCamera : public Camera
    {
      /// \brief Constructor
      public: DepthCamera(const std::string &_namePrefix,
                          Scene *_scene, bool _autoRender = true);

      /// \brief Destructor
      public: virtual ~DepthCamera();

      /// \brief Load the camera with a set of parmeters
      /// \param _sdf The SDF camera info
      public: void Load(sdf::ElementPtr &_sdf);

       /// \brief Load the camera with default parmeters
      public: void Load();

      /// \brief Initialize the camera
      public: void Init();

      /// Finalize the camera
      public: void Fini();

      public: void CreateDepthTexture(const std::string &_textureName);

      /// \brief Render the camera
      public: virtual void PostRender();

      // All things needed to get back z buffer for depth data
      public: virtual const float* GetDepthData();

      /// \brief Connect a to the add entity signal
      public: template<typename T>
              event::ConnectionPtr ConnectNewDepthFrame(T subscriber)
              { return newDepthFrame.Connect(subscriber); }
      public: void DisconnectNewDepthFrame(event::ConnectionPtr &c)
              { newDepthFrame.Disconnect(c); }
      private: virtual void RenderImpl();

      private: void UpdateRenderTarget(Ogre::RenderTarget *target,
               Ogre::Material *material, std::string matName);

      private: float *depthBuffer;

      private: Ogre::Material *depthMaterial;

      private: event::EventT<void(const float *, unsigned int, unsigned int,
                   unsigned int, const std::string &)> newDepthFrame;

      protected: Ogre::Texture *depthTexture;
      protected: Ogre::RenderTarget *depthTarget;
      public: virtual void SetDepthTarget(Ogre::RenderTarget *target);
      protected: Ogre::Viewport *depthViewport;

      private: bool output_points;

      /// \brief Connect a to the add entity signal
      public: template<typename T>
              event::ConnectionPtr ConnectNewRGBPointCloud(T subscriber)
              { return newRGBPointCloud.Connect(subscriber); }
      public: void DisconnectNewRGBPointCloud(event::ConnectionPtr &c)
              { newRGBPointCloud.Disconnect(c); }

      private: float *pcdBuffer;
      protected: Ogre::Viewport *pcdViewport;

      private: event::EventT<void(const float *, unsigned int, unsigned int,
                   unsigned int, const std::string &)> newRGBPointCloud;

      private: Ogre::Material *pcdMaterial;

      protected: Ogre::Texture *pcdTexture;
      protected: Ogre::RenderTarget *pcdTarget;
    };

    /// \}
  }
}
#endif



