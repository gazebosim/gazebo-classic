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
/* Desc: A persepective OGRE Camera with Depth Sensor
 * Author: Nate Koenig
 * Date: 15 July 2003
 */

#ifndef _RENDERING_DEPTHCAMERA_HH_
#define _RENDERING_DEPTHCAMERA_HH_
#include <string>

#include <sdf/sdf.hh>

#include "gazebo/common/Event.hh"
#include "gazebo/common/Time.hh"

#include "gazebo/math/Angle.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/math/Vector2i.hh"

#include "gazebo/rendering/Camera.hh"
#include "gazebo/util/system.hh"

namespace Ogre
{
  class Material;
  class RenderTarget;
  class Texture;
  class Viewport;
}

namespace gazebo
{
  namespace rendering
  {
    class Scene;

    /// \addtogroup gazebo_rendering Rendering
    /// \{

    /// \class DepthCamera DepthCamera.hh rendering/rendering.hh
    /// \brief Depth camera used to render depth data into an image buffer
    class GZ_RENDERING_VISIBLE DepthCamera : public Camera
    {
      /// \brief Constructor
      /// \param[in] _namePrefix Unique prefix name for the camera.
      /// \param[in] _scene Scene that will contain the camera
      /// \param[in] _autoRender Almost everyone should leave this as true.
      public: DepthCamera(const std::string &_namePrefix,
                          ScenePtr _scene, bool _autoRender = true);

      /// \brief Destructor
      public: virtual ~DepthCamera();

      /// \brief Load the camera with a set of parmeters
      /// \param[in] _sdf The SDF camera info
      public: void Load(sdf::ElementPtr _sdf);

       /// \brief Load the camera with default parmeters
      public: void Load();

      /// \brief Initialize the camera
      public: void Init();

      /// Finalize the camera
      public: void Fini();

      /// \brief Create a texture which will hold the depth data
      /// \param[in] _textureName Name of the texture to create
      public: void CreateDepthTexture(const std::string &_textureName);

      /// \brief Render the camera
      public: virtual void PostRender();

      /// \brief All things needed to get back z buffer for depth data
      /// \return The z-buffer as a float array
      public: virtual const float *GetDepthData();

      /// \brief Set the render target, which renders the depth data
      /// \param[in] _target Pointer to the render target
      public: virtual void SetDepthTarget(Ogre::RenderTarget *_target);

      /// \brief Connect a to the new depth image signal
      /// \param[in] _subscriber Subscriber callback function
      /// \return Pointer to the new Connection. This must be kept in scope
      public: template<typename T>
              event::ConnectionPtr ConnectNewDepthFrame(T _subscriber)
              { return newDepthFrame.Connect(_subscriber); }

      /// \brief Disconnect from an depth image singal
      /// \param[in] _c The connection to disconnect
      public: void DisconnectNewDepthFrame(event::ConnectionPtr &_c)
              { newDepthFrame.Disconnect(_c); }

      /// \brief Connect a to the new rgb point cloud signal
      /// \param[in] _subscriber Subscriber callback function
      /// \return Pointer to the new Connection. This must be kept in scope
      public: template<typename T>
              event::ConnectionPtr ConnectNewRGBPointCloud(T _subscriber)
              { return newRGBPointCloud.Connect(_subscriber); }

      /// \brief Disconnect from an rgb point cloud singal
      /// \param[in] _c The connection to disconnect
      public: void DisconnectNewRGBPointCloud(event::ConnectionPtr &c)
              { newRGBPointCloud.Disconnect(c); }


      /// \brief Implementation of the render call
      private: virtual void RenderImpl();

      /// \brief Update a render target
      /// \param[in] _target Render target to update
      /// \param[in] _material Material to use
      /// \param[in] _matName Material name
      private: void UpdateRenderTarget(Ogre::RenderTarget *_target,
                                       Ogre::Material *_material,
                                       const std::string &_matName);

      /// \brief Pointer to the depth texture
      protected: Ogre::Texture *depthTexture;

      /// \brief Pointer to the depth target
      protected: Ogre::RenderTarget *depthTarget;

      /// \brief Pointer to the depth viewport
      protected: Ogre::Viewport *depthViewport;

      /// \brief The depth buffer
      private: float *depthBuffer;

      /// \brief The depth material
      private: Ogre::Material *depthMaterial;

      /// \brief True to generate point clouds
      private: bool outputPoints;

      /// \brief Point cloud data buffer
      private: float *pcdBuffer;

      /// \brief Point cloud view port
      private: Ogre::Viewport *pcdViewport;

      /// \brief Point cloud material
      private: Ogre::Material *pcdMaterial;

      /// \brief Point cloud texture
      private: Ogre::Texture *pcdTexture;

      /// \brief Point cloud texture
      private: Ogre::RenderTarget *pcdTarget;

      /// \brief Event used to signal rgb point cloud data
      private: event::EventT<void(const float *, unsigned int, unsigned int,
                   unsigned int, const std::string &)> newRGBPointCloud;

      /// \brief Event used to signal depth data
      private: event::EventT<void(const float *, unsigned int, unsigned int,
                   unsigned int, const std::string &)> newDepthFrame;
    };
    /// \}
  }
}
#endif
