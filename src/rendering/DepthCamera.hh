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

#include <boost/enable_shared_from_this.hpp>

#include "rendering/Camera.hh"

#include "common/Event.hh"
#include "common/Time.hh"

#include "math/Angle.hh"
#include "math/Pose.hh"
#include "math/Vector2i.hh"

#include "sdf/sdf.h"

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
    /// \brief A set of rendering related class, functions, and definitions
    /// \{

    /// \brief Basic camera sensor
    ///
    /// This is the base class for all cameras.
    class DepthCamera : public Camera , public boost::enable_shared_from_this<DepthCamera>
    {
      /// \brief Constructor
      public: DepthCamera(const std::string &namePrefix, Scene *scene);
    
      /// \brief Destructor
      public: virtual ~DepthCamera();
    
      /// \brief Load the camera with a set of parmeters
      /// \param _sdf The SDF camera info
      public: void Load( sdf::ElementPtr &_sdf );

       /// \brief Load the camera with default parmeters
      public: void Load( );
  
      /// \brief Initialize the camera
      public: void Init();

      /// \brief Update the sensor information
      public: void Update();
    
      /// Finalize the camera
      public: void Fini();

      // All things needed to get back z buffer for depth data
      public: virtual const float *GetDepthData(unsigned int i=0);
      protected: virtual void RenderDepthData();
      protected: Ogre::TexturePtr CreateRTT(const std::string &name, bool depth);
      protected: float *saveDepthBuffer;
      public: Ogre::RenderTarget *depthTarget;
      protected: Ogre::TexturePtr depthTexture;
      private: Ogre::MaterialPtr depthMaterial;
      protected: std::string depthTextureName;
      protected: std::string depthMaterialName;
      protected: bool simulateDepthData;
      private: void CreateDepthTexture( const std::string &textureName );
      public: virtual void PostRender();


    };
    
    /// \}
  }
}
#endif

