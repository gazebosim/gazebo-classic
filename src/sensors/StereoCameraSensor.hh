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
/* Desc: Stereo Camera Sensor
 * Author: Nate Koenig
 * Date: 25 March 2008
 * SVN: $Id$
 */

#ifndef STEREOCAMERASENSOR_HH
#define STEREOCAMERASENSOR_HH

#include <OgrePrerequisites.h>
#include <OgreTexture.h>
#include <OgreMaterial.h>

#include "rendering/Camera.hh"
#include "Sensor.hh"

// Forward Declarations
namespace Ogre
{
  class RenderTarget;
  class Camera;
  class Viewport;
  class SceneNode;
}

namespace gazebo
{
 
  /// \brief Stereo camera sensor
  ///
  /// This sensor is used for simulating a stereo camera.
  class StereoCameraSensor : public Sensor, public Camera
  {
    enum Sides {LEFT, RIGHT, D_LEFT, D_RIGHT};
  
    /// \brief Constructor
    public: StereoCameraSensor(Body *body);
  
    /// \brief Destructor
    public: virtual ~StereoCameraSensor();
  
    /// \brief Load the camera using parameter from an XMLConfig node
    /// \param node The XMLConfig node
    protected: virtual void LoadChild( XMLConfigNode *node );
  
    /// \brief Save the sensor info in XML format
    protected: virtual void SaveChild(std::string &prefix, std::ostream &stream);

    /// \brief Initialize the camera
    protected: virtual void InitChild();
  
    /// \brief Update the sensor information
    protected: virtual void UpdateChild();
  
    /// Finalize the camera
    protected: virtual void FiniChild();
  
    /// \brief Return the material the camera renders to
    public: virtual std::string GetMaterialName() const;
  
    /// \brief Get a pointer to the image data
    /// \param i 0=left, 1=right
    public: virtual const unsigned char *GetImageData(unsigned int i=0);
  
    /// \brief Get a point to the depth data
    /// \param i 0=left, 1=right
    public: const float *GetDepthData(unsigned int i=0);
  
    /// \brief Get the baselien of the camera
    public: double GetBaseline() const;

    // Save the camera frame
    protected: virtual void SaveFrame();
  
    /// \brief Fill all the image buffers
    private: void FillBuffers();
  
    private: Ogre::TexturePtr CreateRTT( const std::string &name, bool depth);
  
    //private: void UpdateAllDependentRenderTargets();
  
    private: Ogre::TexturePtr renderTexture[4];
    private: Ogre::RenderTarget *renderTargets[4];
    private: Ogre::MaterialPtr depthMaterial;
  
    private: std::string textureName[4];
    private: std::string materialName[4];
  
    private: unsigned int depthBufferSize;
    private: unsigned int rgbBufferSize;
    private: float *depthBuffer[2];
    private: unsigned char *rgbBuffer[2];
    private: double baseline;
  
    private: Ogre::Camera *depthCamera;

    /*private: 
             class StereoCameraListener : public Ogre::RenderTargetListener
             {
               public: StereoCameraListener() : Ogre::RenderTargetListener() {}
  
               public: void Init(StereoCameraSensor *sensor, Ogre::RenderTarget *target, bool isLeft);
               public: void preViewportUpdate(const Ogre::RenderTargetViewportEvent &evt);
               public: void postViewportUpdate(const Ogre::RenderTargetViewportEvent &evt);
  
               private: Ogre::Vector3 pos;
               private: StereoCameraSensor *sensor;
               private: Ogre::Camera *camera;
               private: Ogre::RenderTarget *renderTarget;
               private: bool isLeftCamera;
             };
  
  
    private: StereoCameraListener leftCameraListener;
    private: StereoCameraListener rightCameraListener;
    */
  };

/// \}
/// \}
}
#endif

