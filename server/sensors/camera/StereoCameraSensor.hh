/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
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

#include "OgreCamera.hh"
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
  /// \addtogroup gazebo_sensor
  /// \brief Stereo camera sensor
  /// \{
  /// \defgroup gazebo_stereo_camera Stereo Camera
  /// \brief Stereo camera sensor
  // \{
  
  
  /// \brief Stereo camera sensor
  ///
  /// This sensor is used for simulating a stereo camera.
  class StereoCameraSensor : public Sensor, public OgreCamera
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
  
    // Render Depth Data
    protected: virtual void RenderDepthData();
  
    /// \brief Fill all the image buffers
    private: void FillBuffers();
  
    private: Ogre::TexturePtr CreateRTT( const std::string &name, bool depth);
  
    //private: void UpdateAllDependentRenderTargets();
  
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

