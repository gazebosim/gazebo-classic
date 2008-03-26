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
 * SVN: $Id:$
 */

#ifndef STEREOCAMERASENSOR_HH
#define STEREOCAMERASENSOR_HH

#include <OgrePrerequisites.h>
#include <OgreTexture.h>

#include "CameraSensor.hh"

// Forward Declarations
namespace Ogre
{
  class TexturePtr;
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
class StereoCameraSensor : public CameraSensor
{

  /// \brief Constructor
  public: StereoCameraSensor(Body *body);

  /// \brief Destructor
  public: virtual ~StereoCameraSensor();

  /// \brief Load the camera using parameter from an XMLConfig node
  /// \param node The XMLConfig node
  protected: virtual void LoadChild( XMLConfigNode *node );

  /// \brief Initialize the camera
  protected: virtual void InitChild();

  /// \brief Update the sensor information
  protected: virtual void UpdateChild(UpdateParams &params);

  /// Finalize the camera
  protected: virtual void FiniChild();

  /// \brief Return the material the camera renders to
  public: virtual std::string GetMaterialName() const;

  /// \brief Get a pointer to the image data
  public: virtual const unsigned char *GetImageData();

  // Save the camera frame
  protected: virtual void SaveFrame();

  private: Ogre::TexturePtr leftRenderTexture;
  private: Ogre::RenderTarget *leftRenderTarget;
  private: Ogre::TexturePtr rightRenderTexture;
  private: Ogre::RenderTarget *rightRenderTarget;


  private: std::string leftOgreTextureName;
  private: std::string rightOgreTextureName;
  private: std::string ogreMaterialName;

  private: 
           class StereoCameraListener : public Ogre::RenderTargetListener
           {
             public: StereoCameraListener() : Ogre::RenderTargetListener() {}
             public: void Init(StereoCameraSensor *sensor, bool isLeft);
             public: void preViewportUpdate(const Ogre::RenderTargetViewportEvent &evt);
             public: void postViewportUpdate(const Ogre::RenderTargetViewportEvent &evt);

             private: Ogre::Vector3 pos;
             private: StereoCameraSensor *sensor;
             private: Ogre::Camera *camera;
             private: bool isLeftCamera;
           };


  private: StereoCameraListener leftCameraListener;
  private: StereoCameraListener rightCameraListener;
};

/// \}
/// \}
}
#endif

