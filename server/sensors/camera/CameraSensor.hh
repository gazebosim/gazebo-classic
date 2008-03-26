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
/* Desc: A persepective X11 OpenGL Camera Sensor
 * Author: Nate Koenig
 * Date: 15 July 2003
 * CVS: $Id$
 */

#ifndef CAMERASENSOR_HH
#define CAMERASENSOR_HH

#include <OgrePrerequisites.h>
#include <OgreTexture.h>

#include "Pose3d.hh"
#include "Sensor.hh"

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
/// \brief Basic camera sensor
/// \{
/// \defgroup gazebo_camera Camera
/// \brief Basic camera sensor
// \{


/// \brief Basic camera sensor
///
/// This sensor is used for simulating standard monocular cameras; is
/// is used by both camera models (e.g., SonyVID30) and user interface
/// models (e.g., ObserverCam).
class CameraSensor : public Sensor
{
  /// \brief Constructor
  public: CameraSensor(Body *body);

  /// \brief Destructor
  public: virtual ~CameraSensor();

  /// \brief Load the camera using parameter from an XMLConfig node
  /// \param node The XMLConfig node
  protected: virtual void LoadChild( XMLConfigNode *node );

  /// \brief Initialize the camera
  protected: virtual void InitChild();

  /// \brief Update the sensor information
  protected: virtual void UpdateChild(UpdateParams &params);

  /// Finalize the camera
  protected: virtual void FiniChild();

  /// \brief Get the global pose of the camera
  public: Pose3d GetWorldPose() const;

  /// \brief Return the material the camera renders to
  public: virtual std::string GetMaterialName() const = 0;

  /// \brief Translate the camera
  public: void Translate( const Vector3 &direction );

  /// \brief Rotate the camera around the yaw axis
  public: void RotateYaw( float angle );

  /// \brief Rotate the camera around the pitch axis
  public: void RotatePitch( float angle );

  /// \brief Get the camera FOV (horizontal)  
  public: double GetFOV() const;

  /// \brief Get the width of the image
  public: unsigned int GetImageWidth() const;

  /// \brief Get the width of the texture 
  public: unsigned int GetTextureWidth() const;

  /// \brief Get the height of the image
  public: unsigned int GetImageHeight() const;

  /// \brief Get the height of the texture 
  public: unsigned int GetTextureHeight() const;

  /// \brief Get a pointer to the image data
  public: virtual const unsigned char *GetImageData() = 0;

  /// \brief Get the image size in bytes
  public: size_t GetImageByteSize() const;

  /// \brief Get the Z-buffer value at the given image coordinate.
  ///
  /// \param x, y Image coordinate; (0, 0) specifies the top-left corner.
  /// \returns Image z value; note that this is abitrarily scaled and
  /// is @e not the same as the depth value.
  public: double GetZValue(int x, int y);

  /// \brief Enable or disable saving
  public: void EnableSaveFrame(bool enable);

  /// \brief Toggle saving of frames
  public: void ToggleSaveFrame();

  /// \brief Get a pointer to the ogre camera
  public: Ogre::Camera *GetOgreCamera() const;

  // Save the camera frame
  protected: virtual void SaveFrame() = 0;

  protected: double hfov;
  protected: double nearClip, farClip;
  protected: unsigned int imageWidth, imageHeight;
  protected: unsigned int textureWidth, textureHeight;

  protected: Ogre::Camera *camera;
  protected: Ogre::SceneNode *pitchNode;

  protected: Pose3d pose;

  //access to our visual node (convenience member)
  protected: Ogre::SceneNode *sceneNode;

  // Info for saving images
  protected: unsigned char *saveFrameBuffer;
  protected: unsigned int saveCount;
  protected: bool saveFrames;
  protected: std::string savePathname;
};

/// \}
/// \}
}
#endif

