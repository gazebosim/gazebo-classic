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
 * CVS: $Id: Camera.hh,v 1.1.2.1 2006/12/16 22:41:17 natepak Exp $
 */

#ifndef CAMERA_HH
#define CAMERA_HH

#include <OGRE/Ogre.h>

// Forward Declarations
namespace Ogre
{
  class TexturePtr;
  class RenderTarget;
  class Camera;
  class Viewport;
}

/// @brief Basic camera sensor
///
/// This sensor is used for simulating standard monocular cameras; is
/// is used by both camera models (e.g., SonyVID30) and user interface
/// models (e.g., ObserverCam).
class Camera
{
  // Constructor
  public: Camera();

  // Destructor
  public: virtual ~Camera();

  /// @brief Initialize the sensor.
  /// @param width, height image width and height (pixels)
  /// @param hfov horizontal field of view (radians)
  /// @param minDepth, maxDepth near and far depth clipping planes
  ///   (m); minDepth must be greater than zero; maxDepth must be
  ///   greater than minDepth.
  /// @param method Prefered rendering method: SGIX, GLX or XLIB.
  ///  @param zBufferDepth Z buffer depth (in bits) used for rendering (useful
  ///  for some  nvidia cards that do not support other depths than 24 bits)
  /// @returns Zero on success.
  public: int Init(int width, int height, double hfov,
                   double minDepth, double maxDepth, int zBufferDepth);

  /// @brief Finalize the sensor
  public: int Fini();

  /// @brief Update the sensor information
  public: void Update();

  /// @brief Translate the camera
  public: void Translate( const Ogre::Vector3 &direction );

  /// @brief Rotate the camera around the yaw axis
  public: void RotateYaw( float angle );

  /// @brief Rotate the camera around the pitch axis
  public: void RotatePitch( float angle );

  /// @brief Set the pose of the camera (global cs)
  //public: void SetPose(GzPose pose);

  /// @brief Get the camera pose (global cs)
  //public: GzPose GetPose();

  /// @brief Set the camera FOV (horizontal)
  public: void SetFOV(double fov);

  /// @brief Get the camera FOV (horizontal)  
  public: double GetFOV() const;

  /// @brief Get the image dimensions
  public: void GetImageSize(int *w, int *h);

  /// @brief Get a pointer to the image data
  public: const unsigned char *GetImageData();

  /// @brief Get the Z-buffer value at the given image coordinate.
  ///
  /// @param x, y Image coordinate; (0, 0) specifies the top-left corner.
  /// @returns Image z value; note that this is abitrarily scaled and
  /// is @e not the same as the depth value.
  public: double GetZValue(int x, int y);

  /// @brief Compute the change in pose based on two image points.
  ///
  /// This function provides natural feedback when using a mouse to
  /// control the camera pose.  The function computes a change in
  /// camera pose, such that the initial image coordinate a and final
  /// coordinate b map both to the same @e global coordinate.  Thus,
  /// it is possible to "grab" a piece of the terrain and "drag" it to
  /// a new location.
  ///
  /// Naturally, with only two image coordinates the solution is
  /// under-determined (4 constraints and 6 DOF).  We therefore
  /// provide a mode argument specify what kind of transformation
  /// should be affected; the supported modes are translating, zooming
  /// and rotating.
  ///
  /// @param mode Solution method: 0 = translate; 1 = zoom; 2 = rotate.
  /// @param a, b Initial and final image points; the z value on a must be
  /// specified (use GetZValue() for this).
  /// @returns Change in pose (camera frame; post-multiply with
  /// current global pose).
//  public: GzPose CalcCameraDelta(int mode, GzVector a, GzVector b);
  
  // Render the scene from the camera perspective
  private: void Render();

  /// @brief Set the path for saved frames
  public: void SetSavePath(const char *pathname);

  /// @brief Enable or disable saving
  public: void EnableSaveFrame(bool enable);

  // Camera pose
  //private: GzPose cameraPose;

  // Save the camera frame
  private: void SaveFrame();

  // Info for saving images
  private: bool saveEnable;
  private: const char *savePathname;
  private: unsigned int saveCount;

  private: double hfov;
  private: double nearClip, farClip;
  private: int imageWidth, imageHeight;

  private: Ogre::TexturePtr renderTexture;
  private: Ogre::RenderTarget *renderTarget;
  private: Ogre::Viewport *viewport;

  private: Ogre::Camera *camera;
  private: Ogre::SceneNode *translateYawNode;
  private: Ogre::SceneNode *pitchNode;

};

#endif

