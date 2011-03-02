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
/* Desc: A persepective OGRE Camera
 * Author: Nate Koenig
 * Date: 15 July 2003
 * CVS: $Id$
 */

#ifndef CAMERASENSOR_HH
#define CAMERASENSOR_HH

#include <OgrePrerequisites.h>
#include <OgreTexture.h>

#include "Param.hh"
#include "Angle.hh"
#include "Pose3d.hh"

// Forward Declarations
namespace Ogre
{
  class TexturePtr;
  class RenderTarget;
  class Camera;
  class Viewport;
  class SceneNode;
  class AnimationState;
}

namespace gazebo
{
  class XMLConfigNode;
  class Model;
  class Entity;
  class MouseEvent;
  class ViewController;

  /// \addtogroup gazebo_rendering
  /// \brief Basic camera 
  /// \{
  /// \defgroup gazebo_camera OgreCamera
  /// \brief Basic camera sensor
  // \{
  
  
  /// \brief Basic camera sensor
  ///
  /// This is the base class for all cameras.
  class OgreCamera 
  {
    /// \brief Constructor
    public: OgreCamera(const std::string &namePrefix);
  
    /// \brief Destructor
    public: virtual ~OgreCamera();
  
    /// \brief Load the camera using parameter from an XMLConfig node
    /// \param node The XMLConfig node
    public: void LoadCam( XMLConfigNode *node );

    /// \brief Save camera info in xml format
    /// \param stream Output stream
    public: void SaveCam(std::string &prefix, std::ostream &stream);
  
    /// \brief Initialize the camera
    public: void InitCam();

    /// \brief Render the camera
    public: void Render();

    public: void CaptureData();

    /// \brief Update the sensor information
    public: void UpdateCam();
  
    /// Finalize the camera
    public: void FiniCam();

    /// \brief Set to true to enable rendering
    public: void SetRenderingEnabled(bool value);

    /// \brief Get whether the rendering is enabled
    public: bool GetRenderingEnabled() const;
  
    /// \brief Get the global pose of the camera
    public: Pose3d GetCameraWorldPose() const;

    /// \brief Set the global pose of the camera
    public: void SetWorldPose(const Pose3d &pose);

    /// \brief Set the world position
    public: void SetPosition(const Vector3 &pos);
  
    /// \brief Translate the camera
    public: void Translate( const Vector3 &direction );
  
    /// \brief Rotate the camera around the yaw axis
    public: void RotateYaw( float angle );
  
    /// \brief Rotate the camera around the pitch axis
    public: void RotatePitch( float angle );

    /// \brief Set the clip distances
    public: void SetClipDist(float near, float far);

    /// \brief Set the camera FOV (horizontal)  
    public: void SetFOV( float radians );

    /// \brief Get the camera FOV (horizontal)  
    public: Angle GetHFOV() const;

    /// \brief Get the camera FOV (vertical)  
    public: Angle GetVFOV() const;
  
    /// \brief Get the width of the image
    public: unsigned int GetImageWidth() const;
  
    /// \brief Get the width of the texture 
    public: unsigned int GetTextureWidth() const;
  
    /// \brief Get the height of the image
    public: unsigned int GetImageHeight() const;
  
    /// \brief Get the height of the image
    public: int GetImageDepth() const;

    /// \brief Get the height of the image
    public: std::string GetImageFormat() const;

    /// \brief Get the height of the texture 
    public: unsigned int GetTextureHeight() const;
  
    /// \brief Get the image size in bytes
    public: size_t GetImageByteSize() const;
  
    /// \brief Get the Z-buffer value at the given image coordinate.
    ///
    /// \param x, y Image coordinate; (0, 0) specifies the top-left corner.
    /// \returns Image z value; note that this is abitrarily scaled and
    /// is @e not the same as the depth value.
    public: double GetZValue(int x, int y);
  
    /// \brief Get the near clip distance
    public: double GetNearClip();
  
    /// \brief Get the far clip distance
    public: double GetFarClip();
  
    /// \brief Enable or disable saving
    public: void EnableSaveFrame(bool enable);
 
    /// \brief Set the save frame pathname
    public: void SetSaveFramePathname(const std::string &pathname);

    /// \brief Toggle saving of frames
    public: void ToggleSaveFrame();
  
    /// \brief Get a pointer to the ogre camera
    public: Ogre::Camera *GetOgreCamera() const;

    /// \brief Get the viewport width in pixels
    public: unsigned int GetViewportWidth() const;

    /// \brief Get the viewport height in pixels
    public: unsigned int GetViewportHeight() const;

    /// \brief Get the viewport up vector
    public: Vector3 GetUp();

    /// \brief Get the viewport right vector
    public: Vector3 GetRight();

    /// \brief Get the camera position in the world
    public: Vector3 GetCameraPosition();

    /// \brief Get the average FPS
    public: virtual float GetAvgFPS() { return 0;}

    /// \brief Get the triangle count
    public: virtual unsigned int GetTriangleCount() {return 0;}

    /// \brief Set the aspect ratio
    public: void SetAspectRatio( float ratio );

    /// \brief Set whether the user can move the camera via the GUI
    public: void SetUserMovable( bool movable );

    /// \brief Get whether the user can move the camera via the GUI
    public: bool GetUserMovable() const;

    /// \brief Get the name of the camera
    public: std::string GetCameraName();

    /// \brief Set the camera's scene node
    public: void SetCameraSceneNode( Ogre::SceneNode *node );

    /// \brief Get a pointer to the image data
    public: virtual const unsigned char *GetImageData(unsigned int i=0);

    /// \brief Get the camera's name
    public: std::string GetCamName();

    /// \brief Set the camera's name
    public: void SetCamName( const std::string &name );

    /// \brief Move the camera to focus on an entity
    public: void MoveToEntity(Entity *entity);

    /// \brief Set the camera to track an entity
    public: void TrackModel( Model *model );

    /// \brief Set whether to view the world in wireframe
    public: void ShowWireframe(bool s);

    /// \brief Get a world space ray as cast from the camer through the viewport
    public: void GetCameraToViewportRay(int screenx, int screeny,
                                        Vector3 &origin, Vector3 &dir);

    /// \brief Frame started
    public: void FrameStarted(double timeSinceLastFrame);

    /// \brief Get the direction the camera is facing
    public: Vector3 GetDirection() const;

    /// Set the direction of the camera
    public: void SetDirection(Vector3 vec);

    /// \brief Hande a mouse event
    public: void HandleMouseEvent(const MouseEvent &evt);

    /// \brief Get the time of the last render update
    public: gazebo::Time GetLastRenderTime() const; 

    /// \brief if user requests bayer image, post process rgb from ogre to generate bayer formats
    private: void ConvertRGBToBAYER(unsigned char* dst, unsigned char* src, std::string format,int width, int height);


    // Save the camera frame
    protected: virtual void SaveFrame();

    // Render Depth Data
    protected: virtual void RenderDepthData();

    /// \brief set update rate (render rate) of the OgreCamera
    public: void SetUpdateRate(const double &rate);

    private: std::string name;

    protected: ParamT<Angle> *hfovP;
    protected: ParamT<double> *nearClipP, *farClipP, *updateRateP;
    protected: ParamT< Vector2<int> > *imageSizeP;
    protected: unsigned int textureWidth, textureHeight;
  
    protected: Ogre::Camera *camera;
    protected: Ogre::SceneNode *origParentNode;
    protected: Ogre::SceneNode *sceneNode;
    public: Ogre::SceneNode *pitchNode;
  
    private: Pose3d pose;
  
    // Info for saving images
    protected: unsigned char *saveFrameBuffer;
    protected: unsigned char *bayerFrameBuffer;
    protected: unsigned int saveCount;
    protected: ParamT<bool> *saveFramesP;
    protected: ParamT<std::string> *savePathnameP;
    protected: ParamT<std::string> *imageFormatP;
 
    protected: ParamT<std::string> *visMaskP;
    protected: Ogre::PixelFormat imageFormat;
    protected: unsigned int visibilityMask;

    public: Ogre::RenderTarget *renderTarget;

    protected: Ogre::TexturePtr renderTexture;

    protected: std::string ogreTextureName;
    protected: std::string ogreMaterialName;

    private: static unsigned int cameraCounter;
    private: unsigned int myCount;

    protected: std::string cameraName;

    protected: bool captureData;
    protected: bool simulateDepthData;

    private: bool userMovable;
    protected: std::vector<Param*> camParameters;

    protected: bool renderingEnabled;

    protected: Time renderPeriod;
    protected: Time lastUpdate;
    private: Ogre::AnimationState *animState;

    private: ViewController *viewController;

    public: Time GetLastRenderTime() {return this->lastRenderTime;};
    protected: Time lastRenderTime;

  };
  
  /// \}
  /// \}
}
#endif

