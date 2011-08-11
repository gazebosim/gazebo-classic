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
/* Desc: A persepective OGRE Camera
 * Author: Nate Koenig
 * Date: 15 July 2003
 */

#ifndef RENDERING_CAMERA_HH
#define RENDERING_CAMERA_HH

#include <boost/enable_shared_from_this.hpp>

#include "common/Event.hh"
#include "common/Time.hh"

#include "math/Angle.hh"
#include "math/Pose.hh"
#include "math/Vector2i.hh"

#include "sdf/sdf.h"

// Forward Declarations
namespace Ogre
{
  class Texture;
  class RenderTarget;
  class Camera;
  class Viewport;
  class SceneNode;
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
    /// \brief A set of rendering related class, functions, and definitions
    /// \{

    /// \brief Basic camera sensor
    ///
    /// This is the base class for all cameras.
    class Camera : public boost::enable_shared_from_this<Camera>
    {
      /// \brief Constructor
      public: Camera(const std::string &namePrefix, Scene *scene);
    
      /// \brief Destructor
      public: virtual ~Camera();
    
      /// \brief Load the camera with a set of parmeters
      /// \param _sdf The SDF camera info
      public: void Load( sdf::ElementPtr &_sdf );

       /// \brief Load the camera with default parmeters
      public: void Load( );
  
      /// \brief Initialize the camera
      public: void Init();

      /// \brief Render the camera
      public: virtual void Render();
  
      /// \brief Post render
      public: virtual void PostRender();
  
      /// \brief Update the sensor information
      public: void Update();
    
      /// Finalize the camera
      public: void Fini();

      /// \brief Set the ID of the window this camera is rendering into.
      public: void SetWindowId( unsigned int windowId );

      /// \brief Get the ID of the window this camera is rendering into.
      public: unsigned int GetWindowId() const;

      /// \brief Set the scene this camera is viewing
      public: void SetScene( Scene *scene );
  
      /// \brief Set to true to enable rendering
      public: void SetRenderingEnabled(bool value);
  
      /// \brief Get whether the rendering is enabled
      public: bool GetRenderingEnabled() const;
  
  
      /// \brief Get the global pose of the camera
      public: math::Pose GetWorldPose();
  
      /// \brief Get the camera position in the world
      public: math::Vector3 GetWorldPosition() const;
  
      /// \brief Set the global pose of the camera
      public: void SetWorldPose(const math::Pose &pose);
  
      /// \brief Set the world position
      public: void SetWorldPosition(const math::Vector3 &pos);
  
      /// \brief Set the world orientation
      public: void SetWorldRotation(const math::Quaternion &quant);
  
    
      /// \brief Translate the camera
      public: void Translate( const math::Vector3 &direction );
    
      /// \brief Rotate the camera around the yaw axis
      public: void RotateYaw( float angle );
    
      /// \brief Rotate the camera around the pitch axis
      public: void RotatePitch( float angle );
  
  
  
      /// \brief Set the clip distances
      public: void SetClipDist(float near, float far);
  
      /// \brief Set the camera FOV (horizontal)  
      public: void SetFOV( float radians );
  
      /// \brief Get the camera FOV (horizontal)  
      public: math::Angle GetHFOV() const;
  
      /// \brief Get the camera FOV (vertical)  
      public: math::Angle GetVFOV() const;
    
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
  
      /// \brief Get a pointer to the ogre camera
      public: Ogre::Camera *GetOgreCamera() const;
      public: Ogre::Viewport *GetViewport() const;
  
      /// \brief Get the viewport width in pixels
      public: unsigned int GetViewportWidth() const;
  
      /// \brief Get the viewport height in pixels
      public: unsigned int GetViewportHeight() const;
  
      /// \brief Get the viewport up vector
      public: math::Vector3 GetUp();
  
      /// \brief Get the viewport right vector
      public: math::Vector3 GetRight();
  
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
  
      /// \brief Set the camera's scene node
      public: void SetSceneNode( Ogre::SceneNode *node );
  
      /// \brief Get the camera's scene node
      public: Ogre::SceneNode *GetSceneNode() const;
  
      /// \brief Get a pointer to the image data
      public: virtual const unsigned char *GetImageData(unsigned int i=0);
  
      /// \brief Get the camera's name
      public: std::string GetName() const;
  
      /// \brief Set the camera's name
      public: void SetName( const std::string &name );
             
      /// \brief Toggle whether to view the world in wireframe
      public: void ToggleShowWireframe();
  
      /// \brief Set whether to view the world in wireframe
      public: void ShowWireframe(bool s);

      /// \brief Get a world space ray as cast from the camer through the viewport
      public: void GetCameraToViewportRay(int screenx, int screeny,
                                          math::Vector3 &origin, math::Vector3 &dir);
  
          /// \brief Set whether to capture data
      public: void SetCaptureData( bool value );
  
      /// \brief Set the render target
      public: void CreateRenderTexture( const std::string &textureName );
  
      /// \brief Get the scene this camera is in
      public: Scene *GetScene() const;
  
      /// \brief Get point on a plane
      public: math::Vector3 GetWorldPointOnPlane(int x, int y, math::Vector3 planeNorm, double d);
  
      public: void SetRenderTarget( Ogre::RenderTarget *target );

      /// \brief if user requests bayer image, post process rgb from ogre to generate bayer formats
      private: void ConvertRGBToBAYER(unsigned char* dst, unsigned char* src, std::string format,int width, int height);
  
      // Save the camera frame
      protected: virtual void SaveFrame();
  
      // Create the ogre camera
      private: void CreateCamera();
  
      private: std::string name;
      private: math::Pose pose;
      protected: sdf::ElementPtr sdf;

      protected: unsigned int windowId;
  
      protected: unsigned int textureWidth, textureHeight;
    
      protected: Ogre::Camera *camera;
      protected: Ogre::Viewport *viewport;
      protected: Ogre::SceneNode *origParentNode;
      protected: Ogre::SceneNode *sceneNode;
      protected: Ogre::SceneNode *pitchNode;
    
    
      // Info for saving images
      protected: unsigned char *saveFrameBuffer;
      protected: unsigned char *bayerFrameBuffer;
      protected: unsigned int saveCount;
   
      protected: int imageFormat;
  
      protected: Ogre::RenderTarget *renderTarget;
  
      protected: Ogre::Texture *renderTexture;
  
      private: static unsigned int cameraCounter;
      private: unsigned int myCount;
  
      protected: std::string uniqueName;
  
      protected: bool captureData;
  
      private: bool userMovable;
      protected: bool renderingEnabled;
  
      private: bool newData;
  
      protected: common::Time renderPeriod;
      protected: common::Time lastUpdate;
  
      protected: Scene *scene;
  
      protected: std::vector<event::ConnectionPtr> connections;
      private: friend class Scene;
    };
    
    /// \}
  }
}
#endif

