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

#ifndef _RENDERING_CAMERA_HH_
#define _RENDERING_CAMERA_HH_

#include <boost/enable_shared_from_this.hpp>
#include <string>
#include <list>
#include <vector>

#include "common/Event.hh"
#include "common/Time.hh"

#include "math/Angle.hh"
#include "math/Pose.hh"
#include "math/Plane.hh"
#include "math/Vector2i.hh"

#include "msgs/MessageTypes.hh"
#include "rendering/RenderTypes.hh"
#include "sdf/sdf.hh"

// Forward Declarations
namespace Ogre
{
  class Texture;
  class RenderTarget;
  class Camera;
  class Viewport;
  class SceneNode;
  class AnimationState;
  class CompositorInstance;
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
      public: Camera(const std::string &namePrefix, Scene *scene,
                     bool _autoRender = true);

      /// \brief Destructor
      public: virtual ~Camera();

      /// \brief Load the camera with a set of parmeters
      /// \param _sdf The SDF camera info
      public: void Load(sdf::ElementPtr _sdf);

       /// \brief Load the camera with default parmeters
      public: void Load();

      /// \brief Initialize the camera
      public: void Init();

      /// \brief Set the render Hz rate
      public: void SetRenderRate(double _hz);

      /// \brief Render the camera
      public: void Render();
      protected: virtual void RenderImpl();

      /// \brief Post render
      public: virtual void PostRender();

      /// \brief Update the sensor information
      public: virtual void Update();

      /// Finalize the camera
      public: void Fini();

      public: inline bool IsInitialized() const {return this->initialized;}

      /// \brief Set the ID of the window this camera is rendering into.
      public: void SetWindowId(unsigned int windowId);

      /// \brief Get the ID of the window this camera is rendering into.
      public: unsigned int GetWindowId() const;

      /// \brief Set the scene this camera is viewing
      public: void SetScene(Scene *scene);

      /// \brief Get the global pose of the camera
      public: math::Pose GetWorldPose();

      /// \brief Get the camera position in the world
      public: math::Vector3 GetWorldPosition() const;

      /// \brief Get the camera's orientation in the world
      public: math::Quaternion GetWorldRotation() const;

      /// \brief Set the global pose of the camera
      public: virtual void SetWorldPose(const math::Pose &_pose);

      /// \brief Set the world position
      public: void SetWorldPosition(const math::Vector3 &_pos);

      /// \brief Set the world orientation
      public: void SetWorldRotation(const math::Quaternion &_quant);


      /// \brief Translate the camera
      public: void Translate(const math::Vector3 &direction);

      /// \brief Rotate the camera around the yaw axis
      public: void RotateYaw(float angle);

      /// \brief Rotate the camera around the pitch axis
      public: void RotatePitch(float angle);

      /// \brief Set the clip distances
      public: void SetClipDist(float near, float far);

      public: void SetClipDist();

      /// \brief Set the camera FOV (horizontal)
      public: void SetHFOV(float radians);

      /// \brief Get the camera FOV (horizontal)
      public: math::Angle GetHFOV() const;

      /// \brief Get the camera FOV (vertical)
      public: math::Angle GetVFOV() const;

      /// \brief Set the image size
      public: void SetImageSize(unsigned int _w, unsigned int _h);

      /// \brief Set the image height
      public: void SetImageWidth(unsigned int _w);

      /// \brief Set the image height
      public: void SetImageHeight(unsigned int _h);

      /// \brief Get the width of the image
      public: unsigned int GetImageWidth() const;

      /// \brief Get the width of the texture
      public: unsigned int GetTextureWidth() const;

      /// \brief Get the height of the image
      public: unsigned int GetImageHeight() const;

      /// \brief Get the height of the image
      public: unsigned int GetImageDepth() const;

      /// \brief Get the height of the image
      public: std::string GetImageFormat() const;

      /// \brief Get the height of the texture
      public: unsigned int GetTextureHeight() const;

      /// \brief Get the image size in bytes
      public: size_t GetImageByteSize() const;

      public: static size_t GetImageByteSize(unsigned int _width,
                                      unsigned int _height,
                                      const std::string &_format);

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

      /// \brief Save the last frame to disk
      public: bool SaveFrame(const std::string &_filename);

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
      public: void SetAspectRatio(float ratio);

      /// \brief Get the apect ratio
      public: float GetAspectRatio() const;

      /// \brief Set the camera's scene node
      public: void SetSceneNode(Ogre::SceneNode *node);

      /// \brief Get the camera's scene node
      public: Ogre::SceneNode *GetSceneNode() const;

      /// \brief Get a pointer to the image data
      public: virtual const unsigned char *GetImageData(unsigned int i = 0);

      /// \brief Get the camera's name
      public: std::string GetName() const;

      /// \brief Set the camera's name
      public: void SetName(const std::string &name);

      /// \brief Toggle whether to view the world in wireframe
      public: void ToggleShowWireframe();

      /// \brief Set whether to view the world in wireframe
      public: void ShowWireframe(bool s);

      /// \brief Get a world space ray as cast from the camera
      ///        through the viewport
      public: void GetCameraToViewportRay(int screenx, int screeny,
                                          math::Vector3 &origin,
                                          math::Vector3 &dir);

      /// \brief Set whether to capture data
      public: void SetCaptureData(bool _value);

      /// \brief Set the render target
      public: void CreateRenderTexture(const std::string &textureName);

      /// \brief Get the scene this camera is in
      public: Scene *GetScene() const;

      /// \brief Get point on a plane
      /// \return True if a valid point was found
      public: bool GetWorldPointOnPlane(int _x, int _y,
                  const math::Plane &_plane, math::Vector3 &_result);

      public: virtual void SetRenderTarget(Ogre::RenderTarget *target);

      /// \brief Attach the camera to a scene node
      public: void AttachToVisual(const std::string &_visualName,
                  bool _inheritOrientation,
                  double _minDist = 0.0, double _maxDist = 0.0);

      /// \brief Set the camera to track a scene node
      public: void TrackVisual(const std::string &_visualName);

      /// \brief Get the render texture
      public: Ogre::Texture *GetRenderTexture() const;

      /// \brief Get the camera's direction vector
      public: math::Vector3 GetDirection() const;

      /// \brief Connect a to the add entity signal
      public: template<typename T>
              event::ConnectionPtr ConnectNewImageFrame(T subscriber)
              { return newImageFrame.Connect(subscriber); }
      public: void DisconnectNewImageFrame(event::ConnectionPtr &c)
              { newImageFrame.Disconnect(c); }

      public: static bool SaveFrame(const unsigned char *_image,
                  unsigned int _width, unsigned int _height, int _depth,
                  const std::string &_format,
                  const std::string &_filename);


      /// \brief Get the last time the camera was rendered
      public: common::Time GetLastRenderWallTime();

      /// \brief Return true if the visual is within the camera's view
      ///        frustum
      public: bool IsVisible(VisualPtr _visual);

      /// \brief Return true if the visual is within the camera's view
      ///        frustum
      public: bool IsVisible(const std::string &_visualName);

      /// \brief Returns true if initialized
      public: bool GetInitialized() const;

      public: bool MoveToPosition(const math::Vector3 &_end,
                                  double _pitch, double _yaw, double _time);

      public: bool MoveToPositions(const std::vector<math::Pose> &_pts,
                                   double _time,
                                   boost::function<void()> _onComplete = NULL);


      protected: bool TrackVisualImpl(const std::string &_visualName);

      /// \brief Set the camera to track a scene node
      protected: virtual bool TrackVisualImpl(VisualPtr _visual);

      /// \brief Attach the camera to a visual
      protected: virtual bool AttachToVisualImpl(const std::string &_name,
                     bool _inheritOrientation,
                     double _minDist = 0, double _maxDist = 0);

      /// \brief Attach the camera to a visual
      protected: virtual bool AttachToVisualImpl(VisualPtr _visual,
                     bool _inheritOrientation,
                     double _minDist = 0, double _maxDist = 0);

      /// \brief if user requests bayer image, post process rgb from ogre
      ///        to generate bayer formats
      private: void ConvertRGBToBAYER(unsigned char* dst, unsigned char* src,
                   std::string format, int width, int height);

      private: static int GetOgrePixelFormat(const std::string &_format);

      /// Get the next frame filename based on SDF parameters
      protected: std::string GetFrameFilename();

      // Create the ogre camera
      private: void CreateCamera();

      protected: std::string name;
      protected: sdf::ElementPtr sdf;

      protected: unsigned int windowId;

      protected: unsigned int textureWidth, textureHeight;

      protected: Ogre::Camera *camera;
      protected: Ogre::Viewport *viewport;
      protected: Ogre::SceneNode *sceneNode;
      protected: Ogre::SceneNode *pitchNode;


      // Info for saving images
      protected: unsigned char *saveFrameBuffer;
      protected: unsigned char *bayerFrameBuffer;
      protected: unsigned int saveCount;

      protected: int imageFormat;
      protected: int imageWidth, imageHeight;

      protected: Ogre::RenderTarget *renderTarget;

      protected: Ogre::Texture *renderTexture;

      private: static unsigned int cameraCounter;
      private: unsigned int myCount;

      protected: bool captureData;

      protected: bool newData;

      protected: common::Time lastRenderWallTime;

      protected: Scene *scene;

      protected: event::EventT<void(const unsigned char *,
                     unsigned int, unsigned int, unsigned int,
                     const std::string &)> newImageFrame;

      protected: std::vector<event::ConnectionPtr> connections;
      protected: std::list<msgs::Request> requests;
      private: friend class Scene;

      private: sdf::ElementPtr imageElem;

      protected: bool initialized;
      private: VisualPtr trackedVisual;

      protected: Ogre::AnimationState *animState;
      protected: common::Time prevAnimTime;
      protected: boost::function<void()> onAnimationComplete;

      private: Ogre::CompositorInstance *dsGBufferInstance;
      private: Ogre::CompositorInstance *dsMergeInstance;

      private: Ogre::CompositorInstance *dlGBufferInstance;
      private: Ogre::CompositorInstance *dlMergeInstance;

      private: Ogre::CompositorInstance *ssaoInstance;
    };
    /// \}
  }
}
#endif
