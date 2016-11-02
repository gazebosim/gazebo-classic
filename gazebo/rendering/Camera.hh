/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_RENDERING_CAMERA_HH_
#define GAZEBO_RENDERING_CAMERA_HH_

#include <memory>
#include <functional>

#include <boost/enable_shared_from_this.hpp>
#include <string>
#include <utility>
#include <list>
#include <vector>
#include <deque>
#include <sdf/sdf.hh>
#include <ignition/math/Angle.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Pose3.hh>

#include "gazebo/msgs/msgs.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Subscriber.hh"

#include "gazebo/common/Event.hh"
#include "gazebo/common/PID.hh"
#include "gazebo/common/Time.hh"

#include "gazebo/math/Pose.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

// Forward Declarations
namespace Ogre
{
  class Texture;
  class RenderTarget;
  class Camera;
  class Viewport;
  class SceneNode;
  class AnimationState;
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
    class CameraPrivate;

    /// \addtogroup gazebo_rendering Rendering
    /// \brief A set of rendering related class, functions, and definitions
    /// \{

    /// \class Camera Camera.hh rendering/rendering.hh
    /// \brief Basic camera sensor
    ///
    /// This is the base class for all cameras.
    class GZ_RENDERING_VISIBLE Camera :
      public boost::enable_shared_from_this<Camera>
    {
      /// \brief Constructor
      /// \param[in] _namePrefix Unique prefix name for the camera.
      /// \param[in] _scene Scene that will contain the camera
      /// \param[in] _autoRender Almost everyone should leave this as true.
      public: Camera(const std::string &_namePrefix, ScenePtr _scene,
                     bool _autoRender = true);

      /// \brief Destructor
      public: virtual ~Camera();

      /// \brief Load the camera with a set of parmeters
      /// \param[in] _sdf The SDF camera info
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Load the camera with default parmeters
      public: virtual void Load();

      /// \brief Initialize the camera
      public: virtual void Init();

      /// \brief Set the render Hz rate
      /// \param[in] _hz The Hz rate
      public: void SetRenderRate(const double _hz);

      /// \brief Get the render Hz rate
      /// \return The Hz rate
      public: double RenderRate() const;

      /// \brief Render the camera.
      /// Called after the pre-render signal. This function will generate
      /// camera images.
      /// \param[in] _force Force camera to render. Ignore camera update
      /// rate.
      public: virtual void Render(const bool _force = false);

      /// \brief Post render
      ///
      /// Called afer the render signal.
      public: virtual void PostRender();

      /// \internal
      /// \brief Update the camera information. This does not render images.
      ///
      /// This function gets called automatically. There is no need for the
      /// average user to call this function.
      public: virtual void Update();

      /// \brief Finalize the camera.
      ///
      /// This function is called before the camera is destructed
      public: virtual void Fini();

      /// \brief Return true if the camera has been initialized
      /// \return True if initialized was successful
      public: bool Initialized() const;

      /// \internal
      /// \brief Set the ID of the window this camera is rendering into.
      /// \param[in] _windowId The window id of the camera.
      public: void SetWindowId(unsigned int _windowId);

      /// \brief Get the ID of the window this camera is rendering into.
      /// \return The ID of the window.
      public: unsigned int WindowId() const;

      /// \brief Set the scene this camera is viewing
      /// \param[in] _scene Pointer to the scene
      public: void SetScene(ScenePtr _scene);

      /// \brief Get the camera position in the world
      /// \return The world position of the camera
      public: ignition::math::Vector3d WorldPosition() const;

      /// \brief Get the camera's orientation in the world
      /// \return The camera's orientation as an ignition::math::Quaterniond
      public: ignition::math::Quaterniond WorldRotation() const;

      /// \brief Set the global pose of the camera
      /// \param[in] _pose The new math::Pose of the camera
      /// \deprecated See function that accepts an ignition::math parameter.
      public: virtual void SetWorldPose(const math::Pose &_pose)
              GAZEBO_DEPRECATED(8.0);

      /// \brief Set the global pose of the camera
      /// \param[in] _pose The new ignition::math::Pose3d of the camera
      public: virtual void SetWorldPose(const ignition::math::Pose3d &_pose);

      /// \brief Get the world pose.
      /// \return The pose of the camera in the world coordinate frame.
      public: ignition::math::Pose3d WorldPose() const;

      /// \brief Set the world position
      /// \param[in] _pos The new position of the camera
      public: void SetWorldPosition(const ignition::math::Vector3d &_pos);

      /// \brief Set the world orientation
      /// \param[in] _quat The new orientation of the camera
      public: void SetWorldRotation(const ignition::math::Quaterniond &_quat);

      /// \brief Translate the camera
      /// \param[in] _direction The translation vector
      public: void Translate(const ignition::math::Vector3d &_direction);

      /// \brief Rotate the camera around the x-axis
      /// \param[in] _angle Rotation amount
      /// \param[in] _relativeTo Coordinate frame to rotate in. Valid values
      /// are RF_LOCAL, RF_PARENT, and RF_WORLD. Default is RF_LOCAL
      public: void Roll(const ignition::math::Angle &_angle,
                  ReferenceFrame _relativeTo = RF_LOCAL);

      /// \brief Rotate the camera around the y-axis
      /// \param[in] _angle Pitch amount
      /// \param[in] _relativeTo Coordinate frame to rotate in. Valid values
      /// are RF_LOCAL, RF_PARENT, and RF_WORLD. Default is RF_LOCAL
      public: void Pitch(const ignition::math::Angle &_angle,
                  ReferenceFrame _relativeTo = RF_LOCAL);

      /// \brief Rotate the camera around the z-axis
      /// \param[in] _angle Rotation amount
      /// \param[in] _relativeTo Coordinate frame to rotate in. Valid values
      /// are RF_LOCAL, RF_PARENT, and RF_WORLD. Default is RF_WORLD
      public: void Yaw(const ignition::math::Angle &_angle,
                  ReferenceFrame _relativeTo = RF_WORLD);

      /// \brief Set the clip distances
      /// \param[in] _near Near clip distance in meters
      /// \param[in] _far Far clip distance in meters
      public: virtual void SetClipDist(const float _near, const float _far);

      /// \brief Set the camera FOV (horizontal)
      /// \param[in] _angle Horizontal field of view
      public: void SetHFOV(const ignition::math::Angle &_angle);

      /// \brief Get the camera FOV (horizontal)
      /// \return The horizontal field of view
      public: ignition::math::Angle HFOV() const;

      /// \brief Get the camera FOV (vertical)
      /// \return The vertical field of view
      /// \deprecated See function that returns an ignition::math object.
      /// \sa VFOV
      public: ignition::math::Angle VFOV() const;

      /// \brief Set the image size
      /// \param[in] _w Image width
      /// \param[in] _h Image height
      public: void SetImageSize(const unsigned int _w, const unsigned int _h);

      /// \brief Set the image height
      /// \param[in] _w Image width
      public: void SetImageWidth(const unsigned int _w);

      /// \brief Set the image height
      /// \param[in] _h Image height
      public: void SetImageHeight(const unsigned int _h);

      /// \brief Get the width of the image
      /// \return Image width
      public: virtual unsigned int ImageWidth() const;

      /// \brief Get the width of the off-screen render texture
      /// \return Render texture width
      public: unsigned int TextureWidth() const;

      /// \brief Get the height of the image
      /// \return Image height
      public: virtual unsigned int ImageHeight() const;

      /// \brief Get the depth of the image
      /// \return Depth of the image
      public: unsigned int ImageDepth() const;

      /// \brief Get the string representation of the image format.
      /// \return String representation of the image format.
      public: std::string ImageFormat() const;

      /// \brief Get the height of the off-screen render texture
      /// \return Render texture height
      public: unsigned int TextureHeight() const;

      /// \brief Get the image size in bytes
      /// \return Size in bytes
      public: size_t ImageByteSize() const;

      /// \brief Calculate image byte size base on a few parameters.
      /// \param[in] _width Width of an image
      /// \param[in] _height Height of an image
      /// \param[in] _format Image format
      /// \return Size of an image based on the parameters
      public: static size_t ImageByteSize(const unsigned int _width,
                                          const unsigned int _height,
                                          const std::string &_format);

      /// \brief Get the Z-buffer value at the given image coordinate.
      /// \param[in] _x Image coordinate; (0, 0) specifies the top-left corner.
      /// \param[in] _y Image coordinate; (0, 0) specifies the top-left corner.
      /// \returns Image z value; note that this is abitrarily scaled and
      /// is @e not the same as the depth value.
      public: double ZValue(const int _x, const int _y);

      /// \brief Get the near clip distance
      /// \return Near clip distance
      public: double NearClip() const;

      /// \brief Get the far clip distance
      /// \return Far clip distance
      public: double FarClip() const;

      /// \brief Enable or disable saving
      /// \param[in] _enable Set to True to enable saving of frames
      public: void EnableSaveFrame(const bool _enable);

      /// \brief Return the value of this->captureData.
      /// \return True if the camera is set to capture data.
      public: bool CaptureData() const;

      /// \brief Set the save frame pathname
      /// \param[in] _pathname Directory in which to store saved image frames
      public: void SetSaveFramePathname(const std::string &_pathname);

      /// \brief Save the last frame to disk
      /// \param[in] _filename File in which to save a single frame
      /// \return True if saving was successful
      public: bool SaveFrame(const std::string &_filename);

      /// \brief Get a pointer to the ogre camera
      /// \return Pointer to the OGRE camera
      public: Ogre::Camera *OgreCamera() const;

      /// \brief Get a pointer to the Ogre::Viewport
      /// \return Pointer to the Ogre::Viewport
      public: Ogre::Viewport *OgreViewport() const;

      /// \brief Get the viewport width in pixels
      /// \return The viewport width
      public: unsigned int ViewportWidth() const;

      /// \brief Get the viewport height in pixels
      /// \return The viewport height
      public: unsigned int ViewportHeight() const;

      /// \brief Get the viewport up vector
      /// \return The viewport up vector
      public: ignition::math::Vector3d Up() const;

      /// \brief Get the viewport right vector
      /// \return The viewport right vector
      public: ignition::math::Vector3d Right() const;

      /// \brief Get the average FPS
      /// \return The average frames per second
      public: virtual float AvgFPS() const;

      /// \brief Get the triangle count
      /// \return The current triangle count
      public: virtual unsigned int TriangleCount() const;

      /// \brief Set the aspect ratio
      /// \param[in] _ratio The aspect ratio (width / height) in pixels
      public: void SetAspectRatio(float _ratio);

      /// \brief Get the apect ratio
      /// \return The aspect ratio (width / height) in pixels
      public: float AspectRatio() const;

      /// \brief Set the camera's scene node
      /// \param[in] _node The scene nodes to attach the camera to
      public: void SetSceneNode(Ogre::SceneNode *_node);

      /// \brief Get the camera's scene node
      /// \return The scene node the camera is attached to
      public: Ogre::SceneNode *SceneNode() const;

      /// \brief Get a pointer to the image data
      ///
      /// Get the raw image data from a camera's buffer.
      /// \param[in] _i Index of the camera's texture (0 = RGB, 1 = depth).
      /// \return Pointer to the raw data, null if data is not available.
      public: virtual const unsigned char *ImageData(const unsigned int i = 0)
          const;

      /// \brief Get the camera's unscoped name
      /// \return The name of the camera
      public: std::string Name() const;

      /// \brief Get the camera's scoped name (scene_name::camera_name)
      /// \return The name of the camera
      public: std::string ScopedName() const;

      /// \brief Set the camera's name
      /// \param[in] _name New name for the camera
      public: void SetName(const std::string &_name);

      /// \brief Toggle whether to view the world in wireframe
      public: void ToggleShowWireframe();

      /// \brief Set whether to view the world in wireframe
      /// \param[in] _s Set to True to render objects as wireframe
      public: void ShowWireframe(const bool _s);

      /// \brief Get a world space ray as cast from the camera
      /// through the viewport
      /// \param[in] _screenx X coordinate in the camera's viewport, in pixels.
      /// \param[in] _screeny Y coordinate in the camera's viewport, in pixels.
      /// \param[out] _origin Origin in the world coordinate frame of the
      /// resulting ray
      /// \param[out] _dir Direction of the resulting ray
      /// \deprecated See function that accepts ignition::math parameters.
      public: void CameraToViewportRay(const int _screenx, const int _screeny,
                  ignition::math::Vector3d &_origin,
                  ignition::math::Vector3d &_dir) const;

      /// \brief Set whether to capture data
      /// \param[in] _value Set to true to capture data into a memory buffer.
      public: void SetCaptureData(const bool _value);

      /// \brief Capture data once and save to disk
      public: void SetCaptureDataOnce();

      /// \brief Turn on video recording.
      /// \param[in] _format String that represents the video type.
      /// Supported types include: "avi", "ogv", mp4", "v4l2". If using
      /// "v4l2", you must also specify a _filename.
      /// \param[in] _filename Name of the file that stores the video while it
      /// is being created. This is a temporary file when recording to
      /// disk, or a video4linux loopback device like /dev/video0 when
      /// the _format is "v4l2". If blank, a default temporary file is used.
      /// However, the "v4l2" _format must be accompanied with a video
      /// loopback device filename.
      /// \return True on success. The return value is set by
      /// common::VideoEncoder::Start().
      /// \sa common::VideoEncoder::Start
      public: bool StartVideo(const std::string &_format,
                              const std::string &_filename = "");

      /// \brief Turn off video recording
      /// \return True on success. The return value is set by
      /// common::VideoEncoder::Stop().
      /// \sa common::VideoEncoder::Stop
      public: bool StopVideo();

      /// \brief Save the last encoded video to disk
      /// \param[in] _filename File in which to save the encoded video
      /// \return True if saving was successful. The return value is set by
      /// common::VideoEncoder::SaveToFile().
      /// \sa common::VideoEncoder::SaveToFile
      public: bool SaveVideo(const std::string &_filename);

      /// \brief Reset video recording. This will call
      /// common::VideoEncoder::Reset, which will cleanup temporary files and
      /// set video encoding values to their default settings.
      /// \sa common::VideoEncoder::Reset
      /// \return True if reset was succesful. Currently this function will
      /// always return true.
      public: bool ResetVideo();

      /// \brief Set the render target
      /// \param[in] _textureName Name of the new render texture
      public: void CreateRenderTexture(const std::string &_textureName);

      /// \brief Get the scene this camera is in
      /// \return Pointer to scene containing this camera
      public: ScenePtr GetScene() const;

      /// \brief Get point on a plane
      /// \param[in] _x X coordinate in camera's viewport, in pixels
      /// \param[in] _y Y coordinate in camera's viewport, in pixels
      /// \param[in] _plane Plane on which to find the intersecting point
      /// \param[out] _result Point on the plane
      /// \return True if a valid point was found
      public: bool WorldPointOnPlane(const int _x, const int _y,
                  const ignition::math::Planed &_plane,
                  ignition::math::Vector3d &_result);

      /// \brief Set the camera's render target
      /// \param[in] _target Pointer to the render target
      public: virtual void SetRenderTarget(Ogre::RenderTarget *_target);

      /// \brief Attach the camera to a scene node
      /// \param[in] _visualName Name of the visual to attach the camera to
      /// \param[in] _inheritOrientation True means camera acquires the visual's
      /// orientation
      /// \param[in] _minDist Minimum distance the camera is allowed to get to
      /// the visual
      /// \param[in] _maxDist Maximum distance the camera is allowd to get from
      /// the visual
      public: void AttachToVisual(const std::string &_visualName,
                  const bool _inheritOrientation,
                  const double _minDist = 0.0, const double _maxDist = 0.0);

      /// \brief Attach the camera to a scene node
      /// \param[in] _id ID of the visual to attach the camera to
      /// \param[in] _inheritOrientation True means camera acquires the visual's
      /// orientation
      /// \param[in] _minDist Minimum distance the camera is allowed to get to
      /// the visual
      /// \param[in] _maxDist Maximum distance the camera is allowd to get from
      /// the visual
      public: void AttachToVisual(uint32_t _id,
                  const bool _inheritOrientation,
                  const double _minDist = 0.0, const double _maxDist = 0.0);

      /// \brief Set the camera to track a scene node
      /// \param[in] _visualName Name of the visual to track
      public: void TrackVisual(const std::string &_visualName);

      /// \brief Get the render texture
      /// \return Pointer to the render texture
      public: Ogre::Texture *RenderTexture() const;

      /// \brief Get the camera's direction vector
      /// \return Direction the camera is facing
      public: ignition::math::Vector3d Direction() const;

      /// \brief Connect to the new image signal
      /// \param[in] _subscriber Callback that is called when a new image is
      /// generated
      /// \return A pointer to the connection. This must be kept in scope.
      public: event::ConnectionPtr ConnectNewImageFrame(
          std::function<void (const unsigned char *, unsigned int, unsigned int,
          unsigned int, const std::string &)> _subscriber);

      /// \brief Disconnect from an image frame
      /// \param[in] _c The connection to disconnect
      /// \deprecated Use event::~Connection to disconnect
      public: void DisconnectNewImageFrame(event::ConnectionPtr &_c)
              GAZEBO_DEPRECATED(8.0);

      /// \brief Save a frame using an image buffer
      /// \param[in] _image The raw image buffer
      /// \param[in] _width Width of the image
      /// \param[in] _height Height of the image
      /// \param[in] _depth Depth of the image data
      /// \param[in] _format Format the image data is in
      /// \param[in] _filename Name of the file in which to write the frame
      /// \return True if saving was successful
      public: static bool SaveFrame(const unsigned char *_image,
                  const unsigned int _width, const unsigned int _height,
                  const int _depth, const std::string &_format,
                  const std::string &_filename);

      /// \brief Get the last time the camera was rendered
      /// \return Time the camera was last rendered
      public: common::Time LastRenderWallTime() const;

      /// \brief Return true if the visual is within the camera's view
      /// frustum
      /// \param[in] _visual The visual to check for visibility
      /// \return True if the _visual is in the camera's frustum
      public: bool IsVisible(VisualPtr _visual);

      /// \brief Return true if the visual is within the camera's view
      /// frustum
      /// \param[in] _visualName Name of the visual to check for visibility
      /// \return True if the _visual is in the camera's frustum
      public: bool IsVisible(const std::string &_visualName);

      /// \brief Return true if the camera is moving due to an animation.
      public: bool IsAnimating() const;

      /// \brief Move the camera to a position (this is an animated motion).
      /// \sa Camera::MoveToPositions
      /// \param[in] _pose End position of the camera
      /// \param[in] _time Duration of the camera's movement
      public: virtual bool MoveToPosition(const ignition::math::Pose3d &_pose,
                                          const double _time);

      /// \brief Move the camera to a series of poses (this is an
      /// animated motion).
      /// \sa Camera::MoveToPosition
      /// \param[in] _pts Vector of poses to move to
      /// \param[in] _time Duration of the entire move
      /// \param[in] _onComplete Callback that is called when the move is
      /// complete
      public: bool MoveToPositions(
                  const std::vector<ignition::math::Pose3d> &_pts,
                  const double _time,
                  std::function<void()> _onComplete = NULL);

      /// \brief Get the path to saved screenshots.
      /// \return Path to saved screenshots.
      public: std::string ScreenshotPath() const;

      /// \brief Get the distortion model of this camera.
      /// \return Distortion model.
      public: DistortionPtr LensDistortion() const;

      /// \brief Set the type of projection used by the camera.
      /// \param[in] _type The type of projection: "perspective" or
      /// "orthographic".
      /// \return True if successful.
      /// \sa GetProjectionType()
      public: virtual bool SetProjectionType(const std::string &_type);

      /// \brief Return the projection type as a string.
      /// \return "perspective" or "orthographic"
      /// \sa SetProjectionType(const std::string &_type)
      public: std::string ProjectionType() const;

      /// \brief Return the projection matrix of this camera.
      /// \return the projection matrix
      public: ignition::math::Matrix4d ProjectionMatrix() const;

      /// \brief Project 3D world coordinates to 2D screen coordinates
      /// \param[in] _pt 3D world coodinates
      /// \return _pt 2D screen coordinates
      public: ignition::math::Vector2i Project(
          const ignition::math::Vector3d &_pt) const;

      /// \brief Get the visual tracked by this camera.
      /// \return Tracked visual.
      public: VisualPtr TrackedVisual() const;

      /// \brief Get whether this camera is static when tracking a model.
      /// \return True if camera is static when tracking a model.
      /// \sa SetTrackIsStatic(const bool _isStatic)
      public: bool TrackIsStatic() const;

      /// \brief Set whether this camera is static when tracking a model.
      /// \param[in] _isStatic True means camera is static when tracking a
      /// model.
      /// \sa TrackIsStatic()
      public: void SetTrackIsStatic(const bool _isStatic);

      /// \brief Get whether this camera's position is relative to tracked
      /// models.
      /// \return True if camera's position is relative to tracked models.
      /// \sa SetTrackUseModelFrame(const bool _useModelFrame)
      public: bool TrackUseModelFrame() const;

      /// \brief Set whether this camera's position is relative to tracked
      /// models.
      /// \param[in] _useModelFrame True means camera's position is relative to
      /// tracked models.
      /// \sa TrackUseModelFrame()
      public: void SetTrackUseModelFrame(const bool _useModelFrame);

      /// \brief Return the position of the camera when tracking a model.
      /// \return Position of the camera.
      /// \sa SetTrackPosition(const ignition::math::Vector3d &_pos)
      public: ignition::math::Vector3d TrackPosition() const;

      /// \brief Set the position of the camera when tracking a visual.
      /// \param[in] _pos Position of the camera.
      /// \sa TrackPosition()
      public: void SetTrackPosition(const ignition::math::Vector3d &_pos);

      /// \brief Return the minimum distance to the tracked visual.
      /// \return Minimum distance to the model.
      /// \sa SetTrackMinDistance(const double _dist)
      public: double TrackMinDistance() const;

      /// \brief Return the maximum distance to the tracked visual.
      /// \return Maximum distance to the model.
      /// \sa SetTrackMaxDistance(const double _dist)
      public: double TrackMaxDistance() const;

      /// \brief Set the minimum distance between the camera and tracked
      /// visual.
      /// \param[in] _dist Minimum distance between camera and visual.
      /// \sa TrackMinDistance()
      public: void SetTrackMinDistance(const double _dist);

      /// \brief Set the maximum distance between the camera and tracked
      /// visual.
      /// \param[in] _dist Maximum distance between camera and visual.
      /// \sa TrackMaxDistance()
      public: void SetTrackMaxDistance(const double _dist);

      /// \brief Get whether this camera inherits the yaw rotation of the
      /// tracked model.
      /// \return True if the camera inherits the yaw rotation of the tracked
      /// model.
      /// \sa SetTrackInheritYaw(const bool _inheritYaw)
      public: bool TrackInheritYaw() const;

      /// \brief Set whether this camera inherits the yaw rotation of the
      /// tracked model.
      /// \param[in] _inheritYaw True means camera inherits the yaw rotation of
      /// the tracked model.
      /// \sa TrackInheritYaw()
      public: void SetTrackInheritYaw(const bool _inheritYaw);

      /// \brief Implementation of the render call
      protected: virtual void RenderImpl();

      /// \brief Read image data from pixel buffer
      protected: void ReadPixelBuffer();

      /// \brief Implementation of the Camera::TrackVisual call
      /// \param[in] _visualName Name of the visual to track
      /// \return True if able to track the visual
      protected: bool TrackVisualImpl(const std::string &_visualName);

      /// \brief Set the camera to track a scene node
      /// \param[in] _visual The visual to track
      /// \return True if able to track the visual
      protected: virtual bool TrackVisualImpl(VisualPtr _visual);

      /// \brief Attach the camera to a scene node
      /// \param[in] _visualName Name of the visual to attach the camera to
      /// \param[in] _inheritOrientation True means camera acquires the visual's
      /// orientation
      /// \param[in] _minDist Minimum distance the camera is allowed to get to
      /// the visual
      /// \param[in] _maxDist Maximum distance the camera is allowd to get from
      /// the visual
      /// \return True on success
      protected: virtual bool AttachToVisualImpl(const std::string &_name,
                     const bool _inheritOrientation,
                     const double _minDist = 0, const double _maxDist = 0);

      /// \brief Attach the camera to a scene node
      /// \param[in] _id ID of the visual to attach the camera to
      /// \param[in] _inheritOrientation True means camera acquires the visual's
      /// orientation
      /// \param[in] _minDist Minimum distance the camera is allowed to get to
      /// the visual
      /// \param[in] _maxDist Maximum distance the camera is allowd to get from
      /// the visual
      /// \return True on success
      protected: virtual bool AttachToVisualImpl(uint32_t _id,
                     const bool _inheritOrientation,
                     const double _minDist = 0, const double _maxDist = 0);

      /// \brief Attach the camera to a visual
      /// \param[in] _visual The visual to attach the camera to
      /// \param[in] _inheritOrientation True means camera acquires the visual's
      /// orientation
      /// \param[in] _minDist Minimum distance the camera is allowed to get to
      /// the visual
      /// \param[in] _maxDist Maximum distance the camera is allowd to get from
      /// the visual
      /// \return True on success
      protected: virtual bool AttachToVisualImpl(VisualPtr _visual,
                     const bool _inheritOrientation,
                     const double _minDist = 0, const double _maxDist = 0);

      /// \brief Get the next frame filename based on SDF parameters
      /// \return The frame's filename
      protected: std::string FrameFilename();

      /// \brief Internal function used to indicate that an animation has
      /// completed.
      protected: virtual void AnimationComplete();

      /// \brief Update the camera's field of view.
      protected: virtual void UpdateFOV();

      /// \brief Set the clip distance based on stored SDF values
      protected: virtual void SetClipDist();

      /// \brief Tell the camera whether to yaw around its own local Y axis or a
      /// fixed axis of choice.
      /// \param[in] _useFixed If true, the axis passed in the second parameter
      /// will always be the yaw axis no matter what the camera orientation.
      /// If false, the camera yaws around the local Y.
      /// \param[in] _fixedAxis The axis to use if the first parameter is true.
      protected: virtual void SetFixedYawAxis(const bool _useFixed,
          const ignition::math::Vector3d &_fixedAxis =
            ignition::math::Vector3d::UnitY);

      /// \brief if user requests bayer image, post process rgb from ogre
      ///        to generate bayer formats
      /// \param[out] _dst Destination buffer for the image data
      /// \param[in] _src Source image buffer
      /// \param[in] _format Format of the source buffer
      /// \param[in] _width Image width
      /// \param[in] _height Image height
      private: void ConvertRGBToBAYER(unsigned char *_dst,
          const unsigned char *_src, const std::string &_format,
          const int _width, const int _height);

      /// \brief Get the OGRE image pixel format in
      /// \param[in] _format The Gazebo image format
      /// \return Integer representation of the Ogre image format
      private: static int OgrePixelFormat(const std::string &_format);

      /// \brief Receive command message.
      /// \param[in] _msg Camera Command message.
      private: void OnCmdMsg(ConstCameraCmdPtr &_msg);

      /// \brief Create the ogre camera.
      private: void CreateCamera();

      /// \brief Name of the camera.
      protected: std::string name;

      /// \brief Scene scoped name of the camera.
      protected: std::string scopedName;

      /// \brief Scene scoped name of the camera with a unique ID.
      protected: std::string scopedUniqueName;

      /// \brief Camera's SDF values.
      protected: sdf::ElementPtr sdf;

      /// \brief ID of the window that the camera is attached to.
      protected: unsigned int windowId;

      /// \brief Width of the render texture.
      protected: unsigned int textureWidth;

      /// \brief Height of the render texture.
      protected: unsigned int textureHeight;

      /// \brief The OGRE camera
      protected: Ogre::Camera *camera;

      /// \brief Viewport the ogre camera uses.
      protected: Ogre::Viewport *viewport;

      /// \brief Scene node that controls camera position and orientation.
      protected: Ogre::SceneNode *sceneNode;

      /// \brief Buffer for a single image frame.
      protected: unsigned char *saveFrameBuffer;

      /// \brief Buffer for a bayer image frame.
      protected: unsigned char *bayerFrameBuffer;

      /// \brief Number of saved frames.
      protected: unsigned int saveCount;

      /// \brief Path to saved screenshots.
      protected: std::string screenshotPath;

      /// \brief Format for saving images.
      protected: int imageFormat;

      /// \brief Save image width.
      protected: int imageWidth;

      /// \brief Save image height.
      protected: int imageHeight;

      /// \brief Target that renders frames.
      protected: Ogre::RenderTarget *renderTarget;

      /// \brief Texture that receives results from rendering.
      protected: Ogre::Texture *renderTexture;

      /// \brief True to capture frames into an image buffer.
      protected: bool captureData;

      /// \brief True to capture a frame once and save to disk.
      protected: bool captureDataOnce;

      /// \brief True if new data is available.
      protected: bool newData;

      /// \brief Time the last frame was rendered.
      protected: common::Time lastRenderWallTime;

      /// \brief Pointer to the scene.
      protected: ScenePtr scene;

      /// \brief Event triggered when a new frame is generated.
      protected: event::EventT<void(const unsigned char *,
                     unsigned int, unsigned int, unsigned int,
                     const std::string &)> newImageFrame;

      /// \brief The camera's event connections.
      protected: std::vector<event::ConnectionPtr> connections;

      /// \brief List of requests.
      protected: std::list<msgs::Request> requests;

      /// \brief True if initialized.
      protected: bool initialized;

      /// \brief Animation state, used to animate the camera.
      protected: Ogre::AnimationState *animState;

      /// \brief Previous time the camera animation was updated.
      protected: common::Time prevAnimTime;

      /// \brief User callback for when an animation completes.
      protected: std::function<void()> onAnimationComplete;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<CameraPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
