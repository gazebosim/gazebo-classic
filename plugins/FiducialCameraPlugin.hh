/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#ifndef GAZEBO_PLUGINS_FIDUCIAL_CAMERA_PLUGIN_HH_
#define GAZEBO_PLUGINS_FIDUCIAL_CAMERA_PLUGIN_HH_

#include <string>
#include <memory>

#include "gazebo/common/Plugin.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  // Forward declare private class.
  class FiducialCameraPluginPrivate;

  /// \brief A camera sensor plugin for fiducial detection
  /// A fiducial is detected if its center is within the camera frustum and
  /// not occluded by other models in the view. The results are published to
  /// a topic, e.g. ~/camera_model_name/link_name/sensor_name/fiducial
  /// The message format is PosesStamped. The pose's x and y position fields are
  /// the image coordinates of the center of the detected fiducial, and the
  /// orientation is currently always an identity quaternion.
  class GAZEBO_VISIBLE FiducialCameraPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: FiducialCameraPlugin();

    /// \brief Destructor
    public: virtual ~FiducialCameraPlugin();

    // Documentation Inherited.
    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    public: void Init();

    /// \brief Callback when a new camera frame is available
    /// \param[in] _image image data
    /// \param[in] _width image width
    /// \param[in] _height image height
    /// \param[in] _depth image depth
    /// \param[in] _format image format
    public: virtual void OnNewFrame(const unsigned char *_image,
        const unsigned int _width, const unsigned int _height,
        const unsigned int _depth, const std::string &_format);

    /// \brief Helper function to fill the list of fiducials with all models
    /// in the world if none are specified
    private: void PopulateFiducials();

    /// \internal
    /// \brief Pointer to private data.
    private: std::unique_ptr<FiducialCameraPluginPrivate> dataPtr;
  };
}
#endif
