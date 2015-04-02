/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_DEPTH_CAMERA_PLUGIN_HH_
#define _GAZEBO_DEPTH_CAMERA_PLUGIN_HH_

#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/DepthCameraSensor.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/rendering/DepthCamera.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  class GAZEBO_VISIBLE DepthCameraPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: DepthCameraPlugin();

    /// \brief Destructor
    public: virtual ~DepthCameraPlugin();

    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    public: virtual void OnNewDepthFrame(const float *_image,
                unsigned int _width, unsigned int _height,
                unsigned int _depth, const std::string &_format);

    /// \brief Update the controller
    public: virtual void OnNewRGBPointCloud(const float *_pcd,
                unsigned int _width, unsigned int _height,
                unsigned int _depth, const std::string &_format);

    public: virtual void OnNewImageFrame(const unsigned char *_image,
                              unsigned int _width, unsigned int _height,
                              unsigned int _depth, const std::string &_format);

    protected: unsigned int width, height, depth;
    protected: std::string format;

    protected: sensors::DepthCameraSensorPtr parentSensor;
    protected: rendering::DepthCameraPtr depthCamera;

    private: event::ConnectionPtr newDepthFrameConnection;
    private: event::ConnectionPtr newRGBPointCloudConnection;
    private: event::ConnectionPtr newImageFrameConnection;
  };
}
#endif
