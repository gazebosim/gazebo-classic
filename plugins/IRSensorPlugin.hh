/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef GAZEBO_PLUGINS_IRSENSORPLUGIN_HH_
#define GAZEBO_PLUGINS_IRSENSORPLUGIN_HH_

#include <memory>
#include <set>
#include <string>

#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/CameraSensor.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  class IRMaterialHandler;

  /// \def IRMaterialHandlerPtr
  /// \brief Shared pointer to a IRMaterialHandlerPtr object
  using IRMaterialHandlerPtr = std::shared_ptr<IRMaterialHandler>;

  class GAZEBO_VISIBLE IRSensorPlugin : public SensorPlugin
  {
    public: IRSensorPlugin();

    /// \brief Destructor
    public: virtual ~IRSensorPlugin();

    /// \brief Load the plugin
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    public: virtual void OnNewFrame(const unsigned char *_image,
                              unsigned int _width, unsigned int _height,
                              unsigned int _depth, const std::string &_format);

    private: sensors::CameraSensorPtr parentSensor;
    private: rendering::CameraPtr camera;

    private: event::ConnectionPtr newFrameConnection;
    private: IRMaterialHandlerPtr materialHandler;
    private: std::set<std::string> targets;
  };
}
#endif
