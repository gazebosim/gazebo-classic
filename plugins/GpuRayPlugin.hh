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

#ifndef __GAZEBO_GPU_LASER_PLUGIN_HH__
#define __GAZEBO_GPU_LASER_PLUGIN_HH__

#include <string>

#include "common/Plugin.hh"
#include "sensors/GpuRaySensor.hh"
#include "sensors/CameraSensor.hh"
#include "rendering/RenderTypes.hh"
#include "gazebo.h"

namespace gazebo
{
  class GpuRayPlugin : public SensorPlugin
  {
    public: GpuRayPlugin();

    public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);

    public: virtual void OnNewLaserFrame(const float *_image,
                unsigned int _width, unsigned int _height,
                unsigned int _depth, const std::string &_format);

    //public: virtual void OnNewImageFrame(const unsigned char *_image,
    //                          unsigned int _width, unsigned int _height,
    //                          unsigned int _depth, unsigned int cam);

    protected: unsigned int width, height/*, depth*/;
//    protected: std::string format;

    protected: sensors::GpuRaySensorPtr parentSensor;
    protected: rendering::GpuLaserPtr laserCam;

    private: event::ConnectionPtr newLaserFrameConnection;
    //private: event::ConnectionPtr newImageFrameConnection;
  };
}
#endif
