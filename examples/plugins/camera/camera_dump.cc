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
#include "gazebo/gazebo.hh"
#include "plugins/CameraPlugin.hh"

namespace gazebo
{
  class CameraDump : public CameraPlugin
  {
    public: CameraDump() : CameraPlugin(), saveCount(0) {}

    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
    {
      // Don't forget to load the camera plugin
      CameraPlugin::Load(_parent, _sdf);
    }

    // Update the controller
    public: void OnNewFrame(const unsigned char *_image,
        unsigned int _width, unsigned int _height, unsigned int _depth,
        const std::string &_format)
    {
      char tmp[1024];
      snprintf(tmp, sizeof(tmp), "/tmp/%s-%04d.jpg",
          this->parentSensor->GetCamera()->GetName().c_str(), this->saveCount);

      if (this->saveCount < 10)
      {
        this->parentSensor->GetCamera()->SaveFrame(
            _image, _width, _height, _depth, _format, tmp);
        gzmsg << "Saving frame [" << this->saveCount
              << "] as [" << tmp << "]\n";
        this->saveCount++;
      }
    }

    private: int saveCount;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(CameraDump)
}
