/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "plugins/GpuRayPlugin.hh"
#include "gazebo/sensors/GpuRaySensor.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(GpuRayPlugin)

/////////////////////////////////////////////////
GpuRayPlugin::GpuRayPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
void GpuRayPlugin::Load(sensors::SensorPtr _sensor,
                              sdf::ElementPtr /*_sdf*/)
{
  this->parentSensor =
    boost::dynamic_pointer_cast<sensors::GpuRaySensor>(_sensor);

  if (!this->parentSensor)
  {
    gzerr << "GpuRayPlugin not attached to a GpuLaser sensor\n";
    return;
  }

  this->width = this->parentSensor->GetRangeCount();
  this->height = this->parentSensor->GetVerticalRangeCount();

  this->newLaserFrameConnection = this->parentSensor->ConnectNewLaserFrame(
      boost::bind(&GpuRayPlugin::OnNewLaserFrame,
        this, _1, _2, _3, _4, _5));

  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void GpuRayPlugin::OnNewLaserFrame(const float * /*_image*/,
    unsigned int /*_width*/, unsigned int /*_height*/,
    unsigned int /*_depth*/, const std::string &/*_format*/)
{
}
