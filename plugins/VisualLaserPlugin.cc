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
#include "plugins/VisualLaserPlugin.hh"
#include "rendering/VisualLaser.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(VisualLaserPlugin)

/////////////////////////////////////////////////
VisualLaserPlugin::VisualLaserPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
void VisualLaserPlugin::Load(sensors::SensorPtr _sensor,
                              sdf::ElementPtr /*_sdf*/)
{
  this->parentSensor =
    boost::shared_dynamic_cast<sensors::VisualLaserSensor>(_sensor);
  this->laserCam = this->parentSensor->GetLaserCamera();

  if (!this->parentSensor)
  {
    gzerr << "VisualLaserPlugin not attached to a VisualLaser sensor\n";
    return;
  }

  this->width = this->parentSensor->GetRangeCount();
  this->height = this->parentSensor->GetVerticalRangeCount();
//  this->depth = 3;
//  this->format = this->laserCam->GetImageFormat();

  this->newLaserFrameConnection = this->laserCam->ConnectNewLaserFrame(
      boost::bind(&VisualLaserPlugin::OnNewLaserFrame,
        this, _1, _2, _3, _4, _5));

  this->newImageFrameConnection = this->laserCam->ConnectNewImageFrame(
      boost::bind(&VisualLaserPlugin::OnNewImageFrame,
        this, _1, _2, _3, _4, _5));

  this->newImage2FrameConnection = this->laserCam->ConnectNewImage2Frame(
      boost::bind(&VisualLaserPlugin::OnNewImage2Frame,
        this, _1, _2, _3, _4, _5));

  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void VisualLaserPlugin::OnNewLaserFrame(const float *_image,
    unsigned int _width, unsigned int _height,
    unsigned int /*_depth*/, const std::string &/*_format*/)
{
}
    
/////////////////////////////////////////////////
void VisualLaserPlugin::OnNewImageFrame(const unsigned char * /*_image*/,
                              unsigned int /*_width*/,
                              unsigned int /*_height*/,
                              unsigned int /*_depth*/,
                              const std::string &/*_format*/)
{
  /*rendering::Camera::SaveFrame(_image, this->width,
    this->height, this->depth, this->format,
    "/tmp/laserCam/me.jpg");
    */
}

/////////////////////////////////////////////////
void VisualLaserPlugin::OnNewImage2Frame(const unsigned char * /*_image*/,
                              unsigned int /*_width*/,
                              unsigned int /*_height*/,
                              unsigned int /*_depth*/,
                              const std::string &/*_format*/)
{
  /*rendering::Camera::SaveFrame(_image, this->width,
    this->height, this->depth, this->format,
    "/tmp/laserCam/me.jpg");
    */
}
