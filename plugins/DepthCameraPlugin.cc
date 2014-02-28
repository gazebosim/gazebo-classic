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
#include "plugins/DepthCameraPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(DepthCameraPlugin)

/////////////////////////////////////////////////
DepthCameraPlugin::DepthCameraPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
DepthCameraPlugin::~DepthCameraPlugin()
{
  this->parentSensor.reset();
  this->depthCamera.reset();
}

/////////////////////////////////////////////////
void DepthCameraPlugin::Load(sensors::SensorPtr _sensor,
                              sdf::ElementPtr /*_sdf*/)
{
  this->parentSensor =
    boost::dynamic_pointer_cast<sensors::DepthCameraSensor>(_sensor);
  this->depthCamera = this->parentSensor->GetDepthCamera();

  if (!this->parentSensor)
  {
    gzerr << "DepthCameraPlugin not attached to a depthCamera sensor\n";
    return;
  }

  this->width = this->depthCamera->GetImageWidth();
  this->height = this->depthCamera->GetImageHeight();
  this->depth = this->depthCamera->GetImageDepth();
  this->format = this->depthCamera->GetImageFormat();

  this->newDepthFrameConnection = this->depthCamera->ConnectNewDepthFrame(
      boost::bind(&DepthCameraPlugin::OnNewDepthFrame,
        this, _1, _2, _3, _4, _5));

  this->newRGBPointCloudConnection = this->depthCamera->ConnectNewRGBPointCloud(
      boost::bind(&DepthCameraPlugin::OnNewRGBPointCloud,
        this, _1, _2, _3, _4, _5));

  this->newImageFrameConnection = this->depthCamera->ConnectNewImageFrame(
      boost::bind(&DepthCameraPlugin::OnNewImageFrame,
        this, _1, _2, _3, _4, _5));

  this->parentSensor->SetActive(true);
}

/////////////////////////////////////////////////
void DepthCameraPlugin::OnNewDepthFrame(const float *_image,
    unsigned int _width, unsigned int _height,
    unsigned int /*_depth*/, const std::string &/*_format*/)
{
  float min, max;
  min = 1000;
  max = 0;
  for (unsigned int i = 0; i < _width * _height; i++)
  {
    if (_image[i] > max)
      max = _image[i];
    if (_image[i] < min)
      min = _image[i];
  }

  int index =  ((_height * 0.5) * _width) + _width * 0.5;
  printf("W[%u] H[%u] MidPoint[%d] Dist[%f] Min[%f] Max[%f]\n",
      width, height, index, _image[index], min, max);

  /*rendering::Camera::SaveFrame(_image, this->width,
    this->height, this->depth, this->format,
    "/tmp/depthCamera/me.jpg");
    */
}

/////////////////////////////////////////////////
void DepthCameraPlugin::OnNewRGBPointCloud(const float * /*_pcd*/,
                unsigned int /*_width*/, unsigned int /*_height*/,
                unsigned int /*_depth*/, const std::string &/*_format*/)
{
}

/////////////////////////////////////////////////
void DepthCameraPlugin::OnNewImageFrame(const unsigned char * /*_image*/,
                              unsigned int /*_width*/,
                              unsigned int /*_height*/,
                              unsigned int /*_depth*/,
                              const std::string &/*_format*/)
{
  /*rendering::Camera::SaveFrame(_image, this->width,
    this->height, this->depth, this->format,
    "/tmp/depthCamera/me.jpg");
    */
}
