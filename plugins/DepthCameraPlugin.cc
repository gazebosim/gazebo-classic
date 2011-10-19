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
#include "plugins/DepthCameraPlugin.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(DepthCameraPlugin)

DepthCameraPlugin::DepthCameraPlugin() : SensorPlugin() 
{
}

void DepthCameraPlugin::Load( sensors::SensorPtr &_sensor, 
                              sdf::ElementPtr &/*_sdf*/)
{
  this->parentSensor = boost::shared_dynamic_cast<sensors::DepthCameraSensor>(_sensor);
  this->camera = this->parentSensor->GetCamera();

  if (!this->parentSensor)
  {
    gzerr << "DepthCameraPlugin not attached to a camera sensor\n";
    return;
  }

  this->width = this->camera->GetImageWidth();
  this->height = this->camera->GetImageHeight();
  this->depth = this->camera->GetImageDepth();
  this->format = this->camera->GetImageFormat();

  this->newFrameConnection = this->camera->ConnectNewDepthFrame( 
      boost::bind(&DepthCameraPlugin::OnNewFrame, this, _1, _2, _3, _4, _5));

  this->parentSensor->SetActive(true);
}

void DepthCameraPlugin::OnNewFrame(const float *_image,
    unsigned int _width, unsigned int _height, 
    unsigned int /*_depth*/, const std::string &/*_format*/)
{
  float min, max;
  min = 1000;
  max = 0;
  for (unsigned int i=0; i < _width * _height; i++)
  {
    if (_image[i] > max)
      max = _image[i];
    if (_image[i] < min)
      min = _image[i];
  }

  int index =  ((_height * 0.5) * _width) + _width *0.5;
  printf("W[%d] H[%d] MidPoint[%d] Dist[%f] Min[%f] Max[%f]\n", width, height, index, _image[index], min, max);

  /*rendering::Camera::SaveFrame( _image, this->width, 
    this->height, this->depth, this->format, 
    "/tmp/camera/me.jpg" );
    */
}
