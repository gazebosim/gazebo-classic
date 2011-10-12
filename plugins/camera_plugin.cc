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
#include "sensors/CameraSensor.hh"
#include "rendering/Camera.hh"
#include "gazebo.h"

namespace gazebo
{
  class CameraPlugin : public SensorPlugin
  {
    public: CameraPlugin() : SensorPlugin() 
            {
            }

    public: void Load( sensors::SensorPtr &_sensor, sdf::ElementPtr &/*_sdf*/ )
            {
              this->parentSensor = boost::shared_dynamic_cast<sensors::CameraSensor>(_sensor);
              this->camera = this->parentSensor->GetCamera();

              if (!this->parentSensor)
              {
                gzerr << "CameraPlugin not attached to a camera sensor\n";
                return;
              }

              this->width = this->camera->GetImageWidth();
              this->height = this->camera->GetImageHeight();
              this->depth = this->camera->GetImageDepth();
              this->format = this->camera->GetImageFormat();

              this->newFrameConnection = this->camera->ConnectNewFrame( boost::bind(&CameraPlugin::OnNewFrame, this, _1));

              this->parentSensor->SetActive(true);
            }

    public: virtual void OnNewFrame(const unsigned char *_image)
            {
              /*rendering::Camera::SaveFrame( _image, this->width, 
                  this->height, this->depth, this->format, 
                  "/tmp/camera/me.jpg" ); */
            }


    protected: unsigned int width, height, depth;
    protected: std::string format;

    private: sensors::CameraSensorPtr parentSensor;
    private: rendering::CameraPtr camera;

    private: event::ConnectionPtr newFrameConnection;
  };

  GZ_REGISTER_SENSOR_PLUGIN(CameraPlugin)
}
