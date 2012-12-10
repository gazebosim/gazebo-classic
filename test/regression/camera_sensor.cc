/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "ServerFixture.hh"
#include "physics/physics.hh"
#include "sensors/sensors.hh"
#include "common/common.hh"
#include "common/Timer.hh"
#include "scans_cmp.h"
#include "rendering/Camera.hh"
#include "sensors/CameraSensor.hh"

using namespace gazebo;
class CameraSensor : public ServerFixture
{
};

unsigned char* img = NULL;
int image_count = 0;
void OnNewCameraFrame(const unsigned char *_image,
                  unsigned int _width, unsigned int _height,
                  unsigned int _depth,
                  const std::string &/*_format*/)
{
  memcpy(img, _image, _width * _height * _depth);
  image_count+= 1;
}

TEST_F(CameraSensor, EmptyWorld)
{
  // spawn sensors of various sizes to test speed
  {
    Load("worlds/empty.world");
    std::string modelName = "camera_model";
    std::string cameraName = "camera_sensor";
    unsigned int width  = 320;
    unsigned int height = 240;  // 106 fps
    width  = 640;
    height = 480;  // 80.25 fps
    width  = 1280;
    height = 960;  // 41.19 fps
    math::Pose setPose, testPose(
      math::Vector3(-5, 0, 5), math::Quaternion(0, GZ_DTOR(15), 0));
    SpawnCamera(modelName, cameraName, setPose.pos,
        setPose.rot.GetAsEuler(), width, height, 100000);

    sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
    sensors::CameraSensorPtr camSensor =
      boost::shared_dynamic_cast<sensors::CameraSensor>(sensor);
    img = new unsigned char[width * height*3];
    event::ConnectionPtr c =
      camSensor->GetCamera()->ConnectNewImageFrame(
          boost::bind(&::OnNewCameraFrame,
                      _1, _2, _3, _4, _5));
    common::Timer timer;
    timer.Start();
    // time how long it takes to get N images
    int total_images = 2500;
    while (image_count < total_images)
      common::Time::MSleep(10);
    common::Time dt = timer.GetElapsed();
    gzerr << "timer [" << dt.Double()
          << "] seconds rate [" << static_cast<double>(total_images)/dt.Double()
          << "] fps\n";
    camSensor->GetCamera()->DisconnectNewImageFrame(c);
    delete img;
    Unload();
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
