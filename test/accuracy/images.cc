/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include "gazebo/util/ZadeatGTParser.hh"
#include "gazebo/util/ZadeatImgParser.hh"
#include "ServerFixture.hh"

using namespace gazebo;

class ImageAccuracyTest : public ServerFixture
{
};

unsigned char* img = NULL;
int imageCount = 0;

void OnNewCameraFrame(bool* _newImage, unsigned char* _imageDest,
                  const unsigned char *_image,
                  unsigned int _width, unsigned int _height,
                  unsigned int _depth,
                  const std::string &/*_format*/)
{
  memcpy(_imageDest, _image, _width * _height * _depth);
  *_newImage = true;
}

/////////////////////////////////////////////////
TEST_F(ImageAccuracyTest, ZadeatDataset)
{
  /*gazebo::util::ZadeatImgParser *imgParser = new gazebo::util::ZadeatImgParser
    ("/opt/naoDataset/run02/camera_2.strm", "/opt/naoDataset/run02/images/");

  imgParser->Parse();

  return;*/

  Load("worlds/robocup09_spl_field.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  double timestamp;
  double headRoll, headPitch, headYaw;
  double headX, headY, headZ;
  double torsoRoll, torsoPitch, torsoYaw;
  double torsoX, torsoY, torsoZ;

  gazebo::util::ZadeatGTParser *gtParser = new gazebo::util::ZadeatGTParser(
    "/opt/naoDataset/run02/ground_truth_2.csv");

  // spawn sensors of various sizes to test speed
  std::string modelName = "camera_model";
  std::string cameraName = "camera_sensor";
  unsigned int width  = 320;
  unsigned int height = 240;
  double updateRate = 30;

  int counter = 0;
  bool newImage = false;
  img = new unsigned char[width * height * 3];
  math::Pose cameraInitPose(0, 0, 0, 0, 0, 0);

  SpawnCamera(modelName, cameraName, cameraInitPose.pos,
        cameraInitPose.rot.GetAsEuler(), width, height, updateRate);
    sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
    sensors::CameraSensorPtr camSensor =
      boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);

  event::ConnectionPtr c =
    camSensor->GetCamera()->ConnectNewImageFrame(
        boost::bind(&::OnNewCameraFrame, &newImage, img,
          _1, _2, _3, _4, _5));

  while (gtParser->GetNextGT(timestamp, headRoll, headPitch, headYaw,
                              headX, headY, headZ,
                              torsoRoll, torsoPitch, torsoYaw,
                              torsoX, torsoY, torsoZ))
  {

  /*gtParser->GetNextGT(timestamp, headRoll, headPitch, headYaw,
                              headX, headY, headZ,
                              torsoRoll, torsoPitch, torsoYaw,
                              torsoX, torsoY, torsoZ);*/

    // Move the Gazebo camera to that position

    /*headX = 2.36742;
    headY = -1.45686;
    headZ = 0.44459;
    headRoll = -0.0771;
    headPitch = 0.0312;
    headYaw = 2.5127;*/

    // Conversion from mm to m.
    headX /= 1000.0;
    headY /= 1000.0;
    headZ /= 1000.0;

    // Translation and rotation from the center of the head to the lower cam.
    headX += 0.0488;
    headZ -= 0.04409;
    headPitch = GZ_NORMALIZE(headPitch + GZ_DTOR(40.0));

    math::Pose cameraPose(headX, headY, headZ, headRoll, headPitch, headYaw);
    /*std::cout << "Head (" << headX << "," << headY << "," << headZ << ")(" <<
      headRoll << "," << headPitch << "," << headYaw << ")" << std::endl;*/

    rendering::CameraPtr camera = camSensor->GetCamera();
    camera->SetWorldPose(cameraPose);

    newImage = false;
    while (!newImage)
      common::Time::MSleep(1);

    // Save the image on disk
    std::ostringstream ss;
    ss << std::setw(5) << std::setfill('0') << counter / 8;
    std::string s2(ss.str());
    std::string filename = "/opt/naoDataset/run02/gazebo_images/" +
                           ss.str() + ".ppm";
    if (counter % 8 == 0)
    {
      std::cout << filename << std::endl;
      camSensor->SaveFrame(filename);
    }
    ++counter;
    //camSensor->SaveFrame("/tmp/image.ppm");
  }

}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
