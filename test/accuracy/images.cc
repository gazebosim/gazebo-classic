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

void OnNewCameraFrame(int* _imageCounter, unsigned char* _imageDest,
                  const unsigned char *_image,
                  unsigned int _width, unsigned int _height,
                  unsigned int _depth,
                  const std::string &/*_format*/)
{
  memcpy(_imageDest, _image, _width * _height * _depth);
  *_imageCounter += 1;
}

/////////////////////////////////////////////////
TEST_F(ImageAccuracyTest, ZadeatDataset)
{
  /*gazebo::util::ZadeatImgParser *imgParser = new gazebo::util::ZadeatImgParser
    ("/opt/naoDataset/run02/camera_2.strm", "/opt/naoDataset/run02/images/");

  imgParser->Parse();*/

  Load("worlds/empty.world");

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
  unsigned int height = 240;  // 106 fps
  double updateRate = 10;
  math::Pose setPose, testPose(
      math::Vector3(-5, 0, 5), math::Quaternion(0, GZ_DTOR(15), 0));
  SpawnCamera(modelName, cameraName, setPose.pos,
      setPose.rot.GetAsEuler(), width, height, updateRate);
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  imageCount = 0;
  img = new unsigned char[width * height * 3];
  event::ConnectionPtr c =
    camSensor->GetCamera()->ConnectNewImageFrame(
        boost::bind(&::OnNewCameraFrame, &imageCount, img,
          _1, _2, _3, _4, _5));

  int total_images = 1;
  while (imageCount < total_images)
    common::Time::MSleep(10);

  // Save the image
  std::ofstream myFile("/tmp/image.ppm", std::ios::out | std::ios::binary);

  std::string strWidth = boost::lexical_cast<std::string>(width);
  std::string strHeight = boost::lexical_cast<std::string>(height);
  std::string header = "P6 " + strWidth + " " + strHeight + " 255\n";

  // Write a PPM header
  myFile.write(header.c_str(), header.size());

  // Write the data
  uint32_t dataSize = width * height * 3;
  myFile.write(reinterpret_cast<const char *>(img), dataSize);

  myFile.close();

  /*while (gtParser->GetNextGT(timestamp, headRoll, headPitch, headYaw,
                              headX, headY, headZ,
                              torsoRoll, torsoPitch, torsoYaw,
                              torsoX, torsoY, torsoZ))
  {*/

  /*gtParser->GetNextGT(timestamp, headRoll, headPitch, headYaw,
                              headX, headY, headZ,
                              torsoRoll, torsoPitch, torsoYaw,
                              torsoX, torsoY, torsoZ);

  // Move the Gazebo camera to that position
  //math::Pose cameraPose(headX, headY, headZ, headRoll, headPitch, headYaw);

  math::Pose cameraPose(0, 0, 0, 0, 0, 0);

  // Spawn a camera that will do the following
  SpawnCamera("test_camera_model", "test_camera",
      cameraPose.pos, cameraPose.rot.GetAsEuler());

  WaitUntilEntitySpawn("test_camera_model", 100, 50);
  WaitUntilSensorSpawn("test_camera", 100, 100);

  // Take a snapshot
  rendering::ScenePtr scene = rendering::get_scene();
  ASSERT_TRUE(scene);

  rendering::CameraPtr camera = scene->GetCamera("test_camera");*/
  //ASSERT_TRUE(camera);

  /*camera->Render(true);
  camera->PostRender();

  // Store the image
  camera->SaveFrame("/tmp/testImage.png");*/
  //}

}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
