/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/common.hh"
#include "gazebo/common/Timer.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/sensors/MultiCameraSensor.hh"

#include "gazebo/test/ServerFixture.hh"
#include "scans_cmp.h"

using namespace gazebo;
class MultiCameraSensor : public ServerFixture
{
};

unsigned char* img0Left = NULL;
unsigned char* imgt = NULL;
unsigned char* img1Left = NULL;
unsigned char* img2Left = NULL;
unsigned char* img0Right = NULL;
unsigned char* img1Right = NULL;
unsigned char* img2Right = NULL;
int imageCount0Left = 0;
int imageCountt = 0;
int imageCount1Left = 0;
int imageCount2Left = 0;
int imageCount0Right = 0;
int imageCount1Right = 0;
int imageCount2Right = 0;

/////////////////////////////////////////////////
void OnNewFrameTest(int* _imageCounter, unsigned char* _imageDest,
                  const unsigned char *_image,
                  unsigned int _width, unsigned int _height,
                  unsigned int _depth,
                  const std::string &/*_format*/)
{
  memcpy(_imageDest, _image, _width * _height * _depth);
  *_imageCounter += 1;
}

/////////////////////////////////////////////////
TEST_F(MultiCameraSensor, CameraRotationTest)
{
  Load("worlds/camera_rotation_test.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  // get two cameras, one with rotation and one without.
  std::string cameraUnrotated = "multicamera_sensor_unrotated";
  std::string cameraTranslated = "camera_sensor_translated";
  std::string cameraRotated1 = "multicamera_sensor_rotated1";
  std::string cameraRotated2 = "multicamera_sensor_rotated2";

  sensors::SensorPtr sensor = sensors::get_sensor(cameraUnrotated);
  sensors::MultiCameraSensorPtr camSensorUnrotated =
    std::dynamic_pointer_cast<sensors::MultiCameraSensor>(sensor);

  sensor = sensors::get_sensor(cameraTranslated);
  sensors::CameraSensorPtr camSensorTranslated =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);

  sensor = sensors::get_sensor(cameraRotated1);
  sensors::MultiCameraSensorPtr camSensorRotated1 =
    std::dynamic_pointer_cast<sensors::MultiCameraSensor>(sensor);

  sensor = sensors::get_sensor(cameraRotated2);
  sensors::MultiCameraSensorPtr camSensorRotated2 =
    std::dynamic_pointer_cast<sensors::MultiCameraSensor>(sensor);

  unsigned int width  = 1024;
  unsigned int height = 544;
  unsigned int depth = 3;

  // initialize global variables
  imageCount0Left  = 0;
  imageCountt  = 0;
  imageCount1Left  = 0;
  imageCount2Left  = 0;
  imageCount0Right = 0;
  imageCount1Right = 0;
  imageCount2Right = 0;
  img0Left = new unsigned char[width * height * depth];
  imgt = new unsigned char[width * height * depth];
  img1Left = new unsigned char[width * height * depth];
  img2Left = new unsigned char[width * height * depth];
  img0Right = new unsigned char[width * height * depth];
  img1Right = new unsigned char[width * height * depth];
  img2Right = new unsigned char[width * height * depth];

  {
    // connect to camera image updates
    event::ConnectionPtr c0Left =
      camSensorUnrotated->Camera(0)->ConnectNewImageFrame(
          std::bind(&::OnNewFrameTest, &imageCount0Left, img0Left,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            std::placeholders::_4, std::placeholders::_5));
    event::ConnectionPtr ct =
      camSensorTranslated->Camera()->ConnectNewImageFrame(
          std::bind(&::OnNewFrameTest, &imageCountt, imgt,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            std::placeholders::_4, std::placeholders::_5));
    event::ConnectionPtr c1Left =
      camSensorRotated1->Camera(0)->ConnectNewImageFrame(
          std::bind(&::OnNewFrameTest, &imageCount1Left, img1Left,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            std::placeholders::_4, std::placeholders::_5));
    event::ConnectionPtr c2Left =
      camSensorRotated2->Camera(0)->ConnectNewImageFrame(
          std::bind(&::OnNewFrameTest, &imageCount2Left, img2Left,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            std::placeholders::_4, std::placeholders::_5));

    event::ConnectionPtr c0Right =
      camSensorUnrotated->Camera(1)->ConnectNewImageFrame(
          std::bind(&::OnNewFrameTest, &imageCount0Right, img0Right,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            std::placeholders::_4, std::placeholders::_5));
    event::ConnectionPtr c1Right =
      camSensorRotated1->Camera(1)->ConnectNewImageFrame(
          std::bind(&::OnNewFrameTest, &imageCount1Right, img1Right,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            std::placeholders::_4, std::placeholders::_5));
    event::ConnectionPtr c2Right =
      camSensorRotated2->Camera(1)->ConnectNewImageFrame(
          std::bind(&::OnNewFrameTest, &imageCount2Right, img2Right,
            std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
            std::placeholders::_4, std::placeholders::_5));

    // activate camera
    camSensorUnrotated->SetActive(true);
    camSensorTranslated->SetActive(true);
    camSensorRotated1->SetActive(true);
    camSensorRotated2->SetActive(true);

    // Get at least 10 images from each camera
    int waitCount = 0;
    while (imageCount0Left < 10 || imageCountt < 10 ||
           imageCount1Left < 10 || imageCount2Left < 10 ||
           imageCount0Right < 10 ||
           imageCount1Right < 10 || imageCount2Right < 10)
    {
      // wait at most 10 seconds sim time.
      if (++waitCount >= 1000)
        gzerr << "Err [" << imageCount0Left
              << "/10, " << imageCountt
              << "/10, " << imageCount1Left
              << "/10, " << imageCount2Left
              << "/10, " << imageCount0Right
              << "/10, " << imageCount1Right
              << "/10, " << imageCount2Right
              << "/10] images received from cameras\n";
      EXPECT_LT(waitCount, 1000);

      common::Time::MSleep(10);
    }

    // compare unrotated left against translated left, both should be
    // seeing the green block, if not, pose translation in <camera> tag
    // is broken.
    // Note, right camera of translated camera (imgtRight) should also
    // see green block.
    {
      unsigned int diffMax = 0, diffSum = 0;
      double diffAvg = 0.0;

      // compare left images
      this->ImageCompare(img0Left, imgt, width, height, depth,
                         diffMax, diffSum, diffAvg);

      // We expect that there will be some non-zero difference between the two
      // images.
      EXPECT_EQ(diffSum, 0u);

      // We expect that the average difference will be well within 3-sigma.
      EXPECT_NEAR(diffAvg/255., 0.0, 1e-16);
    }

    // compare unrotated left against translated left, both should be
    // seeing the green block, if not, pose translation in <camera> tag
    // is broken.
    // Note, right camera of translated camera (imgtRight) should also
    // see green block.
    {
      unsigned int diffMax = 0, diffSum = 0;
      double diffAvg = -1.0;

      // compare right of translated camera to
      // right of unrotated/untranslated camera images
      // the result should differ as former sees green block and latter
      // sees red block.
      this->ImageCompare(img0Right, imgt, width, height, depth,
                         diffMax, diffSum, diffAvg);

      // use below to construct test for rotated2 left camera offset
      // ignition::math::Quaterniond a(1.2, 1.3, 1.4);
      // gzerr << "test: " << a.RotateVector(
      // ignition::math::Vector3d::UnitY) << "\n";
      // We expect that there will be some non-zero difference between the two
      // images.
      EXPECT_GT(diffSum, 100u);

      // We expect that the average difference will be well within 3-sigma.
      EXPECT_GT(fabs(diffAvg)/255., 0.0549);
    }

    // compare unrotated against rotated1
    {
      unsigned int diffMax = 0, diffSum = 0;
      double diffAvg = 0.0;

      // compare left images
      this->ImageCompare(img0Left, img1Left, width, height, depth,
                         diffMax, diffSum, diffAvg);

      // We expect that there will be some non-zero difference between the two
      // images.
      EXPECT_EQ(diffSum, 0u);

      // We expect that the average difference will be well within 3-sigma.
      EXPECT_NEAR(diffAvg/255., 0.0, 1e-16);
    }

    // compare unrotated against rotated1
    {
      unsigned int diffMax = 0, diffSum = 0;
      double diffAvg = 0.0;
      // compare right images
      this->ImageCompare(img0Right, img1Right, width, height, depth,
                         diffMax, diffSum, diffAvg);

      // We expect that there will be some non-zero difference between the two
      // images.
      EXPECT_EQ(diffSum, 0u);

      // We expect that the average difference will be well within 3-sigma.
      EXPECT_NEAR(diffAvg/255., 0.0, 1e-16);
    }

    // compare unrotated against rotated2
    {
      unsigned int diffMax = 0, diffSum = 0;
      double diffAvg = 0.0;

      // compare left images
      this->ImageCompare(img0Left, img2Left, width, height, depth,
                         diffMax, diffSum, diffAvg);

      // We expect that there will be some non-zero difference between the two
      // images.
      EXPECT_EQ(diffSum, 0u);

      // We expect that the average difference will be well within 3-sigma.
      EXPECT_NEAR(diffAvg/255., 0.0, 1e-16);
    }

    // compare unrotated against rotated2
    {
      unsigned int diffMax = 0, diffSum = 0;
      double diffAvg = 0.0;

      // compare right images
      this->ImageCompare(img0Right, img2Right, width, height, depth,
                         diffMax, diffSum, diffAvg);

      // We expect that there will be some non-zero difference between the two
      // images.
      EXPECT_EQ(diffSum, 0u);

      // We expect that the average difference will be well within 3-sigma.
      EXPECT_NEAR(diffAvg/255., 0.0, 1e-16);
    }

    // activate camera
    camSensorUnrotated->SetActive(false);
    camSensorTranslated->SetActive(false);
    camSensorRotated1->SetActive(false);
    camSensorRotated2->SetActive(false);

    // disconnect callbacks
    c0Left.reset();
    ct.reset();
    c1Left.reset();
    c2Left.reset();
    c0Right.reset();
    c1Right.reset();
    c2Right.reset();
  }

  // cleanup
  delete[] img0Left;
  delete[] imgt;
  delete[] img1Left;
  delete[] img2Left;
  delete[] img0Right;
  delete[] img1Right;
  delete[] img2Right;
}

/////////////////////////////////////////////////
TEST_F(MultiCameraSensor, CameraRotationWorldPoseTest)
{
  // this test checks Camera::GetWorldRotation and other
  // world pose functions.
  // motivated by issue #1087

  Load("worlds/camera_rotation_test.world");

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  // get two cameras, one with rotation and one without.
  std::string modelUnrotated = "cam_x_rot_test_unrotated_cameras_1";
  std::string multicameraUnrotated = "multicamera_sensor_unrotated";

  std::string modelTranslated = "cam_x_rot_test_translated_camera_1";
  std::string cameraTranslated = "camera_sensor_translated";

  std::string modelRotated1 = "cam_x_rot_test_rotated_cameras_1";
  std::string multicameraRotated1 = "multicamera_sensor_rotated1";

  std::string modelRotated2 = "cam_x_rot_test_rotated_cameras_2";
  std::string multicameraRotated2 = "multicamera_sensor_rotated2";

  physics::ModelPtr model1 = world->ModelByName(modelUnrotated);
  sensors::SensorPtr sensor1 = sensors::get_sensor(multicameraUnrotated);
  sensors::MultiCameraSensorPtr multicamera1 =
    std::dynamic_pointer_cast<sensors::MultiCameraSensor>(sensor1);

  physics::ModelPtr model2 = world->ModelByName(modelTranslated);
  sensors::SensorPtr sensor2 = sensors::get_sensor(cameraTranslated);
  sensors::CameraSensorPtr camera2 =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor2);

  physics::ModelPtr model3 = world->ModelByName(modelRotated1);
  sensors::SensorPtr sensor3 = sensors::get_sensor(multicameraRotated1);
  sensors::MultiCameraSensorPtr multicamera3 =
    std::dynamic_pointer_cast<sensors::MultiCameraSensor>(sensor3);

  physics::ModelPtr model4 = world->ModelByName(modelRotated2);
  sensors::SensorPtr sensor4 = sensors::get_sensor(multicameraRotated2);
  sensors::MultiCameraSensorPtr multicamera4 =
    std::dynamic_pointer_cast<sensors::MultiCameraSensor>(sensor4);

  ASSERT_TRUE(model1 != NULL);
  ASSERT_TRUE(model2 != NULL);
  ASSERT_TRUE(model3 != NULL);
  ASSERT_TRUE(model4 != NULL);
  ASSERT_TRUE(multicamera1 != NULL);
  ASSERT_TRUE(camera2 != NULL);
  ASSERT_TRUE(multicamera3 != NULL);
  ASSERT_TRUE(multicamera4 != NULL);

  // check poses in world frame
  // each multicamera have 2 cameras

  // model 1
  // multicamera1 sensor has zero pose offset from the model
  EXPECT_EQ(model1->WorldPose(), multicamera1->Pose() + model1->WorldPose());
  EXPECT_EQ(model1->WorldPose(), sensor1->Pose() + model1->WorldPose());

  // multicamera1 sensor's camera 0 has a pose offset from the sensor
  EXPECT_NE(multicamera1->Pose() + model1->WorldPose(),
      multicamera1->Camera(0)->WorldPose());

  // Get multicamera1's local pose. There is current no GetPose() in Camera,
  // so grab it from it's ogre scene node
  Ogre::SceneNode *cameraNode = multicamera1->Camera(0)->SceneNode();
  ignition::math::Pose3d cameraPose(
      rendering::Conversions::ConvertIgn(cameraNode->getPosition()),
      rendering::Conversions::ConvertIgn(cameraNode->getOrientation()));

  // Wait for the AttachToVisual request msg to be processed so that the camera
  // is attached to the parent visual.
  int sleep = 0;
  int maxSleep = 100;
  while (cameraPose == multicamera1->Camera(0)->WorldPose()
      && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }

  // verify multicamera sensor's camera world pose
  EXPECT_EQ(cameraPose + multicamera1->Pose() + model1->WorldPose(),
    multicamera1->Camera(0)->WorldPose());
  EXPECT_EQ(model1->WorldPose().Rot() * multicamera1->Pose().Rot()
      * cameraPose.Rot(),
      multicamera1->Camera(0)->WorldRotation());

  // multicamera1 sensor's camera 1 has zero pose offset from the sensor
  EXPECT_EQ(multicamera1->Pose() + model1->WorldPose(),
      multicamera1->Camera(1)->WorldPose());

  gzdbg << "model1 [" << model1->WorldPose() << "]\n"
        << "sensor1 ["
        << sensor1->Pose() + model1->WorldPose() << "]\n"
        << "multicamera1 ["
        << multicamera1->Pose() + model1->WorldPose()
        << "]\n"
        << "camera left WorldPose ["
        << multicamera1->Camera(0)->WorldPose() << "]\n"
        << "camera right WorldPose ["
        << multicamera1->Camera(1)->WorldPose() << "]\n";

  // model 2
  // camera2 sensor has zero pose offset from the model
  EXPECT_EQ(model2->WorldPose(), camera2->Pose() + model2->WorldPose());
  EXPECT_EQ(model2->WorldPose(), sensor2->Pose() + model2->WorldPose());

  // camera2 sensor's camera has zero pose offset from the sensor
  EXPECT_EQ(model2->WorldPose(), camera2->Camera()->WorldPose());

  gzdbg << "model2 [" << model2->WorldPose() << "]\n"
        << "sensor2 ["
        << sensor2->Pose() + model2->WorldPose() << "]\n"
        << "camera2 ["
        << camera2->Pose() + model2->WorldPose() << "]\n"
        << "camera WorldPose [" << camera2->Camera()->WorldPose()
        << "]\n";

  // model 3
  // multicamera3 sensor has zero pose offset from the model
  EXPECT_EQ(model3->WorldPose(), multicamera3->Pose() + model3->WorldPose());
  EXPECT_EQ(model3->WorldPose(), sensor3->Pose() + model3->WorldPose());

  // multicamera3 sensor's camera 0 has a pose offset from the sensor
  EXPECT_NE(multicamera3->Pose() + model3->WorldPose(),
      multicamera3->Camera(0)->WorldPose());
  // Get multicamera3 sensor's camera 0 local pose
  cameraNode = multicamera3->Camera(0)->SceneNode();
  cameraPose = ignition::math::Pose3d(
      rendering::Conversions::ConvertIgn(cameraNode->getPosition()),
      rendering::Conversions::ConvertIgn(cameraNode->getOrientation()));

  // Wait for the AttachToVisual request msg to be processed so that the camera
  // is attached to the parent visual.
  sleep = 0;
  maxSleep = 100;
  while (cameraPose == multicamera3->Camera(0)->WorldPose()
      && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }

  // verify multicamera sensor's camera 0 world pose
  EXPECT_EQ(cameraPose + multicamera3->Pose() +
      model3->WorldPose(), multicamera3->Camera(0)->WorldPose());
  EXPECT_EQ(model3->WorldPose().Rot() * multicamera3->Pose().Rot()
      * cameraPose.Rot(),
      multicamera3->Camera(0)->WorldRotation());

  // multicamera3 sensor's camera 1 has zero pose offset from the sensor
  EXPECT_EQ(multicamera3->Pose() + model3->WorldPose(),
      multicamera3->Camera(1)->WorldPose());
  EXPECT_EQ(model3->WorldPose(),
      multicamera3->Camera(1)->WorldPose());

  gzdbg << "model3 [" << model3->WorldPose() << "]\n"
        << "sensor3 ["
        << sensor3->Pose() + model3->WorldPose() << "]\n"
        << "multicamera3 ["
        << multicamera3->Pose() + model3->WorldPose()
        << "]\n"
        << "camera left  WorldPose ["
        << multicamera3->Camera(0)->WorldPose() << "]\n"
        << "camera right WorldPose ["
        << multicamera3->Camera(1)->WorldPose() << "]\n";

  // model 4
  // multicamera4 sensor has zero pose offset from the model
  EXPECT_EQ(model4->WorldPose(),
    multicamera4->Pose() + model4->WorldPose());
  EXPECT_EQ(model4->WorldPose(), sensor4->Pose() + model4->WorldPose());

  // multicamera4 sensor's camera 0 has a pose offset from the sensor
  EXPECT_NE(model4->WorldPose(), multicamera4->Camera(0)->WorldPose());
  // Get multicamera4's camera 0 local pose
  cameraNode = multicamera4->Camera(0)->SceneNode();
  cameraPose = ignition::math::Pose3d(
      rendering::Conversions::ConvertIgn(cameraNode->getPosition()),
      rendering::Conversions::ConvertIgn(cameraNode->getOrientation()));

  // Wait for the AttachToVisual request msg to be processed so that the camera
  // is attached to the parent visual.
  sleep = 0;
  maxSleep = 100;
  while (cameraPose == multicamera4->Camera(0)->WorldPose()
      && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }

  // verify multicamera sensor's camera 0 world pose
  EXPECT_EQ(cameraPose + multicamera4->Pose() + model4->WorldPose(),
    multicamera4->Camera(0)->WorldPose());
  EXPECT_EQ(model4->WorldPose().Rot() * multicamera4->Pose().Rot()
      * cameraPose.Rot(), multicamera4->Camera(0)->WorldRotation());

  // multicamera4 sensor's camera 1 has a pose offset from the sensor
  EXPECT_NE(model4->WorldPose(), multicamera4->Camera(1)->WorldPose());
  // Get multicamera4 sensor's camera 1 local pose
  cameraNode = multicamera4->Camera(1)->SceneNode();
  cameraPose = ignition::math::Pose3d(
      rendering::Conversions::ConvertIgn(cameraNode->getPosition()),
      rendering::Conversions::ConvertIgn(cameraNode->getOrientation()));

  // Wait for the AttachToVisual request msg to be processed so that the camera
  // is attached to the parent visual.
  sleep = 0;
  maxSleep = 100;
  while (cameraPose == multicamera4->Camera(1)->WorldPose()
      && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }

  // verify multicamera4 sensor's camera 1 world pose
  EXPECT_EQ(cameraPose + multicamera4->Pose() + model4->WorldPose(),
    multicamera4->Camera(1)->WorldPose());
  EXPECT_EQ(model4->WorldPose().Rot() * multicamera4->Pose().Rot()
      * cameraPose.Rot(), multicamera4->Camera(1)->WorldRotation());

  gzdbg << "model4 [" << model4->WorldPose() << "]\n"
        << "sensor4 ["
        << sensor4->Pose() + model4->WorldPose() << "]\n"
        << "multicamera4 ["
        << multicamera4->Pose() + model4->WorldPose()
        << "]\n"
        << "camera1 WorldPose [" << multicamera4->Camera(0)->WorldPose()
        << "]\n"
        << "camera2 WorldPose [" << multicamera4->Camera(1)->WorldPose()
        << "]\n";
}

/////////////////////////////////////////////////
TEST_F(MultiCameraSensor, StrictUpdateRate)
{
  LoadArgs(" --lockstep worlds/multicamera_strict_rate_test.world");

  // Make sure the render engine is available.
  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
  {
    gzerr << "No rendering engine, unable to run camera test\n";
    return;
  }

  std::string sensorName = "multicamera_sensor";
  sensors::SensorPtr sensor = sensors::get_sensor(sensorName);
  sensors::MultiCameraSensorPtr multiCamSensor =
    std::dynamic_pointer_cast<sensors::MultiCameraSensor>(sensor);
  EXPECT_TRUE(multiCamSensor != nullptr);
  multiCamSensor->SetActive(true);

  // 3 cameras
  EXPECT_EQ(multiCamSensor->CameraCount(), 3u);
  int imageCount0 = 0;
  unsigned char* img0 = new unsigned char[
      multiCamSensor->ImageWidth(0) * multiCamSensor->ImageHeight(0) * 3];
  event::ConnectionPtr c0 = multiCamSensor->Camera(0)->ConnectNewImageFrame(
        std::bind(&::OnNewFrameTest, &imageCount0, img0,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  int imageCount1 = 0;
  unsigned char* img1 = new unsigned char[
      multiCamSensor->ImageWidth(1) * multiCamSensor->ImageHeight(1) * 3];
  event::ConnectionPtr c1 = multiCamSensor->Camera(1)->ConnectNewImageFrame(
        std::bind(&::OnNewFrameTest, &imageCount1, img1,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  int imageCount2 = 0;
  unsigned char* img2 = new unsigned char[
      multiCamSensor->ImageWidth(2) * multiCamSensor->ImageHeight(2) * 3];
  event::ConnectionPtr c2 = multiCamSensor->Camera(2)->ConnectNewImageFrame(
        std::bind(&::OnNewFrameTest, &imageCount2, img2,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5));

  common::Timer timer;
  timer.Start();

  // how many images produced for 5 seconds (in simulated clock domain)
  double updateRate = multiCamSensor->UpdateRate();
  int totalImagesPerCam = 5 * updateRate;
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  double simT0 = 0.0;

  while (imageCount0 < totalImagesPerCam || imageCount1 < totalImagesPerCam ||
      imageCount2 < totalImagesPerCam)
  {
    // An approximation of when we receive the first image. In reality one
    // iteration before we receive the second image.
    if (imageCount0 == 0)
    {
      simT0 = world->SimTime().Double();
    }
    common::Time::MSleep(1);
  }

  // check that the obtained rate is the one expected
  double dt = world->SimTime().Double() - simT0;
  double rate = static_cast<double>(totalImagesPerCam) / dt;
  gzdbg << "timer [" << dt << "] seconds rate [" << rate << "] fps\n";
  const double tolerance = 0.02;
  EXPECT_GT(rate, updateRate * (1 - tolerance));
  EXPECT_LT(rate, updateRate * (1 + tolerance));

  c0.reset();
  c1.reset();
  c2.reset();
  delete [] img0;
  delete [] img1;
  delete [] img2;
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
