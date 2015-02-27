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

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorsIface.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/common.hh"
#include "gazebo/common/Timer.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/rendering.hh"
#include "gazebo/sensors/MultiCameraSensor.hh"

#include "ServerFixture.hh"
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
    boost::dynamic_pointer_cast<sensors::MultiCameraSensor>(sensor);

  sensor = sensors::get_sensor(cameraTranslated);
  sensors::CameraSensorPtr camSensorTranslated =
    boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);

  sensor = sensors::get_sensor(cameraRotated1);
  sensors::MultiCameraSensorPtr camSensorRotated1 =
    boost::dynamic_pointer_cast<sensors::MultiCameraSensor>(sensor);

  sensor = sensors::get_sensor(cameraRotated2);
  sensors::MultiCameraSensorPtr camSensorRotated2 =
    boost::dynamic_pointer_cast<sensors::MultiCameraSensor>(sensor);

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
      camSensorUnrotated->GetCamera(0)->ConnectNewImageFrame(
          boost::bind(&::OnNewFrameTest, &imageCount0Left, img0Left,
            _1, _2, _3, _4, _5));
    event::ConnectionPtr ct =
      camSensorTranslated->GetCamera()->ConnectNewImageFrame(
          boost::bind(&::OnNewFrameTest, &imageCountt, imgt,
            _1, _2, _3, _4, _5));
    event::ConnectionPtr c1Left =
      camSensorRotated1->GetCamera(0)->ConnectNewImageFrame(
          boost::bind(&::OnNewFrameTest, &imageCount1Left, img1Left,
            _1, _2, _3, _4, _5));
    event::ConnectionPtr c2Left =
      camSensorRotated2->GetCamera(0)->ConnectNewImageFrame(
          boost::bind(&::OnNewFrameTest, &imageCount2Left, img2Left,
            _1, _2, _3, _4, _5));

    event::ConnectionPtr c0Right =
      camSensorUnrotated->GetCamera(1)->ConnectNewImageFrame(
          boost::bind(&::OnNewFrameTest, &imageCount0Right, img0Right,
            _1, _2, _3, _4, _5));
    event::ConnectionPtr c1Right =
      camSensorRotated1->GetCamera(1)->ConnectNewImageFrame(
          boost::bind(&::OnNewFrameTest, &imageCount1Right, img1Right,
            _1, _2, _3, _4, _5));
    event::ConnectionPtr c2Right =
      camSensorRotated2->GetCamera(1)->ConnectNewImageFrame(
          boost::bind(&::OnNewFrameTest, &imageCount2Right, img2Right,
            _1, _2, _3, _4, _5));

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
      // math::Quaternion a(1.2, 1.3, 1.4);
      // gzerr << "test: " << a.RotateVector(math::Vector3(0, 1, 0)) << "\n";

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
    camSensorUnrotated->GetCamera(0)->DisconnectNewImageFrame(c0Left);
    camSensorTranslated->GetCamera()->DisconnectNewImageFrame(ct);
    camSensorRotated1->GetCamera(0)->DisconnectNewImageFrame(c1Left);
    camSensorRotated2->GetCamera(0)->DisconnectNewImageFrame(c2Left);
    camSensorUnrotated->GetCamera(1)->DisconnectNewImageFrame(c0Right);
    camSensorRotated1->GetCamera(1)->DisconnectNewImageFrame(c1Right);
    camSensorRotated2->GetCamera(1)->DisconnectNewImageFrame(c2Right);
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

  physics::ModelPtr model1 = world->GetModel(modelUnrotated);
  sensors::SensorPtr sensor1 = sensors::get_sensor(multicameraUnrotated);
  sensors::MultiCameraSensorPtr multicamera1 =
    boost::dynamic_pointer_cast<sensors::MultiCameraSensor>(sensor1);

  physics::ModelPtr model2 = world->GetModel(modelTranslated);
  sensors::SensorPtr sensor2 = sensors::get_sensor(cameraTranslated);
  sensors::CameraSensorPtr camera2 =
    boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor2);

  physics::ModelPtr model3 = world->GetModel(modelRotated1);
  sensors::SensorPtr sensor3 = sensors::get_sensor(multicameraRotated1);
  sensors::MultiCameraSensorPtr multicamera3 =
    boost::dynamic_pointer_cast<sensors::MultiCameraSensor>(sensor3);

  physics::ModelPtr model4 = world->GetModel(modelRotated2);
  sensors::SensorPtr sensor4 = sensors::get_sensor(multicameraRotated2);
  sensors::MultiCameraSensorPtr multicamera4 =
    boost::dynamic_pointer_cast<sensors::MultiCameraSensor>(sensor4);

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
  EXPECT_EQ(model1->GetWorldPose(),
    multicamera1->GetPose() + model1->GetWorldPose());
  EXPECT_EQ(model1->GetWorldPose(),
    sensor1->GetPose() + model1->GetWorldPose());

  // multicamera1 sensor's camera 0 has a pose offset from the sensor
  EXPECT_NE(multicamera1->GetPose() + model1->GetWorldPose(),
      multicamera1->GetCamera(0)->GetWorldPose());

  // Get multicamera1's local pose. There is current no GetPose() in Camera,
  // so grab it from it's ogre scene node
  Ogre::SceneNode *cameraNode = multicamera1->GetCamera(0)->GetSceneNode();
  math::Pose cameraPose(
      rendering::Conversions::Convert(cameraNode->getPosition()),
      rendering::Conversions::Convert(cameraNode->getOrientation()));

  // Wait for the AttachToVisual request msg to be processed so that the camera
  // is attached to the parent visual.
  int sleep = 0;
  int maxSleep = 100;
  while (cameraPose == multicamera1->GetCamera(0)->GetWorldPose()
      && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }

  // verify multicamera sensor's camera world pose
  EXPECT_EQ(cameraPose + multicamera1->GetPose() + model1->GetWorldPose(),
    multicamera1->GetCamera(0)->GetWorldPose());
  EXPECT_EQ(model1->GetWorldPose().rot * multicamera1->GetPose().rot
      * cameraPose.rot, multicamera1->GetCamera(0)->GetWorldRotation());

  // multicamera1 sensor's camera 1 has zero pose offset from the sensor
  EXPECT_EQ(multicamera1->GetPose() + model1->GetWorldPose(),
      multicamera1->GetCamera(1)->GetWorldPose());

  gzdbg << "model1 [" << model1->GetWorldPose() << "]\n"
        << "sensor1 [" << sensor1->GetPose() + model1->GetWorldPose() << "]\n"
        << "multicamera1 [" << multicamera1->GetPose() + model1->GetWorldPose()
        << "]\n"
        << "camera left WorldPose ["
        << multicamera1->GetCamera(0)->GetWorldPose() << "]\n"
        << "camera right WorldPose ["
        << multicamera1->GetCamera(1)->GetWorldPose() << "]\n";

  // model 2
  // camera2 sensor has zero pose offset from the model
  EXPECT_EQ(model2->GetWorldPose(),
    camera2->GetPose() + model2->GetWorldPose());
  EXPECT_EQ(model2->GetWorldPose(),
    sensor2->GetPose() + model2->GetWorldPose());

  // camera2 sensor's camera has zero pose offset from the sensor
  EXPECT_EQ(model2->GetWorldPose(), camera2->GetCamera()->GetWorldPose());

  gzdbg << "model2 [" << model2->GetWorldPose() << "]\n"
        << "sensor2 [" << sensor2->GetPose() + model2->GetWorldPose() << "]\n"
        << "camera2 [" << camera2->GetPose() + model2->GetWorldPose() << "]\n"
        << "camera WorldPose [" << camera2->GetCamera()->GetWorldPose()
        << "]\n";

  // model 3
  // multicamera3 sensor has zero pose offset from the model
  EXPECT_EQ(model3->GetWorldPose(),
      multicamera3->GetPose() + model3->GetWorldPose());
  EXPECT_EQ(model3->GetWorldPose(),
      sensor3->GetPose() + model3->GetWorldPose());

  // multicamera3 sensor's camera 0 has a pose offset from the sensor
  EXPECT_NE(multicamera3->GetPose() + model3->GetWorldPose(),
      multicamera3->GetCamera(0)->GetWorldPose());
  // Get multicamera3 sensor's camera 0 local pose
  cameraNode = multicamera3->GetCamera(0)->GetSceneNode();
  cameraPose = math::Pose(
      rendering::Conversions::Convert(cameraNode->getPosition()),
      rendering::Conversions::Convert(cameraNode->getOrientation()));

  // Wait for the AttachToVisual request msg to be processed so that the camera
  // is attached to the parent visual.
  sleep = 0;
  maxSleep = 100;
  while (cameraPose == multicamera3->GetCamera(0)->GetWorldPose()
      && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }

  // verify multicamera sensor's camera 0 world pose
  EXPECT_EQ(cameraPose + multicamera3->GetPose() + model3->GetWorldPose(),
    multicamera3->GetCamera(0)->GetWorldPose());
  EXPECT_EQ(model3->GetWorldPose().rot * multicamera3->GetPose().rot
      * cameraPose.rot, multicamera3->GetCamera(0)->GetWorldRotation());

  // multicamera3 sensor's camera 1 has zero pose offset from the sensor
  EXPECT_EQ(multicamera3->GetPose() + model3->GetWorldPose(),
      multicamera3->GetCamera(1)->GetWorldPose());
  EXPECT_EQ(model3->GetWorldPose(), multicamera3->GetCamera(1)->GetWorldPose());

  gzdbg << "model3 [" << model3->GetWorldPose() << "]\n"
        << "sensor3 [" << sensor3->GetPose() + model3->GetWorldPose() << "]\n"
        << "multicamera3 [" << multicamera3->GetPose() + model3->GetWorldPose()
        << "]\n"
        << "camera left  WorldPose ["
        << multicamera3->GetCamera(0)->GetWorldPose() << "]\n"
        << "camera right WorldPose ["
        << multicamera3->GetCamera(1)->GetWorldPose() << "]\n";

  // model 4
  // multicamera4 sensor has zero pose offset from the model
  EXPECT_EQ(model4->GetWorldPose(),
    multicamera4->GetPose() + model4->GetWorldPose());
  EXPECT_EQ(model4->GetWorldPose(),
    sensor4->GetPose() + model4->GetWorldPose());

  // multicamera4 sensor's camera 0 has a pose offset from the sensor
  EXPECT_NE(model4->GetWorldPose(), multicamera4->GetCamera(0)->GetWorldPose());
  // Get multicamera4's camera 0 local pose
  cameraNode = multicamera4->GetCamera(0)->GetSceneNode();
  cameraPose = math::Pose(
      rendering::Conversions::Convert(cameraNode->getPosition()),
      rendering::Conversions::Convert(cameraNode->getOrientation()));

  // Wait for the AttachToVisual request msg to be processed so that the camera
  // is attached to the parent visual.
  sleep = 0;
  maxSleep = 100;
  while (cameraPose == multicamera4->GetCamera(0)->GetWorldPose()
      && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }

  // verify multicamera sensor's camera 0 world pose
  EXPECT_EQ(cameraPose + multicamera4->GetPose() + model4->GetWorldPose(),
    multicamera4->GetCamera(0)->GetWorldPose());
  EXPECT_EQ(model4->GetWorldPose().rot * multicamera4->GetPose().rot
      * cameraPose.rot, multicamera4->GetCamera(0)->GetWorldRotation());

  // multicamera4 sensor's camera 1 has a pose offset from the sensor
  EXPECT_NE(model4->GetWorldPose(), multicamera4->GetCamera(1)->GetWorldPose());
  // Get multicamera4 sensor's camera 1 local pose
  cameraNode = multicamera4->GetCamera(1)->GetSceneNode();
  cameraPose = math::Pose(
      rendering::Conversions::Convert(cameraNode->getPosition()),
      rendering::Conversions::Convert(cameraNode->getOrientation()));

  // Wait for the AttachToVisual request msg to be processed so that the camera
  // is attached to the parent visual.
  sleep = 0;
  maxSleep = 100;
  while (cameraPose == multicamera4->GetCamera(1)->GetWorldPose()
      && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    sleep++;
  }

  // verify multicamera4 sensor's camera 1 world pose
  EXPECT_EQ(cameraPose + multicamera4->GetPose() + model4->GetWorldPose(),
    multicamera4->GetCamera(1)->GetWorldPose());
  EXPECT_EQ(model4->GetWorldPose().rot * multicamera4->GetPose().rot
      * cameraPose.rot, multicamera4->GetCamera(1)->GetWorldRotation());

  gzdbg << "model4 [" << model4->GetWorldPose() << "]\n"
        << "sensor4 [" << sensor4->GetPose() + model4->GetWorldPose() << "]\n"
        << "multicamera4 [" << multicamera4->GetPose() + model4->GetWorldPose()
        << "]\n"
        << "camera1 WorldPose [" << multicamera4->GetCamera(0)->GetWorldPose()
        << "]\n"
        << "camera2 WorldPose [" << multicamera4->GetCamera(1)->GetWorldPose()
        << "]\n";
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
