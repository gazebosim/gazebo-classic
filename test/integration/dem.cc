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

#include <ignition/math/Angle.hh>
#include "gazebo/common/Dem.hh"
#include "gazebo/sensors/GpsSensor.hh"
#include "gazebo/test/ServerFixture.hh"

#define DOUBLE_TOL 1e-3

using namespace gazebo;
class Dem_TEST : public ServerFixture
{
  /////////////////////////////////////////////////
  public: void OnNewCameraFrame(int* _imageCounter, unsigned char* _imageDest,
                    const unsigned char *_image,
                    unsigned int _width, unsigned int _height,
                    unsigned int _depth,
                    const std::string &/*_format*/)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    memcpy(_imageDest, _image, _width * _height * _depth);
    *_imageCounter += 1;
  }

  public: unsigned char* img = NULL;
  private: std::mutex mutex;
};

#ifdef HAVE_GDAL
/////////////////////////////////////////////////
/// \brief Test the integration between GPS and a DEM terrain.
TEST_F(Dem_TEST, GPS)
{
  ignition::math::Angle latitude, longitude;
  double elevation;
  common::Dem dem;
  boost::filesystem::path path = "file://media/dem/volcano.tif";

  // Load a DEM world with a GPS sensor (without noise) attached to a box.
  Load("worlds/dem_gps.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  physics::ModelPtr model = world->ModelByName("box1");
  ASSERT_TRUE(model != NULL);

  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  // Update the sensor manager so that it can process new sensors.
  mgr->Update();

  // Get a pointer to the GPS sensor.
  sensors::GpsSensorPtr sensor =
    std::dynamic_pointer_cast<sensors::GpsSensor>(
        mgr->GetSensor("gps"));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != NULL);
  EXPECT_TRUE(sensor->IsActive());
  sensor->Update(true);

  // Get the georeference coordinates of the DEM's origin
  dem.Load(path.string());
  dem.GetGeoReferenceOrigin(latitude, longitude);
  elevation = dem.GetElevation(0.0, 0.0);

  EXPECT_NEAR(sensor->Latitude().Degree(), latitude.Degree(), DOUBLE_TOL);
  EXPECT_NEAR(sensor->Longitude().Degree(), longitude.Degree(), DOUBLE_TOL);

  // Sensor altitude is the elevation of the terrain + the sensor position.
  EXPECT_NEAR(sensor->Altitude(),
      elevation + model->WorldPose().Pos().Z(), 1);
}

/////////////////////////////////////////////////
/// \brief Test the integration between a camera and a DEM terrain.
TEST_F(Dem_TEST, Camera)
{
  // Load a DEM world with a GPS sensor (without noise) attached to a box.
  Load("worlds/heightmap_dem.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // spawn camera sensor to capture an image of heightmap
  std::string modelName = "camera_model";
  std::string cameraName = "camera_sensor";
  unsigned int width  = 320;
  unsigned int height = 240;
  double updateRate = 10;
  ignition::math::Pose3d testPose(
      ignition::math::Vector3d(0, 0, 100),
      ignition::math::Quaterniond(0, 1.57, 0));
  SpawnCamera(modelName, cameraName, testPose.Pos(),
      testPose.Rot().Euler(), width, height, updateRate);

  // Get a pointer to the CameraSensor.
  sensors::SensorPtr sensor = sensors::get_sensor(cameraName);
  sensors::CameraSensorPtr camSensor =
    std::dynamic_pointer_cast<sensors::CameraSensor>(sensor);

  // Make sure the above dynamic cast worked.
  ASSERT_TRUE(camSensor != NULL);
  EXPECT_TRUE(camSensor->IsActive());
  camSensor->Update(true);

  int imageCount = 0;
  this->img = new unsigned char[width*height*3];
  event::ConnectionPtr c =
      camSensor->Camera()->ConnectNewImageFrame(
      std::bind(&Dem_TEST::OnNewCameraFrame, this, &imageCount, this->img,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
      std::placeholders::_4, std::placeholders::_5));

  // grab some images
  int sleep = 0;
  int maxSleep = 500;
  int total_images = 10;
  while (imageCount < total_images && sleep++ < maxSleep )
    common::Time::MSleep(10);
  EXPECT_GE(imageCount, total_images);

  c.reset();

  unsigned int rSum = 0;
  unsigned int gSum = 0;
  unsigned int bSum = 0;
  for (unsigned int i = 0; i < height*width*3; i+=3)
  {
    unsigned int r = img[i];
    unsigned int g = img[i+1];
    unsigned int b = img[i+2];
    rSum += r;
    gSum += g;
    bSum += b;
  }

  // verify that red is the dominant color in the image
  EXPECT_GT(rSum, gSum);
  EXPECT_GT(rSum, bSum);

  delete [] this->img;
}
#endif

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
