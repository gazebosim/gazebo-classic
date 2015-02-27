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

#include <boost/filesystem.hpp>
#include "gazebo/common/Dem.hh"
#include "gazebo/math/Angle.hh"
#include "gazebo/sensors/GpsSensor.hh"
#include "ServerFixture.hh"

#define DOUBLE_TOL 1e-3

using namespace gazebo;
class Dem_TEST : public ServerFixture
{
};

#ifdef HAVE_GDAL
/////////////////////////////////////////////////
/// \brief Test the integration between GPS and a DEM terrain.
TEST_F(Dem_TEST, GPS)
{
  math::Angle latitude, longitude;
  double elevation;
  common::Dem dem;
  boost::filesystem::path path = "file://media/dem/volcano.tif";

  // Load a DEM world with a GPS sensor (without noise) attached to a box.
  Load("worlds/dem_gps.world");
  sensors::SensorManager *mgr = sensors::SensorManager::Instance();

  // Update the sensor manager so that it can process new sensors.
  mgr->Update();

  // Get a pointer to the GPS sensor.
  sensors::GpsSensorPtr sensor =
    boost::dynamic_pointer_cast<sensors::GpsSensor>(
        mgr->GetSensor("gps"));

  // Make sure the above dynamic cast worked.
  EXPECT_TRUE(sensor != NULL);
  EXPECT_TRUE(sensor->IsActive());
  sensor->Update(true);

  // Get the georeference coordinates of the DEM's origin
  dem.Load(path.string());
  dem.GetGeoReferenceOrigin(latitude, longitude);
  elevation = dem.GetElevation(0.0, 0.0);

  EXPECT_NEAR(sensor->GetLatitude().Degree(), latitude.Degree(), DOUBLE_TOL);
  EXPECT_NEAR(sensor->GetLongitude().Degree(), longitude.Degree(), DOUBLE_TOL);
  EXPECT_NEAR(sensor->GetAltitude(), elevation, 1);
}
#endif

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
