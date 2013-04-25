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

#include <gtest/gtest.h>
#include "gazebo/sdf/sdf.hh"
#include "gazebo/math/Angle.hh"
#include "test/ServerFixture.hh"

using namespace gazebo;
class RaySensor_TEST : public ServerFixture
{
};

static std::string raySensorString =
"<sdf version='1.3'>"
"  <sensor name='laser' type='ray'>"
"    <always_on>1</always_on>"
"    <visualize>1</visualize>"
"    <update_rate>20.000000</update_rate>"
"    <ray>"
"      <scan>"
"        <horizontal>"
"          <samples>640</samples>"
"          <resolution>1.000000</resolution>"
"          <min_angle>-2.2689</min_angle>"
"          <max_angle>2.2689</max_angle>"
"        </horizontal>"
"      </scan>"
"      <range>"
"        <min>0.08</min>"
"        <max>10.0</max>"
"        <resolution>0.01</resolution>"
"      </range>"
"    </ray>"
"  </sensor>"
"</sdf>";

/////////////////////////////////////////////////
/// \brief Test Creation of a Ray sensor
TEST_F(RaySensor_TEST, CreateLaser)
{
    Load("worlds/empty.world");
  

    sensors::SensorManager *mgr = sensors::SensorManager::Instance();

    sdf::ElementPtr sdf(new sdf::Element);
    sdf::initFile("sensor.sdf", sdf);
    sdf::readString(raySensorString, sdf);

    // Create the Ray sensor
    std::string sensorName = mgr->CreateSensor(sdf, "default",
        "ground_plane::link");

    // Update the sensor manager so that it can process new sensors.
    mgr->Update();

    // Get a pointer to the Ray sensor
    sensors::RaySensorPtr sensor = boost::dynamic_pointer_cast<sensors::RaySensor>
      (mgr->GetSensor(sensorName));

    // Update the sensor
    sensor->Update(true);

    // Uncomment this sleep will fix the segfault
    // sleep(1);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
