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

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/common.hh"
#include "gazebo/test/helper_physics_generator.hh"

#define LASER_TOL 1e-5
#define DOUBLE_TOL 1e-6

using namespace gazebo;
class NoiseTest : public ServerFixture,
                  public testing::WithParamInterface<const char*>
{
  public: void NoisePlugin(const std::string &_physicsEngine);
};


void NoiseTest::NoisePlugin(const std::string &_physicsEngine)
{
  // Test ray sensor with noise applied
  Load("worlds/empty.world", false, _physicsEngine);

  std::string raySensorName = "raySensor";
  std::string modelName = "rayModel";
  std::string pluginFileName = "libRaySensorNoisePlugin.so";
  double maxRange = 5.0;

  msgs::Factory msg;
  std::ostringstream newModelStr;
  newModelStr << "<sdf version='" << SDF_VERSION << "'>"
    << "<model name ='" << modelName << "'>"
    << "<static>true</static>"
    << "<pose> 0 0 0.5 0 0 0 </pose>"
    << "<link name ='body'>"
    << "<collision name='parent_collision'>"
    << "  <geometry>"
    << "    <cylinder>"
    << "      <radius>0.02</radius>"
    << "      <length>0.03</length>"
    << "    </cylinder>"
    << "  </geometry>"
    << "</collision>"
    << "  <sensor name ='" << raySensorName << "' type ='ray'>"
    << "    <ray>"
    << "      <scan>"
    << "        <horizontal>"
    << "          <samples>100</samples>"
    << "          <resolution>1</resolution>"
    << "          <min_angle>-1</min_angle>"
    << "          <max_angle>1</max_angle>"
    << "        </horizontal>"
    << "      </scan>"
    << "      <range>"
    << "        <min>0.1</min>"
    << "        <max>" << maxRange << "</max>"
    << "      </range>"
    << "      <noise>"
    << "        <type>custom</type>"
    << "      </noise>"
    << "    </ray>"
    << "    <plugin name ='laser' filename='" << pluginFileName << "'/>"
    << "  </sensor>"
    << "</link>"
    << "</model>"
    << "</sdf>";

  msg.set_sdf(newModelStr.str());
  this->factoryPub->Publish(msg);

  WaitUntilEntitySpawn(modelName, 100, 100);
  WaitUntilSensorSpawn(raySensorName, 100, 100);

  sensors::SensorPtr sensor = sensors::get_sensor(raySensorName);
  sensors::RaySensorPtr raySensor =
    boost::dynamic_pointer_cast<sensors::RaySensor>(sensor);

  EXPECT_TRUE(raySensor != NULL);

  raySensor->Init();
  raySensor->Update(true);

  // Expect at least one value to be non-max (empty world).
  // Expect the range to be within (max-noise) < max < (max+noise), see
  // custom noise model in RaySensorNoisePlugin.
  // Noise rate value also taken directly from plugin.
  double fixedNoiseRate = 0.005;
  double noise = maxRange*fixedNoiseRate;
  for (int i = 0; i < raySensor->GetRayCount(); ++i)
  {
    double range = raySensor->GetRange(i);
    if (std::isinf(range))
    {
      continue;
    }

    EXPECT_TRUE(range >= maxRange - noise);
    EXPECT_TRUE(range <= maxRange + noise);
  }
}

TEST_P(NoiseTest, NoisePlugin)
{
  NoisePlugin(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, NoiseTest, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
