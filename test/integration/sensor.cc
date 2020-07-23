/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include <string.h>
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class SensorTest : public ServerFixture
{
};

std::mutex g_mutex;
unsigned int g_messageCount = 0;

////////////////////////////////////////////////////////////////////////
void SensorCallback(const ConstIMUSensorPtr &/*_msg*/)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  g_messageCount++;
}

/////////////////////////////////////////////////
// This tests getting links from a model.
TEST_F(SensorTest, GetScopedName)
{
  Load("worlds/camera_pose_test.world");

  sensors::SensorPtr sensor = sensors::get_sensor("cam1");
  ASSERT_TRUE(sensor != NULL);

  std::string sensorName = sensor->ScopedName();
  EXPECT_EQ(sensorName, std::string("default::rotated_box::link::cam1"));
}

/////////////////////////////////////////////////
// Make sure sensors can run without asserting in a world with a large
// step size.
TEST_F(SensorTest, FastSensor)
{
  Load("worlds/fast_sensor_test.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // This test will cause an assertion if maxSensorUpdate in
  // SensorManager::SensorContainer::RunLoop() is set improperly
}

/////////////////////////////////////////////////
// Make sure sensors update rates are respected
// Spawn two sensors, one after another, with different update rates and
// verify the rates are correctly throttled
TEST_F(SensorTest, MaxUpdateRate)
{
  Load("worlds/empty.world");

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_NE(nullptr, world);

  auto spawnSensorWithUpdateRate = [&](const std::string &_name,
      const ignition::math::Pose3d &_pose, double _rate)
  {
    std::ostringstream newModelStr;
    newModelStr << "<sdf version='" << SDF_VERSION << "'>"
      << "<model name ='" << _name << "'>\n"
      << "<static>true</static>\n"
      << "<pose>" << _pose << "</pose>\n"
      << "<link name ='body'>\n"
      << "<inertial>\n"
      << "<mass>0.1</mass>\n"
      << "</inertial>\n"
      << "<collision name='parent_collision'>\n"
      << "  <pose>0 0 0.0205 0 0 0</pose>\n"
      << "  <geometry>\n"
      << "    <cylinder>\n"
      << "      <radius>0.021</radius>\n"
      << "      <length>0.029</length>\n"
      << "    </cylinder>\n"
      << "  </geometry>\n"
      << "</collision>\n"
      << "  <sensor name ='" << _name << "' type ='imu'>\n"
      << "    <update_rate>" << _rate << "</update_rate>\n"
      << "    <topic>" << _name << "</topic>\n"
      << "    <imu>\n"
      << "    </imu>\n"
      << "  </sensor>\n"
      << "</link>\n"
      << "</model>\n"
      << "</sdf>\n";

    SpawnSDF(newModelStr.str());
  };

  transport::NodePtr node = transport::NodePtr(new transport::Node());
  node->Init();

  g_messageCount = 0;

  // spawn first sensor with low update rate
  spawnSensorWithUpdateRate("sensor1", ignition::math::Pose3d::Zero, 5);

  transport::SubscriberPtr sub = node->Subscribe("~/sensor1/body/sensor1/imu",
      SensorCallback);

  // wait for messages
  int sleep = 0;
  int maxSleep = 1000;
  double t0 = 0.0;
  while (g_messageCount < 30 && sleep++ < maxSleep)
  {
    if (g_messageCount == 0)
      t0 = world->SimTime().Double();
    common::Time::MSleep(10);
  }

  // verify update rate by checking the time it takes to receive n msgs
  double elapsed = world->SimTime().Double() - t0;
  EXPECT_NEAR(6.0, elapsed, 0.5);

  // disconnect first sensor
  sub.reset();

  g_messageCount = 0;

  // spawn another sensor with higher update rate
  spawnSensorWithUpdateRate("sensor2", ignition::math::Pose3d::Zero, 10);
  sub = node->Subscribe("~/sensor2/body/sensor2/imu", SensorCallback);

  // wait for more msgs
  sleep = 0;
  t0 = 0.0;
  while (g_messageCount < 30 && sleep++ < maxSleep)
  {
    if (g_messageCount == 0)
      t0 = world->SimTime().Double();
    common::Time::MSleep(10);
  }

  // verify update rate by checking the time it takes to receive n msgs
  elapsed = world->SimTime().Double() - t0;
  EXPECT_NEAR(3.0, elapsed, 0.5);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
