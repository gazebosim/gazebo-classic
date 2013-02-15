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
#include "scans_cmp.h"

using namespace gazebo;
class LaserTest : public ServerFixture
{
};

TEST_F(LaserTest, Stationary_EmptyWorld)
{
  Load("worlds/empty.world");
  std::ostringstream laserModel;
    laserModel << "<sdf version='" << SDF_VERSION << "'>"
      "<model name='box'>"
        "<static>true</static>"
        "<link name='link'>"
        "<pose>0 0 0.5 0 0 0</pose>"
        "<sensor name='laser' type='ray'>"
          "<always_on>true</always_on>"
          "<update_rate>10</update_rate>"
          "<visualize>true</visualize>"
          "<topic>~/laser_scan</topic>"
          "<ray>"
            "<scan>"
              "<horizontal>"
                "<samples>640</samples>"
                "<resolution>1</resolution>"
                "<min_angle>-2.27</min_angle>"
                "<max_angle>2.27</max_angle>"
              "</horizontal>"
            "</scan>"
            "<range>"
              "<min>0.0</min>"
              "<max>10</max>"
              "<resolution>0.01</resolution>"
            "</range>"
          "</ray>"
        "</sensor>"
      "</link>"
    "</model>"
  "</sdf>";

  SpawnSDF(laserModel.str());
  while (!HasEntity("box"))
    common::Time::MSleep(10);

  sensors::RaySensorPtr laser =
    boost::shared_static_cast<sensors::RaySensor>(
        sensors::SensorManager::Instance()->GetSensor(
          "default::box::link::laser"));
  EXPECT_TRUE(laser);
  laser->Update(true);
  while (laser->GetRange(0) == 0)
    common::Time::MSleep(100);

  EXPECT_EQ(640, laser->GetRayCount());
  EXPECT_EQ(640, laser->GetRangeCount());

  for (int i = 0; i < laser->GetRangeCount(); ++i)
  {
    EXPECT_EQ(10, laser->GetRange(i));
  }

  // Spawn a box and test for proper laser scan
  {
    SpawnBox("test_box", math::Vector3(1, 1, 1),
        math::Vector3(2, 0, 0.5), math::Vector3(0, 0, 0));
    common::Time::MSleep(1000);

    laser->Update(true);

    double diffMax, diffSum, diffAvg;
    std::vector<double> scan;
    laser->GetRanges(scan);

    DoubleCompare(box_scan, &scan[0], 640, diffMax, diffSum, diffAvg);
    EXPECT_LT(diffMax, 2e-6);
    EXPECT_LT(diffSum, 1e-4);
    EXPECT_LT(diffAvg, 2e-6);

    // This line will print the current scan. Use this to generate
    // a new test scan sample
    // PrintScan("box_scan", &scan[0], 640);
  }

  // Move the laser to point down on the ground plane,
  {
    common::Time prevTime;
    physics::WorldPtr world = physics::get_world("default");
    EXPECT_TRUE(world);

    physics::ModelPtr model = world->GetModel("box");

    prevTime = laser->GetLastUpdateTime();
    model->SetWorldPose(math::Pose(0, 0, 1.0, 0, M_PI*0.5, 0));
    while (laser->GetLastUpdateTime() <= prevTime)
      common::Time::MSleep(10);

    double diffMax, diffSum, diffAvg;

    std::vector<double> scan, scan2;

    laser->Update(false);
    for (unsigned int j = 0; j < 5; ++j)
    {
      laser->Update(true);
      laser->GetRanges(scan);
      laser->Update(true);
      laser->GetRanges(scan2);

      DoubleCompare(&scan[0], &scan2[0], 640, diffMax, diffSum, diffAvg);
      EXPECT_LT(diffMax, 1e-6);
      EXPECT_LT(diffSum, 1e-6);
      EXPECT_LT(diffAvg, 1e-6);
    }
    laser->Update(true);

    DoubleCompare(plane_scan, &scan[0], 640, diffMax, diffSum, diffAvg);
    EXPECT_LT(diffMax, 2e-6);
    EXPECT_LT(diffSum, 1e-5);
    EXPECT_LT(diffAvg, 2e-6);

    // This line will print the current scan. Use this to generate
    // a new test scan sample
    // PrintScan("plane_scan", &scan[0], 640);
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
