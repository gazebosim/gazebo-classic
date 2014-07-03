/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include <map>
#include <string>
#include <vector>

#include "test/ServerFixture.hh"
#include "gazebo/math/SignalStats.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"

using namespace gazebo;

const double g_angle_y_tol = 0.2;
const double g_angle_z_tol = 0.2;

class PhysicsTest : public ServerFixture,
                    public testing::WithParamInterface<const char*>
{
  public: void InertiaRatioPendulum(const std::string &_physicsEngine);
};

// Double pendulum with large inertia ratio and lateral gravity component
void PhysicsTest::InertiaRatioPendulum(const std::string &_physicsEngine)
{
  Load("worlds/inertia_ratio_pendulum.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // verify lateral gravity
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  math::Vector3 g = physics->GetGravity();
  EXPECT_EQ(g, math::Vector3(0.1, 0, -9.81));

  // get model
  physics::ModelPtr model = world->GetModel("inertia_ratio");
  ASSERT_TRUE(model != NULL);

  // get links
  physics::LinkPtr upperLink = model->GetLink("upper_link");
  physics::LinkPtr lowerLink = model->GetLink("lower_link");
  ASSERT_TRUE(upperLink != NULL);
  ASSERT_TRUE(lowerLink != NULL);

  math::Vector3Stats upperAngles;
  math::Vector3Stats lowerAngles;
  upperAngles.InsertStatistic("max");
  upperAngles.InsertStatistic("rms");
  lowerAngles.InsertStatistic("max");
  lowerAngles.InsertStatistic("rms");

  for (int i = 0; i < 3000; ++i)
  {
    world->Step(1);

    // Record out of plane angles
    upperAngles.InsertData(upperLink->GetWorldPose().rot.GetAsEuler());
    lowerAngles.InsertData(lowerLink->GetWorldPose().rot.GetAsEuler());
  }

  {
    // upper pitch
    std::map<std::string, double> map = upperAngles.y.GetMap();
    EXPECT_NEAR(map["max"], 0.0, g_angle_y_tol);
    std::string prefix = "upper_pitch_";
    for (std::map<std::string, double>::iterator iter = map.begin();
         iter != map.end(); ++iter)
    {
      std::ostringstream stream;
      stream << iter->second;
      RecordProperty(prefix + iter->first, stream.str());
    }
  }

  {
    // upper yaw
    std::map<std::string, double> map = upperAngles.z.GetMap();
    EXPECT_NEAR(map["max"], 0.0, g_angle_z_tol);
    std::string prefix = "upper_yaw_";
    for (std::map<std::string, double>::iterator iter = map.begin();
         iter != map.end(); ++iter)
    {
      std::ostringstream stream;
      stream << iter->second;
      RecordProperty(prefix + iter->first, stream.str());
    }
  }

  {
    // lower pitch
    std::map<std::string, double> map = lowerAngles.y.GetMap();
    EXPECT_NEAR(map["max"], 0.0, g_angle_y_tol);
    std::string prefix = "lower_pitch_";
    for (std::map<std::string, double>::iterator iter = map.begin();
         iter != map.end(); ++iter)
    {
      std::ostringstream stream;
      stream << iter->second;
      RecordProperty(prefix + iter->first, stream.str());
    }
  }

  {
    // lower yaw
    std::map<std::string, double> map = lowerAngles.z.GetMap();
    EXPECT_NEAR(map["max"], 0.0, g_angle_z_tol);
    std::string prefix = "lower_yaw_";
    for (std::map<std::string, double>::iterator iter = map.begin();
         iter != map.end(); ++iter)
    {
      std::ostringstream stream;
      stream << iter->second;
      RecordProperty(prefix + iter->first, stream.str());
    }
  }
}

TEST_F(PhysicsTest, InertiaRatioPendulum)
{
  InertiaRatioPendulum("ode");
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
