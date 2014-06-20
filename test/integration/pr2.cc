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
#include "ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "helper_physics_generator.hh"

using namespace gazebo;
class PR2Test : public ServerFixture,
                public testing::WithParamInterface<const char*>
{
  public: void StaticPR2(std::string _physicsEngine);
  public: void Load(std::string _physicsEngine);
};

void PR2Test::Load(std::string _physicsEngine)
{
  if (_physicsEngine == "simbody")
  {
    gzerr << "Abort test since simbody does not support screw joints in PR2, "
          << "Please see issue #857.\n";
    return;
  }
  if (_physicsEngine == "dart")
  {
    gzerr << "Abort test since dart does not support ray sensor in PR2, "
          << "Please see issue #911.\n";
    return;
  }

  // Cleanup test directory.
  boost::filesystem::remove_all("/tmp/gazebo_test");
  boost::filesystem::create_directories("/tmp/gazebo_test");

  ServerFixture::Load("worlds/empty.world", false, _physicsEngine);
  SpawnModel("model://pr2");

  int i;
  for (i = 0; i < 1000 && !this->HasEntity("pr2"); ++i)
    common::Time::MSleep(1000);
  EXPECT_LT(i, 1000);

  if (rendering::RenderEngine::Instance()->GetRenderPathType() ==
      rendering::RenderEngine::NONE)
    return;


  sensors::SensorPtr sensor =
    sensors::get_sensor("narrow_stereo_gazebo_l_stereo_camera_sensor");
  EXPECT_TRUE(sensor);

  sensors::CameraSensorPtr camSensor =
    boost::dynamic_pointer_cast<sensors::CameraSensor>(sensor);
  EXPECT_TRUE(camSensor);

  while (!camSensor->SaveFrame("/tmp/gazebo_test/frame_10.jpg"))
    common::Time::MSleep(100);

  physics::get_world("default")->GetPhysicsEngine()->SetGravity(
      math::Vector3(-0.5, 0, -0.1));
  for (int i = 11; i < 200; i++)
  {
    std::ostringstream filename;
    filename << "/tmp/gazebo_test/frame_" << i << ".jpg";
    camSensor->SaveFrame(filename.str());
    common::Time::MSleep(100);
  }

  // Cleanup test directory.
  boost::filesystem::remove_all("/tmp/gazebo_test");
}

TEST_P(PR2Test, Load)
{
  Load(GetParam());
}

////////////////////////////////////////////////////////////////////////
// StaticPR2:
// Issue #586 noted a segfault when loading a pr2 as static and stepping
// physics forward. This test loads a world with several objects and
// steps time forward.
////////////////////////////////////////////////////////////////////////
void PR2Test::StaticPR2(std::string _physicsEngine)
{
  if (_physicsEngine == "simbody")
  {
    gzerr << "Abort test since simbody does not support screw joints in PR2, "
          << "Please see issue #857.\n";
    return;
  }
  if (_physicsEngine == "dart")
  {
    gzerr << "Abort test since dart does not support ray sensor in PR2, "
          << "Please see issue #911.\n";
    return;
  }

  ServerFixture::Load("worlds/static_pr2.world", true, _physicsEngine);

  // The body of this is copied from PhysicsTest::EmptyWorld
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  // simulate 1 step
  world->StepWorld(1);
  double t = world->GetSimTime().Double();
  // verify that time moves forward
  EXPECT_GT(t, 0);

  // simulate a few steps
  int steps = 20;
  world->StepWorld(steps);
  double dt = world->GetPhysicsEngine()->GetMaxStepSize();
  EXPECT_GT(dt, 0);
  t = world->GetSimTime().Double();
  EXPECT_GT(t, 0.99*dt*static_cast<double>(steps+1));
}

TEST_P(PR2Test, StaticPR2)
{
  StaticPR2(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, PR2Test, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
