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

#include <boost/filesystem.hpp>
#include "ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "helper_physics_generator.hh"

using namespace gazebo;
class PR2Test : public ServerFixture,
                public testing::WithParamInterface<const char*>
{
  public: void LoadPR2(std::string _physicsEngine);
  public: void ScrewJoint(std::string _physicsEngine);
  public: void StaticPR2(std::string _physicsEngine);
};

////////////////////////////////////////////////////////////////////////
void PR2Test::LoadPR2(std::string _physicsEngine)
{
  if (_physicsEngine == "dart")
  {
    gzerr << "Abort test since dart does not support ray sensor in PR2, "
          << "Please see issue #911.\n";
    return;
  }

  // Cleanup test directory.
  common::SystemPaths *paths = common::SystemPaths::Instance();
  boost::filesystem::remove_all(paths->GetDefaultTestPath());
  boost::filesystem::create_directories(paths->GetDefaultTestPath());

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

  while (!camSensor->SaveFrame(paths->GetDefaultTestPath() + "/frame_10.jpg"))
    common::Time::MSleep(100);

  physics::get_world("default")->GetPhysicsEngine()->SetGravity(
      math::Vector3(-0.5, 0, -0.1));
  for (int i = 11; i < 200; i++)
  {
    std::ostringstream filename;
    filename << paths->GetDefaultTestPath() << "/frame_" << i << ".jpg";
    camSensor->SaveFrame(filename.str());
    common::Time::MSleep(100);
  }

  // Cleanup test directory.
  boost::filesystem::remove_all(paths->GetDefaultTestPath());
}

TEST_P(PR2Test, LoadPR2)
{
  LoadPR2(GetParam());
}

////////////////////////////////////////////////////////////////////////
void PR2Test::ScrewJoint(std::string _physicsEngine)
{
  if (_physicsEngine == "bullet")
  {
    gzerr << "Abort test since bullet screw joints don't work yet\n";
    return;
  }
  if (_physicsEngine == "dart")
  {
    gzerr << "Abort test since dart does not support ray sensor in PR2, "
          << "Please see issue #911.\n";
    return;
  }

  ServerFixture::Load("worlds/pr2.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  physics::ModelPtr model = world->GetModel("pr2");
  ASSERT_TRUE(model != NULL);

  physics::LinkPtr link = model->GetLink("torso_lift_link");
  ASSERT_TRUE(link != NULL);

  physics::JointPtr motor = model->GetJoint("torso_lift_motor_screw_joint");
  ASSERT_TRUE(motor != NULL);
  motor->SetEffortLimit(0, 1000);

  physics::JointPtr screw =
    model->GetJoint("torso_lift_screw_torso_lift_joint");
  ASSERT_TRUE(screw != NULL);

  // Let it settle a bit
  world->Step(300);

  // Set a fixed velocity on revolute joint that connects to the screw joint
  unsigned int steps = 1000;
  for (unsigned int i = 0; i < steps; ++i)
  {
    motor->SetForce(0, 10);
    world->Step(1);
  }

  // Expect torso to lift at least 1 mm/s
  gzdbg << "motor " << motor->GetVelocity(0) << std::endl;
  gzdbg << "screw " << screw->GetVelocity(0) << std::endl;
  gzdbg << "link  " << link->GetWorldLinearVel() << std::endl;
  EXPECT_GT(link->GetWorldLinearVel().z, 1e-3);
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
  world->Step(1);
  double t = world->GetSimTime().Double();
  // verify that time moves forward
  EXPECT_GT(t, 0);

  // simulate a few steps
  int steps = 20;
  world->Step(steps);
  double dt = world->GetPhysicsEngine()->GetMaxStepSize();
  EXPECT_GT(dt, 0);
  t = world->GetSimTime().Double();
  EXPECT_GT(t, 0.99*dt*static_cast<double>(steps+1));
}

TEST_P(PR2Test, ScrewJoint)
{
  ScrewJoint(GetParam());
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
