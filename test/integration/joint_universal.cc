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

#include "gazebo/physics/physics.hh"
#include "ServerFixture.hh"
#include "helper_physics_generator.hh"

using namespace gazebo;

class JointTestUniversal : public ServerFixture,
                        public testing::WithParamInterface<const char*>
{
  public: void UniversalJoint1(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void JointTestUniversal::UniversalJoint1(const std::string &_physicsEngine)
{
  if (_physicsEngine == "dart")
  {
    gzerr << "DART Universal Joint is not yet working.  See issue #1011.\n";
    return;
  }

  // Load our universal joint test world
  Load("worlds/universal_joint_test.world", true, _physicsEngine);

  // Get a pointer to the world, make sure world loads
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Verify physics engine type
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);

  physics->SetGravity(math::Vector3(0, 0, 0));

  // simulate 1 step
  world->Step(1);
  double t = world->GetSimTime().Double();

  // get time step size
  double dt = world->GetPhysicsEngine()->GetMaxStepSize();
  EXPECT_GT(dt, 0);
  gzlog << "dt : " << dt << "\n";

  // verify that time moves forward
  EXPECT_DOUBLE_EQ(t, dt);
  gzlog << "t after one step : " << t << "\n";

  // get model, joint and links
  physics::ModelPtr model_1 = world->GetModel("model_1");
  physics::LinkPtr link_00 = model_1->GetLink("link_00");
  physics::LinkPtr link_01 = model_1->GetLink("link_01");
  physics::JointPtr joint_00 = model_1->GetJoint("joint_00");
  physics::JointPtr joint_01 = model_1->GetJoint("joint_01");

  // both initial angles should be zero
  EXPECT_EQ(joint_00->GetAngle(0), 0);
  EXPECT_EQ(joint_00->GetAngle(1), 0);

  // move child link to it's initial location
  link_00->SetWorldPose(math::Pose(0, 0, 2, 0, 0, 0));
  EXPECT_EQ(joint_00->GetAngle(0), 0);
  EXPECT_EQ(joint_00->GetAngle(1), 0);
  EXPECT_EQ(joint_00->GetGlobalAxis(0), math::Vector3(1, 0, 0));
  EXPECT_EQ(joint_00->GetGlobalAxis(1), math::Vector3(0, 1, 0));
  gzdbg << "joint angles [" << joint_00->GetAngle(0)
        << ", " << joint_00->GetAngle(1)
        << "] axis1 [" << joint_00->GetGlobalAxis(0)
        << "] axis2 [" << joint_00->GetGlobalAxis(1)
        << "]\n";

  // move child link 45deg about x
  link_00->SetWorldPose(math::Pose(0, 0, 2, 0.25*M_PI, 0, 0));
  EXPECT_EQ(joint_00->GetAngle(0), 0.25*M_PI);
  EXPECT_EQ(joint_00->GetAngle(1), 0);
  EXPECT_EQ(joint_00->GetGlobalAxis(0), math::Vector3(1, 0, 0));
  EXPECT_EQ(joint_00->GetGlobalAxis(1),
    math::Vector3(0, cos(0.25*M_PI), sin(0.25*M_PI)));
  gzdbg << "joint angles [" << joint_00->GetAngle(0)
        << ", " << joint_00->GetAngle(1)
        << "] axis1 [" << joint_00->GetGlobalAxis(0)
        << "] axis2 [" << joint_00->GetGlobalAxis(1)
        << "]\n";

  // move child link 45deg about y
  link_00->SetWorldPose(math::Pose(0, 0, 2, 0, 0.25*M_PI, 0));
  EXPECT_EQ(joint_00->GetAngle(0), 0);
  EXPECT_EQ(joint_00->GetAngle(1), 0.25*M_PI);
  EXPECT_EQ(joint_00->GetGlobalAxis(0), math::Vector3(1, 0, 0));
  EXPECT_EQ(joint_00->GetGlobalAxis(1), math::Vector3(0, 1, 0));
  gzdbg << "joint angles [" << joint_00->GetAngle(0)
        << ", " << joint_00->GetAngle(1)
        << "] axis1 [" << joint_00->GetGlobalAxis(0)
        << "] axis2 [" << joint_00->GetGlobalAxis(1)
        << "]\n";

  // move child link 90deg about both x and "rotated y axis" (z)
  link_00->SetWorldPose(math::Pose(0, 0, 2, 0.5*M_PI, 0, 0.5*M_PI));
  EXPECT_EQ(joint_00->GetAngle(0), 0.5*M_PI);
  EXPECT_EQ(joint_00->GetAngle(1), 0.5*M_PI);
  EXPECT_EQ(joint_00->GetGlobalAxis(0), math::Vector3(1, 0, 0));
  EXPECT_EQ(joint_00->GetGlobalAxis(1),
    math::Vector3(0, cos(0.5*M_PI), sin(0.5*M_PI)));
  gzdbg << "joint angles [" << joint_00->GetAngle(0)
        << ", " << joint_00->GetAngle(1)
        << "] axis1 [" << joint_00->GetGlobalAxis(0)
        << "] axis2 [" << joint_00->GetGlobalAxis(1)
        << "]\n";
}

/////////////////////////////////////////////////
TEST_P(JointTestUniversal, UniversalJoint1)
{
  UniversalJoint1(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, JointTestUniversal,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
