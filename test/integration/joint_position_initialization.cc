/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include "gazebo/physics/physics.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

#define TOL 0.00001
using namespace gazebo;

class JointKinematicTest : public ServerFixture,
                           public testing::WithParamInterface<const char*>
{
  /// \brief Test setting joint initial positions.
  /// \param[in] _physicsEngine physics engine type [bullet|dart|ode|simbody]
  public: void JointInitialPositionTest(const std::string &_physicsEngine);
};

//////////////////////////////////////////////////
void JointKinematicTest::JointInitialPositionTest(
  const std::string &_physicsEngine)
{
  // init random seed
  srand(time(NULL));

  if (_physicsEngine == "bullet")
  {
    gzerr << "Bullet Joint::SetPosition affected by issue #1194.\n";
    return;
  }

  if (_physicsEngine == "dart")
  {
    gzerr << "DART Joint::SetPosition not yet working.\n";
    return;
  }

  if (_physicsEngine == "simbody")
  {
    gzerr << "Simbody Joint::SetPosition not yet working.\n";
    return;
  }

  // Load our screw joint test world
  Load("worlds/initial_joint_position.world", true, _physicsEngine);

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

  // get pointer to model
  physics::ModelPtr model = world->GetModel("model_2");
  while (!model)
  {
    model = world->GetModel("model_2");
    gzdbg << "waiting for model_2 to spawn\n";
    sleep(1);
  }
  world->SetPaused(false);

  // intentionally break the joint using Link::SetWorldPose
  // let it conflict with Physics pose updates and make sure
  // internal model state stays consistent

  physics::Link_V links;
  links.push_back(model->GetLink("link_1"));
  links.push_back(model->GetLink("link_2"));
  links.push_back(model->GetLink("link_3"));
  links.push_back(model->GetLink("link_4"));
  links.push_back(model->GetLink("link_5"));
  links.push_back(model->GetLink("link_2a"));
  links.push_back(model->GetLink("link_2b"));
  links.push_back(model->GetLink("link_3a"));
  links.push_back(model->GetLink("link_4a"));
  links.push_back(model->GetLink("link_5a"));

  for (physics::Link_V::iterator li = links.begin(); li != links.end(); ++li)
  {
    EXPECT_TRUE((*li)  != NULL);
  }

  physics::Joint_V joints;
  joints.push_back(model->GetJoint("model_2::joint_01"));
  joints.push_back(model->GetJoint("model_2::joint_12"));
  joints.push_back(model->GetJoint("model_2::joint_23"));
  joints.push_back(model->GetJoint("model_2::joint_34"));
  joints.push_back(model->GetJoint("model_2::joint_45"));
  joints.push_back(model->GetJoint("model_2::joint_52"));
  joints.push_back(model->GetJoint("model_2::joint_22a"));
  joints.push_back(model->GetJoint("model_2::joint_2a2b"));
  joints.push_back(model->GetJoint("model_2::joint_2b3a"));
  joints.push_back(model->GetJoint("model_2::joint_3a4a"));
  joints.push_back(model->GetJoint("model_2::joint_4a5a"));
  joints.push_back(model->GetJoint("model_2::joint_5a2b"));

  for (physics::Joint_V::iterator ji = joints.begin(); ji != joints.end(); ++ji)
  {
    EXPECT_TRUE((*ji)  != NULL);
  }

  world->Step(1000);
  for (physics::Joint_V::iterator ji = joints.begin(); ji != joints.end(); ++ji)
  {
    EXPECT_NEAR((*ji)->GetVelocity(0), 0, TOL);
    if ((*ji)->GetName() == "joint_01")
    {
      EXPECT_NEAR((*ji)->GetAngle(0).Radian(), 0.5, TOL);
    }
    else if ((*ji)->GetName() == "joint_12")
    {
      EXPECT_NEAR((*ji)->GetAngle(0).Radian(), 0.7071, TOL);
    }
    else if ((*ji)->GetName() == "joint_23")
    {
      // loop joint, so should stay 0
      EXPECT_NEAR((*ji)->GetAngle(0).Radian(), 0.0, TOL);
    }
    else if ((*ji)->GetName() == "joint_34")
    {
      // loop joint, so should stay 0
      EXPECT_NEAR((*ji)->GetAngle(0).Radian(), 0.0, TOL);
    }
    else if ((*ji)->GetName() == "joint_45")
    {
      // loop joint, so should stay 0
      EXPECT_NEAR((*ji)->GetAngle(0).Radian(), 0.0, TOL);
    }
    else if ((*ji)->GetName() == "joint_52")
    {
      // loop joint, so should stay 0
      EXPECT_NEAR((*ji)->GetAngle(0).Radian(), 0.0, TOL);
    }
    else if ((*ji)->GetName() == "joint_22a")
    {
      EXPECT_NEAR((*ji)->GetAngle(0).Radian(), 0.7071, TOL);
    }
    else if ((*ji)->GetName() == "joint_2a2b")
    {
      EXPECT_NEAR((*ji)->GetAngle(0).Radian(), -0.7071, TOL);
    }
    else if ((*ji)->GetName() == "joint_2b3a")
    {
      // loop joint, so should stay 0
      EXPECT_NEAR((*ji)->GetAngle(0).Radian(), 0, TOL);
    }
    else if ((*ji)->GetName() == "joint_3a4a")
    {
      // loop joint, so should stay 0
      EXPECT_NEAR((*ji)->GetAngle(0).Radian(), 0, TOL);
    }
    else if ((*ji)->GetName() == "joint_4a5a")
    {
      // loop joint, so should stay 0
      EXPECT_NEAR((*ji)->GetAngle(0).Radian(), 0, TOL);
    }
    else if ((*ji)->GetName() == "joint_5a2b")
    {
      // loop joint, so should stay 0
      EXPECT_NEAR((*ji)->GetAngle(0).Radian(), 0, TOL);
    }
  }
}

TEST_P(JointKinematicTest, JointInitialPositionTest)
{
  JointInitialPositionTest(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, JointKinematicTest,
  PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
