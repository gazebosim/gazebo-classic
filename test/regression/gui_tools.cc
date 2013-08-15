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
#include <string.h>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "ServerFixture.hh"

using namespace gazebo;

class GuiToolsTest : public ServerFixture
{
  public: void MoveTool(const std::string &_physicsEngine);
  public: void SetGravity(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void GuiToolsTest::SetGravity(const std::string &_physicsEngine)
{
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  math::Vector3 g = physics->GetGravity();

  // Assume gravity vector points down z axis only.
  EXPECT_EQ(g.x, 0);
  EXPECT_EQ(g.y, 0);
  EXPECT_LE(g.z, -9.8);

  // Set Gravity by publishing to "~/physics"
  transport::PublisherPtr physicsPub =
    this->node->Advertise<msgs::Physics>("~/physics");
  msgs::Physics msg;
  // it doesn't actually seem to matter what type you set
  msg.set_type(msgs::Physics::Type_MIN);

  std::vector<math::Vector3> gravity;
  gravity.push_back(math::Vector3(0, 0, 9.81));
  gravity.push_back(math::Vector3(0, 0, -20));
  gravity.push_back(math::Vector3(0, 0, 20));
  gravity.push_back(math::Vector3(0, 0, 0));
  gravity.push_back(math::Vector3(0, 0, -9.81));
  gravity.push_back(math::Vector3(1, 1, 9.81));
  gravity.push_back(math::Vector3(2, 3, -20));
  gravity.push_back(math::Vector3(2, -3, 20));
  gravity.push_back(math::Vector3(-2, 3, 0));
  gravity.push_back(math::Vector3(-2, -3, -9.81));

  for (std::vector<math::Vector3>::iterator iter = gravity.begin();
       iter != gravity.end(); ++iter)
  {
    msgs::Set(msg.mutable_gravity(), *iter);
    physicsPub->Publish(msg);

    world->StepWorld(10);
    common::Time::MSleep(10);
    world->StepWorld(10);

    EXPECT_EQ(*iter, physics->GetGravity());
  }
}

/////////////////////////////////////////////////
void GuiToolsTest::MoveTool(const std::string &_physicsEngine)
{
  Load("worlds/empty.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // set gravity to zero
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  physics->SetGravity(math::Vector3::Zero);

  // spawn a box
  std::string name = "test_box";
  double z0 = 5;
  math::Vector3 pos = math::Vector3(0, 0, z0);
  math::Vector3 size = math::Vector3(1, 1, 1);
  SpawnBox("test_box", size, pos, math::Vector3::Zero);

  // advertise on "~/model/modify"
  transport::PublisherPtr modelPub =
    this->node->Advertise<msgs::Model>("~/model/modify");

  // list of poses to move to
  std::vector<math::Pose> poses;
  poses.push_back(math::Pose(5, 0, z0, 0, 0, 0));
  poses.push_back(math::Pose(0, 8, z0, 0, 0, 0));
  poses.push_back(math::Pose(-99, 0, z0, 0, 0, 0));
  poses.push_back(math::Pose(0, 999, z0, 0, 0, 0));
  poses.push_back(math::Pose(123.456, 456.123, z0*10, 0.1, -0.2, 0.3));
  poses.push_back(math::Pose(-123.456, 456.123, z0*10, 0.2, 0.4, -0.6));
  poses.push_back(math::Pose(123.456, -456.123, z0*10, 0.3, -0.6, 0.9));
  poses.push_back(math::Pose(-123.456, -456.123, z0*10, -0.4, 0.8, -1.2));

  physics::ModelPtr model = world->GetModel(name);
  if (model != NULL)
  {
    math::Pose initialPose = model->GetWorldPose();
    EXPECT_EQ(pos, initialPose.pos);

    msgs::Model msg;
    msg.set_name(name);
    msg.set_id(model->GetId());

    for (std::vector<math::Pose>::iterator iter = poses.begin();
         iter != poses.end(); ++iter)
    {
      msgs::Set(msg.mutable_pose(), *iter);
      modelPub->Publish(msg);

      world->StepWorld(50);
      common::Time::MSleep(100);
      world->StepWorld(50);

      EXPECT_EQ(*iter, model->GetWorldPose());
    }
  }
  else
  {
    EXPECT_TRUE(model);
    gzerr << "Could not get pointer to " << name << std::endl;
  }
}

/////////////////////////////////////////////////
TEST_P(GuiToolsTest, SetGravity)
{
  SetGravity(GetParam());
}

/////////////////////////////////////////////////
TEST_P(GuiToolsTest, MoveTool)
{
  MoveTool(GetParam());
}

INSTANTIATE_TEST_CASE_P(TestODE, GuiToolsTest,
    ::testing::Values("ode"));

#ifdef HAVE_BULLET
INSTANTIATE_TEST_CASE_P(TestBullet, GuiToolsTest,
    ::testing::Values("bullet"));
#endif  // HAVE_BULLET

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
