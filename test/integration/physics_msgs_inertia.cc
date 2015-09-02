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
#include <string.h>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;

class InertiaMsgsTest : public ServerFixture,
                        public testing::WithParamInterface<const char*>
{
  /// \brief Set inertia parameters over ~/model/modify
  /// and verify that Inertial accessors register the change.
  /// \param[in] _physicsEngine Type of physics engine to use.
  public: void InertiaAccessors(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void InertiaMsgsTest::InertiaAccessors(const std::string &_physicsEngine)
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
    msgs::Set(msg.mutable_gravity(), (*iter).Ign());
    physicsPub->Publish(msg);

    while (*iter != physics->GetGravity())
    {
      world->Step(1);
      common::Time::MSleep(1);
    }

    EXPECT_EQ(*iter, physics->GetGravity());
  }
}

/////////////////////////////////////////////////
TEST_P(InertiaMsgsTest, InertiaAccessors)
{
  InertiaAccessors(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, InertiaMsgsTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
