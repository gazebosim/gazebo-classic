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
  public: void InertialAccessors(const std::string &_physicsEngine);
};

/////////////////////////////////////////////////
void InertiaMsgsTest::InertialAccessors(const std::string &_physicsEngine)
{
  Load("worlds/seesaw.world", true, _physicsEngine);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // check the gravity vector
  physics::PhysicsEnginePtr physics = world->GetPhysicsEngine();
  ASSERT_TRUE(physics != NULL);
  EXPECT_EQ(physics->GetType(), _physicsEngine);
  math::Vector3 g = physics->GetGravity();

  const std::string modelName("cube1");
  auto model = world->GetModel(modelName);
  ASSERT_TRUE(model != NULL);
  auto link = model->GetLink();
  ASSERT_TRUE(link != NULL);
  auto inertial = link->GetInertial();
  ASSERT_TRUE(inertial != NULL);
  const double mass = inertial->GetMass();
  const math::Vector3 cog = inertial->GetCoG();
  const math::Vector3 Ixxyyzz = inertial->GetPrincipalMoments();
  const math::Vector3 Ixyxzyz = inertial->GetProductsofInertia();
  EXPECT_DOUBLE_EQ(mass, 45.56250000000001);
  EXPECT_EQ(cog, math::Vector3::Zero);
  EXPECT_EQ(Ixxyyzz, 1.537734375*math::Vector3::One);
  EXPECT_EQ(Ixyxzyz, math::Vector3::Zero);

  // new inertial values
  msgs::Model msg;
  msg.set_name(modelName);
  msg.add_link();
  auto msgLink = msg.mutable_link(0);
  msgLink->set_name("link");
  msgLink->set_id(link->GetId());
  auto msgInertial = msgLink->mutable_inertial();
  msgInertial->set_mass(99.9);
  msgInertial->set_ixx(12.3);
  msgInertial->set_ixy(0.123);
  msgInertial->set_ixz(0.456);
  msgInertial->set_iyy(13.4);
  msgInertial->set_iyz(0.789);
  msgInertial->set_izz(15.6);
  const ignition::math::Vector3d newCog(1.1, -2.2, 3.3);
  msgs::Set(msgInertial->mutable_pose(), ignition::math::Pose3d(
    newCog, ignition::math::Quaterniond()));

  // Set inertial properties by publishing to "~/model/modify"
  transport::PublisherPtr modelPub =
    this->node->Advertise<msgs::Model>("~/model/modify");
  modelPub->WaitForConnection();
  modelPub->Publish(msg, true);

  while (newCog != inertial->GetCoG().Ign())
  {
    world->Step(1);
    common::Time::MSleep(1);
    modelPub->Publish(msg, true);
  }
}

/////////////////////////////////////////////////
TEST_P(InertiaMsgsTest, InertialAccessors)
{
  InertialAccessors(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, InertiaMsgsTest,
                        PHYSICS_ENGINE_VALUES);

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
