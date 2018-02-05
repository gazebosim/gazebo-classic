/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include "gazebo/test/ServerFixture.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/test/helper_physics_generator.hh"

using namespace gazebo;
class AttachLightTest : public ServerFixture,
                        public testing::WithParamInterface<const char*>
{
  /// \brief Test AttachLightPlugin by verifying light pose against link pose
  /// \param[in] _physicsEngine Name of physics engine
  public: void AttachLightPlugin(const std::string &_physicsEngine);

  /// \brief Test Light as child of Link by verifying light pose against link
  /// pose
  /// \param[in] _physicsEngine Name of physics engine
  public: void LinkLight(const std::string &_physicsEngine);
};


/////////////////////////////////////////////////
void AttachLightTest::AttachLightPlugin(const std::string &_physicsEngine)
{
  // Test plugin for attaching lights to links
  this->Load("worlds/attach_lights.world", true, _physicsEngine);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Get the double pendulum model
  physics::ModelPtr pendulumModel =
      world->ModelByName("double_pendulum_with_lights");
  ASSERT_TRUE(pendulumModel != nullptr);

  // Get the links
  physics::LinkPtr upperLink =
      pendulumModel->GetLink("double_pendulum_with_base::upper_link");
  ASSERT_TRUE(upperLink != nullptr);
  physics::LinkPtr lowerLink =
      pendulumModel->GetLink("double_pendulum_with_base::lower_link");
  ASSERT_TRUE(lowerLink != nullptr);

  // Get the lights
  physics::LightPtr pointLight = world->LightByName("point");
  ASSERT_TRUE(pointLight != nullptr);
  physics::LightPtr pointLight2 = world->LightByName("point2");
  ASSERT_TRUE(pointLight2 != nullptr);
  physics::LightPtr spotLight = world->LightByName("spot");
  ASSERT_TRUE(spotLight != nullptr);

  // step the world
  world->Step(1);

  // Get the initial light pose offset relative to link
  ignition::math::Pose3d pointLightPose = pointLight->WorldPose() -
      upperLink->WorldPose();
  ignition::math::Pose3d pointLight2Pose = pointLight2->WorldPose() -
      upperLink->WorldPose();
  ignition::math::Pose3d spotLightPose = spotLight->WorldPose() -
      lowerLink->WorldPose();

  // verify light pose against link pose.
  // NOTE: there seem to be race condition when verifying pose using
  // WorldPose in the test thread so do the verification in the update
  // callback which is guaranteed to be done in the physics thread
  int iteration = 0;
  auto verifyPose = [&]()
  {
    ignition::math::Pose3d upperLinkPose = upperLink->WorldPose();
    ignition::math::Pose3d lowerLinkPose = lowerLink->WorldPose();

    EXPECT_EQ(pointLight->WorldPose(), pointLightPose + upperLinkPose);
    EXPECT_EQ(pointLight2->WorldPose(), pointLight2Pose + upperLinkPose);
    EXPECT_EQ(spotLight->WorldPose(), spotLightPose + lowerLinkPose);
    iteration++;
  };
  auto connection = event::Events::ConnectWorldUpdateEnd(std::bind(verifyPose));

  // verify pose for 1000 iterations
  for (unsigned int i = 0; i < 1000u; ++i)
    world->Step(1);

  // verify that update is called
  EXPECT_EQ(iteration, 1000);
}

/////////////////////////////////////////////////
void AttachLightTest::LinkLight(const std::string &_physicsEngine)
{
  // Test plugin for attaching lights to links
  this->Load("worlds/attach_lights.world", true, _physicsEngine);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Get the sphere model
  physics::ModelPtr sphereModel =
      world->ModelByName("sphere_with_light");
  ASSERT_TRUE(sphereModel != nullptr);

  // Get the link
  physics::LinkPtr sphereLink =
      sphereModel->GetLink("link");
  ASSERT_TRUE(sphereLink != nullptr);

  // Get the light in the link
  // TODO change this to link->LightByName("point") when API is added
  physics::LightPtr pointLight =
      world->LightByName("sphere_with_light::link::point");
  ASSERT_TRUE(pointLight != nullptr);

  // step the world
  world->Step(1);

  // Get the initial light pose offset relative to link
  ignition::math::Pose3d pointLightPose = pointLight->RelativePose();

  // verify light pose against link pose.
  // NOTE: there seem to be race condition when verifying pose using
  // WorldPose in the test thread so do the verification in the update
  // callback which is guaranteed to be done in the physics thread
  int iteration = 0;
  auto verifyPose = [&]()
  {
    ignition::math::Pose3d sphereLinkPose = sphereLink->WorldPose();

    EXPECT_EQ(pointLightPose + sphereLinkPose,
        pointLight->WorldPose());
    iteration++;
  };
  auto connection = event::Events::ConnectWorldUpdateEnd(std::bind(verifyPose));

  // verify pose for 1500 iterations
  for (unsigned int i = 0; i < 1500u; ++i)
    world->Step(1);

  // verify that update is called
  EXPECT_EQ(iteration, 1500);
}

/////////////////////////////////////////////////
TEST_P(AttachLightTest, AttachLightPlugin)
{
  AttachLightPlugin(GetParam());
}

/////////////////////////////////////////////////
TEST_P(AttachLightTest, LinkLight)
{
  LinkLight(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, AttachLightTest, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
