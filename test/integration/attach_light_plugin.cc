/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/common.hh"
#include "gazebo/test/helper_physics_generator.hh"

// why the large tolerance? It seems that for some physics engines the light
// pose is one timestep behind the link pose.
#define ATTACH_POS_TOL 1e-2
#define ATTACH_ROT_TOL 1e-2

using namespace gazebo;
class AttachLightTest : public ServerFixture,
                        public testing::WithParamInterface<const char*>
{
  public: void AttachLightPlugin(const std::string &_physicsEngine);
};


void AttachLightTest::AttachLightPlugin(const std::string &_physicsEngine)
{
  // Test plugin for attaching lights to links
  this->Load("worlds/attach_lights.world", true, _physicsEngine);

  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != nullptr);

  // Get the double pendulum model
  physics::ModelPtr pendulumModel =
      world->GetModel("double_pendulum_with_lights");
  ASSERT_TRUE(pendulumModel != nullptr);

  // Get the links
  physics::LinkPtr upperLink =
      pendulumModel->GetLink("double_pendulum_with_base::upper_link");
  ASSERT_TRUE(upperLink != nullptr);
  physics::LinkPtr lowerLink =
      pendulumModel->GetLink("double_pendulum_with_base::lower_link");
  ASSERT_TRUE(lowerLink != nullptr);

  // Get the lights
  physics::LightPtr pointLight = world->Light("point");
  ASSERT_TRUE(pointLight != nullptr);
  physics::LightPtr pointLight2 = world->Light("point2");
  ASSERT_TRUE(pointLight2 != nullptr);
  physics::LightPtr spotLight = world->Light("spot");
  ASSERT_TRUE(spotLight != nullptr);

  // step the world once
  world->Step(1);

  // Get the initial light pose offset relative to link
  ignition::math::Pose3d pointLightPose = pointLight->GetWorldPose().Ign() -
      upperLink->GetWorldPose().Ign();
  ignition::math::Pose3d pointLight2Pose = pointLight2->GetWorldPose().Ign() -
      upperLink->GetWorldPose().Ign();
  ignition::math::Pose3d spotLightPose = spotLight->GetWorldPose().Ign() -
      lowerLink->GetWorldPose().Ign();

  // verify pose for 1000 iterations (1 second);
  for (unsigned int i = 0; i < 1000; ++i)
  {
    world->Step(1);

    // verify point light pose
    auto p = pointLight->GetWorldPose().Ign();
    auto p2 = pointLightPose + upperLink->GetWorldPose().Ign();
    // verify pos
    EXPECT_NEAR(p.Pos().X(), p2.Pos().X(), ATTACH_POS_TOL);
    EXPECT_NEAR(p.Pos().Y(), p2.Pos().Y(), ATTACH_POS_TOL);
    EXPECT_NEAR(p.Pos().Z(), p2.Pos().Z(), ATTACH_POS_TOL);
    // verify rot and account for the case when angle is close to 2pi
    EXPECT_TRUE(ignition::math::equal(
        p.Rot().Euler().X(), p2.Rot().Euler().X(), ATTACH_ROT_TOL) ||
        ignition::math::equal(
        p.Rot().Euler().X(), p2.Rot().Euler().X() - 2*M_PI, ATTACH_ROT_TOL));
    EXPECT_TRUE(ignition::math::equal(
        p.Rot().Euler().Y(), p2.Rot().Euler().Y(), ATTACH_ROT_TOL) ||
        ignition::math::equal(
        p.Rot().Euler().Y(), p2.Rot().Euler().Y() - 2*M_PI, ATTACH_ROT_TOL));
    EXPECT_TRUE(ignition::math::equal(
        p.Rot().Euler().Z(), p2.Rot().Euler().Z(), ATTACH_ROT_TOL) ||
        ignition::math::equal(
        p.Rot().Euler().Z(), p2.Rot().Euler().Z() - 2*M_PI, ATTACH_ROT_TOL));

    // verify point2 light pose
    p = pointLight2->GetWorldPose().Ign();
    p2 = pointLight2Pose + upperLink->GetWorldPose().Ign();
    // verify pos
    EXPECT_NEAR(p.Pos().X(), p2.Pos().X(), ATTACH_POS_TOL);
    EXPECT_NEAR(p.Pos().Y(), p2.Pos().Y(), ATTACH_POS_TOL);
    EXPECT_NEAR(p.Pos().Z(), p2.Pos().Z(), ATTACH_POS_TOL);
    // verify rot and account for the case when angle is close to 2pi
    EXPECT_TRUE(ignition::math::equal(
        p.Rot().Euler().X(), p2.Rot().Euler().X(), ATTACH_ROT_TOL) ||
        ignition::math::equal(
        p.Rot().Euler().X(), p2.Rot().Euler().X() - 2*M_PI, ATTACH_ROT_TOL));
    EXPECT_TRUE(ignition::math::equal(
        p.Rot().Euler().Y(), p2.Rot().Euler().Y(), ATTACH_ROT_TOL) ||
        ignition::math::equal(
        p.Rot().Euler().Y(), p2.Rot().Euler().Y() - 2*M_PI, ATTACH_ROT_TOL));
    EXPECT_TRUE(ignition::math::equal(
        p.Rot().Euler().Z(), p2.Rot().Euler().Z(), ATTACH_ROT_TOL) ||
        ignition::math::equal(
        p.Rot().Euler().Z(), p2.Rot().Euler().Z() - 2*M_PI, ATTACH_ROT_TOL));

    // verify spot light pose
    p = spotLight->GetWorldPose().Ign();
    p2 = spotLightPose + lowerLink->GetWorldPose().Ign();
    // verify pos
    EXPECT_NEAR(p.Pos().X(), p2.Pos().X(), ATTACH_POS_TOL);
    EXPECT_NEAR(p.Pos().Y(), p2.Pos().Y(), ATTACH_POS_TOL);
    EXPECT_NEAR(p.Pos().Z(), p2.Pos().Z(), ATTACH_POS_TOL);
    // verify rot and account for the case when angle is close to 2pi
    EXPECT_TRUE(ignition::math::equal(
        p.Rot().Euler().X(), p2.Rot().Euler().X(), ATTACH_ROT_TOL) ||
        ignition::math::equal(
        p.Rot().Euler().X(), p2.Rot().Euler().X() - 2*M_PI, ATTACH_ROT_TOL));
    EXPECT_TRUE(ignition::math::equal(
        p.Rot().Euler().Y(), p2.Rot().Euler().Y(), ATTACH_ROT_TOL) ||
        ignition::math::equal(
        p.Rot().Euler().Y(), p2.Rot().Euler().Y() - 2*M_PI, ATTACH_ROT_TOL));
    EXPECT_TRUE(ignition::math::equal(
        p.Rot().Euler().Z(), p2.Rot().Euler().Z(), ATTACH_ROT_TOL) ||
        ignition::math::equal(
        p.Rot().Euler().Z(), p2.Rot().Euler().Z() - 2*M_PI, ATTACH_ROT_TOL));
  }
}

TEST_P(AttachLightTest, AttachLightPlugin)
{
  AttachLightPlugin(GetParam());
}

INSTANTIATE_TEST_CASE_P(PhysicsEngines, AttachLightTest, PHYSICS_ENGINE_VALUES);

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
