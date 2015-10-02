/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include <gazebo/rendering/rendering.hh>
#include "test/ServerFixture.hh"

using namespace gazebo;

const double g_tolerance = 1e-4;
class Issue1694Test : public ServerFixture
{
  /// \brief Check acceleration value.
  /// \param[in] _accel Acceleration to check.
  /// \param[in] _expected Expected value.
  public: static void CheckAccel(const math::Vector3 &_accel,
                                 const math::Vector3 &_expected);
};

/////////////////////////////////////////////////
void Issue1694Test::CheckAccel(const math::Vector3 &_accel,
                               const math::Vector3 &_expected)
{
  EXPECT_NEAR(_accel.x, _expected.x, g_tolerance);
  EXPECT_NEAR(_accel.y, _expected.y, g_tolerance);
  EXPECT_NEAR(_accel.z, _expected.z, g_tolerance);
}

/////////////////////////////////////////////////
// \brief Test for issue #1694
TEST_F(Issue1694Test, WorldAccel)
{
  Load("worlds/issue_1694.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr model = world->GetModel("box_model");
  ASSERT_TRUE(model != NULL);

  physics::LinkPtr link = model->GetLink("box_link");
  ASSERT_TRUE(link != NULL);

  const math::Vector3 g = world->GetPhysicsEngine()->GetGravity();

  world->Step(1);

  // Expect box to still be falling
  CheckAccel(link->GetRelativeLinearAccel(), g);
  CheckAccel(link->GetWorldLinearAccel(), g);

  world->Step(3000);

  // The box should be resting on the ground
  CheckAccel(link->GetRelativeLinearAccel(), math::Vector3());
  CheckAccel(link->GetWorldLinearAccel(), math::Vector3());
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
