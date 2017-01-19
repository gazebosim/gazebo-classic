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
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;

const double g_tolerance = 1e-4;
class Issue1694Test : public ServerFixture
{
  /// \brief Check acceleration value.
  /// \param[in] _accel Acceleration to check.
  /// \param[in] _expected Expected value.
  public: static void CheckAccel(const ignition::math::Vector3d &_accel,
                                 const ignition::math::Vector3d &_expected);
};

/////////////////////////////////////////////////
void Issue1694Test::CheckAccel(const ignition::math::Vector3d &_accel,
                               const ignition::math::Vector3d &_expected)
{
  EXPECT_NEAR(_accel.X(), _expected.X(), g_tolerance);
  EXPECT_NEAR(_accel.Y(), _expected.Y(), g_tolerance);
  EXPECT_NEAR(_accel.Z(), _expected.Z(), g_tolerance);
}

/////////////////////////////////////////////////
// \brief Test for issue #1694
TEST_F(Issue1694Test, WorldAccel)
{
  Load("worlds/issue_1694.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr model = world->ModelByName("box_model");
  ASSERT_TRUE(model != NULL);

  physics::LinkPtr link = model->GetLink("box_link");
  ASSERT_TRUE(link != NULL);

  const auto g = world->Gravity();

  world->Step(1);

  // Expect box to still be falling
  CheckAccel(link->RelativeLinearAccel(), g);
  CheckAccel(link->WorldLinearAccel(), g);

  world->Step(3000);

  // The box should be resting on the ground
  CheckAccel(link->RelativeLinearAccel(),
      ignition::math::Vector3d::Zero);
  CheckAccel(link->WorldLinearAccel(),
      ignition::math::Vector3d::Zero);
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
