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

#include "ServerFixture.hh"
#include "gazebo/math/Box.hh"

using namespace gazebo;

class Issue1146Test : public ServerFixture
{
};

/////////////////////////////////////////////////
// \brief Test for issue #1146
TEST_F(Issue1146Test, Reset)
{
  Load("worlds/box_plane_low_friction_test.world", true);
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr model = world->GetModel("box");
  ASSERT_TRUE(model != NULL);

  physics::LinkPtr link = model->GetLink("link");
  ASSERT_TRUE(link != NULL);

  physics::CollisionPtr coll = link->GetCollision("collision");
  ASSERT_TRUE(coll != NULL);

  EXPECT_EQ(coll->GetCollisionBoundingBox(),
      math::Box(math::Vector3(-0.5, -0.5, 0), math::Vector3(0.5, 0.5, 1)));

  // Move the box
  model->SetWorldPose(math::Pose(10, 15, 20, 0, 0, 0));

  EXPECT_EQ(coll->GetCollisionBoundingBox(),
      math::Box(math::Vector3(9.5, 14.5, 19.5),
                math::Vector3(10.5, 15.5, 20.5)));
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
