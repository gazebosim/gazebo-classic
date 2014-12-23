/*
 * Copyright 2014 Open Source Robotics Foundation
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

using namespace gazebo;

class SetWorldPoseTest : public ServerFixture {};

/////////////////////////////////////////////////
TEST_F(SetWorldPoseTest, Stress)
{
  Load("worlds/box_plane_low_friction_test.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  physics::ModelPtr model = world->GetModel("box");
  ASSERT_TRUE(model != NULL);

  math::Pose pose(1, 2, 3, 0, 0, 0);

  common::Time startTime = common::Time::GetWallTime();
  for (unsigned int i = 0; i < 10000000; ++i)
  {
    model->SetWorldPose(pose);
  }
  common::Time endTime = common::Time::GetWallTime();

  gzdbg << "Time elapsed while setting world pose ["
        << endTime - startTime << "]\n";

  EXPECT_LT(endTime - startTime, common::Time(15, 0));
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
