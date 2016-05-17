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

#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class PhysicsBase : public ServerFixture
{
};

/////////////////////////////////////////////////
// \brief Test URI for various entities.
TEST_F(PhysicsBase, URI)
{
  this->Load("worlds/joints.world");

  // Get a pointer to the world
  auto world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  // Check light URI
  auto light = world->Light("sun");
  ASSERT_TRUE(light != NULL);
  EXPECT_EQ(light->URI().Str(), "data://world/default/light/sun");

  // Check model URI
  auto model = world->GetModel("revolute_model");
  ASSERT_TRUE(model != NULL);
  EXPECT_EQ(model->URI().Str(), "data://world/default/model/revolute_model");

  // Check joint URI
  auto joint = model->GetJoint("revolute_joint");
  ASSERT_TRUE(joint != NULL);
  EXPECT_EQ(joint->URI().Str(),
      "data://world/default/model/revolute_model/joint/revolute_joint");

  // Check link URI
  auto link = model->GetLink("body1");
  ASSERT_TRUE(link != NULL);
  EXPECT_EQ(link->URI().Str(),
      "data://world/default/model/revolute_model/link/body1");

  // Check collision URI
  auto collision = link->GetCollision("geom");
  ASSERT_TRUE(collision != NULL);
  EXPECT_EQ(collision->URI().Str(),
      "data://world/default/model/revolute_model/link/body1/collision/geom");
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
