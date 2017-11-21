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

using namespace gazebo;
class SDFTest : public ServerFixture
{
};

/*
TEST_F(PR2Test, Param)
{
  sdf::ParamT<char> charP("c", 'a', 0);
  sdf::ParamT<bool> boolP("b", true, 0);
  sdf::ParamT<float> floatP("f", 1.0, 0);
  sdf::ParamT<double> doubleP("d", 1.0, 0);
  sdf::ParamT<int> intP("i", 1, 0);
  sdf::ParamT<unsigned int> uintP("ui", 1, 0);
  sdf::ParamT<std::string> stringP("s", "default", 0);
  sdf::ParamT<common::Color> colorP("c", common::Color(.1, .2, .3, 1), 0);
  sdf::ParamT<ignition::math::Vector3d> vec3P("v3",
      ignition::math::Vector3d(1, 2, 3), 0);
  sdf::ParamT<ignition::math::Pose3d> poseP("v3", ignition:math::Posed(
      ignition::math::Vector3d(1, 2, 3),
      ignition::math::Quaterniond(0, 0, M_PI)), 0);

  EXPECT_TRUE(boolP.IsBool());
  EXPECT_TRUE(intP.IsInt());
  EXPECT_TRUE(uintP.IsUInt());
  EXPECT_TRUE(floatP.IsFloat());
  EXPECT_TRUE(doubleP.IsDouble());
  EXPECT_TRUE(doubleP.IsDouble());
}
*/

TEST_F(SDFTest, WorldGetSDF)
{
  // load the empty world
  Load("worlds/empty.world");
  physics::WorldPtr world = physics::get_world("default");
  ASSERT_TRUE(world != NULL);
  world->SetPaused(true);

  // get the world sdf and assert that it is not null
  sdf::ElementPtr worldSdf = world->SDF();
  ASSERT_NE(worldSdf, nullptr);

  // only one top-level element expected, which is "world"
  EXPECT_FALSE(worldSdf->GetNextElement());
  EXPECT_EQ(worldSdf->GetName(), "world");

  // get the first (and only) model child, which has to be the ground floor
  sdf::ElementPtr modelSdf = worldSdf->GetFirstElement();
  ASSERT_NE(modelSdf, nullptr);
  // now get the first model child of the world element
  modelSdf = modelSdf->GetNextElement("model");
  ASSERT_NE(modelSdf, nullptr);
  EXPECT_FALSE(modelSdf->GetNextElement("model"));
  ASSERT_TRUE(modelSdf->HasAttribute("name"));
  EXPECT_EQ(modelSdf->GetAttribute("name")->GetAsString(), "ground_plane");
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
