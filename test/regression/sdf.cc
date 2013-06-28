/*
 * Copyright 2012 Open Source Robotics Foundation
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
class RMLTest : public ServerFixture
{
};

TEST_F(PR2Test, Param)
{
  rml::ParamT<char> charP("c", 'a', 0);
  rml::ParamT<bool> boolP("b", true, 0);
  rml::ParamT<float> floatP("f", 1.0, 0);
  rml::ParamT<double> doubleP("d", 1.0, 0);
  rml::ParamT<int> intP("i", 1, 0);
  rml::ParamT<unsigned int> uintP("ui", 1, 0);
  rml::ParamT<std::string> stringP("s", "default", 0);
  rml::ParamT<common::Color> colorP("c", common::Color(.1, .2, .3, 1), 0);
  rml::ParamT<math::Vector3> vec3P("v3", math::Vector3(1, 2, 3), 0);
  rml::ParamT<math::Pose> poseP("v3", math::Pose(math::Vector3(1, 2, 3),
        math::Quaternion(0, 0, M_PI)), 0);

  EXPECT_TRUE(boolP.IsBool());
  EXPECT_TRUE(intP.IsInt());
  EXPECT_TRUE(uintP.IsUInt());
  EXPECT_TRUE(floatP.IsFloat());
  EXPECT_TRUE(doubleP.IsDouble());
  EXPECT_TRUE(doubleP.IsDouble());
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
