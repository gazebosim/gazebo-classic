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

#include <gtest/gtest.h>
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/rendering/RenderEngine.hh"

using namespace gazebo;
class RenderEngine_TEST : public RenderingFixture
{
};

/////////////////////////////////////////////////
TEST_F(RenderEngine_TEST, FSAATest)
{
  Load("worlds/empty.world");

  rendering::ScenePtr scene = rendering::get_scene("default");

  if (!scene)
    scene = rendering::create_scene("default", false);

  EXPECT_TRUE(scene != nullptr);

  // get all supported fsaa levels
  std::vector<unsigned int> fsaa =
      rendering::RenderEngine::Instance()->FSAALevels();

  // verify there is data
  EXPECT_GE(fsaa.size(), 1u);

  for (auto const f : fsaa)
  {
    // verify the value is a multiple of 2
    EXPECT_EQ(f % 2u, 0u);
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
