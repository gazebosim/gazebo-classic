/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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
#include <stdio.h>

const std::string cmakeSourceDir = CMAKE_SOURCE_DIR;
const std::string worldEditPath = "examples/plugins/world_edit";

TEST(ExamplePlugins, WorldEdit)
{
  char cmd[1024];

  snprintf(cmd, sizeof(cmd), "cmake %s",
    std::string(cmakeSourceDir + '/' + worldEditPath).c_str());
  ASSERT_EQ(system(cmd), 0);
  snprintf(cmd, sizeof(cmd), "make");
  ASSERT_EQ(system(cmd), 0);

  // Should also try to run it
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
