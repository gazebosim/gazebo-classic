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
#include <gtest/gtest.h>
#include <stdio.h>
#include "test_config.h"

TEST(PkgConfig, Config)
{
  char cmd[1024];

  // Delete previous build directory to not reuse previous CMakeCache
  snprintf(cmd, sizeof(cmd), "rm -fr %s/test/pkgconfig/plugin",
           PROJECT_BINARY_PATH);
  ASSERT_EQ(system(cmd), 0);

  // Create a build directory in the project binary directory so that we
  // don't pollute the source tree
  snprintf(cmd, sizeof(cmd), "mkdir %s/test/pkgconfig/plugin",
           PROJECT_BINARY_PATH);
  ASSERT_EQ(system(cmd), 0);

  // Run cmake
  snprintf(
    cmd, sizeof(cmd),
    "cd %s/test/pkgconfig/plugin; cmake %s -DGAZEBO_EXPECTED_VERSION=%s",
    PROJECT_BINARY_PATH, SOURCE_DIR, GAZEBO_EXPECTED_VERSION);
  ASSERT_EQ(system(cmd), 0);

  // Make
  snprintf(cmd, sizeof(cmd), "cd %s/test/pkgconfig/plugin; make",
           PROJECT_BINARY_PATH);
  ASSERT_EQ(system(cmd), 0);
}

#ifndef WIN32
TEST(PkgConfig, CC)
{
  char cmd[1024];

  snprintf(cmd, sizeof(cmd),
     "cc -fPIC -shared `pkg-config --cflags gazebo`\
     `pkg-config --libs gazebo` -o hello_world.so \
     %s/../../testfiles/hello_world.cc", SOURCE_DIR);
  ASSERT_EQ(system(cmd), 0);
}
#endif

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
