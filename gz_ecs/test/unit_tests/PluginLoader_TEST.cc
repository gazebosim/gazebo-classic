/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <algorithm>
#include <gtest/gtest.h>
#include "gazebo/plugin/PluginLoader.hh"
#include "gazebo/test/util/DummyPlugins.hh"


/////////////////////////////////////////////////
TEST(PluginLoader, InitialNoSearchPaths)
{
  gazebo::plugin::PluginLoader pm;
  EXPECT_EQ(0, pm.SearchPaths().size());
}

/////////////////////////////////////////////////
TEST(PluginLoader, InitialNoInterfacesImplemented)
{
  gazebo::plugin::PluginLoader pm;
  EXPECT_EQ(0, pm.InterfacesImplemented().size());
}

/////////////////////////////////////////////////
TEST(PluginLoader, AddOneSearchPath)
{
  gazebo::plugin::PluginLoader pm;
  pm.AddSearchPath("./");
  ASSERT_EQ(1, pm.SearchPaths().size());
  EXPECT_EQ("./", pm.SearchPaths()[0]);
}

/////////////////////////////////////////////////
TEST(PluginLoader, SearchPathGetsTrailingSlash)
{
  gazebo::plugin::PluginLoader pm;
  pm.AddSearchPath("/usr/local/lib");
  ASSERT_EQ(1, pm.SearchPaths().size());
  EXPECT_EQ("/usr/local/lib/", pm.SearchPaths()[0]);
}

/////////////////////////////////////////////////
TEST(PluginLoader, SearchPathUsesForwardSlashes)
{
  gazebo::plugin::PluginLoader pm;
  pm.AddSearchPath("C:\\user\\alice\\gazebolibs\\");
  ASSERT_EQ(1, pm.SearchPaths().size());
  EXPECT_EQ("C:/user/alice/gazebolibs/", pm.SearchPaths()[0]);
}

/////////////////////////////////////////////////
TEST(PluginLoader, LoadNonexistantLibrary)
{
  gazebo::plugin::PluginLoader pm;
  pm.AddSearchPath("../util");
  EXPECT_FALSE(pm.LoadLibrary("DoesNotExist"));
}

/////////////////////////////////////////////////
TEST(PluginLoader, LoadExistingLibrary)
{
  gazebo::plugin::PluginLoader pm;
  pm.AddSearchPath("../util");
  // The search path is a little fragile
  // It requires running from directory containing this test executable
  // `make test` does this, so it's only an issue if running the test solo
  EXPECT_TRUE(pm.LoadLibrary("GZDummyPlugins"));

  ASSERT_EQ(1, pm.InterfacesImplemented().size());
  ASSERT_EQ("::gazebo::test::util::DummyPluginBase",
            pm.InterfacesImplemented()[0]);
  ASSERT_EQ(1,
      pm.PluginsImplementing("::gazebo::test::util::DummyPluginBase").size());
  std::unique_ptr<gazebo::test::util::DummyPluginBase> plugin =
    pm.Instantiate<gazebo::test::util::DummyPluginBase>(
      "gazebo::test::util::DummyPlugin",
      "gazebo::test::util::DummyPluginBase");
  ASSERT_NE(nullptr, plugin.get());
  EXPECT_EQ(std::string("DummyPlugin"), plugin->MyNameIs());
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
