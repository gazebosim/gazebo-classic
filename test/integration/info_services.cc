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

#include <string>

#include <ignition/msgs/plugin_v.pb.h>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/transport/Node.hh>

#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;

class InfoServicesTest : public ServerFixture
{
};

/////////////////////////////////////////////////
// Check getting information about a specific model plugin.
bool g_specificModelPlugin = false;
void ReceiveSpecificModelPlugin(const ignition::msgs::Plugin_V &_plugins,
    const bool _success)
{
  EXPECT_FALSE(g_specificModelPlugin);
  EXPECT_TRUE(_success);
  ASSERT_EQ(_plugins.plugins().size(), 1);
  EXPECT_EQ(_plugins.plugins(0).name(), "buoyancy");
  EXPECT_EQ(_plugins.plugins(0).filename(), "libBuoyancyPlugin.so");
  EXPECT_EQ(_plugins.plugins(0).innerxml(),
      "<fluid_density>1000</fluid_density>\n");

  g_specificModelPlugin = true;
}

/////////////////////////////////////////////////
// Check getting information about all plugins in a model.
bool g_allModelPlugins = false;
void ReceiveAllModelPlugins(const ignition::msgs::Plugin_V &_plugins,
    const bool _success)
{
  EXPECT_FALSE(g_allModelPlugins);
  EXPECT_TRUE(_success);
  ASSERT_EQ(_plugins.plugins().size(), 5);
  EXPECT_EQ(_plugins.plugins(0).name(), "submarine_propeller_1");
  EXPECT_EQ(_plugins.plugins(1).name(), "submarine_propeller_2");
  EXPECT_EQ(_plugins.plugins(2).name(), "submarine_propeller_3");
  EXPECT_EQ(_plugins.plugins(3).name(), "submarine_propeller_4");
  EXPECT_EQ(_plugins.plugins(4).name(), "buoyancy");

  g_allModelPlugins = true;
}

/////////////////////////////////////////////////
// Check using a bad URI
bool g_badURI = false;
void ReceiveBadURI(const ignition::msgs::Plugin_V &/*_plugins*/,
    const bool _success)
{
  EXPECT_FALSE(g_badURI);
  EXPECT_FALSE(_success);

  g_badURI = true;
}

/////////////////////////////////////////////////
// Check using an incomplete URI
bool g_badURI2 = false;
void ReceiveBadURI2(const ignition::msgs::Plugin_V &/*_plugins*/,
    const bool _success)
{
  EXPECT_FALSE(g_badURI2);
  EXPECT_FALSE(_success);

  g_badURI2 = true;
}

/////////////////////////////////////////////////
// Check getting information about a world that doesn't exist.
bool g_badWorld = false;
void ReceiveBadWorld(const ignition::msgs::Plugin_V &/*_plugins*/,
    const bool _success)
{
  EXPECT_FALSE(g_badWorld);
  EXPECT_FALSE(_success);

  g_badWorld = true;
}

/////////////////////////////////////////////////
// Check getting information about all plugins in a model that doesn't exist.
bool g_badModel = false;
void ReceiveBadModel(const ignition::msgs::Plugin_V &/*_plugins*/,
    const bool _success)
{
  EXPECT_FALSE(g_badModel);
  EXPECT_FALSE(_success);

  g_badModel = true;
}

/////////////////////////////////////////////////
// Check getting information about a model plugin that doesn't exist.
bool g_badModelPlugin = false;
void ReceiveBadModelPlugin(const ignition::msgs::Plugin_V &/*_plugins*/,
    const bool _success)
{
  EXPECT_FALSE(g_badModelPlugin);
  EXPECT_FALSE(_success);

  g_badModelPlugin = true;
}

/////////////////////////////////////////////////
// Check using an element which is not handled
bool g_badElement = false;
void ReceiveBadElement(const ignition::msgs::Plugin_V &/*_plugins*/,
    const bool _success)
{
  EXPECT_FALSE(g_badElement);
  EXPECT_FALSE(_success);

  g_badElement = true;
}

/////////////////////////////////////////////////
// Request info about model plugins
TEST_F(InfoServicesTest, ModelPlugins)
{
  this->Load("worlds/underwater.world");

  ignition::transport::Node ignNode;

  std::string pluginInfoService("/physics/info/plugin");
  ignition::msgs::StringMsg req;

  // Request info on a specific model plugin
  req.set_data("data://world/default/model/submarine/plugin/buoyancy");

  EXPECT_FALSE(g_specificModelPlugin);
  ignNode.Request(pluginInfoService, req, ReceiveSpecificModelPlugin);
  EXPECT_TRUE(g_specificModelPlugin);

  // Request info on all plugins in a model
  req.set_data("data://world/default/model/submarine/plugin");

  EXPECT_FALSE(g_allModelPlugins);
  ignNode.Request(pluginInfoService, req, ReceiveAllModelPlugins);
  EXPECT_TRUE(g_allModelPlugins);

  // Request with bad URI
  req.set_data("data:/world/default/model/submarine/plugins");

  EXPECT_FALSE(g_badURI);
  ignNode.Request(pluginInfoService, req, ReceiveBadURI);
  EXPECT_TRUE(g_badURI);

  // Request info with incomplete URI
  req.set_data("data://world/");

  EXPECT_FALSE(g_badURI2);
  ignNode.Request(pluginInfoService, req, ReceiveBadURI2);
  EXPECT_TRUE(g_badURI2);

  // Request info on a world that doesn't exist
  req.set_data("data://world/i_dont_exist/model/submarine/plugins");

  EXPECT_FALSE(g_badWorld);
  ignNode.Request(pluginInfoService, req, ReceiveBadWorld);
  EXPECT_TRUE(g_badWorld);

  // Request info on a model that doesn't exist
  req.set_data("data://world/default/model/i_dont_exit/plugins");

  EXPECT_FALSE(g_badModel);
  ignNode.Request(pluginInfoService, req, ReceiveBadModel);
  EXPECT_TRUE(g_badModel);

  // Request info on a model plugin that doesn't exist
  req.set_data("data://world/default/model/submarine/plugins/i_dont_exist");

  EXPECT_FALSE(g_badModelPlugin);
  ignNode.Request(pluginInfoService, req, ReceiveBadModelPlugin);
  EXPECT_TRUE(g_badModelPlugin);

  // Request using a segment not handled
  req.set_data("data://world/default/light/sun/plugins");

  EXPECT_FALSE(g_badElement);
  ignNode.Request(pluginInfoService, req, ReceiveBadElement);
  EXPECT_TRUE(g_badElement);
}

/////////////////////////////////////////////////
// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
