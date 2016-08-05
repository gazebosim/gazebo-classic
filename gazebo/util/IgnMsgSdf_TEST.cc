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

#include "gazebo/util/IgnMsgSdf.hh"
#include "test/util.hh"

using namespace gazebo;

class IgnMsgSdfTest : public gazebo::testing::AutoLogFixture {};

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, ConvertNotSupportedType)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("plugin.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <plugin name='plugin_name' filename='plugin_filename'>\
         </plugin>\
       </sdf>", sdf));

  auto msg = util::Convert<ignition::msgs::WorldControl>(sdf);
  EXPECT_FALSE(msg.has_pause());
  EXPECT_FALSE(msg.has_step());
  EXPECT_FALSE(msg.has_multi_step());
  EXPECT_FALSE(msg.has_reset());
  EXPECT_FALSE(msg.has_seed());
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, ConvertWrongType)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("gui.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <gui fullscreen='true'>\
         </gui>\
       </sdf>", sdf));

  // Error message is printed
  auto msg = util::Convert<ignition::msgs::Plugin>(sdf);
  EXPECT_FALSE(msg.has_name());
  EXPECT_FALSE(msg.has_filename());
  EXPECT_FALSE(msg.has_innerxml());
  EXPECT_EQ(msg.DebugString(), "");
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, PluginSdfToIgnMsg)
{
  sdf::ElementPtr sdf(new sdf::Element());
  sdf::initFile("plugin.sdf", sdf);
  ASSERT_TRUE(sdf::readString(
      "<sdf version='" SDF_VERSION "'>\
         <plugin name='plugin_name' filename='plugin_filename'>\
           <param1>1</param1>\
           <param2>true</param2>\
         </plugin>\
       </sdf>", sdf));
  auto msg = util::Convert<ignition::msgs::Plugin>(sdf);

  EXPECT_TRUE(msg.has_name());
  EXPECT_EQ(msg.name(), "plugin_name");
  EXPECT_TRUE(msg.has_filename());
  EXPECT_EQ(msg.filename(), "plugin_filename");
  EXPECT_TRUE(msg.has_innerxml());
  EXPECT_EQ(msg.innerxml(), "<param1>1</param1>\n<param2>true</param2>\n");
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, PluginIgnMsgToSdf)
{
  std::string name = "plugin_name";
  std::string filename = "plugin_filename";
  std::string innerxml = "<plugin_param>param</plugin_param>";

  ignition::msgs::Plugin msg;
  msg.set_name(name);
  msg.set_filename(filename);
  msg.set_innerxml(innerxml);

  auto pluginSDF = util::Convert(msg);

  EXPECT_TRUE(pluginSDF->HasAttribute("name"));
  EXPECT_EQ(pluginSDF->Get<std::string>("name"), name);

  EXPECT_TRUE(pluginSDF->HasAttribute("filename"));
  EXPECT_EQ(pluginSDF->Get<std::string>("filename"), filename);

  EXPECT_TRUE(pluginSDF->HasElement("plugin_param"));
  EXPECT_EQ(pluginSDF->Get<std::string>("plugin_param"), "param");
}

/////////////////////////////////////////////////
TEST_F(IgnMsgSdfTest, PluginToFromSDF)
{
  std::string name = "plugin_name";
  std::string filename = "plugin_filename";
  std::string innerxml = "<plugin_param>param</plugin_param>\n";

  // Create message
  ignition::msgs::Plugin msg1;
  msg1.set_name(name);
  msg1.set_filename(filename);
  msg1.set_innerxml(innerxml);

  // To SDF
  auto sdf1 = util::Convert(msg1);

  // Back to Msg
  auto msg2 = util::Convert<ignition::msgs::Plugin>(sdf1);
  EXPECT_EQ(msg2.name(), name);
  EXPECT_EQ(msg2.filename(), filename);
  EXPECT_EQ(msg2.innerxml(), innerxml);

  // Back to SDF
  sdf::ElementPtr sdf2;
  sdf2.reset(new sdf::Element);
  sdf::initFile("plugin.sdf", sdf2);
  util::Convert(msg2, sdf2);
  EXPECT_TRUE(sdf2 != NULL);
  EXPECT_EQ(sdf2->Get<std::string>("name"), name);
  EXPECT_EQ(sdf2->Get<std::string>("filename"), filename);
  EXPECT_TRUE(sdf2->HasElement("plugin_param"));
  EXPECT_EQ(sdf2->Get<std::string>("plugin_param"), "param");
}
