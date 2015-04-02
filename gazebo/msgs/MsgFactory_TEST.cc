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
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/msgs/MsgFactory.hh"
#include "test_config.h"
#include "test/util.hh"

class MsgFactory : public gazebo::testing::AutoLogFixture { };

std::string custom_exec(const std::string &_cmd)
{
  FILE* pipe = popen(_cmd.c_str(), "r");

  if (!pipe)
    return "ERROR";

  char buffer[128];
  std::string result = "";

  while (!feof(pipe))
  {
    if (fgets(buffer, 128, pipe) != NULL)
      result += buffer;
  }

  pclose(pipe);
  return result;
}

/////////////////////////////////////////////////
/// Check for null when asked for a bad message type
TEST_F(MsgFactory, BadMsgType)
{
  boost::shared_ptr<google::protobuf::Message> msg =
    gazebo::msgs::MsgFactory::NewMsg("bad");
  EXPECT_EQ(NULL, msg.get());
}

/////////////////////////////////////////////////
/// Check to make sure that NewMsg works properly
TEST_F(MsgFactory, NewMsg)
{
  gazebo::msgs::Vector3d goodMsg;
  goodMsg.set_x(1.1);
  goodMsg.set_y(2.2);
  goodMsg.set_z(3.3);

  std::string serializedData;
  goodMsg.SerializeToString(&serializedData);

  gazebo::msgs::Vector3dPtr msg =
    boost::dynamic_pointer_cast<gazebo::msgs::Vector3d>(
        gazebo::msgs::MsgFactory::NewMsg("gazebo.msgs.Vector3d"));

  msg->ParseFromString(serializedData);
  EXPECT_NEAR(msg->x(), 1.1, 1e-6);
  EXPECT_NEAR(msg->y(), 2.2, 1e-6);
  EXPECT_NEAR(msg->z(), 3.3, 1e-6);
}

/////////////////////////////////////////////////
/// Check to make sure that all the proto files have been registered
TEST_F(MsgFactory, AllTypes)
{
  std::vector<std::string> types;
  gazebo::msgs::MsgFactory::GetMsgTypes(types);
  std::string protoCount = custom_exec(std::string("ls ") +
      PROJECT_SOURCE_PATH + "/gazebo/msgs/*.proto | wc -l");
  boost::trim(protoCount);

  EXPECT_EQ(boost::lexical_cast<size_t>(protoCount), types.size());
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
