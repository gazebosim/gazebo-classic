/*
 * Copyright (C) 2013-2015 Open Source Robotics Foundation
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
#include <boost/filesystem.hpp>
#include <fstream>

#include "ServerFixture.hh"

using namespace gazebo;
class OgreLog : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(OgreLog, PubSub)
{
  Load("worlds/empty.world");

  boost::filesystem::path logPath =
    common::SystemPaths::Instance()->GetLogPath();
  logPath /= "ogre.log";
  std::ifstream ogreLog(logPath.string().c_str(), std::ios::in);
  ASSERT_TRUE(ogreLog.is_open());

  while (!ogreLog.eof())
  {
    std::string line;
    std::getline(ogreLog, line);
    EXPECT_EQ(line.find("Error"), std::string::npos);
    EXPECT_EQ(line.find("error"), std::string::npos);
    EXPECT_EQ(line.find("ERROR"), std::string::npos);
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
