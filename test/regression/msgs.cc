/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include <string.h>
#include "msgs/msgs.h"
#include "ServerFixture.hh"

using namespace gazebo;
class MsgsTest : public ServerFixture
{
};

TEST_F(MsgsTest, Convert)
{
  {
    msgs::Request *request = msgs::CreateRequest("help", "me");
    EXPECT_STREQ("help", request->request().c_str());
    EXPECT_STREQ("me", request->data().c_str());
    EXPECT_TRUE(request->id() > 0);
  }

  {
    common::Time t = common::Time::GetWallTime();
    msgs::Header msg;
    msgs::Stamp(&msg); 
    EXPECT_EQ(t.sec, msg.stamp().sec());
    EXPECT_TRUE(t.nsec <= msg.stamp().nsec());
  }

  {
    common::Time t = common::Time::GetWallTime();
    msgs::Time msg;
    msgs::Stamp(&msg); 
    EXPECT_EQ(t.sec, msg.sec());
    EXPECT_TRUE(t.nsec <= msg.nsec());
  }

  {
    msgs::Vector3d msg = msgs::Convert(math::Vector3(1, 2, 3));
    EXPECT_EQ(1, msg.x());
    EXPECT_EQ(2, msg.y());
    EXPECT_EQ(3, msg.z());
  }
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
