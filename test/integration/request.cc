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

#include <ignition/transport/Node.hh>
#include "gazebo/test/ServerFixture.hh"
#include "gazebo/transport/Request.hh"

using namespace gazebo;

class RequestTest : public ServerFixture
{
};

bool g_modelDeleted = false;
bool g_linkDeleted = false;
bool g_collisionDeleted = false;
bool g_directResponse = false;
bool g_success = false;
size_t g_requestId = 0;

/////////////////////////////////////////////////
void OnNotification(const ignition::msgs::Operation &_msg)
{
  EXPECT_TRUE(_msg.has_type());
  EXPECT_EQ(_msg.type(), ignition::msgs::Operation::DELETE);

  EXPECT_TRUE(_msg.has_op_delete());

  auto uri = _msg.op_delete().uri();

  if (uri.find("box") != std::string::npos)
  {
    //EXPECT_NEQ(uri.find("model"), std::string::npos);

    if (_msg.has_id() && _msg.id() == g_requestId)
    {
      g_directResponse = true;

      EXPECT_TRUE(_msg.has_success());
      g_success = _msg.success();
    }
    else
      g_modelDeleted = true;
  }
  else if (uri.find("link") != std::string::npos)
  {
//    EXPECT_EQ(type, ignition::msgs::Types::LINK);
    g_linkDeleted = true;
  }
  else if (uri.find("collision") != std::string::npos)
  {
  //  EXPECT_EQ(type, ignition::msgs::Types::COLLISION);
    g_collisionDeleted = true;
  }
  else if (uri.find("non_existing") != std::string::npos)
  {
    //EXPECT_EQ(type, ignition::msgs::Types::NONE);
    if (_msg.has_id() && _msg.id() == g_requestId)
    {
      g_directResponse = true;

      EXPECT_TRUE(_msg.has_success());
      g_success = _msg.success();
    }
  }
  else
  {
    EXPECT_TRUE(false) << "Unknown entity deleted [" << uri << "]";
  }
}

/////////////////////////////////////////////////
TEST_F(RequestTest, RequestDelete)
{
  // Load a world with entities
  this->Load("worlds/shapes.world");

  EXPECT_FALSE(g_modelDeleted);
  EXPECT_FALSE(g_linkDeleted);
  EXPECT_FALSE(g_collisionDeleted);
  EXPECT_FALSE(g_directResponse);
  EXPECT_FALSE(g_success);

  // Subscribe to notifications
  ignition::transport::Node ignNode;
  ignNode.Subscribe("/notification", &OnNotification);

  // Request deletion of existing model
  g_requestId = transport::RequestDelete(common::URI("box"));
  EXPECT_NE(g_requestId, 0u);

  // Wait for the request to be completed
  int maxSleep = 10;
  int sleep = 0;
  while (!g_modelDeleted &&
         !g_linkDeleted &&
         !g_collisionDeleted &&
         !g_directResponse &&
         !g_success
      && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    ++sleep;
  }
  EXPECT_TRUE(g_modelDeleted);
  EXPECT_TRUE(g_linkDeleted);
  EXPECT_TRUE(g_collisionDeleted);
  EXPECT_TRUE(g_directResponse);
  EXPECT_TRUE(g_success);

  // Reset variables
  g_modelDeleted = false;
  g_linkDeleted = false;
  g_collisionDeleted = false;
  g_directResponse = false;
  g_success = true;
  g_requestId = 0;

  // Request deletion of non existing model
  g_requestId = transport::RequestDelete(common::URI("non_existing"));
  EXPECT_NE(g_requestId, 0u);

  // Wait for the request to be completed
  sleep = 0;
  while (!g_modelDeleted &&
         !g_linkDeleted &&
         !g_collisionDeleted &&
         !g_directResponse &&
         g_success
      && sleep < maxSleep)
  {
    common::Time::MSleep(100);
    ++sleep;
  }
  EXPECT_FALSE(g_modelDeleted);
  EXPECT_FALSE(g_linkDeleted);
  EXPECT_FALSE(g_collisionDeleted);
  EXPECT_TRUE(g_directResponse);
  EXPECT_FALSE(g_success);
}

/////////////////////////////////////////////////
// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
