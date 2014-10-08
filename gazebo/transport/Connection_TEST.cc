/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include <string>
#include <stdlib.h>

#include "gazebo/transport/Connection.hh"
#include "test/util.hh"

using namespace gazebo;

class Connection : public gazebo::testing::AutoLogFixture { };

/////////////////////////////////////////////////
TEST_F(Connection, IPWhiteList)
{
  transport::Connection *connection = new transport::Connection();
  EXPECT_TRUE(connection->GetIPWhiteList().empty());
  delete connection;
  connection = NULL;

  // Get original value
  char *ipEnv = getenv("GAZEBO_IP_WHITE_LIST");

  // Set a new IP White List Value
  setenv("GAZEBO_IP_WHITE_LIST", "192.168.1.1", 1);
  connection = new transport::Connection();
  std::string localAddress = connection->GetLocalAddress();
  EXPECT_EQ(connection->GetIPWhiteList(),
      std::string(",") + localAddress + ",127.0.0.1,192.168.1.1,");
  delete connection;
  connection = NULL;

  // Restore value
  if (ipEnv)
    setenv("GAZEBO_IP_WHITE_LIST", ipEnv, 1);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
