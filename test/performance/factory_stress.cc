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
#include "gazebo/transport/transport.hh"
#include "ServerFixture.hh"

using namespace gazebo;
class FactoryStressTest : public ServerFixture
{
};

/////////////////////////////////////////////////
void OnWorldStats(ConstWorldStatisticsPtr &/*_msg*/)
{
}

/////////////////////////////////////////////////
TEST_F(FactoryStressTest, Bookshelf)
{
  Load("worlds/empty.world");

  gazebo::transport::SubscriberPtr sub =
    this->node->Subscribe("~/world_stats", &OnWorldStats);

  for (int i = 0; i < 100; ++i)
  {
    SpawnModel("model://bookshelf");
    gazebo::common::Time::MSleep(500);
    RemoveModel("bookshelf");
  }

  sub.reset();
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
