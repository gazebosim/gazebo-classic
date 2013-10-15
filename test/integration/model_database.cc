 /*
 * Copyright 2013 Open Source Robotics Foundation
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

#include <unistd.h>
#include "gazebo/common/ModelDatabase.hh"
#include "ServerFixture.hh"

using namespace gazebo;

int g_onModels = 0;
int g_onModels1 = 0;
int g_onModels2 = 0;

boost::shared_ptr<bool> g_boolRef;
boost::shared_ptr<bool> g_boolRef1;
boost::shared_ptr<bool> g_boolRef2;

class ModelDatabaseTest : public ServerFixture
{
};

void OnModels(const std::map<std::string, std::string> & /*_models*/)
{
  g_onModels++;
  g_boolRef.reset();
}

void OnModels1(const std::map<std::string, std::string> & /*_models*/)
{
  g_onModels1++;
}

void OnModels2(const std::map<std::string, std::string> & /*_models*/)
{
  g_onModels2++;
}

/////////////////////////////////////////////////
// Check the the first callback is called once, and the second twice
TEST_F(ModelDatabaseTest, GetModels)
{
  g_onModels = 0;
  g_onModels1 = 0;

  Load("worlds/empty.world");

  g_boolRef = common::ModelDatabase::Instance()->GetModels(
        boost::bind(&OnModels, _1));

  g_boolRef1 = common::ModelDatabase::Instance()->GetModels(
        boost::bind(&OnModels1, _1));

  while (g_onModels == 0 || g_onModels1 <= 1)
    common::Time::MSleep(500);

  EXPECT_EQ(g_onModels, 1);
  EXPECT_EQ(g_onModels1, 2);
}

/////////////////////////////////////////////////
// Check that the second and third callbacks are received three times
TEST_F(ModelDatabaseTest, GetModelsTriple)
{
  g_onModels = 0;
  g_onModels1 = 0;
  g_onModels2 = 0;

  Load("worlds/empty.world");

  g_boolRef = common::ModelDatabase::Instance()->GetModels(
        boost::bind(&OnModels, _1));

  g_boolRef1 = common::ModelDatabase::Instance()->GetModels(
        boost::bind(&OnModels1, _1));

  g_boolRef2 = common::ModelDatabase::Instance()->GetModels(
        boost::bind(&OnModels2, _1));

  while (!g_onModels && !g_onModels1 && !g_onModels2)
    common::Time::MSleep(500);

  EXPECT_EQ(g_onModels, 1);
  EXPECT_EQ(g_onModels1, 3);
  EXPECT_EQ(g_onModels1, 3);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
