 /*
 * Copyright (C) 2013 Open Source Robotics Foundation
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

#include "gazebo/common/ModelDatabase.hh"
#include "gazebo/test/ServerFixture.hh"
#include "test_config.h"

using namespace gazebo;
using namespace boost::placeholders;

int g_onModels = 0;
int g_onModels1 = 0;
int g_onModels2 = 0;

event::ConnectionPtr g_Connection;
event::ConnectionPtr g_Connection1;
event::ConnectionPtr g_Connection2;

class ModelDatabaseTest : public ServerFixture
{
};

void OnModels(const std::map<std::string, std::string> & /*_models*/)
{
  g_onModels++;
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

  g_Connection = common::ModelDatabase::Instance()->GetModels(
        boost::bind(&OnModels, _1));

  g_Connection1 = common::ModelDatabase::Instance()->GetModels(
        boost::bind(&OnModels1, _1));

  while (g_onModels == 0 || g_onModels1 == 0)
    common::Time::MSleep(500);

  EXPECT_EQ(g_onModels, 1);
  EXPECT_EQ(g_onModels1, 1);
}

/////////////////////////////////////////////////
// Check the the first callback is called once, and the second twice
TEST_F(ModelDatabaseTest, GetModelsTwice)
{
  g_onModels = 0;
  g_onModels1 = 0;

  Load("worlds/empty.world");

  g_Connection = common::ModelDatabase::Instance()->GetModels(
        boost::bind(&OnModels, _1));

  g_Connection1 = common::ModelDatabase::Instance()->GetModels(
        boost::bind(&OnModels1, _1));

  while (g_onModels == 0 || g_onModels1 == 0)
    common::Time::MSleep(500);

  EXPECT_EQ(g_onModels, 1);
  EXPECT_EQ(g_onModels1, 1);


  // Reset bool reference, so now only g_onModels1 should increment
  g_Connection.reset();

  common::ModelDatabase::Instance()->GetModels(boost::bind(&OnModels, _1));

  while (g_onModels1 == 1)
    common::Time::MSleep(500);

  EXPECT_EQ(g_onModels, 1);
  EXPECT_EQ(g_onModels1, 2);
}

/////////////////////////////////////////////////
// Check that the second and third callbacks are received three times
TEST_F(ModelDatabaseTest, GetModelsThrice)
{
  g_onModels = 0;
  g_onModels1 = 0;
  g_onModels2 = 0;

  Load("worlds/empty.world");

  g_Connection = common::ModelDatabase::Instance()->GetModels(
        boost::bind(&OnModels, _1));

  g_Connection1 = common::ModelDatabase::Instance()->GetModels(
        boost::bind(&OnModels1, _1));

  g_Connection2 = common::ModelDatabase::Instance()->GetModels(
        boost::bind(&OnModels2, _1));

  while (g_onModels == 0 || g_onModels1 == 0 || g_onModels2 == 0)
  {
    common::Time::MSleep(1000);
  }

  EXPECT_EQ(g_onModels, 1);
  EXPECT_EQ(g_onModels1, 1);
  EXPECT_EQ(g_onModels2, 1);

  // Reset bool reference, so now only g_onModels1 and g_onModels2 should
  // increment
  g_Connection.reset();

  common::ModelDatabase::Instance()->GetModels(
      boost::bind(&OnModels, _1));

  while (g_onModels1 == 1 || g_onModels2 == 1)
    common::Time::MSleep(500);

  EXPECT_EQ(g_onModels, 1);
  EXPECT_EQ(g_onModels1, 2);
  EXPECT_EQ(g_onModels2, 2);

  // Reset bool reference, so now only g_onModels2 should increment
  g_Connection1.reset();

  common::ModelDatabase::Instance()->GetModels(
      boost::bind(&OnModels, _1));

  while (g_onModels2 == 2)
    common::Time::MSleep(500);

  EXPECT_EQ(g_onModels, 1);
  EXPECT_EQ(g_onModels1, 2);
  EXPECT_EQ(g_onModels2, 3);
}

/////////////////////////////////////////////////
TEST_F(ModelDatabaseTest, Version)
{
  // add test model cococan to path so that the model database
  // can pick it up
  gazebo::common::SystemPaths::Instance()->AddModelPaths(
    PROJECT_SOURCE_PATH "/test/models/testdb");

  std::string uri = "model://cococan";
  std::string model;
  model = gazebo::common::ModelDatabase::Instance()->GetModelFile(uri);
  // this model hias multiple sdf files. 1_4 is the correct one
  EXPECT_TRUE(model.find("model-1_4.sdf") != std::string::npos);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
