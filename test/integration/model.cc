/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class ModelTest : public ServerFixture
{
};

/////////////////////////////////////////////////
// This tests getting links from a model.
TEST_F(ModelTest, GetLinksV)
{
  Load("worlds/joint_test.world");
  physics::ModelPtr model = GetModel("model_1");

  // This for-loop would cause a seg-fault in gazebo 3.0 and before.
  for (physics::Link_V::const_iterator iter = model->GetLinks().begin();
       iter != model->GetLinks().end(); ++iter)
  {
    EXPECT_TRUE(*iter != NULL);
    EXPECT_FALSE((*iter)->GetName().empty());
    EXPECT_STREQ((*iter)->GetName().c_str(), "link_1");
  }

  EXPECT_EQ(model->GetLinks().size(), 1u);
}

/////////////////////////////////////////////////
// This tests getting the scoped name of a model.
TEST_F(ModelTest, GetScopedName)
{
  Load("worlds/simple_arm_test.world");

  physics::ModelPtr model = GetModel("simple_arm");

  std::string modelName = model->GetScopedName();
  EXPECT_EQ(modelName, std::string("simple_arm"));

  modelName = model->GetScopedName(true);
  EXPECT_EQ(modelName, std::string("default::simple_arm"));
}

/////////////////////////////////////////////////
// This tests creating new links within a model.
TEST_F(ModelTest, CreateLink)
{
  Load("worlds/joint_test.world");
  physics::ModelPtr model = GetModel("model_1");

  const std::string linkName = "link_name";
  physics::LinkPtr link = model->CreateLink(linkName);
  ASSERT_TRUE(link != NULL);
  EXPECT_EQ(link->GetName(), linkName);
  EXPECT_EQ(link, model->GetLink(linkName));

  // Make sure we cannot create a second link with the same name
  physics::LinkPtr link2 = model->CreateLink(linkName);
  EXPECT_TRUE(link2 == NULL);

  // GetLink should still return the original link
  EXPECT_EQ(link, model->GetLink(linkName));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
