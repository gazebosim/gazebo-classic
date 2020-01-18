/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

std::vector<msgs::Model> g_modelMsgs;
std::mutex g_mutex;
void ModelInfoCallback(ConstModelPtr &_model)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  g_modelMsgs.push_back(*_model.get());
}


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

/////////////////////////////////////////////////
// Test setting scale of model and verify visual msg
TEST_F(ModelTest, SetScale)
{
  Load("worlds/shapes.world");
  physics::ModelPtr model = GetModel("sphere");
  ASSERT_NE(model, nullptr);

  physics::WorldPtr world = model->GetWorld();
  ASSERT_NE(world, nullptr);

  std::string modelTopic = "~/model/info";
  transport::SubscriberPtr sub = node->Subscribe(modelTopic, ModelInfoCallback);

  double s = 0.25;
  ignition::math::Vector3d scale(s, s, s);
  model->SetScale(scale, true);

  // publish msg
  int sleep = 0;
  bool receivedMsgs = false;
  while (!receivedMsgs && sleep++ < 100)
  {
    world->Step(1);
    common::Time::MSleep(100);
    {
      std::lock_guard<std::mutex> lock(g_mutex);
      receivedMsgs = !g_modelMsgs.empty();
    }
  }

  EXPECT_FALSE(g_modelMsgs.empty());

  // verify geometry of the visual msg in the link
  std::string visualName = "sphere::link::visual";
  physics::LinkPtr link = model->GetLink();
  msgs::Visual visualMsg = link->GetVisualMessage(visualName);
  EXPECT_EQ("sphere::link::visual", visualMsg.name());
  double expectedRadius = s * 0.5;
  EXPECT_DOUBLE_EQ(expectedRadius, visualMsg.geometry().sphere().radius());

  // verify scale
  msgs::Model modelMsg = g_modelMsgs[0];
  EXPECT_TRUE(modelMsg.has_scale());
  EXPECT_DOUBLE_EQ(s, modelMsg.scale().x());
  EXPECT_DOUBLE_EQ(s, modelMsg.scale().y());
  EXPECT_DOUBLE_EQ(s, modelMsg.scale().z());

  // verify geometry of the visual msg received
  ASSERT_GE(modelMsg.link_size(), 1);
  msgs::Link linkMsg = modelMsg.link(0);
  msgs::Visual receivedVisualMsg;
  for (int i = 0; i < linkMsg.visual_size(); ++i)
  {
    if (linkMsg.visual(i).name() == visualName)
    {
      receivedVisualMsg = linkMsg.visual(i);
      break;
    }
  }
  EXPECT_EQ(visualName, receivedVisualMsg.name());
  EXPECT_DOUBLE_EQ(expectedRadius,
    receivedVisualMsg.geometry().sphere().radius());

  g_modelMsgs.clear();
}


int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
