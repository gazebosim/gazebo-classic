/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/SelectionObj.hh"
#include "test/ServerFixture.hh"

using namespace gazebo;
class SelectionObj_TEST : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(SelectionObj_TEST, SelectionObjTest)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");

  if (!scene)
    scene = gazebo::rendering::create_scene("default", false);

  EXPECT_TRUE(scene != NULL);

  // Test calling constructor and Load functions and make sure
  // there are no segfaults
  rendering::SelectionObjPtr obj;
  obj.reset(new rendering::SelectionObj("obj", scene->GetWorldVisual()));
  obj->Load();

  // Selection none = no handles visible
  obj->SetMode(rendering::SelectionObj::SELECTION_NONE);
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::TRANS));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::ROT));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::SCALE));

  // Translate handles visible
  obj->SetMode(rendering::SelectionObj::TRANS);
  EXPECT_TRUE(obj->GetHandleVisible(rendering::SelectionObj::TRANS));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::ROT));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::SCALE));

  // Rotate handles visible
  obj->SetMode(rendering::SelectionObj::ROT);
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::TRANS));
  EXPECT_TRUE(obj->GetHandleVisible(rendering::SelectionObj::ROT));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::SCALE));

  // Scale handles visible
  obj->SetMode(rendering::SelectionObj::SCALE);
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::TRANS));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::ROT));
  EXPECT_TRUE(obj->GetHandleVisible(rendering::SelectionObj::SCALE));

  // Set child handles
  obj->SetHandleVisible(rendering::SelectionObj::TRANS_X, true);
  EXPECT_TRUE(obj->GetHandleVisible(rendering::SelectionObj::TRANS_X));

  obj->SetHandleVisible(rendering::SelectionObj::ROT_Y, false);
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::ROT_Y));

  obj->SetHandleVisible(rendering::SelectionObj::SCALE_Z, true);
  EXPECT_TRUE(obj->GetHandleVisible(rendering::SelectionObj::SCALE_Z));
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
