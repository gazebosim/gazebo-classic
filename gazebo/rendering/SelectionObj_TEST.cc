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
#include "gazebo/test/ServerFixture.hh"

using namespace gazebo;
class SelectionObj_TEST : public RenderingFixture
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
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::TRANS_X));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::TRANS_Y));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::TRANS_Z));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::ROT));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::ROT_X));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::ROT_Y));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::ROT_Z));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::SCALE));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::SCALE_X));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::SCALE_Y));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::SCALE_Z));

  // Translate group visible
  obj->SetMode(rendering::SelectionObj::TRANS);
  EXPECT_TRUE(obj->GetHandleVisible(rendering::SelectionObj::TRANS));
  EXPECT_TRUE(obj->GetHandleVisible(rendering::SelectionObj::TRANS_X));
  EXPECT_TRUE(obj->GetHandleVisible(rendering::SelectionObj::TRANS_Y));
  EXPECT_TRUE(obj->GetHandleVisible(rendering::SelectionObj::TRANS_Z));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::ROT));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::ROT_X));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::ROT_Y));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::ROT_Z));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::SCALE));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::SCALE_X));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::SCALE_Y));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::SCALE_Z));

  // Rotate group visible
  obj->SetMode(rendering::SelectionObj::ROT);
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::TRANS));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::TRANS_X));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::TRANS_Y));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::TRANS_Z));
  EXPECT_TRUE(obj->GetHandleVisible(rendering::SelectionObj::ROT));
  EXPECT_TRUE(obj->GetHandleVisible(rendering::SelectionObj::ROT_X));
  EXPECT_TRUE(obj->GetHandleVisible(rendering::SelectionObj::ROT_Y));
  EXPECT_TRUE(obj->GetHandleVisible(rendering::SelectionObj::ROT_Z));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::SCALE));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::SCALE_X));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::SCALE_Y));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::SCALE_Z));

  // Scale group visible
  obj->SetMode(rendering::SelectionObj::SCALE);
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::TRANS));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::TRANS_X));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::TRANS_Y));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::TRANS_Z));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::ROT));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::ROT_X));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::ROT_Y));
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::ROT_Z));
  EXPECT_TRUE(obj->GetHandleVisible(rendering::SelectionObj::SCALE));
  EXPECT_TRUE(obj->GetHandleVisible(rendering::SelectionObj::SCALE_X));
  EXPECT_TRUE(obj->GetHandleVisible(rendering::SelectionObj::SCALE_Y));
  EXPECT_TRUE(obj->GetHandleVisible(rendering::SelectionObj::SCALE_Z));

  // Set child handles
  obj->SetHandleVisible(rendering::SelectionObj::TRANS_X, true);
  EXPECT_TRUE(obj->GetHandleVisible(rendering::SelectionObj::TRANS_X));

  obj->SetHandleVisible(rendering::SelectionObj::ROT_Y, false);
  EXPECT_FALSE(obj->GetHandleVisible(rendering::SelectionObj::ROT_Y));

  obj->SetHandleVisible(rendering::SelectionObj::SCALE_Z, true);
  EXPECT_TRUE(obj->GetHandleVisible(rendering::SelectionObj::SCALE_Z));
}

/////////////////////////////////////////////////
TEST_F(SelectionObj_TEST, LoadFini)
{
  Load("worlds/empty.world");

  gazebo::rendering::ScenePtr scene = gazebo::rendering::get_scene("default");

  if (!scene)
    scene = gazebo::rendering::create_scene("default", false);

  EXPECT_TRUE(scene != NULL);

  // Create and load visual
  rendering::SelectionObjPtr obj;
  obj.reset(new rendering::SelectionObj("obj", scene->GetWorldVisual()));
  obj->Load();
  EXPECT_TRUE(obj != NULL);

  // Check that it was added to the scene (by Load)
  EXPECT_EQ(scene->GetVisual("obj"), obj);

  // Remove it from the scene (Fini is called)
  scene->RemoveVisual(obj);

  // Check that it was removed
  EXPECT_TRUE(scene->GetVisual("obj") == NULL);

  // Reset pointer
  obj.reset();
  EXPECT_TRUE(obj == NULL);

  // Create another visual with the same name
  rendering::SelectionObjPtr obj2;
  obj2.reset(new rendering::SelectionObj("obj", scene->GetWorldVisual()));
  obj2->Load();
  EXPECT_TRUE(obj2 != NULL);

  // Check that the scene returns the new visual
  EXPECT_EQ(scene->GetVisual("obj"), obj2);

  // Remove it from the scene (Fini is called)
  scene->RemoveVisual(obj2);

  // Check that it was removed
  EXPECT_TRUE(scene->GetVisual("obj") == NULL);

  // Reset pointer
  obj2.reset();
  EXPECT_TRUE(obj2 == NULL);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
