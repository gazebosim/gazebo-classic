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

#include "gazebo/gui/MainWindow.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/model/ModelEditorTypes.hh"
#include "gazebo/gui/model/EditorMaterialSwitcher.hh"
#include "gazebo/gui/model/EditorMaterialSwitcher_TEST.hh"

using namespace gazebo;

/////////////////////////////////////////////////
void EditorMaterialSwitcher_TEST::CreateMaterialSwitcher()
{
  // set NULL camera - should not work and the material scheme should be empty
  gui::EditorMaterialSwitcherPtr emptyMaterialSwitcher(
      new gui::EditorMaterialSwitcher(NULL));
  QCOMPARE(emptyMaterialSwitcher->MaterialScheme(), std::string(""));

  std::string scheme = "material_scheme";
  emptyMaterialSwitcher->SetMaterialScheme(scheme);
  QCOMPARE(emptyMaterialSwitcher->MaterialScheme(), std::string(""));


  // create a material swticher with a camera
  this->Load("test/worlds/box.world", false, false, true);
  rendering::ScenePtr scene =
      gazebo::rendering::get_scene(gazebo::physics::get_world()->GetName());

  rendering::CameraPtr camera = scene->CreateCamera("test_camera", false);
  camera->Load();
  camera->Init();
  camera->CreateRenderTexture("test_render_target");

  // construvtor
  gui::EditorMaterialSwitcherPtr materialSwitcher(
      new gui::EditorMaterialSwitcher(camera));

  QVERIFY(materialSwitcher != NULL);

  // verify scheme
  materialSwitcher->SetMaterialScheme(scheme);
  QCOMPARE(materialSwitcher->MaterialScheme(), scheme);

  scene->RemoveCamera(camera->GetName());
}

// Generate a main function for the test
QTEST_MAIN(EditorMaterialSwitcher_TEST)
