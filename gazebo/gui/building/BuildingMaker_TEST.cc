/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#include "gazebo/gui/qt.h"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/building/BuildingEditorEvents.hh"
#include "gazebo/gui/building/BuildingMaker.hh"
#include "gazebo/gui/building/BuildingMaker_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void BuildingMaker_TEST::Attach()
{
  // Create a building maker
  auto buildingMaker = new gazebo::gui::BuildingMaker();
  QVERIFY(buildingMaker != NULL);

  // Attach a window to a wall
  buildingMaker->AttachManip("window", "wall");
  QVERIFY(buildingMaker->IsAttached("window"));
  QVERIFY(!buildingMaker->IsAttached("wall"));
  QVERIFY(!buildingMaker->IsAttached("door"));

  // Attach a door to a wall
  buildingMaker->AttachManip("door", "wall");
  QVERIFY(buildingMaker->IsAttached("window"));
  QVERIFY(!buildingMaker->IsAttached("wall"));
  QVERIFY(buildingMaker->IsAttached("door"));

  // Detach the window from the wall
  buildingMaker->DetachFromParent("window");
  QVERIFY(!buildingMaker->IsAttached("window"));
  QVERIFY(!buildingMaker->IsAttached("wall"));
  QVERIFY(buildingMaker->IsAttached("door"));

  // Reattach window to the wall
  buildingMaker->AttachManip("window", "wall");
  QVERIFY(buildingMaker->IsAttached("window"));
  QVERIFY(!buildingMaker->IsAttached("wall"));
  QVERIFY(buildingMaker->IsAttached("door"));

  // Detach all of the wall's children
  buildingMaker->DetachAllChildren("wall");
  QVERIFY(!buildingMaker->IsAttached("window"));
  QVERIFY(!buildingMaker->IsAttached("wall"));
  QVERIFY(!buildingMaker->IsAttached("door"));

  delete buildingMaker;
}

/////////////////////////////////////////////////
void BuildingMaker_TEST::Layers()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  // Load an empty world
  this->Load("worlds/empty.world", false, false, false);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Create a building maker
  auto buildingMaker = new gazebo::gui::BuildingMaker();
  QVERIFY(buildingMaker != NULL);

  // Add a wall on each level
  int levelCount = 3;
  for (int i = 0; i < levelCount; ++i)
  {
    // Change level
    gazebo::gui::editor::Events::changeBuildingLevel(i);

    auto wall = buildingMaker->AddWall(QVector3D(1, 1, 1),
        QVector3D(0, 0, i+0.5), 0);
    QVERIFY(wall == "Wall_" + std::to_string(i));
    gzdbg << "Created [" << wall << "]" << std::endl;
  }

  // Generate SDF
  buildingMaker->GenerateSDF();
  auto sdfStr = buildingMaker->ModelSDF();

  sdf::SDF sdf;
  sdf.SetFromString(sdfStr);

  QVERIFY(sdf.Root() != NULL);
  QVERIFY(sdf.Root()->HasElement("model"));
  QVERIFY(sdf.Root()->GetElement("model")->HasElement("link"));

  // Check each level
  int count = 0;
  auto link = sdf.Root()->GetElement("model")->GetElement("link");
  while (link)
  {
    QVERIFY(link != NULL);
    QVERIFY(link->HasAttribute("name"));
    QVERIFY(link->Get<std::string>("name") == "Wall_" + std::to_string(count));
    QVERIFY(link->HasElement("visual"));
    QVERIFY(link->GetElement("visual")->HasElement("meta"));
    QVERIFY(link->GetElement("visual")->GetElement("meta")
        ->HasElement("layer"));
    QCOMPARE(link->GetElement("visual")->GetElement("meta")->GetElement("layer")
        ->Get<int>(), count);

    gzdbg << "Checked wall [Wall_" << count << "]" << std::endl;

    link = link->GetNextElement("link");
    count++;
  }
  QVERIFY(link == NULL);
  QCOMPARE(count, levelCount);

  delete buildingMaker;
}

// Generate a main function for the test
QTEST_MAIN(BuildingMaker_TEST)
