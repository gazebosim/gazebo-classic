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

// Generate a main function for the test
QTEST_MAIN(BuildingMaker_TEST)
