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

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/UserCmdHistory.hh"
#include "visual_pose.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void VisualPoseTest::VisualPose()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("test/worlds/visual_pose.world", true, false, true);

  // Get world
  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  QVERIFY(world != NULL);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Get scene
  auto scene = gazebo::gui::get_active_camera()->GetScene();
  QVERIFY(scene != NULL);

  // Get box model visual (rendering)
  auto boxModelVis = scene->GetVisual("box");
  QVERIFY(boxModelVis != NULL);
  auto boxModelVisPose = boxModelVis->GetPose().Ign();

  // Get box link visual (rendering)
  auto boxLinkVis = scene->GetVisual("box::link");
  QVERIFY(boxLinkVis != NULL);
  auto boxLinkVisPose = boxLinkVis->GetPose().Ign();

  // Get box visual visual (rendering)
  auto boxVisualVis = scene->GetVisual("box::link::visual");
  QVERIFY(boxVisualVis != NULL);
  auto boxVisualVisPose = boxVisualVis->GetPose().Ign();

  // Get box model (physics)
  auto boxModel = world->GetModel("box");
  QVERIFY(boxModel != NULL);
  QVERIFY(boxModel->GetWorldPose().Ign() == boxModelVisPose);

  // Get box link (physics)
  auto boxLink = boxModel->GetLink("link");
  QVERIFY(boxLink != NULL);
  QVERIFY(boxLink->GetRelativePose().Ign() == boxLinkVisPose);

  // Get box visual id (physics)
  uint32_t boxVisualId;
  QVERIFY(boxLink->VisualId("visual", boxVisualId));
  QVERIFY(boxVisualId != 0u);

  // Get box visual pose (physics)
  ignition::math::Pose3d boxVisualPose;
  QVERIFY(boxLink->VisualPose(boxVisualId, boxVisualPose));
  QVERIFY(boxVisualPose == ignition::math::Pose3d(0, 2, 0, 0, 0, 0));

  // Set box visual pose (physics)
  ignition::math::Pose3d newVisualPose(1, -2, 3, -0.1, 0.2, -0.3);
  QVERIFY(boxLink->SetVisualPose(boxVisualId, newVisualPose));

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check that only visual pose changed (physics)
  QVERIFY(boxLink->VisualPose(boxVisualId, boxVisualPose));
  QVERIFY(boxVisualPose == newVisualPose);
  QVERIFY(boxLinkVisPose == boxLink->GetRelativePose().Ign());
  QVERIFY(boxModelVisPose == boxModel->GetRelativePose().Ign());

  // Check that only visual pose changed (rendering)
  QVERIFY(newVisualPose == boxVisualVis->GetPose().Ign());
  QVERIFY(boxModelVisPose == boxModelVis->GetPose().Ign());
  QVERIFY(boxLinkVisPose == boxLinkVis->GetPose().Ign());

  // Clean up
  delete mainWindow;
  mainWindow = NULL;
}

// Generate a main function for the test
QTEST_MAIN(VisualPoseTest)
