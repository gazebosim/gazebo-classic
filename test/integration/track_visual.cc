/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include "track_visual.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void TrackVisualTest::TrackVisual()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("test/worlds/track_visual.world", true, false, false);

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

  // Get user camera
  auto cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);

  // Get scene
  auto scene = cam->GetScene();
  QVERIFY(scene != NULL);

  // Get box model visual
  auto boxModelVis = scene->GetVisual("box");
  QVERIFY(boxModelVis != NULL);
  auto boxModelVisPose = boxModelVis->GetPose().Ign();

  // Get tracked model visual
  auto trackedModelVis = cam->TrackedVisual();
  QVERIFY(trackedModelVis == boxModelVis);

  // Get box model
  auto boxModel = world->GetModel("box");
  QVERIFY(boxModel != NULL);
  QVERIFY(boxModel->GetWorldPose().Ign() == boxModelVisPose);

  // Get box model pose
  auto boxModelPose = boxModel->GetWorldPose();
  QVERIFY(boxModelPose == ignition::math::Pose3d(0, 0, 0.5, 0, 0, 0));

  // Check that camera's position is static when tracking a model
  QVERIFY(true == cam->TrackIsStatic());

  // Check that camera's position is relative to tracked model
  QVERIFY(true == cam->TrackUseModelFrame());

  // Check that the camera inherits the yaw rotation of tracked model
  QVERIFY(true == cam->TrackInheritYaw());

  // Get camera's relative position
  auto camTrackPosition = cam->TrackPosition();
  QVERIFY(camTrackPosition == ignition::math::Vector3d(-1, -1, -1));

  // Get camera's pose
  auto camPose = cam->WorldPose();

  // Set box model pose
  ignition::math::Pose3d newVisualPose(1, -2, 0.5, -0.1, 0.2, -0.3);
  boxModel->SetWorldPose(newVisualPose);

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check that user camera's pose changed
  auto newCamPose = cam->WorldPose();
  QVERIFY(newCamPose != camPose);

  // Change camera tracking parameters
  cam->SetTrackIsStatic(false);
  QVERIFY(false == cam->TrackIsStatic());
  cam->SetTrackUseModelFrame(false);
  QVERIFY(false == cam->TrackUseModelFrame());
  cam->SetTrackInheritYaw(false);
  QVERIFY(false == cam->TrackInheritYaw());
  auto camPosition = ignition::math::Vector3d(0, 0, 0);
  cam->SetTrackPosition(camPosition);
  QVERIFY(camPosition == cam->TrackPosition());
  double minDistance = 1;
  cam->SetTrackMinDistance(minDistance);
  QCOMPARE(minDistance, cam->TrackMinDistance());
  double maxDistance = 10;
  cam->SetTrackMaxDistance(maxDistance);
  QCOMPARE(maxDistance, cam->TrackMaxDistance());

  // Stop following visual
  cam->TrackVisual("");

  // Set new box model pose
  newVisualPose = ignition::math::Pose3d(-1, 2, 0.5, 0.1, -0.2, 0.3);
  boxModel->SetWorldPose(newVisualPose);

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check that camera does not track any visual
  QVERIFY(!cam->TrackedVisual());

  // Check that user camera's pose has not changed
  QVERIFY(cam->WorldPose() == newCamPose);

  // Clean up
  delete mainWindow;
  mainWindow = NULL;
}

// Generate a main function for the test
QTEST_MAIN(TrackVisualTest)
