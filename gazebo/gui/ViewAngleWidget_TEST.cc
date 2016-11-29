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

#include <ignition/math/Pose3.hh>

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/GuiIface.hh"

#include "gazebo/gui/ViewAngleWidget.hh"
#include "gazebo/gui/ViewAngleWidget_TEST.hh"

/////////////////////////////////////////////////
void ViewAngleWidget_TEST::EmptyWorld()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, false);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get camera
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);

  // Get camera default pose and check that's the current pose
  ignition::math::Pose3d defaultPose = cam->DefaultPose().Ign();
  QVERIFY(defaultPose == cam->WorldPose());

  // Get the camera distance from the world origin (zoom level)
  double dist = defaultPose.Pos().Length();

  // Get the view angle widget
  gazebo::gui::ViewAngleWidget *viewAngleWidget =
      mainWindow->findChild<gazebo::gui::ViewAngleWidget *>("viewAngleWidget");
  QVERIFY(viewAngleWidget != NULL);

  QApplication::postEvent(viewAngleWidget, new QShowEvent());

  this->ProcessEventsAndDraw(mainWindow);

  // Get buttons
  QList<QToolButton *> buttons =
      viewAngleWidget->findChildren<QToolButton *>();
  QVERIFY(buttons.size() == 7u);

  // Trigger the top view button
  buttons[0]->click();

  this->ProcessEventsAndDraw(mainWindow, 50);

  // Check the camera position
  double tol = 1e-6;
  ignition::math::Pose3d pose = cam->WorldPose();
  QVERIFY((pose.Pos() - ignition::math::Vector3d(0, 0, dist)).Length() < tol);

  // Trigger the front view button
  buttons[2]->click();

  this->ProcessEventsAndDraw(mainWindow, 50);

  // Check the camera position
  pose = cam->WorldPose();
  QVERIFY((pose.Pos() - ignition::math::Vector3d(dist, 0, 0)).Length() < tol);

  // Trigger the left view button
  buttons[5]->click();

  this->ProcessEventsAndDraw(mainWindow, 50);

  // Check the camera position
  pose = cam->WorldPose();
  QVERIFY((pose.Pos() - ignition::math::Vector3d(0, dist, 0)).Length() < tol);

  // Trigger the bottom view button
  buttons[1]->click();

  this->ProcessEventsAndDraw(mainWindow, 50);

  // Check the camera position
  pose = cam->WorldPose();
  QVERIFY((pose.Pos() - ignition::math::Vector3d(0, 0, -dist)).Length() < tol);

  // Trigger the back view button
  buttons[3]->click();

  this->ProcessEventsAndDraw(mainWindow, 50);

  // Check the camera position
  pose = cam->WorldPose();
  QVERIFY((pose.Pos() - ignition::math::Vector3d(-dist, 0, 0)).Length() < tol);

  // Trigger the right view button
  buttons[4]->click();

  this->ProcessEventsAndDraw(mainWindow, 50);

  // Check the camera position
  pose = cam->WorldPose();
  QVERIFY((pose.Pos() - ignition::math::Vector3d(0, -dist, 0)).Length() < tol);

  // Trigger the reset view button
  buttons[6]->click();

  this->ProcessEventsAndDraw(mainWindow, 50);

  // Check the camera position
  pose = cam->WorldPose();
  QVERIFY((pose.Pos() - defaultPose.Pos()).Length() < tol);

  // Clean up
  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void ViewAngleWidget_TEST::Projections()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, false);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get camera
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);

  // Get the view angle widget
  gazebo::gui::ViewAngleWidget *viewAngleWidget =
      mainWindow->findChild<gazebo::gui::ViewAngleWidget *>("viewAngleWidget");
  QVERIFY(viewAngleWidget != NULL);

  // Get the combobox
  QList<QComboBox *> comboBoxes =
      viewAngleWidget->findChildren<QComboBox *>();
  QVERIFY(comboBoxes.size() == 1u);

  // Check that it is in perspective projection
  QVERIFY(gazebo::gui::g_cameraPerspectiveAct->isChecked());
  QVERIFY(!gazebo::gui::g_cameraOrthoAct->isChecked());
  QVERIFY(comboBoxes[0]->currentText() == "Perspective");

  // Trigger ortho and see it changed
  gazebo::gui::g_cameraOrthoAct->trigger();

  this->ProcessEventsAndDraw(mainWindow);

  // Check that it is in orthographic projection
  QVERIFY(!gazebo::gui::g_cameraPerspectiveAct->isChecked());
  QVERIFY(gazebo::gui::g_cameraOrthoAct->isChecked());
  QVERIFY(comboBoxes[0]->currentText() == "Orthographic");

  // Trigger perpective and see it changed
  gazebo::gui::g_cameraPerspectiveAct->trigger();

  this->ProcessEventsAndDraw(mainWindow);

  // Check that it is in perspective projection
  QVERIFY(gazebo::gui::g_cameraPerspectiveAct->isChecked());
  QVERIFY(!gazebo::gui::g_cameraOrthoAct->isChecked());
  QVERIFY(comboBoxes[0]->currentText() == "Perspective");

  // Change combobox
  comboBoxes[0]->setCurrentIndex(1);

  this->ProcessEventsAndDraw(mainWindow);

  // Check that it is in orthographic projection
  QVERIFY(!gazebo::gui::g_cameraPerspectiveAct->isChecked());
  QVERIFY(gazebo::gui::g_cameraOrthoAct->isChecked());
  QVERIFY(comboBoxes[0]->currentText() == "Orthographic");

  // Change combobox
  comboBoxes[0]->setCurrentIndex(0);

  this->ProcessEventsAndDraw(mainWindow);

  // Check that it is in perspective projection
  QVERIFY(gazebo::gui::g_cameraPerspectiveAct->isChecked());
  QVERIFY(!gazebo::gui::g_cameraOrthoAct->isChecked());
  QVERIFY(comboBoxes[0]->currentText() == "Perspective");

  // Clean up
  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(ViewAngleWidget_TEST)
