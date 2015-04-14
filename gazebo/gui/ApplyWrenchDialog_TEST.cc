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

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"

#include "gazebo/gui/ApplyWrenchDialog.hh"
#include "gazebo/gui/ApplyWrenchDialog_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void ApplyWrenchDialog_TEST::ApplyForceTorqueFromDialog()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != NULL);

  // Get the box
  gazebo::rendering::VisualPtr boxModelVis = scene->GetVisual("box");
  QVERIFY(boxModelVis != NULL);
  gazebo::rendering::VisualPtr boxLinkVis = scene->GetVisual("box::link");
  QVERIFY(boxLinkVis != NULL);
  gazebo::math::Pose boxModelPose = boxModelVis->GetWorldPose();

  // Initialize dialog
  gazebo::gui::ApplyWrenchDialog *applyWrenchDialog =
      new gazebo::gui::ApplyWrenchDialog;
  applyWrenchDialog->Init("box", "box::link");

  // Get spins
  QList<QDoubleSpinBox *> spins =
      applyWrenchDialog->findChildren<QDoubleSpinBox *>();
  QVERIFY(spins.size() == 11u);

  // Get buttons
  QList<QPushButton *> buttons =
      applyWrenchDialog->findChildren<QPushButton *>();
  QVERIFY(buttons.size() == 6u);

  QPushButton *applyForceButton = NULL;
  QPushButton *applyTorqueButton = NULL;
  QPushButton *applyAllButton = NULL;
  QPushButton *cancelButton = NULL;
  for (auto it : buttons)
  {
    QVERIFY(it);
    if (it->text().toLower().toStdString() == "apply force")
      applyForceButton = it;
    else if (it->text().toLower().toStdString() == "apply torque")
      applyTorqueButton = it;
    else if (it->text().toLower().toStdString() == "apply all")
      applyAllButton = it;
    else if (it->text().toLower().toStdString() == "cancel")
      cancelButton = it;
  }
  QVERIFY(applyForceButton);
  QVERIFY(applyTorqueButton);
  QVERIFY(applyAllButton);

  // Set and apply force on X axis
  spins[0]->setValue(1000);
  applyForceButton->click();

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check that box moved
  QVERIFY(boxModelPose != boxModelVis->GetWorldPose());

  // Save current pose
  boxModelPose = boxModelVis->GetWorldPose();
  QVERIFY(boxModelPose == boxModelVis->GetWorldPose());

  // Set and apply torque about -Z axis
  spins[9]->setValue(-1000);
  applyTorqueButton->click();

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check that box moved
  QVERIFY(boxModelPose != boxModelVis->GetWorldPose());

  // Save current pose
  boxModelPose = boxModelVis->GetWorldPose();
  QVERIFY(boxModelPose == boxModelVis->GetWorldPose());

  // Apply force and torque
  applyAllButton->click();

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check that box moved
  QVERIFY(boxModelPose.pos != boxModelVis->GetWorldPose().pos);
  QVERIFY(boxModelPose.rot != boxModelVis->GetWorldPose().rot);

  // Close dialog
  cancelButton->click();

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  delete applyWrenchDialog;

  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(ApplyWrenchDialog_TEST)
