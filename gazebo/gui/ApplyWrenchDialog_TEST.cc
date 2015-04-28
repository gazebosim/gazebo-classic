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

  // World with one model which has 2 links, no ground plane and gravity is off
  this->Load("worlds/multilink_shape.world", false, false, false);

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

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != NULL);

  // Get the model
  gazebo::rendering::VisualPtr modelVis = scene->GetVisual("multilink");
  QVERIFY(modelVis != NULL);

  // Get the box link
  gazebo::rendering::VisualPtr boxLinkVis =
      scene->GetVisual("multilink::box_link");
  QVERIFY(boxLinkVis != NULL);
  gazebo::math::Pose boxLinkPose = boxLinkVis->GetWorldPose();
  QVERIFY(boxLinkPose == boxLinkVis->GetWorldPose());

  // Get the sphere link
  gazebo::rendering::VisualPtr sphereLinkVis =
      scene->GetVisual("multilink::sphere_link");
  QVERIFY(sphereLinkVis != NULL);
  gazebo::math::Pose sphereLinkPose = sphereLinkVis->GetWorldPose();
  QVERIFY(sphereLinkPose == sphereLinkVis->GetWorldPose());

  // Initialize dialog with the box link
  gazebo::gui::ApplyWrenchDialog *applyWrenchDialog =
      new gazebo::gui::ApplyWrenchDialog;
  applyWrenchDialog->Init("multilink", "multilink::box_link");

  // Get combo box
  QList<QComboBox *> comboBoxes =
      applyWrenchDialog->findChildren<QComboBox *>();
  QVERIFY(comboBoxes.size() == 1u);

  // Check the combo box's items
  QVERIFY(comboBoxes[0]->count() == 2u);
  QVERIFY(comboBoxes[0]->itemText(0) == "box_link");
  QVERIFY(comboBoxes[0]->itemText(1) == "sphere_link");
  QVERIFY(comboBoxes[0]->currentIndex() == 0u);

  // Get radio buttons
  QList<QRadioButton *> radioButtons =
      applyWrenchDialog->findChildren<QRadioButton *>();
  QVERIFY(radioButtons.size() == 2u);

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
  QPushButton *clearForceButton = NULL;
  QPushButton *clearTorqueButton = NULL;
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
    else if (it->text().toLower().toStdString() == "clear" && !clearForceButton)
      clearForceButton = it;
    else if (it->text().toLower().toStdString() == "clear")
      clearTorqueButton = it;
    else if (it->text().toLower().toStdString() == "cancel")
      cancelButton = it;
  }
  QVERIFY(applyForceButton);
  QVERIFY(applyTorqueButton);
  QVERIFY(applyAllButton);
  QVERIFY(clearForceButton);
  QVERIFY(clearTorqueButton);
  QVERIFY(cancelButton);

  // Set and apply force on X axis, magnitude 1000
  spins[0]->setValue(1.0);
  spins[3]->setValue(1000.0);
  applyForceButton->click();

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check that force spin was updated according to magnitude
  QCOMPARE(spins[0]->value(), 1000.0);

  // Check that link moved on X axis
  QVERIFY(boxLinkPose.pos.x < boxLinkVis->GetWorldPose().pos.x);
  QVERIFY(boxLinkPose.pos.y - boxLinkVis->GetWorldPose().pos.y < 1e-6);
  QVERIFY(boxLinkPose.pos.z - boxLinkVis->GetWorldPose().pos.z < 1e-6);
  QCOMPARE(boxLinkPose.rot, boxLinkVis->GetWorldPose().rot);

  // Save current pose
  boxLinkPose = boxLinkVis->GetWorldPose();
  QVERIFY(boxLinkPose == boxLinkVis->GetWorldPose());

  // Set and apply torque about -Z axis, magnitude 1000
  spins[9]->setValue(-1.0);
  spins[10]->setValue(1000.0);
  applyTorqueButton->click();

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check that torque spin was updated according to magnitude
  QCOMPARE(spins[9]->value(), -1000.0);

  // Check that link rotated
  QVERIFY(boxLinkPose.pos.x < boxLinkVis->GetWorldPose().pos.x);
  QVERIFY(boxLinkPose.pos.y - boxLinkVis->GetWorldPose().pos.y < 1e-6);
  QVERIFY(boxLinkPose.pos.z - boxLinkVis->GetWorldPose().pos.z < 1e-6);
  QVERIFY(boxLinkPose.rot != boxLinkVis->GetWorldPose().rot);

  // Save current pose
  boxLinkPose = boxLinkVis->GetWorldPose();
  QVERIFY(boxLinkPose == boxLinkVis->GetWorldPose());

  // Apply force and torque
  applyAllButton->click();

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check that link translated and rotated
  QVERIFY(boxLinkPose.pos != boxLinkVis->GetWorldPose().pos);
  QVERIFY(boxLinkPose.rot != boxLinkVis->GetWorldPose().rot);

  // Change link
  comboBoxes[0]->setCurrentIndex(1);
  QVERIFY(comboBoxes[0]->currentText() == "sphere_link");

  // Clear force
  clearForceButton->click();
  QCOMPARE(spins[0]->value(), 0.0);
  QCOMPARE(spins[1]->value(), 0.0);
  QCOMPARE(spins[2]->value(), 0.0);
  QCOMPARE(spins[3]->value(), 0.0);

  // Clear torque
  clearTorqueButton->click();
  QCOMPARE(spins[7]->value(), 0.0);
  QCOMPARE(spins[8]->value(), 0.0);
  QCOMPARE(spins[9]->value(), 0.0);
  QCOMPARE(spins[10]->value(), 0.0);

  // Apply zero force and torque
  applyAllButton->click();

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check that link didn't move
  QVERIFY(sphereLinkPose.pos == sphereLinkVis->GetWorldPose().pos);
  QVERIFY(sphereLinkPose.rot == sphereLinkVis->GetWorldPose().rot);

  // Set and apply force on Y axis with an offset on X
  spins[1]->setValue(1000.0);
  spins[4]->setValue(1.0);
  applyForceButton->click();

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check that link translated and rotated
  QVERIFY(sphereLinkPose.pos != sphereLinkVis->GetWorldPose().pos);
  QVERIFY(sphereLinkPose.rot != sphereLinkVis->GetWorldPose().rot);

  // Select CoM as application point
  radioButtons[0]->click();

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Check that force offset spins were updated
  QCOMPARE(spins[4]->value(), 0.0);
  QCOMPARE(spins[5]->value(), 0.0);
  QCOMPARE(spins[6]->value(), 0.0);

  // Close dialog
  cancelButton->click();

  // Process some events and draw the screen
  for (size_t i = 0; i < 10; ++i)
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
