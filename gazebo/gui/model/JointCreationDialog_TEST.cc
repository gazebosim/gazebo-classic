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

#include "gazebo/gui/ConfigWidget.hh"
#include "gazebo/gui/model/JointCreationDialog.hh"
#include "gazebo/gui/model/JointCreationDialog_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void JointCreationDialog_TEST::Type()
{
  // Create a joint maker
  auto jointMaker = new gazebo::gui::JointMaker();
  QVERIFY(jointMaker != NULL);

  // Create a dialog
  auto jointCreationDialog = new gazebo::gui::JointCreationDialog(jointMaker);
  QVERIFY(jointCreationDialog != NULL);

  // Open it
  jointCreationDialog->Open(gazebo::gui::JointMaker::JOINT_HINGE);
  QVERIFY(jointCreationDialog->isVisible());

  // Check there are 8 radio buttons for joint types
  auto radioButtons = jointCreationDialog->findChildren<QRadioButton *>();
  QCOMPARE(radioButtons.size(), 8);

  // Get the config widget
  auto configWidget =
      jointCreationDialog->findChild<gazebo::gui::ConfigWidget *>();
  QVERIFY(configWidget != NULL);

  // Check that only the correct button is checked (Revolute - 1)
  for (int i = 0; i < radioButtons.size(); ++i)
  {
    if (i == 1)
      QVERIFY(radioButtons[i]->isChecked());
    else
      QVERIFY(!radioButtons[i]->isChecked());
  }

  // Check there's one joint axis widget
  QVERIFY(configWidget->GetWidgetVisible("axis1"));
  QVERIFY(!configWidget->GetWidgetVisible("axis2"));

  // Set type to ball joint
  radioButtons[6]->click();

  // Check that only the correct button is checked (Ball - 6)
  for (int i = 0; i < radioButtons.size(); ++i)
  {
    if (i == 6)
      QVERIFY(radioButtons[i]->isChecked());
    else
      QVERIFY(!radioButtons[i]->isChecked());
  }

  // Check there's no joint axis widget
  QVERIFY(!configWidget->GetWidgetVisible("axis1"));
  QVERIFY(!configWidget->GetWidgetVisible("axis2"));

  // Set type to Revolute 2 joint
  radioButtons[2]->click();

  // Check that only the correct button is checked (Revolute 2 - 2)
  for (int i = 0; i < radioButtons.size(); ++i)
  {
    if (i == 2)
      QVERIFY(radioButtons[i]->isChecked());
    else
      QVERIFY(!radioButtons[i]->isChecked());
  }

  // Check there are both joint axis widgets
  QVERIFY(configWidget->GetWidgetVisible("axis1"));
  QVERIFY(configWidget->GetWidgetVisible("axis2"));

  delete jointCreationDialog;
  delete jointMaker;
}

// Generate a main function for the test
QTEST_MAIN(JointCreationDialog_TEST)
