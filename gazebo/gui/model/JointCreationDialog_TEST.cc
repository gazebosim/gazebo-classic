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
#include "gazebo/gui/model/ModelEditorEvents.hh"
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

/////////////////////////////////////////////////
void JointCreationDialog_TEST::Links()
{
  // Create a joint maker
  auto jointMaker = new gazebo::gui::JointMaker();
  QVERIFY(jointMaker != NULL);

  // Add links to list
  std::vector<std::string> scopedLinkNames =
      {"model::link1", "model::link2", "model::link3"};
  std::vector<std::string> linkNames;
  for (auto scopedName : scopedLinkNames)
  {
    gazebo::gui::model::Events::linkInserted(scopedName);

    linkNames.push_back(scopedName.substr(scopedName.find("::")+2));
  }

  // Create a dialog
  auto jointCreationDialog = new gazebo::gui::JointCreationDialog(jointMaker);
  QVERIFY(jointCreationDialog != NULL);

  // Open it
  jointCreationDialog->Open(gazebo::gui::JointMaker::JOINT_HINGE);
  QVERIFY(jointCreationDialog->isVisible());

  // Get the config widget
  auto configWidget =
      jointCreationDialog->findChild<gazebo::gui::ConfigWidget *>();
  QVERIFY(configWidget != NULL);

  // Get the parent and child widgets
  auto parentWidget = configWidget->ConfigChildWidgetByName("parentCombo");
  QVERIFY(parentWidget != NULL);
  auto childWidget = configWidget->ConfigChildWidgetByName("childCombo");
  QVERIFY(childWidget != NULL);

  // Get parent and child combo boxes
  auto parentCombo = parentWidget->findChild<QComboBox *>();
  QVERIFY(parentCombo != NULL);
  auto childCombo = childWidget->findChild<QComboBox *>();
  QVERIFY(childCombo != NULL);

  // Check that each combo box has an empty option plus 3 link options
  QVERIFY(parentCombo->count() == 4u);
  QVERIFY(childCombo->count() == 4u);

  for (int i = 1; i < parentCombo->count(); ++i)
  {
    QVERIFY(parentCombo->itemText(i).toStdString() == linkNames[i-1]);
    QVERIFY(childCombo->itemText(i).toStdString() == linkNames[i-1]);
  }

  // Check there are no links selected yet
  QVERIFY(configWidget->GetEnumWidgetValue("parentCombo") == "");
  QVERIFY(configWidget->GetEnumWidgetValue("childCombo") == "");

  // Check only parent is enabled
  QVERIFY(!configWidget->GetWidgetReadOnly("parentCombo"));
  QVERIFY(configWidget->GetWidgetReadOnly("childCombo"));
  QVERIFY(configWidget->GetWidgetReadOnly("axis1"));
  QVERIFY(configWidget->GetWidgetReadOnly("axis2"));
  QVERIFY(configWidget->GetWidgetReadOnly("align"));
  QVERIFY(configWidget->GetWidgetReadOnly("joint_pose"));
  QVERIFY(configWidget->GetWidgetReadOnly("relative_pose_general"));

  // Get push buttons (reset, cancel, create)
  auto pushButtons = jointCreationDialog->findChildren<QPushButton *>();
  QCOMPARE(pushButtons.size(), 3);

  // Check that create and reset buttons are disabled
  for (auto button : pushButtons)
  {
    if (button->text() == "Create" ||
        (button->text().toStdString()).find("Reset") != std::string::npos)
    {
      QVERIFY(!button->isEnabled());
    }
    else
    {
      QVERIFY(button->isEnabled());
    }
  }

  // Set parent from 3D scene
  jointCreationDialog->SetParent("model::link1");

  // Check that the parent link was selected
  QVERIFY(configWidget->GetEnumWidgetValue("parentCombo") == "link1");
  QVERIFY(configWidget->GetEnumWidgetValue("childCombo") == "");

  // Check that now child is also enabled
  QVERIFY(!configWidget->GetWidgetReadOnly("parentCombo"));
  QVERIFY(!configWidget->GetWidgetReadOnly("childCombo"));
  QVERIFY(configWidget->GetWidgetReadOnly("axis1"));
  QVERIFY(configWidget->GetWidgetReadOnly("axis2"));
  QVERIFY(configWidget->GetWidgetReadOnly("align"));
  QVERIFY(configWidget->GetWidgetReadOnly("joint_pose"));
  QVERIFY(configWidget->GetWidgetReadOnly("relative_pose_general"));

  // Check that create and reset buttons are disabled
  for (auto button : pushButtons)
  {
    if (button->text() == "Create" ||
        (button->text().toStdString()).find("Reset") != std::string::npos)
    {
      QVERIFY(!button->isEnabled());
    }
    else
    {
      QVERIFY(button->isEnabled());
    }
  }

  // Set child from 3D scene
  jointCreationDialog->SetChild("model::link2");

  // Check that the child link was selected
  QVERIFY(configWidget->GetEnumWidgetValue("parentCombo") == "link1");
  QVERIFY(configWidget->GetEnumWidgetValue("childCombo") == "link2");

  // Check that now all widgets are enabled
  QVERIFY(!configWidget->GetWidgetReadOnly("parentCombo"));
  QVERIFY(!configWidget->GetWidgetReadOnly("childCombo"));
  QVERIFY(!configWidget->GetWidgetReadOnly("axis1"));
  QVERIFY(!configWidget->GetWidgetReadOnly("axis2"));
  QVERIFY(!configWidget->GetWidgetReadOnly("align"));
  QVERIFY(!configWidget->GetWidgetReadOnly("joint_pose"));
  QVERIFY(!configWidget->GetWidgetReadOnly("relative_pose_general"));

  for (auto button : pushButtons)
    QVERIFY(button->isEnabled());

  // Get swap button
  auto swapButton =
      jointCreationDialog->findChild<QToolButton *>("JointCreationSwapButton");
  QVERIFY(swapButton != NULL);

  // Trigger swap
  swapButton->click();

  // Check that the parent link was selected
  QVERIFY(configWidget->GetEnumWidgetValue("parentCombo") == "link2");
  QVERIFY(configWidget->GetEnumWidgetValue("childCombo") == "link1");

  // Set child from dialog, same as parent
  childCombo->setCurrentIndex(1);

  // Check that the child link was selected
  QVERIFY(configWidget->GetEnumWidgetValue("parentCombo") == "link2");
  QVERIFY(configWidget->GetEnumWidgetValue("childCombo") == "link2");

  // Check that create button is disabled
  for (auto button : pushButtons)
  {
    if (button->text() == "Create")
    {
      QVERIFY(!button->isEnabled());
    }
    else
    {
      QVERIFY(button->isEnabled());
    }
  }

  delete jointCreationDialog;
  delete jointMaker;
}

// Generate a main function for the test
QTEST_MAIN(JointCreationDialog_TEST)
