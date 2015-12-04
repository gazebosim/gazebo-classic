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
  jointCreationDialog->SetParent(scopedLinkNames[0]);

  // Check that the parent link was selected
  QVERIFY(configWidget->GetEnumWidgetValue("parentCombo") == linkNames[0]);
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
  jointCreationDialog->SetChild(scopedLinkNames[1]);

  // Check that the child link was selected
  QVERIFY(configWidget->GetEnumWidgetValue("parentCombo") == linkNames[0]);
  QVERIFY(configWidget->GetEnumWidgetValue("childCombo") == linkNames[1]);

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
  QVERIFY(configWidget->GetEnumWidgetValue("parentCombo") == linkNames[1]);
  QVERIFY(configWidget->GetEnumWidgetValue("childCombo") == linkNames[0]);

  // Set child from dialog, same as parent
  childCombo->setCurrentIndex(1);

  // Check that the child link was selected
  QVERIFY(configWidget->GetEnumWidgetValue("parentCombo") == linkNames[1]);
  QVERIFY(configWidget->GetEnumWidgetValue("childCombo") == linkNames[1]);

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

/////////////////////////////////////////////////
void JointCreationDialog_TEST::Axis()
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
  jointCreationDialog->Open(gazebo::gui::JointMaker::JOINT_HINGE2);
  QVERIFY(jointCreationDialog->isVisible());

  // Get push buttons (reset, cancel, create)
  auto pushButtons = jointCreationDialog->findChildren<QPushButton *>();
  QCOMPARE(pushButtons.size(), 3);

  // Get the config widget
  auto configWidget =
      jointCreationDialog->findChild<gazebo::gui::ConfigWidget *>();
  QVERIFY(configWidget != NULL);

  // Set child and parent from 3D scene
  jointCreationDialog->SetParent(scopedLinkNames[0]);
  jointCreationDialog->SetChild(scopedLinkNames[1]);

  // Check that all widgets are enabled
  QVERIFY(!configWidget->GetWidgetReadOnly("parentCombo"));
  QVERIFY(!configWidget->GetWidgetReadOnly("childCombo"));
  QVERIFY(!configWidget->GetWidgetReadOnly("axis1"));
  QVERIFY(!configWidget->GetWidgetReadOnly("axis2"));
  QVERIFY(!configWidget->GetWidgetReadOnly("align"));
  QVERIFY(!configWidget->GetWidgetReadOnly("joint_pose"));
  QVERIFY(!configWidget->GetWidgetReadOnly("relative_pose_general"));

  for (auto button : pushButtons)
    QVERIFY(button->isEnabled());

  // Check that both joint axis widgets are visible
  QVERIFY(configWidget->GetWidgetVisible("axis1"));
  QVERIFY(configWidget->GetWidgetVisible("axis2"));

  // Check default values
  QVERIFY(configWidget->GetVector3WidgetValue("axis1") ==
      ignition::math::Vector3d::UnitX);
  QVERIFY(configWidget->GetVector3WidgetValue("axis2") ==
      ignition::math::Vector3d::UnitY);

  // Set an axis to be zero
  auto axis1Widget = configWidget->ConfigChildWidgetByName("axis1");
  QVERIFY(axis1Widget != NULL);

  auto axis1Spins = axis1Widget->findChildren<QDoubleSpinBox *>();
  QVERIFY(axis1Spins.size() == 3u);

  axis1Spins[0]->setValue(0.0);
  QTest::keyClick(axis1Spins[1], Qt::Key_Enter);

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

  // Set it back to a valid value
  axis1Spins[2]->setValue(1.0);
  QTest::keyClick(axis1Spins[1], Qt::Key_Enter);

  // Check that all buttons are enabled again
  for (auto button : pushButtons)
    QVERIFY(button->isEnabled());

  delete jointCreationDialog;
  delete jointMaker;
}

/////////////////////////////////////////////////
void JointCreationDialog_TEST::Align()
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
  jointCreationDialog->Open(gazebo::gui::JointMaker::JOINT_HINGE2);
  QVERIFY(jointCreationDialog->isVisible());

  // Get push buttons (reset, cancel, create)
  auto pushButtons = jointCreationDialog->findChildren<QPushButton *>();
  QCOMPARE(pushButtons.size(), 3);

  // Get the config widget
  auto configWidget =
      jointCreationDialog->findChild<gazebo::gui::ConfigWidget *>();
  QVERIFY(configWidget != NULL);

  // Set child and parent from 3D scene
  jointCreationDialog->SetParent(scopedLinkNames[0]);
  jointCreationDialog->SetChild(scopedLinkNames[1]);

  // Check that all widgets are enabled
  QVERIFY(!configWidget->GetWidgetReadOnly("parentCombo"));
  QVERIFY(!configWidget->GetWidgetReadOnly("childCombo"));
  QVERIFY(!configWidget->GetWidgetReadOnly("axis1"));
  QVERIFY(!configWidget->GetWidgetReadOnly("axis2"));
  QVERIFY(!configWidget->GetWidgetReadOnly("align"));
  QVERIFY(!configWidget->GetWidgetReadOnly("joint_pose"));
  QVERIFY(!configWidget->GetWidgetReadOnly("relative_pose_general"));

  for (auto button : pushButtons)
    QVERIFY(button->isEnabled());

  // Get align widget and check it has all the buttons
  auto alignWidget = configWidget->ConfigChildWidgetByName("align");
  QVERIFY(alignWidget != NULL);

  auto alignButtons = alignWidget->findChildren<QToolButton *>();
  QVERIFY(alignButtons.size() == 9u);

  // Check that only one button per axis can be checked at a time
  alignButtons[0]->click();
  QVERIFY(alignButtons[0]->isChecked());
  QVERIFY(!alignButtons[1]->isChecked());
  QVERIFY(!alignButtons[2]->isChecked());

  alignButtons[1]->click();
  QVERIFY(!alignButtons[0]->isChecked());
  QVERIFY(alignButtons[1]->isChecked());
  QVERIFY(!alignButtons[2]->isChecked());

  // Check that checked button is toggled on click
  alignButtons[1]->click();
  QVERIFY(!alignButtons[0]->isChecked());
  QVERIFY(!alignButtons[1]->isChecked());
  QVERIFY(!alignButtons[2]->isChecked());

  // Check a button per axis and check they are checked at the same time
  alignButtons[0]->click();
  alignButtons[3]->click();
  alignButtons[6]->click();
  QVERIFY(alignButtons[0]->isChecked());
  QVERIFY(alignButtons[3]->isChecked());
  QVERIFY(alignButtons[6]->isChecked());

  // Check that all buttons are disabled when the link is changed
  auto parentWidget = configWidget->ConfigChildWidgetByName("parentCombo");
  QVERIFY(parentWidget != NULL);
  auto parentCombo = parentWidget->findChild<QComboBox *>();
  QVERIFY(parentCombo != NULL);
  parentCombo->setCurrentIndex(2);

  for (auto button : alignButtons)
    QVERIFY(!button->isChecked());
}

/////////////////////////////////////////////////
void JointCreationDialog_TEST::RelativePose()
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
  jointCreationDialog->Open(gazebo::gui::JointMaker::JOINT_HINGE2);
  QVERIFY(jointCreationDialog->isVisible());

  // Get push buttons (reset, cancel, create)
  auto pushButtons = jointCreationDialog->findChildren<QPushButton *>();
  QCOMPARE(pushButtons.size(), 3);

  // Get the config widget
  auto configWidget =
      jointCreationDialog->findChild<gazebo::gui::ConfigWidget *>();
  QVERIFY(configWidget != NULL);

  // Check the default value
  QVERIFY(configWidget->GetPoseWidgetValue("relative_pose") ==
      ignition::math::Pose3d::Zero);

  // Set child and parent from 3D scene
  jointCreationDialog->SetParent(scopedLinkNames[0]);
  jointCreationDialog->SetChild(scopedLinkNames[1]);

  // Check that all widgets are enabled
  QVERIFY(!configWidget->GetWidgetReadOnly("parentCombo"));
  QVERIFY(!configWidget->GetWidgetReadOnly("childCombo"));
  QVERIFY(!configWidget->GetWidgetReadOnly("axis1"));
  QVERIFY(!configWidget->GetWidgetReadOnly("axis2"));
  QVERIFY(!configWidget->GetWidgetReadOnly("align"));
  QVERIFY(!configWidget->GetWidgetReadOnly("joint_pose"));
  QVERIFY(!configWidget->GetWidgetReadOnly("relative_pose_general"));

  for (auto button : pushButtons)
    QVERIFY(button->isEnabled());

  // Update the relative pose from 3D
  ignition::math::Pose3d pose(1, -0.2, 3.3, 0.1, -0.2, 0);
  jointCreationDialog->UpdateRelativePose(pose);

  // Check the widget was updated
  QVERIFY(configWidget->GetPoseWidgetValue("relative_pose") == pose);

  // Get reset button
  auto resetButton =
      jointCreationDialog->findChild<QPushButton *>("JointCreationResetButton");
  QVERIFY(resetButton != NULL);

  // Trigger reset
  resetButton->click();
}

// Generate a main function for the test
QTEST_MAIN(JointCreationDialog_TEST)
