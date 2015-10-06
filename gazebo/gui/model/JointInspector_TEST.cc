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

#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/JointMaker.hh"
#include "gazebo/gui/model/JointInspector.hh"
#include "gazebo/gui/model/JointInspector_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void JointInspector_TEST::AddRemoveLink()
{
  // Create a joint maker
  gazebo::gui::JointMaker *jointMaker = new gazebo::gui::JointMaker();
  QVERIFY(jointMaker != NULL);

  // Create a joint inspector
  gazebo::gui::JointInspector *jointInspector =
      new gazebo::gui::JointInspector(jointMaker);
  QVERIFY(jointInspector != NULL);

  // Get combo boxes
  QList<QComboBox *> comboBoxes = jointInspector->findChildren<QComboBox *>();
  unsigned int boxCount = comboBoxes.size();
  QVERIFY(boxCount >= 2);

  // Check parent and child combo boxes
  QComboBox *parentBox = comboBoxes[boxCount-2];
  QComboBox *childBox = comboBoxes[boxCount-1];
  QCOMPARE(parentBox->count(), 0);
  QCOMPARE(childBox->count(), 0);

  // Send link inserted event
  gazebo::gui::model::Events::linkInserted("model::link1");

  // Check parent and child combo boxes
  QCOMPARE(parentBox->count(), 1);
  QCOMPARE(childBox->count(), 1);
  QVERIFY(parentBox->itemText(0) == "link1");
  QVERIFY(childBox->itemText(0) == "link1");

  // Send link inserted event
  gazebo::gui::model::Events::linkInserted("model::link2");

  // Check parent and child combo boxes
  QCOMPARE(parentBox->count(), 2);
  QCOMPARE(childBox->count(), 2);
  QVERIFY(parentBox->itemText(1) == "link2");
  QVERIFY(childBox->itemText(1) == "link2");

  // Send link removed event
  gazebo::gui::model::Events::linkRemoved("model::link1");

  // Check parent and child combo boxes
  QCOMPARE(parentBox->count(), 1);
  QCOMPARE(childBox->count(), 1);
  QVERIFY(parentBox->itemText(0) == "link2");
  QVERIFY(childBox->itemText(0) == "link2");

  delete jointInspector;
  delete jointMaker;
}

/////////////////////////////////////////////////
void JointInspector_TEST::Swap()
{
  // Create a joint maker
  gazebo::gui::JointMaker *jointMaker = new gazebo::gui::JointMaker();
  QVERIFY(jointMaker != NULL);

  // Add links to list
  gazebo::gui::model::Events::linkInserted("model::link1");
  gazebo::gui::model::Events::linkInserted("model::link2");

  // Create a joint inspector
  gazebo::gui::JointInspector *jointInspector =
      new gazebo::gui::JointInspector(jointMaker);
  QVERIFY(jointInspector != NULL);

  // Open it so link boxes are updated with new links
  jointInspector->Open();

  // Get combo boxes
  QList<QComboBox *> comboBoxes = jointInspector->findChildren<QComboBox *>();
  unsigned int boxCount = comboBoxes.size();
  QVERIFY(boxCount >= 2);

  // Check parent and child combo boxes
  QComboBox *parentBox = comboBoxes[boxCount-2];
  QComboBox *childBox = comboBoxes[boxCount-1];
  QCOMPARE(parentBox->count(), 2);
  QCOMPARE(childBox->count(), 2);

  // Select parent and child links
  parentBox->setCurrentIndex(0);
  childBox->setCurrentIndex(1);
  QVERIFY(parentBox->currentText() == "link1");
  QVERIFY(childBox->currentText() == "link2");

  // Get swap button
  QList<QToolButton *> toolButtons =
      jointInspector->findChildren<QToolButton *>();
  QVERIFY(toolButtons.size() == 2);

  // Trigger swap
  toolButtons[1]->click();

  // Check parent and child links
  QVERIFY(parentBox->currentText() == "link2");
  QVERIFY(childBox->currentText() == "link1");

  delete jointInspector;
  delete jointMaker;
}

/////////////////////////////////////////////////
void JointInspector_TEST::RemoveButton()
{
  // Create a joint maker
  gazebo::gui::JointMaker *jointMaker = new gazebo::gui::JointMaker();
  QVERIFY(jointMaker != NULL);

  // Create a joint inspector
  gazebo::gui::JointInspector *jointInspector =
      new gazebo::gui::JointInspector(jointMaker);
  QVERIFY(jointInspector != NULL);

  // Open it
  jointInspector->Open();
  QVERIFY(jointInspector->isVisible());

  // Get buttons
  QList<QToolButton *> toolButtons =
      jointInspector->findChildren<QToolButton *>();
  QVERIFY(toolButtons.size() == 2);
  QVERIFY(toolButtons[0]->text() == "");

  // Trigger remove
  toolButtons[0]->click();

  // Check joint inspector disappeared
  QVERIFY(!jointInspector->isVisible());

  delete jointInspector;
  delete jointMaker;
}

// Generate a main function for the test
QTEST_MAIN(JointInspector_TEST)
