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

#include "gazebo/gui/model/LinkInspector.hh"
#include "gazebo/gui/model/LinkInspector_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void LinkInspector_TEST::RemoveButton()
{
  // Create a link inspector
  gazebo::gui::LinkInspector *linkInspector =
      new gazebo::gui::LinkInspector();
  QVERIFY(linkInspector != NULL);

  // Open it
  linkInspector->Open();
  QVERIFY(linkInspector->isVisible());

  // Get buttons
  QList<QToolButton *> toolButtons =
      linkInspector->findChildren<QToolButton *>();

  // 3 tool buttons: remove link, remove visual, remove collision
  QVERIFY(toolButtons.size() == 3);
  QVERIFY(toolButtons[0]->text() == "");

  // Trigger remove
  toolButtons[0]->click();

  // Check link inspector disappeared
  QVERIFY(!linkInspector->isVisible());

  delete linkInspector;
}

/////////////////////////////////////////////////
void LinkInspector_TEST::AppliedSignal()
{
  // Create a link inspector
  gazebo::gui::LinkInspector *linkInspector =
      new gazebo::gui::LinkInspector();
  QVERIFY(linkInspector != NULL);

  // Connect signals
  connect(linkInspector, SIGNAL(Applied()), this, SLOT(OnApply()));

  // Open it
  linkInspector->Open();
  QVERIFY(linkInspector->isVisible());
  QCOMPARE(g_appliedSignalCount, 0u);

  // Get spins
  QList<QDoubleSpinBox *> spins =
      linkInspector->findChildren<QDoubleSpinBox *>();
  QVERIFY(spins.size() == 19);

  // Edit link pose (13~18)
  spins[18]->setValue(2.0);
  QTest::keyClick(spins[18], Qt::Key_Enter);
  QCOMPARE(g_appliedSignalCount, 1u);

  delete linkInspector;
}

/////////////////////////////////////////////////
void LinkInspector_TEST::OnApply()
{
  g_appliedSignalCount++;
}

// Generate a main function for the test
QTEST_MAIN(LinkInspector_TEST)
