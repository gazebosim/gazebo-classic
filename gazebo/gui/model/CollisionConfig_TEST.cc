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

#include "gazebo/gui/model/CollisionConfig.hh"
#include "gazebo/gui/model/CollisionConfig_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void CollisionConfig_TEST::AppliedSignal()
{
  // Create a link inspector
  gazebo::gui::CollisionConfig *collisionConfig =
      new gazebo::gui::CollisionConfig();
  QVERIFY(collisionConfig != NULL);

  // Connect signals
  connect(collisionConfig, SIGNAL(Applied()), this, SLOT(OnApply()));

  // Init it
  collisionConfig->Init();
  QCOMPARE(g_appliedSignalCount, 0u);
  QCOMPARE(collisionConfig->GetCollisionCount(), 0u);

  // Get push buttons
  QList<QPushButton *> pushButtons =
      collisionConfig->findChildren<QPushButton *>();
  QVERIFY(pushButtons.size() == 1);

  // Add a collision
  pushButtons[0]->click();
  QCOMPARE(collisionConfig->GetCollisionCount(), 1u);

  // Get spins
  QList<QDoubleSpinBox *> spins =
      collisionConfig->findChildren<QDoubleSpinBox *>();
  QVERIFY(spins.size() == 32);

  // Get combo boxes
  QList<QComboBox *> combos =
      collisionConfig->findChildren<QComboBox *>();
  QVERIFY(combos.size() == 1);

  // Edit collision pose (2~7)
  spins[2]->setValue(2.0);
  QTest::keyClick(spins[2], Qt::Key_Enter);
  QCOMPARE(g_appliedSignalCount, 1u);

  // Edit geometry (0)
  combos[0]->setCurrentIndex(2);
  QTest::keyClick(combos[0], Qt::Key_Enter);
  QCOMPARE(g_appliedSignalCount, 2u);

  delete collisionConfig;
}

/////////////////////////////////////////////////
void CollisionConfig_TEST::OnApply()
{
  g_appliedSignalCount++;
}

// Generate a main function for the test
QTEST_MAIN(CollisionConfig_TEST)
