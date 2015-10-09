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

#include "gazebo/gui/model/VisualConfig.hh"
#include "gazebo/gui/model/VisualConfig_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void VisualConfig_TEST::AppliedSignal()
{
  // Create a link inspector
  gazebo::gui::VisualConfig *visualConfig =
      new gazebo::gui::VisualConfig();
  QVERIFY(visualConfig != NULL);

  // Connect signals
  connect(visualConfig, SIGNAL(Applied()), this, SLOT(OnApply()));

  // Init it
  visualConfig->Init();
  QCOMPARE(g_appliedSignalCount, 0u);
  QCOMPARE(visualConfig->GetVisualCount(), 0u);

  // Add a visual
  visualConfig->AddVisual("visual_name");
  QCOMPARE(visualConfig->GetVisualCount(), 1u);

  // Get spins
  QList<QDoubleSpinBox *> spins =
      visualConfig->findChildren<QDoubleSpinBox *>();
  QVERIFY(spins.size() == 32);

  // Get combo boxes
  QList<QComboBox *> combos =
      visualConfig->findChildren<QComboBox *>();
  QVERIFY(combos.size() == 3);

  // Edit transparency (0)
  spins[0]->setValue(0.5);
  QTest::keyClick(spins[0], Qt::Key_Enter);
  QCOMPARE(g_appliedSignalCount, 1u);

  // Edit visual pose (2~7)
  spins[2]->setValue(2.0);
  QTest::keyClick(spins[2], Qt::Key_Enter);
  QCOMPARE(g_appliedSignalCount, 2u);

  // Edit geometry (0)
  combos[0]->setCurrentIndex(2);
  QTest::keyClick(combos[0], Qt::Key_Enter);
  QCOMPARE(g_appliedSignalCount, 3u);

  // Edit color (13~28)
  spins[15]->setValue(0.3);
  QTest::keyClick(spins[15], Qt::Key_Enter);
  QCOMPARE(g_appliedSignalCount, 4u);

  delete visualConfig;
}

/////////////////////////////////////////////////
void VisualConfig_TEST::OnApply()
{
  g_appliedSignalCount++;
}

// Generate a main function for the test
QTEST_MAIN(VisualConfig_TEST)
