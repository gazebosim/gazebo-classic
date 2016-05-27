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

#include "gazebo/gui/ConfigWidget.hh"
#include "gazebo/gui/model/VisualConfig.hh"
#include "gazebo/gui/model/VisualConfig_TEST.hh"

#include "test_config.h"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
void VisualConfig_TEST::Initialization()
{
  VisualConfig vc;

  QCOMPARE(vc.GetVisualCount(), 0u);
  QVERIFY(vc.GetData("NotFound") == NULL);
}

/////////////////////////////////////////////////
void VisualConfig_TEST::VisualUpdates()
{
  VisualConfig vc;
  msgs::Visual v1, v2, v3;

  vc.AddVisual("v1", &v1);
  vc.AddVisual("v2", &v2);
  vc.AddVisual("v3", &v3);

  QCOMPARE(vc.GetVisualCount(), 3u);

  QVERIFY(vc.GetData("v1") != NULL);
  QVERIFY(vc.GetData("v2") != NULL);
  QVERIFY(vc.GetData("v3") != NULL);
  QVERIFY(vc.GetData("NotFound") == NULL);

  msgs::VisualPtr visualMsgPtr(new msgs::Visual);
  visualMsgPtr->set_transparency(0.50);

  vc.UpdateVisual("v1", visualMsgPtr);
  bool foundConfig = false;

  for (const auto &it : vc.ConfigData())
  {
    if (it.second->name == "v1")
    {
      const VisualConfigData *configData = it.second;
      QCOMPARE(configData->configWidget->DoubleWidgetValue("transparency"),
          0.50);
      foundConfig = true;
      break;
    }
  }
  QVERIFY(foundConfig);

  vc.Reset();

  QCOMPARE(vc.GetVisualCount(), 0u);

  QVERIFY(vc.GetData("v1") == NULL);
  QVERIFY(vc.GetData("v2") == NULL);
  QVERIFY(vc.GetData("v3") == NULL);
}

/////////////////////////////////////////////////
void VisualConfig_TEST::GeometryUpdates()
{
  VisualConfig vc;
  msgs::Visual v1;

  vc.AddVisual("v1", &v1);

  const ignition::math::Vector3d size1(5, 10, 15);

  vc.SetGeometry("v1", size1, "unit_box");

  ignition::math::Vector3d size2;
  std::string uri;

  vc.Geometry("v1", size2, uri);

  QCOMPARE(5.0, size2.X());
  QCOMPARE(10.0, size2.Y());
  QCOMPARE(15.0, size2.Z());
  QCOMPARE(uri, std::string(""));

  ignition::math::Vector3d size3(0, 0, 0);

  vc.Geometry("NotFound", size3, uri);

  QCOMPARE(0.0, size3.X());
  QCOMPARE(0.0, size3.Y());
  QCOMPARE(0.0, size3.Z());
  QCOMPARE(uri, std::string(""));
}

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

  // Get push buttons
  QList<QPushButton *> pushButtons =
      visualConfig->findChildren<QPushButton *>();
  QVERIFY(pushButtons.size() == 1);

  // Add a collision
  pushButtons[0]->click();
  QCOMPARE(visualConfig->GetVisualCount(), 1u);

  // Get spins
  QList<QDoubleSpinBox *> spins =
      visualConfig->findChildren<QDoubleSpinBox *>();
  QVERIFY(spins.size() == 32);

  // Get combo boxes
  QList<QComboBox *> combos =
      visualConfig->findChildren<QComboBox *>();
  QVERIFY(combos.size() == 4);

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

/////////////////////////////////////////////////
void VisualConfig_TEST::Restore()
{
  VisualConfig vc;
  msgs::Visual v1, v2, v3;

  vc.AddVisual("v1", &v1);
  vc.AddVisual("v2", &v2);

  QCOMPARE(vc.GetVisualCount(), 2u);
  QVERIFY(vc.GetData("v1") != NULL);
  QVERIFY(vc.GetData("v2") != NULL);
  QVERIFY(vc.GetData("v3") == NULL);

  // Set this as the original data
  vc.Init();

  // Add a visual and restore
  vc.AddVisual("v3", &v3);
  QCOMPARE(vc.GetVisualCount(), 3u);
  QVERIFY(vc.GetData("v1") != NULL);
  QVERIFY(vc.GetData("v2") != NULL);
  QVERIFY(vc.GetData("v3") != NULL);

  vc.RestoreOriginalData();
  QCOMPARE(vc.GetVisualCount(), 2u);
  QVERIFY(vc.GetData("v1") != NULL);
  QVERIFY(vc.GetData("v2") != NULL);
  QVERIFY(vc.GetData("v3") == NULL);

  // Remove a visual and restore
  auto button = vc.findChild<QToolButton *>("removeVisualButton_0");
  QVERIFY(button);

  // Note that the confirmation dialog has been disabled for tests (issue #1963)
  button->click();

  QCOMPARE(vc.GetVisualCount(), 1u);
  QVERIFY(vc.GetData("v1") == NULL);
  QVERIFY(vc.GetData("v2") != NULL);
  QVERIFY(vc.GetData("v3") == NULL);

  vc.RestoreOriginalData();
  QCOMPARE(vc.GetVisualCount(), 2u);
  QVERIFY(vc.GetData("v1") != NULL);
  QVERIFY(vc.GetData("v2") != NULL);
  QVERIFY(vc.GetData("v3") == NULL);
}

// Generate a main function for the test
QTEST_MAIN(VisualConfig_TEST)
