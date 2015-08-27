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
#include "gazebo/gui/ConfigWidget.hh"

#include "test_config.h"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
void VisualConfig_TEST::Initialization()
{
  gazebo::gui::VisualConfig vc;

  QCOMPARE(vc.GetVisualCount(), (unsigned int) 0);
  QVERIFY(vc.GetData("NotFound") == NULL);
}

/////////////////////////////////////////////////
void VisualConfig_TEST::VisualUpdates()
{
  gazebo::gui::VisualConfig vc;

  msgs::Visual v1, v2, v3;

  vc.AddVisual("v1", &v1);
  vc.AddVisual("v2", &v2);
  vc.AddVisual("v3", &v3);

  QCOMPARE(vc.GetVisualCount(), (unsigned int) 3);

  QVERIFY(vc.GetData("v1") != NULL);
  QVERIFY(vc.GetData("v2") != NULL);
  QVERIFY(vc.GetData("v3") != NULL);
  QVERIFY(vc.GetData("NotFound") == NULL);

  msgs::VisualPtr visualMsgPtr(new msgs::Visual);
  visualMsgPtr->set_transparency(0.50);

  vc.UpdateVisual("v1", visualMsgPtr);

  // Access visual as white box to verify update

  bool found = false;
  for (auto &it : vc.configs)
  {
    if (it.second->name == "v1")
    {
      VisualConfigData *vcd = it.second;
      ConfigWidget *cw = vcd->configWidget;

      QCOMPARE(cw->GetDoubleWidgetValue("transparency"), (double)0.50);
      found = true;
    }
  }
  QVERIFY(found);

  vc.Reset();

  QCOMPARE(vc.GetVisualCount(), (unsigned int) 0);

  QVERIFY(vc.GetData("v1") == NULL);
  QVERIFY(vc.GetData("v2") == NULL);
  QVERIFY(vc.GetData("v3") == NULL);

}

/////////////////////////////////////////////////
void VisualConfig_TEST::GeometryUpdates()
{
  gazebo::gui::VisualConfig vc;
  msgs::Visual v1;

  vc.AddVisual("v1", &v1);

  const ignition::math::Vector3d size1(5, 10, 15);

  vc.SetGeometry("v1", size1, "unit_box");

  ignition::math::Vector3d size2;
  std::string uri;

  vc.GetGeometry("v1", size2, uri);

  QCOMPARE(5.0, size2.X());
  QCOMPARE(10.0, size2.Y());
  QCOMPARE(15.0, size2.Z());

  ignition::math::Vector3d size3(0, 0, 0);

  vc.GetGeometry("NotFound", size3, uri);

  QCOMPARE(0.0, size3.X());
  QCOMPARE(0.0, size3.Y());
  QCOMPARE(0.0, size3.Z());

}

// Generate a main function for the test
QTEST_MAIN(VisualConfig_TEST)
