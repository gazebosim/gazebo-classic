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

#include "gazebo/gui/model/LinkConfig.hh"
#include "gazebo/gui/model/LinkConfig_TEST.hh"
#include "gazebo/gui/ConfigWidget.hh"

#include "test_config.h"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
void LinkConfig_TEST::Initialization()
{
  LinkConfig lc;
  const ConfigWidget *cw = lc.GetConfigWidget();

  QVERIFY(cw != NULL);

  QCOMPARE(cw->GetDoubleWidgetValue("inertial::mass"), 1.0);
  QCOMPARE(cw->GetDoubleWidgetValue("inertial::ixx"), 1.0);
  QCOMPARE(cw->GetDoubleWidgetValue("inertial::iyy"), 1.0);
  QCOMPARE(cw->GetDoubleWidgetValue("inertial::izz"), 1.0);

  QCOMPARE(cw->GetBoolWidgetValue("gravity"), true);
  QCOMPARE(cw->GetBoolWidgetValue("self_collide"), false);
  QCOMPARE(cw->GetBoolWidgetValue("kinematic"), false);
}

/////////////////////////////////////////////////
void LinkConfig_TEST::LinkMsgUpdate()
{
  gazebo::gui::LinkConfig lc;
  msgs::LinkPtr linkMsgPtr(new msgs::Link);
  const ConfigWidget *cw = lc.GetConfigWidget();

  QVERIFY(cw != NULL);

  linkMsgPtr->set_gravity(false);
  linkMsgPtr->set_self_collide(true);
  linkMsgPtr->set_kinematic(true);

  lc.Update(linkMsgPtr);

  QCOMPARE(cw->GetBoolWidgetValue("gravity"), false);
  QCOMPARE(cw->GetBoolWidgetValue("self_collide"), true);
  QCOMPARE(cw->GetBoolWidgetValue("kinematic"), true);
}

/////////////////////////////////////////////////
void LinkConfig_TEST::PoseUpdate()
{
  gazebo::gui::LinkConfig lc;
  const ignition::math::Pose3d pose(5.0, 10.0, 15.0, -1, -1, -1);
  const ConfigWidget *cw = lc.GetConfigWidget();

  QVERIFY(cw != NULL);

  lc.SetPose(pose);
  math::Pose p = cw->GetPoseWidgetValue("pose");

  QCOMPARE(p.pos.x, 5.0);
  QCOMPARE(p.pos.y, 10.0);
  QCOMPARE(p.pos.z, 15.0);
}

/////////////////////////////////////////////////
void LinkConfig_TEST::MassUpdate()
{
  gazebo::gui::LinkConfig lc;
  const ConfigWidget *cw = lc.GetConfigWidget();

  QVERIFY(cw != NULL);

  lc.SetMass(50.0);

  QCOMPARE(cw->GetDoubleWidgetValue("inertial::mass"), 50.0);
}

/////////////////////////////////////////////////
void LinkConfig_TEST::InertiaMatrixUpdate()
{
  gazebo::gui::LinkConfig lc;
  const ConfigWidget *cw = lc.GetConfigWidget();

  QVERIFY(cw != NULL);

  lc.SetInertiaMatrix(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);

  QCOMPARE(cw->GetDoubleWidgetValue("inertial::ixx"), 1.0);
  QCOMPARE(cw->GetDoubleWidgetValue("inertial::iyy"), 2.0);
  QCOMPARE(cw->GetDoubleWidgetValue("inertial::izz"), 3.0);

  QCOMPARE(cw->GetDoubleWidgetValue("inertial::ixy"), 4.0);
  QCOMPARE(cw->GetDoubleWidgetValue("inertial::ixz"), 5.0);
  QCOMPARE(cw->GetDoubleWidgetValue("inertial::iyz"), 6.0);
}

/////////////////////////////////////////////////
void LinkConfig_TEST::InertialPoseUpdate()
{
  gazebo::gui::LinkConfig lc;
  const ignition::math::Pose3d pose(5.0, 10.0, 15.0, -1, -1, -1);
  const ConfigWidget *cw = lc.GetConfigWidget();

  QVERIFY(cw != NULL);

  lc.SetInertialPose(pose);
  math::Pose p = cw->GetPoseWidgetValue("inertial::pose");

  QCOMPARE(p.pos.x, 5.0);
  QCOMPARE(p.pos.y, 10.0);
  QCOMPARE(p.pos.z, 15.0);
}

// Generate a main function for the test
QTEST_MAIN(LinkConfig_TEST)
