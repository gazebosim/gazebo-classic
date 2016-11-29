/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#include "gazebo/gui/GLWidget.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/model/JointInspector.hh"
#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/JointMaker.hh"
#include "gazebo/gui/model/JointMaker_TEST.hh"

using namespace gazebo;

/////////////////////////////////////////////////
void JointMaker_TEST::JointState()
{
  this->Load("worlds/empty.world");

  gui::JointMaker *jointMaker = new gui::JointMaker();
  QCOMPARE(jointMaker->State(), gui::JointMaker::JOINT_NONE);

  jointMaker->AddJoint(gui::JointMaker::JOINT_HINGE);
  QCOMPARE(jointMaker->State(), gui::JointMaker::JOINT_HINGE);

  jointMaker->Reset();
  QCOMPARE(jointMaker->State(), gui::JointMaker::JOINT_NONE);

  jointMaker->AddJoint(gui::JointMaker::JOINT_SLIDER);
  QCOMPARE(jointMaker->State(), gui::JointMaker::JOINT_SLIDER);

  jointMaker->Stop();
  QCOMPARE(jointMaker->State(), gui::JointMaker::JOINT_NONE);

  delete jointMaker;
}

/////////////////////////////////////////////////
void JointMaker_TEST::CreateRemoveJoint()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

  gui::JointMaker *jointMaker = new gui::JointMaker();
  QCOMPARE(jointMaker->State(), gui::JointMaker::JOINT_NONE);
  QCOMPARE(jointMaker->JointCount(), 0u);

  gui::MainWindow *mainWindow = new gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  rendering::UserCameraPtr cam = gui::get_active_camera();
  Q_ASSERT(cam);
  rendering::ScenePtr scene = cam->GetScene();
  Q_ASSERT(scene);

  rendering::VisualPtr boxLink = scene->GetVisual("box::link");
  rendering::VisualPtr sphereLink = scene->GetVisual("sphere::link");
  rendering::VisualPtr cylinderLink = scene->GetVisual("cylinder::link");

  Q_ASSERT(boxLink.get());
  Q_ASSERT(sphereLink.get());
  Q_ASSERT(cylinderLink.get());

  // Add a revolute joint
  jointMaker->AddJoint(gui::JointMaker::JOINT_HINGE);
  gui::JointData *revoluteJointData =
      jointMaker->CreateJoint(boxLink, sphereLink);
  jointMaker->CreateHotSpot(revoluteJointData);
  QCOMPARE(jointMaker->JointCount(), 1u);

  // Add a prismatic joint
  jointMaker->AddJoint(gui::JointMaker::JOINT_SLIDER);
  gui::JointData *prismaticJointData =
      jointMaker->CreateJoint(sphereLink, cylinderLink);
  jointMaker->CreateHotSpot(prismaticJointData);
  QCOMPARE(jointMaker->JointCount(), 2u);

  // Add a screw joint
  jointMaker->AddJoint(gui::JointMaker::JOINT_SCREW);
  gui::JointData *screwJointData =
      jointMaker->CreateJoint(cylinderLink, boxLink);
  jointMaker->CreateHotSpot(screwJointData);
  QCOMPARE(jointMaker->JointCount(), 3u);

  // Remove the screw joint
  jointMaker->RemoveJoint(screwJointData->hotspot->GetName());
  QCOMPARE(jointMaker->JointCount(), 2u);

  // Add a ball joint
  jointMaker->AddJoint(gui::JointMaker::JOINT_BALL);
  gui::JointData *ballJointData =
      jointMaker->CreateJoint(cylinderLink, boxLink);
  jointMaker->CreateHotSpot(ballJointData);
  QCOMPARE(jointMaker->JointCount(), 3u);

  // Remove the two joints connected to the sphere
  jointMaker->RemoveJointsByLink(sphereLink->GetName());
  QCOMPARE(jointMaker->JointCount(), 1u);

  // Remove the last joint
  jointMaker->RemoveJoint(ballJointData->hotspot->GetName());
  QCOMPARE(jointMaker->JointCount(), 0u);

  delete jointMaker;
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void JointMaker_TEST::CreateRemoveNestedJoint()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/deeply_nested_models.world", false, false, false);

  gui::JointMaker *jointMaker = new gui::JointMaker();
  QCOMPARE(jointMaker->State(), gui::JointMaker::JOINT_NONE);
  QCOMPARE(jointMaker->JointCount(), 0u);

  gui::MainWindow *mainWindow = new gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  rendering::UserCameraPtr cam = gui::get_active_camera();
  Q_ASSERT(cam);
  rendering::ScenePtr scene = cam->GetScene();
  Q_ASSERT(scene);

  // Get the top level model link
  gazebo::rendering::VisualPtr link00Vis =
      scene->GetVisual("model_00::link_00");
  QVERIFY(link00Vis != NULL);

  // Get the nested model links
  gazebo::rendering::VisualPtr link01Vis =
      scene->GetVisual("model_00::model_01::link_01");
  QVERIFY(link01Vis != NULL);
  gazebo::rendering::VisualPtr link02Vis =
      scene->GetVisual("model_00::model_01::model_02::link_02");
  QVERIFY(link02Vis != NULL);
  gazebo::rendering::VisualPtr link03Vis =
      scene->GetVisual("model_00::model_01::model_02::model_03::link_03");
  QVERIFY(link03Vis != NULL);

  unsigned int jointCount = jointMaker->JointCount();

  // Remove joints connected to the each link
  jointMaker->RemoveJointsByLink(link00Vis->GetName());
  QVERIFY(jointMaker->JointCount() <= jointCount);
  jointCount = jointMaker->JointCount();

  jointMaker->RemoveJointsByLink(link01Vis->GetName());
  QVERIFY(jointMaker->JointCount() <= jointCount);
  jointCount = jointMaker->JointCount();

  jointMaker->RemoveJointsByLink(link02Vis->GetName());
  QVERIFY(jointMaker->JointCount() <= jointCount);
  jointCount = jointMaker->JointCount();

  jointMaker->RemoveJointsByLink(link03Vis->GetName());
  QVERIFY(jointMaker->JointCount() <= jointCount);

  // no more joints left
  QCOMPARE(jointMaker->JointCount(), 0u);

  // Add a revolute joint
  jointMaker->AddJoint(gui::JointMaker::JOINT_HINGE);
  gui::JointData *revoluteJointData =
      jointMaker->CreateJoint(link00Vis, link01Vis);
  jointMaker->CreateHotSpot(revoluteJointData);
  QCOMPARE(jointMaker->JointCount(), 1u);

  // Add a prismatic joint
  jointMaker->AddJoint(gui::JointMaker::JOINT_SLIDER);
  gui::JointData *prismaticJointData =
      jointMaker->CreateJoint(link01Vis, link02Vis);
  jointMaker->CreateHotSpot(prismaticJointData);
  QCOMPARE(jointMaker->JointCount(), 2u);

  // Add a screw joint
  jointMaker->AddJoint(gui::JointMaker::JOINT_SCREW);
  gui::JointData *screwJointData =
      jointMaker->CreateJoint(link00Vis, link02Vis);
  jointMaker->CreateHotSpot(screwJointData);
  QCOMPARE(jointMaker->JointCount(), 3u);

  // Remove the screw joint
  jointMaker->RemoveJoint(screwJointData->hotspot->GetName());
  QCOMPARE(jointMaker->JointCount(), 2u);

  // Add a ball joint
  jointMaker->AddJoint(gui::JointMaker::JOINT_BALL);
  gui::JointData *ballJointData =
      jointMaker->CreateJoint(link00Vis, link03Vis);
  jointMaker->CreateHotSpot(ballJointData);
  QCOMPARE(jointMaker->JointCount(), 3u);

  // Remove the two joints connected to link01
  jointMaker->RemoveJointsByLink(link01Vis->GetName());
  QCOMPARE(jointMaker->JointCount(), 1u);

  // Remove the last joint
  jointMaker->RemoveJoint(ballJointData->hotspot->GetName());
  QCOMPARE(jointMaker->JointCount(), 0u);

  delete jointMaker;
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void JointMaker_TEST::JointDefaultProperties()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

  gui::JointMaker *jointMaker = new gui::JointMaker();
  QCOMPARE(jointMaker->State(), gui::JointMaker::JOINT_NONE);
  QCOMPARE(jointMaker->JointCount(), 0u);

  gui::MainWindow *mainWindow = new gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  rendering::UserCameraPtr cam = gui::get_active_camera();
  Q_ASSERT(cam);
  rendering::ScenePtr scene = cam->GetScene();
  Q_ASSERT(scene);

  rendering::VisualPtr boxLink = scene->GetVisual("box::link");
  rendering::VisualPtr sphereLink = scene->GetVisual("sphere::link");
  rendering::VisualPtr cylinderLink = scene->GetVisual("cylinder::link");

  Q_ASSERT(boxLink.get());
  Q_ASSERT(sphereLink.get());
  Q_ASSERT(cylinderLink.get());

  // Add a revolute2 joint
  jointMaker->AddJoint(gui::JointMaker::JOINT_HINGE2);
  gui::JointData *revoluteJointData =
      jointMaker->CreateJoint(boxLink, sphereLink);
  jointMaker->CreateHotSpot(revoluteJointData);
  QCOMPARE(jointMaker->JointCount(), 1u);

  // verify connected joints
  std::vector<gui::JointData *> boxJointData =
      jointMaker->JointDataByLink("box::link");
  QCOMPARE(static_cast<unsigned int>(boxJointData.size()), 1u);

  gui::JointData *rev2joint = boxJointData[0];
  QVERIFY(rev2joint != NULL);
  QVERIFY(rev2joint->inspector != NULL);

  // verify default values
  QVERIFY(msgs::ConvertJointType(rev2joint->jointMsg->type()) == "revolute2");
  QCOMPARE(msgs::ConvertIgn(rev2joint->jointMsg->pose()),
      ignition::math::Pose3d::Zero);
  QVERIFY(ignition::math::equal(rev2joint->jointMsg->cfm(), 0.0));
  QVERIFY(ignition::math::equal(rev2joint->jointMsg->bounce(), 0.0));
  QVERIFY(ignition::math::equal(rev2joint->jointMsg->fudge_factor(), 0.0));
  QVERIFY(ignition::math::equal(rev2joint->jointMsg->limit_cfm(), 0.0));
  QVERIFY(ignition::math::equal(rev2joint->jointMsg->limit_erp(), 0.2));
  QVERIFY(ignition::math::equal(rev2joint->jointMsg->suspension_cfm(), 0.0));
  QVERIFY(ignition::math::equal(rev2joint->jointMsg->suspension_erp(), 0.2));

  msgs::Axis rev2Axis1Msg = rev2joint->jointMsg->axis1();
  QCOMPARE(msgs::ConvertIgn(rev2Axis1Msg.xyz()),
      ignition::math::Vector3d(1, 0, 0));
  QVERIFY(ignition::math::equal(rev2Axis1Msg.limit_lower(), -IGN_DBL_MAX));
  QVERIFY(ignition::math::equal(rev2Axis1Msg.limit_upper(), IGN_DBL_MAX));
  QVERIFY(ignition::math::equal(rev2Axis1Msg.limit_effort(), -1.0));
  QVERIFY(ignition::math::equal(rev2Axis1Msg.limit_velocity(), -1.0));
  QVERIFY(ignition::math::equal(rev2Axis1Msg.damping(), 0.0));
  QVERIFY(ignition::math::equal(rev2Axis1Msg.friction(), 0.0));
  QCOMPARE(rev2Axis1Msg.use_parent_model_frame(), false);

  msgs::Axis rev2Axis2Msg = rev2joint->jointMsg->axis2();
  QCOMPARE(msgs::ConvertIgn(rev2Axis2Msg.xyz()),
      ignition::math::Vector3d(0, 1, 0));
  QVERIFY(ignition::math::equal(rev2Axis2Msg.limit_lower(), -IGN_DBL_MAX));
  QVERIFY(ignition::math::equal(rev2Axis2Msg.limit_upper(), IGN_DBL_MAX));
  QVERIFY(ignition::math::equal(rev2Axis2Msg.limit_effort(), -1.0));
  QVERIFY(ignition::math::equal(rev2Axis2Msg.limit_velocity(), -1.0));
  QVERIFY(ignition::math::equal(rev2Axis2Msg.damping(), 0.0));
  QVERIFY(ignition::math::equal(rev2Axis2Msg.friction(), 0.0));
  QCOMPARE(rev2Axis2Msg.use_parent_model_frame(), false);

  // Add a prismatic joint
  jointMaker->AddJoint(gui::JointMaker::JOINT_SLIDER);
  gui::JointData *prismaticJointData =
      jointMaker->CreateJoint(sphereLink, cylinderLink);
  jointMaker->CreateHotSpot(prismaticJointData);
  QCOMPARE(jointMaker->JointCount(), 2u);

  // verify connected joints
  std::vector<gui::JointData *> sphereJointData =
      jointMaker->JointDataByLink("sphere::link");
  QCOMPARE(static_cast<unsigned int>(sphereJointData.size()), 2u);

  std::vector<gui::JointData *> cylinderJointData =
      jointMaker->JointDataByLink("cylinder::link");
  QCOMPARE(static_cast<unsigned int>(cylinderJointData.size()), 1u);

  gui::JointData *prisJoint = cylinderJointData[0];
  QVERIFY(prisJoint != NULL);
  QVERIFY(prisJoint->inspector != NULL);

  // verify default values
  QVERIFY(msgs::ConvertJointType(prisJoint->jointMsg->type()) == "prismatic");
  QCOMPARE(msgs::ConvertIgn(prisJoint->jointMsg->pose()),
      ignition::math::Pose3d::Zero);
  QVERIFY(ignition::math::equal(prisJoint->jointMsg->cfm(), 0.0));
  QVERIFY(ignition::math::equal(prisJoint->jointMsg->bounce(), 0.0));
  QVERIFY(ignition::math::equal(prisJoint->jointMsg->fudge_factor(), 0.0));
  QVERIFY(ignition::math::equal(prisJoint->jointMsg->limit_cfm(), 0.0));
  QVERIFY(ignition::math::equal(prisJoint->jointMsg->limit_erp(), 0.2));
  QVERIFY(ignition::math::equal(prisJoint->jointMsg->suspension_cfm(), 0.0));
  QVERIFY(ignition::math::equal(prisJoint->jointMsg->suspension_erp(), 0.2));

  msgs::Axis prisAxis1Msg = prisJoint->jointMsg->axis1();
  QCOMPARE(msgs::ConvertIgn(prisAxis1Msg.xyz()),
      ignition::math::Vector3d(1, 0, 0));
  QVERIFY(ignition::math::equal(prisAxis1Msg.limit_lower(), -IGN_DBL_MAX));
  QVERIFY(ignition::math::equal(prisAxis1Msg.limit_upper(), IGN_DBL_MAX));
  QVERIFY(ignition::math::equal(prisAxis1Msg.limit_effort(), -1.0));
  QVERIFY(ignition::math::equal(prisAxis1Msg.limit_velocity(), -1.0));
  QVERIFY(ignition::math::equal(prisAxis1Msg.damping(), 0.0));
  QVERIFY(ignition::math::equal(prisAxis1Msg.friction(), 0.0));
  QCOMPARE(prisAxis1Msg.use_parent_model_frame(), false);

  // Add a gearbox joint
  jointMaker->AddJoint(gui::JointMaker::JOINT_GEARBOX);
  gui::JointData *gearboxJointData =
      jointMaker->CreateJoint(boxLink, cylinderLink);
  jointMaker->CreateHotSpot(gearboxJointData);
  QCOMPARE(jointMaker->JointCount(), 3u);

  // verify connected joints
  boxJointData =
      jointMaker->JointDataByLink("box::link");
  QCOMPARE(static_cast<unsigned int>(boxJointData.size()), 2u);

  cylinderJointData =
      jointMaker->JointDataByLink("cylinder::link");
  QCOMPARE(static_cast<unsigned int>(cylinderJointData.size()), 2u);

  gui::JointData *gearboxJoint = cylinderJointData[0];
  QVERIFY(gearboxJoint != NULL);
  QVERIFY(gearboxJoint->inspector != NULL);

  // verify default values
  QVERIFY(msgs::ConvertJointType(gearboxJoint->jointMsg->type()) == "gearbox");
  QCOMPARE(msgs::ConvertIgn(gearboxJoint->jointMsg->pose()),
      ignition::math::Pose3d::Zero);
  QVERIFY(ignition::math::equal(gearboxJoint->jointMsg->cfm(), 0.0));
  QVERIFY(ignition::math::equal(gearboxJoint->jointMsg->bounce(), 0.0));
  QVERIFY(ignition::math::equal(gearboxJoint->jointMsg->fudge_factor(), 0.0));
  QVERIFY(ignition::math::equal(gearboxJoint->jointMsg->limit_cfm(), 0.0));
  QVERIFY(ignition::math::equal(gearboxJoint->jointMsg->limit_erp(), 0.2));
  QVERIFY(ignition::math::equal(gearboxJoint->jointMsg->suspension_cfm(), 0.0));
  QVERIFY(ignition::math::equal(gearboxJoint->jointMsg->suspension_erp(), 0.2));

  msgs::Axis gearboxAxis1Msg = gearboxJoint->jointMsg->axis1();
  QCOMPARE(msgs::ConvertIgn(gearboxAxis1Msg.xyz()),
      ignition::math::Vector3d(0, 0, 1));
  QVERIFY(ignition::math::equal(gearboxAxis1Msg.limit_lower(), -IGN_DBL_MAX));
  QVERIFY(ignition::math::equal(gearboxAxis1Msg.limit_upper(), IGN_DBL_MAX));
  QVERIFY(ignition::math::equal(gearboxAxis1Msg.limit_effort(), -1.0));
  QVERIFY(ignition::math::equal(gearboxAxis1Msg.limit_velocity(), -1.0));
  QVERIFY(ignition::math::equal(gearboxAxis1Msg.damping(), 0.0));
  QVERIFY(ignition::math::equal(gearboxAxis1Msg.friction(), 0.0));
  QCOMPARE(gearboxAxis1Msg.use_parent_model_frame(), false);

  msgs::Axis gearboxAxis2Msg = gearboxJoint->jointMsg->axis2();
  QCOMPARE(msgs::ConvertIgn(gearboxAxis2Msg.xyz()),
      ignition::math::Vector3d(0, 0, 1));
  QVERIFY(ignition::math::equal(gearboxAxis2Msg.limit_lower(), -IGN_DBL_MAX));
  QVERIFY(ignition::math::equal(gearboxAxis2Msg.limit_upper(), IGN_DBL_MAX));
  QVERIFY(ignition::math::equal(gearboxAxis2Msg.limit_effort(), -1.0));
  QVERIFY(ignition::math::equal(gearboxAxis2Msg.limit_velocity(), -1.0));
  QVERIFY(ignition::math::equal(gearboxAxis2Msg.damping(), 0.0));
  QVERIFY(ignition::math::equal(gearboxAxis2Msg.friction(), 0.0));
  QCOMPARE(gearboxAxis2Msg.use_parent_model_frame(), false);

  // Add a fixed joint
  jointMaker->AddJoint(gui::JointMaker::JOINT_FIXED);
  gui::JointData *fixedJointData =
      jointMaker->CreateJoint(boxLink, cylinderLink);
  jointMaker->CreateHotSpot(fixedJointData);
  QCOMPARE(jointMaker->JointCount(), 4u);

  // verify connected joints
  boxJointData =
      jointMaker->JointDataByLink("box::link");
  QCOMPARE(static_cast<unsigned int>(boxJointData.size()), 3u);

  cylinderJointData =
      jointMaker->JointDataByLink("cylinder::link");
  QCOMPARE(static_cast<unsigned int>(cylinderJointData.size()), 3u);

  gui::JointData *fixedJoint = cylinderJointData[1];
  QVERIFY(fixedJoint != NULL);
  QVERIFY(fixedJoint->inspector != NULL);

  // verify default values
  QVERIFY(msgs::ConvertJointType(fixedJoint->jointMsg->type()) == "fixed");
  QCOMPARE(msgs::ConvertIgn(fixedJoint->jointMsg->pose()),
      ignition::math::Pose3d::Zero);
  QVERIFY(ignition::math::equal(fixedJoint->jointMsg->cfm(), 0.0));
  QVERIFY(ignition::math::equal(fixedJoint->jointMsg->bounce(), 0.0));
  QVERIFY(ignition::math::equal(fixedJoint->jointMsg->fudge_factor(), 0.0));
  QVERIFY(ignition::math::equal(fixedJoint->jointMsg->limit_cfm(), 0.0));
  QVERIFY(ignition::math::equal(fixedJoint->jointMsg->limit_erp(), 0.2));
  QVERIFY(ignition::math::equal(fixedJoint->jointMsg->suspension_cfm(), 0.0));
  QVERIFY(ignition::math::equal(fixedJoint->jointMsg->suspension_erp(), 0.2));

  // fixed joint has no axes.
  QVERIFY(!fixedJoint->jointMsg->has_axis1());
  QVERIFY(!fixedJoint->jointMsg->has_axis2());

  delete jointMaker;
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void JointMaker_TEST::ShowJoints()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

  gui::JointMaker *jointMaker = new gui::JointMaker();

  gui::MainWindow *mainWindow = new gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  rendering::UserCameraPtr cam = gui::get_active_camera();
  Q_ASSERT(cam);
  rendering::ScenePtr scene = cam->GetScene();
  Q_ASSERT(scene);

  rendering::VisualPtr boxLink = scene->GetVisual("box::link");
  rendering::VisualPtr sphereLink = scene->GetVisual("sphere::link");
  rendering::VisualPtr cylinderLink = scene->GetVisual("cylinder::link");

  Q_ASSERT(boxLink.get());
  Q_ASSERT(sphereLink.get());
  Q_ASSERT(cylinderLink.get());

  // Add a revolute joint
  jointMaker->AddJoint(gui::JointMaker::JOINT_HINGE);
  gui::JointData *revoluteJointData =
      jointMaker->CreateJoint(boxLink, sphereLink);
  jointMaker->CreateHotSpot(revoluteJointData);
  QCOMPARE(jointMaker->JointCount(), 1u);

  // Add a prismatic joint
  jointMaker->AddJoint(gui::JointMaker::JOINT_SLIDER);
  gui::JointData *prismaticJointData =
      jointMaker->CreateJoint(sphereLink, cylinderLink);
  jointMaker->CreateHotSpot(prismaticJointData);
  QCOMPARE(jointMaker->JointCount(), 2u);

  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(revoluteJointData->hotspot != NULL);
  QVERIFY(prismaticJointData->hotspot != NULL);
  QVERIFY(revoluteJointData->jointVisual != NULL);
  QVERIFY(prismaticJointData->jointVisual != NULL);

  // toggle joint visualization and verify
  jointMaker->ShowJoints(false);
  QVERIFY(!revoluteJointData->hotspot->GetVisible());
  QVERIFY(!prismaticJointData->hotspot->GetVisible());
  QVERIFY(!revoluteJointData->jointVisual->GetVisible());
  QVERIFY(!prismaticJointData->jointVisual->GetVisible());

  jointMaker->ShowJoints(true);
  QVERIFY(revoluteJointData->hotspot->GetVisible());
  QVERIFY(prismaticJointData->hotspot->GetVisible());
  QVERIFY(revoluteJointData->jointVisual->GetVisible());
  QVERIFY(prismaticJointData->jointVisual->GetVisible());

  jointMaker->ShowJoints(false);
  QVERIFY(!revoluteJointData->hotspot->GetVisible());
  QVERIFY(!prismaticJointData->hotspot->GetVisible());
  QVERIFY(!revoluteJointData->jointVisual->GetVisible());
  QVERIFY(!prismaticJointData->jointVisual->GetVisible());

  delete jointMaker;
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void JointMaker_TEST::Selection()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

  gui::JointMaker *jointMaker = new gui::JointMaker();

  QCOMPARE(jointMaker->State(), gui::JointMaker::JOINT_NONE);
  QCOMPARE(jointMaker->JointCount(), 0u);

  gui::MainWindow *mainWindow = new gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  rendering::UserCameraPtr cam = gui::get_active_camera();
  Q_ASSERT(cam);
  rendering::ScenePtr scene = cam->GetScene();
  Q_ASSERT(scene);

  rendering::VisualPtr boxLink = scene->GetVisual("box::link");
  rendering::VisualPtr sphereLink = scene->GetVisual("sphere::link");
  rendering::VisualPtr cylinderLink = scene->GetVisual("cylinder::link");

  Q_ASSERT(boxLink.get());
  Q_ASSERT(sphereLink.get());
  Q_ASSERT(cylinderLink.get());

  // Add a revolute joint
  jointMaker->AddJoint(gui::JointMaker::JOINT_HINGE);
  gui::JointData *revoluteJointData =
      jointMaker->CreateJoint(boxLink, sphereLink);
  jointMaker->CreateHotSpot(revoluteJointData);
  QCOMPARE(jointMaker->JointCount(), 1u);

  // Add a prismatic joint
  jointMaker->AddJoint(gui::JointMaker::JOINT_SLIDER);
  gui::JointData *prismaticJointData =
      jointMaker->CreateJoint(sphereLink, cylinderLink);
  jointMaker->CreateHotSpot(prismaticJointData);
  QCOMPARE(jointMaker->JointCount(), 2u);

  // Add a screw joint
  jointMaker->AddJoint(gui::JointMaker::JOINT_SCREW);
  gui::JointData *screwJointData =
      jointMaker->CreateJoint(cylinderLink, boxLink);
  jointMaker->CreateHotSpot(screwJointData);
  QCOMPARE(jointMaker->JointCount(), 3u);

  // verify initial selected state
  QVERIFY(!revoluteJointData->hotspot->GetHighlighted());
  QVERIFY(!prismaticJointData->hotspot->GetHighlighted());
  QVERIFY(!screwJointData->hotspot->GetHighlighted());

  // select the joints and verify that they are selected
  jointMaker->SetSelected(revoluteJointData->hotspot, true);
  QVERIFY(revoluteJointData->hotspot->GetHighlighted());

  jointMaker->SetSelected(prismaticJointData->hotspot, true);
  QVERIFY(prismaticJointData->hotspot->GetHighlighted());

  jointMaker->SetSelected(screwJointData->hotspot, true);
  QVERIFY(screwJointData->hotspot->GetHighlighted());

  // deselect and verify
  jointMaker->SetSelected(revoluteJointData->hotspot, false);
  QVERIFY(!revoluteJointData->hotspot->GetHighlighted());

  jointMaker->SetSelected(prismaticJointData->hotspot, false);
  QVERIFY(!prismaticJointData->hotspot->GetHighlighted());

  jointMaker->SetSelected(screwJointData->hotspot, false);
  QVERIFY(!screwJointData->hotspot->GetHighlighted());

  // select one and verify all
  jointMaker->SetSelected(prismaticJointData->hotspot, true);
  QVERIFY(prismaticJointData->hotspot->GetHighlighted());
  QVERIFY(!revoluteJointData->hotspot->GetHighlighted());
  QVERIFY(!screwJointData->hotspot->GetHighlighted());

  delete jointMaker;
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void JointMaker_TEST::JointMaterial()
{
  this->Load("worlds/empty.world");

  gui::JointMaker *jointMaker = new gui::JointMaker();

  // all currently supported joint types.
  std::vector<std::string> jointTypes;
  jointTypes.push_back("revolute");
  jointTypes.push_back("revolute2");
  jointTypes.push_back("prismatic");
  jointTypes.push_back("ball");
  jointTypes.push_back("universal");
  jointTypes.push_back("screw");
  jointTypes.push_back("gearbox");

  // verify joint materials are not empty and they are all unique
  std::set<std::string> jointMaterials;
  for (auto &j : jointTypes)
  {
    std::string mat = jointMaker->JointMaterial(j);
    QVERIFY(mat != "");
    QVERIFY(jointMaterials.find(mat) == jointMaterials.end());
    jointMaterials.insert(mat);
  }
  delete jointMaker;
}

/////////////////////////////////////////////////
void JointMaker_TEST::LinkList()
{
  this->Load("worlds/empty.world");

  gui::JointMaker *jointMaker = new gui::JointMaker();
  QVERIFY(jointMaker != NULL);

  // Check there are no links in the beginning
  auto linkList = jointMaker->LinkList();
  QVERIFY(linkList.empty());

  // Send notification that a link has been inserted
  gui::model::Events::linkInserted("model::link1");
  QTest::qWait(200);

  // Check it was received
  linkList = jointMaker->LinkList();
  QVERIFY(linkList.size() == 1);
  QVERIFY(linkList.find("model::link1") != linkList.end());
  QVERIFY(linkList["model::link1"] == "link1");

  // Send notification that another link has been inserted
  gui::model::Events::linkInserted("model::link2");
  QTest::qWait(200);

  // Check it was received
  linkList = jointMaker->LinkList();
  QVERIFY(linkList.size() == 2);
  QVERIFY(linkList.find("model::link2") != linkList.end());
  QVERIFY(linkList["model::link2"] == "link2");

  // Send notification that a link has been removed
  gui::model::Events::linkRemoved("model::link1");
  QTest::qWait(200);

  // Check it was received
  linkList = jointMaker->LinkList();
  QVERIFY(linkList.size() == 1);
  QVERIFY(linkList.find("model::link1") == linkList.end());

  delete jointMaker;
}

/////////////////////////////////////////////////
void JointMaker_TEST::UpdateMsg()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

  // Create joint maker
  auto jointMaker = new gui::JointMaker();
  QVERIFY(jointMaker != NULL);
  QCOMPARE(jointMaker->JointCount(), 0u);

  // Create main window
  auto mainWindow = new gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  auto cam = gui::get_active_camera();
  Q_ASSERT(cam);
  auto scene = cam->GetScene();
  Q_ASSERT(scene);

  auto boxLink = scene->GetVisual("box::link");
  auto sphereLink = scene->GetVisual("sphere::link");

  Q_ASSERT(boxLink.get());
  Q_ASSERT(sphereLink.get());

  // Add a revolute joint
  jointMaker->AddJoint(gui::JointMaker::JOINT_HINGE);
  auto jointData = jointMaker->CreateJoint(boxLink, sphereLink);
  jointMaker->CreateHotSpot(jointData);
  QCOMPARE(jointMaker->JointCount(), 1u);

  // Check data was properly generated
  auto name1 = jointData->name;
  QVERIFY(name1 == "link_JOINT_0");

  auto type1 = jointData->type;
  QCOMPARE(type1, gui::JointMaker::JOINT_HINGE);

  auto msg1 = jointData->jointMsg;
  QVERIFY(msg1 != NULL);
  QVERIFY(msg1->name() == name1);
  QVERIFY(gui::JointMaker::ConvertJointType(
      msgs::ConvertJointType(msg1->type())) == type1);
  QVERIFY(msg1->has_axis1());
  QVERIFY(!msg1->has_axis2());
  QCOMPARE(msgs::ConvertIgn(msg1->pose()), ignition::math::Pose3d::Zero);

  auto parentVis = jointData->parent;
  QVERIFY(parentVis != NULL);

  auto childVis = jointData->child;
  QVERIFY(childVis != NULL);

  auto hotspot = jointData->hotspot;
  QVERIFY(hotspot != NULL);

  auto visual = jointData->visual;
  QVERIFY(visual != NULL);

  auto line = jointData->line;
  QVERIFY(line != NULL);

  auto handles = jointData->handles;
  QVERIFY(handles != NULL);

  auto inspector = jointData->inspector;
  QVERIFY(inspector != NULL);

  QVERIFY(jointData->dirty);

  // Check there's no joint visual until update
  auto jointVisual = jointData->jointVisual;
  QVERIFY(jointVisual == NULL);

  jointData->Update();

  QVERIFY(!jointData->dirty);

  jointVisual = jointData->jointVisual;
  QVERIFY(jointVisual != NULL);

  // Change data and update - 1 axis -> 2 axes
  std::string name2 = "new_name";
  auto type2 = gui::JointMaker::JOINT_UNIVERSAL;
  auto pose2 = ignition::math::Pose3d(1, 2, -3, 0, -0.2, 1);

  jointData->name = name2;
  jointData->type = type2;
  msgs::Set(jointData->jointMsg->mutable_pose(), pose2);

  jointData->Update();

  // Verify changes
  auto msg2 = jointData->jointMsg;
  QVERIFY(msg1 != msg2);

  QVERIFY(msg1->name() != msg2->name());
  QVERIFY(name2 == msg2->name());

  QVERIFY(msg1->type() != msg2->type());
  QVERIFY(gui::JointMaker::ConvertJointType(
      msgs::ConvertJointType(msg2->type())) == type2);
  QVERIFY(msg2->has_axis1());
  QVERIFY(msg2->has_axis2());
  QCOMPARE(msgs::ConvertIgn(msg2->pose()), pose2);

  // Change data and update - 2 axes -> 0 axes
  std::string name3 = "new_name2";
  gui::JointMaker::JointType type3 = gui::JointMaker::JOINT_BALL;
  auto pose3 = ignition::math::Pose3d(4, -5, 6, -0.1, 0, 0);

  jointData->name = name3;
  jointData->type = type3;
  msgs::Set(jointData->jointMsg->mutable_pose(), pose3);

  jointData->Update();

  // Verify changes
  auto msg3 = jointData->jointMsg;
  QVERIFY(msg2 != msg3);

  QVERIFY(msg2->name() != msg3->name());
  QVERIFY(name3 == msg3->name());

  QVERIFY(msg2->type() != msg3->type());
  QVERIFY(gui::JointMaker::ConvertJointType(
      msgs::ConvertJointType(msg3->type())) == type3);
  QVERIFY(!msg3->has_axis1());
  QVERIFY(!msg3->has_axis2());
  QCOMPARE(msgs::ConvertIgn(msg3->pose()), pose3);

  // Change data and update - 0 axes -> 1 axis
  std::string name4 = "new_name3";
  gui::JointMaker::JointType type4 = gui::JointMaker::JOINT_SLIDER;
  auto pose4 = ignition::math::Pose3d(-2, 0, 3, 0.4, 1, 0.5);

  jointData->name = name4;
  jointData->type = type4;
  msgs::Set(jointData->jointMsg->mutable_pose(), pose4);

  jointData->Update();

  // Verify changes
  auto msg4 = jointData->jointMsg;
  QVERIFY(msg3 != msg4);

  QVERIFY(msg3->name() != msg4->name());
  QVERIFY(name4 == msg4->name());

  QVERIFY(msg3->type() != msg4->type());
  QVERIFY(gui::JointMaker::ConvertJointType(
      msgs::ConvertJointType(msg4->type())) == type4);
  QVERIFY(msg4->has_axis1());
  QVERIFY(!msg4->has_axis2());
  QCOMPARE(msgs::ConvertIgn(msg4->pose()), pose4);

  delete jointMaker;
  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(JointMaker_TEST)
