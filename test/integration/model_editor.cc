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

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/model/JointMaker.hh"
#include "gazebo/gui/model/ModelCreator.hh"

#include "model_editor.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void ModelEditorTest::SaveModelPose()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, false);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != NULL);

  // Start never saved
  gazebo::gui::ModelCreator *modelCreator = new gazebo::gui::ModelCreator();

  // Inserting a link and it still is never saved
  modelCreator->AddShape(gazebo::gui::ModelCreator::ENTITY_CYLINDER);
  gazebo::rendering::VisualPtr cylinder =
      scene->GetVisual("ModelPreview_0_0::link_0");
  QVERIFY(cylinder != NULL);

  // Move cylinder to new pose
  ignition::math::Pose3d cylinderPose(1, 1, 2, 4, 5, 6);
  cylinder->SetWorldPose(cylinderPose);
  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }
  QVERIFY(cylinder->GetWorldPose() == cylinderPose);

  // Insert another link
  modelCreator->AddShape(gazebo::gui::ModelCreator::ENTITY_BOX);
  gazebo::rendering::VisualPtr box =
      scene->GetVisual("ModelPreview_0_0::link_1");
  QVERIFY(box != NULL);

  // Move box to new pose
  ignition::math::Pose3d boxPose(2, 1, 0, 0, 0, 0);
  box->SetWorldPose(boxPose);
  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }
  QVERIFY(box->GetWorldPose() == boxPose);

  // Add a revolute joint
  gazebo::gui::JointMaker *jointMaker = modelCreator->GetJointMaker();
  QVERIFY(jointMaker != NULL);
  jointMaker->AddJoint(gazebo::gui::JointMaker::JOINT_HINGE);
  auto jointData = jointMaker->CreateJoint(cylinder, box);
  jointMaker->CreateHotSpot(jointData);
  QCOMPARE(jointMaker->JointCount(), 1u);

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }
  QVERIFY(jointData->hotspot->GetWorldPose().pos.Ign() ==
      cylinderPose.Pos() + (boxPose.Pos() - cylinderPose.Pos())*0.5);

  // Save all changes
  modelCreator->SaveModelFiles();
  QCOMPARE(modelCreator->GetCurrentSaveState(),
      gazebo::gui::ModelCreator::ALL_SAVED);

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // verify pose again
  QVERIFY(cylinder->GetWorldPose() == cylinderPose);
  QVERIFY(box->GetWorldPose() == boxPose);
  QVERIFY(jointData->hotspot->GetWorldPose().pos.Ign() ==
      cylinderPose.Pos() + (boxPose.Pos() - cylinderPose.Pos())*0.5);

  delete modelCreator;
  modelCreator = NULL;
  mainWindow->close();
  delete mainWindow;
  mainWindow = NULL;
}

// Generate a main function for the test
QTEST_MAIN(ModelEditorTest)
