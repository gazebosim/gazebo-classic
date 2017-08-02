/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#include "gazebo/gui/JointControlWidget.hh"
#include "gazebo/gui/MainWindow.hh"
#include "joint_control_widget.hh"

#include "test_config.h"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
void JointControlWidgetTest::SetGetParameters()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/nested_joint_test.world", true, false, false);

  // Create the main window.
  MainWindow *mainWindow = new MainWindow();
  QVERIFY(mainWindow != NULL);

  JointControlWidget *jointControlWidget =
      mainWindow->findChild<JointControlWidget *>();
  QVERIFY(jointControlWidget != NULL);

  // Select a model and get the joint control sliders
  jointControlWidget->SetModelName("model_1::model_2");
  this->ProcessEventsAndDraw(mainWindow);

  JointForceControl *slider =
      jointControlWidget->findChild<JointForceControl *>();
  QVERIFY(slider != NULL);

  JointPIDPosControl *pidPosSlider =
      jointControlWidget->findChild<JointPIDPosControl *>();
  QVERIFY(pidPosSlider != NULL);

  JointPIDVelControl *pidVelSlider =
      jointControlWidget->findChild<JointPIDVelControl *>();
  QVERIFY(pidVelSlider != NULL);

  // Set the joint control parameters
  slider->SetForce(3.0);

  pidPosSlider->SetPositionTarget(12.3);
  pidPosSlider->SetPGain(4.1);
  pidPosSlider->SetIGain(1.1);
  pidPosSlider->SetDGain(9.1);

  pidVelSlider->SetVelocityTarget(3.21);
  pidVelSlider->SetPGain(4.2);
  pidVelSlider->SetIGain(1.2);
  pidVelSlider->SetDGain(9.2);

  // Switch to a different model to forget the parameters
  jointControlWidget->SetModelName("ground_plane");
  this->ProcessEventsAndDraw(mainWindow);

  // Switch back to the first model
  jointControlWidget->SetModelName("model_1::model_2");
  this->ProcessEventsAndDraw(mainWindow);

  QList<QDoubleSpinBox *> spinBoxes =
      jointControlWidget->findChildren<QDoubleSpinBox *>();
  QVERIFY(spinBoxes.size() == 9u);

  // Check that the control parameters have been reloaded
  std::list<double> pidParams = {3.0, 12.3, 4.1, 1.1, 9.1, 3.21, 4.2, 1.2, 9.2};
  for (auto spinBox : spinBoxes)
  {
    pidParams.remove(spinBox->value());
  }
  QVERIFY(pidParams.size() == 0u);

  // Done
  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(JointControlWidgetTest)
