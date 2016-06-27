/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#include "gazebo/gui/GLWidget.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/model/ModelCreator.hh"

#include "view_control.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void MouseZoom(QWidget *_widget)
{
  // There seem to be a problem simulating mouse drag using QTest
  // so use raw QMouseEvent.

  // move the mouse cursor to the center of the widget
  QPoint midPt(_widget->width()*0.5, _widget->height()*0.5);
  QMouseEvent *pressEvent = new QMouseEvent(QEvent::MouseButtonPress, midPt,
      _widget->mapToGlobal(midPt), Qt::RightButton, Qt::RightButton,
      Qt::NoModifier);
  QApplication::postEvent(_widget, pressEvent);

  // move the cursor in y for a quarter of the widget height
  double destY = 0.25;
  unsigned int steps = 10;
  for (unsigned int i = 0; i < steps; ++i)
  {
    // compute the next y pos to move the mouse cursor to.
    double dy = 0.5 - (destY * (1.0 / steps) * (i+1));

    QPoint movePt(_widget->width()*0.5, _widget->height()*dy);
    QMouseEvent *moveEvent = new QMouseEvent(QEvent::MouseMove, movePt,
        _widget->mapToGlobal(movePt), Qt::RightButton, Qt::RightButton,
        Qt::NoModifier);
    QApplication::postEvent(_widget, moveEvent);

    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
  }

  QPoint releasePt(_widget->width()*0.5, _widget->height()*destY);
  QMouseEvent *releaseEvent = new QMouseEvent(QEvent::MouseButtonRelease,
      releasePt, _widget->mapToGlobal(releasePt), Qt::RightButton,
      Qt::RightButton, Qt::NoModifier);
  QApplication::postEvent(_widget, releaseEvent);
}

/////////////////////////////////////////////////
void ViewControlTest::MouseZoomSimulation()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != nullptr);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  // Process some events and draw the screen
  this->ProcessEventsAndDraw(mainWindow);

  // Get camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != nullptr);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != nullptr);

  // Get box visual and position it
  gazebo::rendering::VisualPtr boxVis = scene->GetVisual("box");
  QVERIFY(boxVis != nullptr);

  cam->SetWorldPose(ignition::math::Pose3d(
      ignition::math::Vector3d(-1.5, 0.0, 0.5),
      ignition::math::Quaterniond(0, 0, 0)));

  // Process some events and draw the screen
  this->ProcessEventsAndDraw(mainWindow);

  // make sure the box is visible
  QVERIFY(cam->IsVisible(boxVis));

  // Get the glwidget
  auto glWidget = mainWindow->findChild<gazebo::gui::GLWidget *>("GLWidget");
  QVERIFY(glWidget != nullptr);

  // Zoom in on the model.
  MouseZoom(glWidget);

  // Process some events and draw the screen
  this->ProcessEventsAndDraw(mainWindow);

  // verify the camera did not zoom past the model
  QVERIFY(cam->IsVisible(boxVis));

  // Clean up
  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void ViewControlTest::MouseZoomModelEditor()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, false);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != nullptr);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  // Process some events and draw the screen
  this->ProcessEventsAndDraw(mainWindow);

  // Get camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != nullptr);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != nullptr);

  // Process some events and draw the screen
  this->ProcessEventsAndDraw(mainWindow);

  // create the model editor
  gazebo::gui::ModelCreator *modelCreator = new gazebo::gui::ModelCreator();

  // Inserting a link
  modelCreator->AddShape(gazebo::gui::ModelCreator::ENTITY_CYLINDER);
  gazebo::rendering::VisualPtr cylinder =
      scene->GetVisual("ModelPreview_0_0::link_0");
  QVERIFY(cylinder != nullptr);

  // Process some events and draw the screen
  this->ProcessEventsAndDraw(mainWindow);

  cam->SetWorldPose(ignition::math::Pose3d(
      ignition::math::Vector3d(-1.5, 0.0, 0.5),
      ignition::math::Quaterniond(0, 0, 0)));

  // Process some events and draw the screen
  this->ProcessEventsAndDraw(mainWindow);

  // make sure the cylinder is visible
  QVERIFY(cam->IsVisible(cylinder));

  // Get the glwidget
  auto glWidget = mainWindow->findChild<gazebo::gui::GLWidget *>("GLWidget");
  QVERIFY(glWidget != nullptr);

  // Zoom in on the link.
  MouseZoom(glWidget);

  // Process some events and draw the screen
  this->ProcessEventsAndDraw(mainWindow);

  // verify the camera did not zoom past the link
  QVERIFY(cam->IsVisible(cylinder));

  // Clean up
  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(ViewControlTest)
