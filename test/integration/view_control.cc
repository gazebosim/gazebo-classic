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

#include <memory>
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
  unsigned int steps = 30;
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
      ignition::math::Quaterniond::Identity));

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
  std::unique_ptr<gazebo::gui::ModelCreator> modelCreator(
                                              new gazebo::gui::ModelCreator());
  // Inserting a link
  modelCreator->AddShape(gazebo::gui::ModelCreator::ENTITY_CYLINDER);
  gazebo::rendering::VisualPtr cylinder =
      scene->GetVisual("ModelPreview_0_0::link_0");
  QVERIFY(cylinder != nullptr);

  // Process some events and draw the screen
  this->ProcessEventsAndDraw(mainWindow);

  cam->SetWorldPose(ignition::math::Pose3d(
      ignition::math::Vector3d(-1.5, 0.0, 0.5),
      ignition::math::Quaterniond::Identity));

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

/////////////////////////////////////////////////
void ViewControlTest::MouseZoomTerrain()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/heightmap.world", false, false, false);

  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != nullptr);
  // Create the main window.
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  std::string boxName = "box4";

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != nullptr);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != nullptr);

  this->ProcessEventsAndDraw(mainWindow);

  gazebo::rendering::VisualPtr boxVis = scene->GetVisual(boxName);
  QVERIFY(boxVis != nullptr);

  auto glWidget = mainWindow->findChild<gazebo::gui::GLWidget *>("GLWidget");
  QVERIFY(glWidget != nullptr);

  // move camera so that it faces the box but partially occluded by terrain
  cam->SetWorldPose(ignition::math::Pose3d(
      ignition::math::Vector3d(62.93, -65.14, 9.49),
      ignition::math::Quaterniond(0, -0.5, 1.57)));

  this->ProcessEventsAndDraw(mainWindow);

  // make sure the box is visible and not completely occluded
  QVERIFY(cam->IsVisible(boxVis));
  ignition::math::Pose3d boxPt =
      ignition::math::Pose3d(0.0, -0.5, 0.4, 0.0, 0.0, 0.0) +
      boxVis->WorldPose();
  auto vis = cam->Visual(cam->Project(boxPt.Pos()));
  QVERIFY(vis != nullptr);
  QCOMPARE(vis->GetRootVisual(), boxVis);

  // zoom in. It should not zoom pass the terrain.
  QTest::mouseClick(glWidget, Qt::LeftButton, 0,
      QPoint(glWidget->width()*0.5, glWidget->height()*0.5));
  MouseZoom(glWidget);

  // Process some events and draw the screen
  this->ProcessEventsAndDraw(mainWindow);

  // Make sure the box is still in the frustum but it should now be
  // completely occluded.
  QVERIFY(cam->IsVisible(boxVis));
  vis = cam->Visual(cam->Project(boxPt.Pos()));
  QVERIFY(vis == nullptr);

  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void ViewControlTest::MouseZoomBoundingBox()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  // Load a world with a large cordless drill mesh. Place a camera
  // inside its bounding box and test view control using the mouse.
  // Mouse clicks on the mesh should return valid contact points that
  // can be used as focus points for moving the camera.
  this->Load("worlds/camera_mesh_bbox.world", false, false, false);

  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != nullptr);
  // Create the main window.
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != nullptr);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != nullptr);

  this->ProcessEventsAndDraw(mainWindow);

  std::string modelName = "drill";
  gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelName);
  QVERIFY(modelVis != nullptr);

  auto glWidget = mainWindow->findChild<gazebo::gui::GLWidget *>("GLWidget");
  QVERIFY(glWidget != nullptr);

  ignition::math::Pose3d camWorldPose(
      ignition::math::Vector3d(0, 0.3, 1.0),
      ignition::math::Quaterniond(0, -1.57, 0));

  // place camera inside bounding box of cordless drill and look up
  cam->SetWorldPose(camWorldPose);

  this->ProcessEventsAndDraw(mainWindow);

  // verify valid contact point
  ignition::math::Vector3d pos;
  scene->FirstContact(cam, ignition::math::Vector2i(0, 0), pos);
  QVERIFY(pos.Z() < 1.5);

  // zoom in. It should not zoom pass the drill
  QTest::mouseClick(glWidget, Qt::LeftButton, 0,
      QPoint(glWidget->width()*0.5, glWidget->height()*0.5));
  MouseZoom(glWidget);

  // Process some events and draw the screen
  this->ProcessEventsAndDraw(mainWindow);

  // Make sure the camera is still inside bounding box of drill
  ignition::math::Vector3d camPose = cam->WorldPose().Pos();
  QVERIFY(ignition::math::equal(camPose.X(), camWorldPose.Pos().X(), 1e-3));
  QVERIFY(ignition::math::equal(camPose.Y(), camWorldPose.Pos().Y(), 1e-3));
  QVERIFY(camPose.Z() < 1.5);

  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(ViewControlTest)
