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

#include <ignition/math/Vector2.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/Mesh.hh"
#include "gazebo/common/MeshManager.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GLWidget.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"

#include "gazebo/gui/model/ModelEditorPalette.hh"
#include "gazebo/gui/model/ModelCreator.hh"

#include "gazebo/rendering/RayQuery.hh"

#include "mouse_pick.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void MouseDrag(QWidget *_widget, Qt::MouseButton _button,
    const ignition::math::Vector2d &_start,
    const ignition::math::Vector2d &_end)
{
  // move the mouse cursor to the _start pos
  QTest::mouseMove(_widget,
      QPoint(_widget->width()*_start.X(), _widget->height()*_start.Y()));

  // There seem to be a problem simulating mouse drag using QTest
  // so use raw QMouseEvent.
  QPoint startPt(_widget->width()*_start.X(), _widget->height()*_start.Y());
  QMouseEvent *pressEvent = new QMouseEvent(QEvent::MouseButtonPress, startPt,
      _widget->mapToGlobal(startPt), _button, _button, Qt::NoModifier);
  QApplication::postEvent(_widget, pressEvent);

  ignition::math::Vector2d dist = _end - _start;

  // move the cursor to _end pos over 10 steps
  unsigned int steps = 10;
  for (unsigned int i = 0; i < steps; ++i)
  {
    // compute next pos to move the cursor to
    ignition::math::Vector2d nextPos = _start + dist * (1.0 / steps) * (i+1);

    QPoint movePt(_widget->width()*nextPos.X(), _widget->height()*nextPos.Y());

    QMouseEvent *moveEvent = new QMouseEvent(QEvent::MouseMove, movePt,
        _widget->mapToGlobal(movePt), _button, _button, Qt::NoModifier);
    QApplication::postEvent(_widget, moveEvent);

    gazebo::common::Time::MSleep(10);
    QCoreApplication::processEvents();
  }

  QPoint releasePt(_widget->width()*_end.X(), _widget->height()*_end.Y());
  QMouseEvent *releaseEvent = new QMouseEvent(QEvent::MouseButtonRelease,
      releasePt, _widget->mapToGlobal(releasePt), _button, _button,
      Qt::NoModifier);
  QApplication::postEvent(_widget, releaseEvent);
}

/////////////////////////////////////////////////
void MousePickingTest::Shapes()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  // Create the main window.
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  std::string model01Name = "cylinder";
  std::string model02Name = "box";
  std::string model03Name = "sphere";

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != NULL);

  this->ProcessEventsAndDraw(mainWindow);

  gazebo::rendering::VisualPtr model01Vis = scene->GetVisual(model01Name);
  QVERIFY(model01Vis != NULL);
  gazebo::rendering::VisualPtr model02Vis = scene->GetVisual(model02Name);
  QVERIFY(model02Vis != NULL);
  gazebo::rendering::VisualPtr model03Vis = scene->GetVisual(model03Name);
  QVERIFY(model03Vis != NULL);

  // move camera to look at the shapes from +x
  cam->SetWorldPose(ignition::math::Pose3d(
      ignition::math::Vector3d(-5, 0.0, 0.5),
      ignition::math::Quaterniond::Identity));

  auto glWidget = mainWindow->findChild<gazebo::gui::GLWidget *>("GLWidget");
  QVERIFY(glWidget != NULL);

  // mouse picking in arrow mode
  gazebo::gui::g_arrowAct->trigger();

  this->ProcessEventsAndDraw(mainWindow);

  // pick the first model - sphere
  auto pickPt = cam->Project(model01Vis->WorldPose().Pos());
  auto pt = QPoint(pickPt.X(), pickPt.Y());
  QTest::mouseMove(glWidget, pt);
  QTest::mouseClick(glWidget, Qt::LeftButton, 0, pt);

  this->ProcessEventsAndDraw(mainWindow, 10, 30);

  QVERIFY(model01Vis->GetHighlighted());
  QVERIFY(!model02Vis->GetHighlighted());
  QVERIFY(!model03Vis->GetHighlighted());

  // pick the second model - box
  pickPt = cam->Project(model02Vis->WorldPose().Pos());
  pt = QPoint(pickPt.X(), pickPt.Y());
  QTest::mouseMove(glWidget, pt);
  QTest::mouseClick(glWidget, Qt::LeftButton, 0, pt);

  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(!model01Vis->GetHighlighted());
  QVERIFY(model02Vis->GetHighlighted());
  QVERIFY(!model03Vis->GetHighlighted());

  // pick the third model - cylinder
  pickPt = cam->Project(model03Vis->WorldPose().Pos());
  pt = QPoint(pickPt.X(), pickPt.Y());
  QTest::mouseMove(glWidget, pt);
  QTest::mouseClick(glWidget, Qt::LeftButton, 0, pt);

  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(!model01Vis->GetHighlighted());
  QVERIFY(!model02Vis->GetHighlighted());
  QVERIFY(model03Vis->GetHighlighted());

  // pick near the edge of box
  pickPt = cam->Project(model02Vis->WorldPose().Pos() +
      ignition::math::Vector3d(-0.5, 0.49, 0));
  pt = QPoint(pickPt.X(), pickPt.Y());
  QTest::mouseMove(glWidget, pt);
  QTest::mouseClick(glWidget, Qt::LeftButton, 0, pt);

  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(!model01Vis->GetHighlighted());
  QVERIFY(model02Vis->GetHighlighted());
  QVERIFY(!model03Vis->GetHighlighted());

  // pick just outside the edge of box and verify nothing is selected
  pickPt = cam->Project(model02Vis->WorldPose().Pos() +
      ignition::math::Vector3d(-0.5, 0.51, 0));
  pt = QPoint(pickPt.X(), pickPt.Y());
  QTest::mouseMove(glWidget, pt);
  QTest::mouseClick(glWidget, Qt::LeftButton, 0, pt);

  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(!model01Vis->GetHighlighted());
  QVERIFY(!model02Vis->GetHighlighted());
  QVERIFY(!model03Vis->GetHighlighted());

  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void MousePickingTest::ModelEditorSelection()
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

  // Get camera
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != nullptr);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != nullptr);

  // Process some events and draw the screen
  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(gazebo::gui::g_editModelAct != NULL);

  // switch to editor mode
  gazebo::gui::g_editModelAct->toggle();

  // get the model creator
  auto modelEditorPalette =
      mainWindow->findChild<gazebo::gui::ModelEditorPalette *>();
  QVERIFY(modelEditorPalette != nullptr);
  gazebo::gui::ModelCreator *modelCreator = modelEditorPalette->ModelCreator();
  QVERIFY(modelCreator != nullptr);

  // insert a cylinder link
  modelCreator->AddShape(gazebo::gui::ModelCreator::ENTITY_CYLINDER);
  gazebo::rendering::VisualPtr cylinder =
      scene->GetVisual("ModelPreview_0::link_0");
  QVERIFY(cylinder != nullptr);

  // Process some events and draw the screen
  this->ProcessEventsAndDraw(mainWindow);

  // move camera to look at cylinder
  cam->SetWorldPose(ignition::math::Pose3d(
      ignition::math::Vector3d(-5, 0.0, 0.5),
      ignition::math::Quaterniond::Identity));

  // Process some events and draw the screen
  this->ProcessEventsAndDraw(mainWindow);

  auto glWidget = mainWindow->findChild<gazebo::gui::GLWidget *>("GLWidget");
  QVERIFY(glWidget != nullptr);

  // click on the cylinder
  QTest::mouseMove(glWidget,
      QPoint(glWidget->width()*0.5, glWidget->height()*0.5));
  QTest::mouseClick(glWidget, Qt::LeftButton, 0,
      QPoint(glWidget->width()*0.5, glWidget->height()*0.5));

  // Process some events and draw the screen
  this->ProcessEventsAndDraw(mainWindow);

  // make sure the cylinder is selected
  QVERIFY(cylinder->GetHighlighted());

  // pan the camera by dragging over the ground plane in x
  double dragHeight = 0.99;
  MouseDrag(glWidget, Qt::LeftButton,
      ignition::math::Vector2d(0.5, dragHeight),
      ignition::math::Vector2d(0.25, dragHeight));

  this->ProcessEventsAndDraw(mainWindow);

  // verify the cylinder is still selected.
  QVERIFY(cylinder->GetHighlighted());

  // Clean up
  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void MousePickingTest::Transparency()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  // Create the main window.
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  std::string model01Name = "cylinder";
  std::string model02Name = "box";
  std::string model03Name = "sphere";

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != NULL);

  cam->SetCaptureData(true);

  this->ProcessEventsAndDraw(mainWindow);

  gazebo::rendering::VisualPtr model01Vis = scene->GetVisual(model01Name);
  QVERIFY(model01Vis != NULL);
  gazebo::rendering::VisualPtr model02Vis = scene->GetVisual(model02Name);
  QVERIFY(model02Vis != NULL);
  gazebo::rendering::VisualPtr model03Vis = scene->GetVisual(model03Name);
  QVERIFY(model03Vis != NULL);

  gazebo::rendering::VisualPtr model01LinkVis =
      scene->GetVisual(model01Name + "::link");
  QVERIFY(model01LinkVis != NULL);
  gazebo::rendering::VisualPtr model02LinkVis =
      scene->GetVisual(model02Name + "::link");
  QVERIFY(model02LinkVis != NULL);
  gazebo::rendering::VisualPtr model03LinkVis =
      scene->GetVisual(model03Name + "::link");
  QVERIFY(model03LinkVis != NULL);

  model01Vis->SetTransparency(0.5);
  model02Vis->SetTransparency(0.5);
  model03Vis->SetTransparency(0.5);

  auto glWidget = mainWindow->findChild<gazebo::gui::GLWidget *>("GLWidget");
  QVERIFY(glWidget != NULL);

  // mouse picking in arrow mode
  gazebo::gui::g_arrowAct->trigger();

  cam->SetWorldPose(ignition::math::Pose3d(0, 3.0, 0.5, 0, 0, -1.57));

  this->ProcessEventsAndDraw(mainWindow);

  QTest::mouseMove(glWidget,
      QPoint(glWidget->width()*0.5, glWidget->height()*0.5));
  QTest::mouseClick(glWidget, Qt::LeftButton, 0,
      QPoint(glWidget->width()*0.5, glWidget->height()*0.5));

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  QVERIFY(!model01Vis->GetHighlighted());
  QVERIFY(!model02Vis->GetHighlighted());
  QVERIFY(model03Vis->GetHighlighted());

  cam->SetWorldPose(ignition::math::Pose3d(0, -3.0, 0.5, 0, 0, 1.57));

  this->ProcessEventsAndDraw(mainWindow);

  QTest::mouseMove(glWidget,
      QPoint(glWidget->width()*0.5, glWidget->height()*0.5));
  QTest::mouseClick(glWidget, Qt::LeftButton, 0,
      QPoint(glWidget->width()*0.5, glWidget->height()*0.5));

  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(model01Vis->GetHighlighted());
  QVERIFY(!model02Vis->GetHighlighted());
  QVERIFY(!model03Vis->GetHighlighted());

  // try mouse picking in translate mode
  gazebo::gui::g_translateAct->trigger();

  cam->SetWorldPose(ignition::math::Pose3d(0.1, 3.0, 0.6, 0, 0, -1.57));

  this->ProcessEventsAndDraw(mainWindow);

  QTest::mouseMove(glWidget,
      QPoint(glWidget->width()*0.5, glWidget->height()*0.5));
  QTest::mouseClick(glWidget, Qt::LeftButton, 0,
      QPoint(glWidget->width()*0.5, glWidget->height()*0.5));

  this->ProcessEventsAndDraw(mainWindow);

  // TODO ModelManipulator uses gui::get_entity_id to differentiate between
  // model and link but because g_main_win is not available in QTestFixture
  // the link is selected instead of the model
  QVERIFY(!model01Vis->GetHighlighted() && !model01LinkVis->GetHighlighted());
  QVERIFY(!model02Vis->GetHighlighted() && !model02LinkVis->GetHighlighted());
  QVERIFY(model03Vis->GetHighlighted() || model03LinkVis->GetHighlighted());

  cam->SetWorldPose(ignition::math::Pose3d(0.1, -3.0, 0.6, 0, 0, 1.57));

  this->ProcessEventsAndDraw(mainWindow);

  QTest::mouseMove(glWidget,
      QPoint(glWidget->width()*0.5, glWidget->height()*0.5));
  QTest::mouseClick(glWidget, Qt::LeftButton, 0,
      QPoint(glWidget->width()*0.5, glWidget->height()*0.5));

  this->ProcessEventsAndDraw(mainWindow);

  // TODO ModelManipulator uses gui::get_entity_id to differentiate between
  // model and link but because g_main_win is not available in QTestFixture
  // the link is selected instead of the model
  QVERIFY(model01Vis->GetHighlighted() || model01LinkVis->GetHighlighted());
  QVERIFY(!model02Vis->GetHighlighted() && !model02LinkVis->GetHighlighted());
  QVERIFY(!model03Vis->GetHighlighted() && !model03LinkVis->GetHighlighted());

  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void MousePickingTest::InvalidMesh()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  // Create the main window.
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  std::string modelName = "box";

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != nullptr);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != nullptr);

  this->ProcessEventsAndDraw(mainWindow);

  gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelName);
  QVERIFY(modelVis != nullptr);

  // get the box mesh
  gazebo::common::Mesh *mesh = const_cast<gazebo::common::Mesh *>(
      gazebo::common::MeshManager::Instance()->GetMesh("unit_box"));

  // create invalid submesh and add to unit_box mesh
  // submesh must have no. of indices that is a multiple of 3 (triangle)
  gazebo::common::SubMesh *subMesh = new gazebo::common::SubMesh();
  subMesh->SetName("invalid");
  subMesh->SetPrimitiveType(gazebo::common::SubMesh::TRIANGLES);
  subMesh->AddIndex(0);
  subMesh->AddIndex(1);
  subMesh->AddIndex(2);
  subMesh->AddIndex(3);
  subMesh->AddVertex(ignition::math::Vector3d::Zero);
  subMesh->AddVertex(ignition::math::Vector3d::Zero);
  subMesh->AddVertex(ignition::math::Vector3d::Zero);
  subMesh->AddVertex(ignition::math::Vector3d::Zero);
  subMesh->AddNormal(ignition::math::Vector3d::UnitX);
  subMesh->AddNormal(ignition::math::Vector3d::UnitX);
  subMesh->AddNormal(ignition::math::Vector3d::UnitX);
  subMesh->AddNormal(ignition::math::Vector3d::UnitX);
  mesh->AddSubMesh(subMesh);

  // move camera to look at the shapes from +x
  cam->SetWorldPose(ignition::math::Pose3d(
      ignition::math::Vector3d(-5, 0.0, 0.5),
      ignition::math::Quaterniond(0, 0, 0)));

  this->ProcessEventsAndDraw(mainWindow);

  // pick the box
  auto pickPt = cam->Project(modelVis->WorldPose().Pos());

  gazebo::rendering::RayQuery rayQuery(cam);
  ignition::math::Vector3d intersect;
  ignition::math::Triangle3d triangle;
  QVERIFY(rayQuery.SelectMeshTriangle(pickPt.X(), pickPt.Y(), modelVis,
      intersect, triangle));
  QVERIFY(triangle.Valid());

  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void MousePickingTest::DistantMovement()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

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

  // move camera to a distance far away from origin
  ignition::math::Pose3d camPose(
      ignition::math::Vector3d(-160000, 0.0, 0.0),
      ignition::math::Quaterniond(0, 0, 0));
  cam->SetWorldPose(camPose);

  QCOMPARE(cam->WorldPose(), camPose);

  // set a large clip distance
  double nearClip = 0.125;
  double farClip = 200000.0;
  cam->SetClipDist(nearClip, farClip);

  QVERIFY(qFuzzyCompare(cam->NearClip(), nearClip));
  QVERIFY(qFuzzyCompare(cam->FarClip(), farClip));

  this->ProcessEventsAndDraw(mainWindow);

  auto glWidget = mainWindow->findChild<gazebo::gui::GLWidget *>("GLWidget");
  QVERIFY(glWidget != nullptr);

  // test orbiting
  MouseDrag(glWidget, Qt::MiddleButton,
      ignition::math::Vector2d(0.75, 0.5),
      ignition::math::Vector2d(0.25, 0.5));

  this->ProcessEventsAndDraw(mainWindow);

  // test panning
  MouseDrag(glWidget, Qt::LeftButton,
      ignition::math::Vector2d(0.5, 0.75),
      ignition::math::Vector2d(0.5, 0.25));

  QVERIFY(cam->WorldPose() != camPose);

  this->ProcessEventsAndDraw(mainWindow);

  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(MousePickingTest)
