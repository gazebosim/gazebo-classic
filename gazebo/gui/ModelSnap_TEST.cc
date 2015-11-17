/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include "gazebo/rendering/RayQuery.hh"
#include "gazebo/math/Helpers.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/ModelSnap.hh"
#include "gazebo/gui/ModelSnap_TEST.hh"

#include "test_config.h"


/////////////////////////////////////////////////
gazebo::math::Vector2i GetScreenSpaceCoords(gazebo::math::Vector3 _pt,
    gazebo::rendering::CameraPtr _cam)
{
  // Convert from 3D world pos to 2D screen pos
  Ogre::Vector3 pos = _cam->GetOgreCamera()->getProjectionMatrix() *
      _cam->GetOgreCamera()->getViewMatrix() *
      gazebo::rendering::Conversions::Convert(_pt);

  gazebo::math::Vector2i screenPos;
  screenPos.x = ((pos.x / 2.0) + 0.5) * _cam->GetViewportWidth();
  screenPos.y = (1 - ((pos.y / 2.0) + 0.5)) * _cam->GetViewportHeight();

  return screenPos;
}

/////////////////////////////////////////////////
void ModelSnap_TEST::Snap()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, true);

  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  // Create the main window.
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  std::string model01Name = "cylinder";
  std::string model02Name = "box";
  std::string model03Name = "sphere";

  gazebo::rendering::Events::createScene("default");

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

  gazebo::rendering::VisualPtr model01Vis = scene->GetVisual(model01Name);
  QVERIFY(model01Vis != NULL);
  gazebo::rendering::VisualPtr model02Vis = scene->GetVisual(model02Name);
  QVERIFY(model02Vis != NULL);
  gazebo::rendering::VisualPtr model03Vis = scene->GetVisual(model03Name);
  QVERIFY(model03Vis != NULL);

  this->SetPause(true);

  // set-up the ray query
  gazebo::rendering::RayQuery rayQuery(cam);

  // select any triangle on the sphere
  gazebo::math::Vector2i srcPt = GetScreenSpaceCoords(
      model03Vis->WorldPose().Pos() + ignition::math::Vector3d(0.5, 0, 0), cam);
  gazebo::math::Vector3 intersect;
  std::vector<gazebo::math::Vector3> verticesSrc;
  rayQuery.SelectMeshTriangle(srcPt.x, srcPt.y, model03Vis, intersect,
      verticesSrc);

  // select the front face of the box
  gazebo::math::Vector2i destPt = GetScreenSpaceCoords(
      model02Vis->WorldPose().Pos() + ignition::math::Vector3d(0.5, 0, 0), cam);
  std::vector<gazebo::math::Vector3> verticesDest;
  rayQuery.SelectMeshTriangle(destPt.x, destPt.y, model02Vis, intersect,
      verticesDest);

  // Snap the sphere to the front face of the box.
  gazebo::gui::ModelSnap::Instance()->Snap(
      verticesSrc, verticesDest, model03Vis);

  double xDiff = 0;
  double yDiff = 0;
  double zDiff = 0;

  // The sphere should now be in front of the box
  // Given that they are both unit shapes, the x value sphere will roughly be
  // within the box's x + 1.0.
  // The tolerance is higher as we did not select a triangle
  // that lies exactly [0.5, 0 , 0] from the center of the sphere.
  QVERIFY(ignition::math::equal(model03Vis->WorldPose().Pos().X(),
      model02Vis->WorldPose().Pos().X() + 1.0, 1e-2));

  // The y and z pos of the sphere should be within the y and z bounds of the
  // box.
  yDiff = model03Vis->WorldPose().Pos().Y() - model02Vis->WorldPose().Pos().Y();
  zDiff = model03Vis->WorldPose().Pos().Z() - model02Vis->WorldPose().Pos().Z();
  QVERIFY(fabs(yDiff) <= 0.5);
  QVERIFY(fabs(zDiff) <= 0.5);

  gazebo::gui::ModelSnap::Instance()->Reset();

  // select the spherical face of the cylinder
  gazebo::math::Vector2i srcPt2 = GetScreenSpaceCoords(
      model01Vis->WorldPose().Pos() +
      ignition::math::Vector3d(0.5, 0, 0.0), cam);
  verticesSrc.clear();
  rayQuery.SelectMeshTriangle(srcPt2.x, srcPt2.y, model01Vis, intersect,
      verticesSrc);

  // select the top face of the box
  gazebo::math::Vector2i destPt2 = GetScreenSpaceCoords(
      model02Vis->WorldPose().Pos() +
      ignition::math::Vector3d(0.0, 0, 0.5), cam);
  verticesDest.clear();
  rayQuery.SelectMeshTriangle(destPt2.x, destPt2.y, model02Vis, intersect,
      verticesDest);

  // Snap the cylinder to the top of the box.
  gazebo::gui::ModelSnap::Instance()->Snap(
      verticesSrc, verticesDest, model01Vis);

  // The cylinder should now be on top of the box
  // Given that they are both unit shapes, the height of the cylinder will now
  // be 1.0 + 0.5 = 1.5.
  QVERIFY(ignition::math::equal(model01Vis->WorldPose().Pos().Z(), 1.5));

  // The x and y pos of the cyinder should be within the x and y bounds of the
  // box.
  xDiff = model01Vis->WorldPose().Pos().X() - model02Vis->WorldPose().Pos().X();
  yDiff = model01Vis->WorldPose().Pos().Y() - model02Vis->WorldPose().Pos().Y();
  QVERIFY(fabs(xDiff) <= 0.5);
  QVERIFY(fabs(yDiff) <= 0.5);

  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(ModelSnap_TEST)
