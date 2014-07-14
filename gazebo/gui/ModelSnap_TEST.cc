/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

  // set-up the ray query
  gazebo::rendering::RayQuery rayQuery(cam);

  // select any triangle on the sphere
  gazebo::math::Vector3 intersect;
  std::vector<gazebo::math::Vector3> verticesSrc;
  rayQuery.SelectMeshTriangle(1430, 485, model03Vis, intersect, verticesSrc);

  // select the front face of the box
  std::vector<gazebo::math::Vector3> verticesDest;
  rayQuery.SelectMeshTriangle(1230, 535, model02Vis, intersect, verticesDest);

  // Snap the sphere to the front face of the box.
  gazebo::gui::ModelSnap::Instance()->Snap(
      verticesSrc, verticesDest, model03Vis);

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  double xDiff = 0;
  double yDiff = 0;
  double zDiff = 0;

  // The sphere should now be in front of the box
  // Given that they are both unit shapes, the x value sphere will roughly be
  // within the box's x + 1.0.
  // The tolerance is higher as we did not select a triangle
  // that lies exactly [0.5, 0 , 0] from the center of the sphere.
  QVERIFY(gazebo::math::equal(model03Vis->GetWorldPose().pos.x,
      model02Vis->GetWorldPose().pos.x + 1.0, 1e-2));

  // The y and z pos of the sphere should be within the y and z bounds of the
  // box.
  yDiff = model03Vis->GetWorldPose().pos.y - model02Vis->GetWorldPose().pos.y;
  zDiff = model03Vis->GetWorldPose().pos.z - model02Vis->GetWorldPose().pos.z;
  QVERIFY(fabs(yDiff) <= 0.5);
  QVERIFY(fabs(zDiff) <= 0.5);


  gazebo::gui::ModelSnap::Instance()->Reset();

  // select the spherical face of the cylinder
  verticesSrc.clear();
  rayQuery.SelectMeshTriangle(930, 630, model01Vis, intersect, verticesSrc);

  // select the top face of the box
  verticesDest.clear();
  rayQuery.SelectMeshTriangle(1145, 395, model02Vis, intersect, verticesDest);

  // Snap the cylinder to the top of the box.
  gazebo::gui::ModelSnap::Instance()->Snap(
      verticesSrc, verticesDest, model01Vis);

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // The cylinder should now be on top of the box
  // Given that they are both unit shapes, the height of the cylinder will now
  // be 1.0 + 0.5 = 1.5.
  QVERIFY(gazebo::math::equal(model01Vis->GetWorldPose().pos.z, 1.5));

  // The x and y pos of the cyinder should be within the x and y bounds of the
  // box.
  xDiff = model01Vis->GetWorldPose().pos.x - model02Vis->GetWorldPose().pos.x;
  yDiff = model01Vis->GetWorldPose().pos.y - model02Vis->GetWorldPose().pos.y;
  QVERIFY(fabs(xDiff) <= 0.5);
  QVERIFY(fabs(yDiff) <= 0.5);

  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(ModelSnap_TEST)
