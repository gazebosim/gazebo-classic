/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include <ignition/math/Helpers.hh>

#include "gazebo/rendering/RayQuery.hh"
#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GLWidget.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/ModelSnap.hh"
#include "gazebo/gui/ModelSnap_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
bool FindRedColor(gazebo::rendering::CameraPtr _cam)
{
  // Get camera data
  const unsigned char *data = _cam->ImageData();
  unsigned int width = _cam->ImageWidth();
  unsigned int height = _cam->ImageHeight();
  unsigned int depth = _cam->ImageDepth();

  // scan image and find red pixels
  for (unsigned int y = 0; y < height; ++y)
  {
    for (unsigned int x = 0; x < width*depth; x += depth)
    {
      int r = data[y*width*depth + x];
      int g = data[y*width*depth + x+1];
      int b = data[y*width*depth + x+2];

      if (r > g && r > b && r > 200 && g < 200 && b < 200)
        return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
void ModelSnap_TEST::Highlight()
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

  std::string model02Name = "box";
  std::string model03Name = "sphere";

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != NULL);

  // set cam pose so it doesn't see the red line representing the world x axis.
  cam->SetWorldPose(ignition::math::Pose3d(
      ignition::math::Vector3d(2.18, 0.65, 1.06),
      ignition::math::Quaterniond(0, 0.14, -3.14)));

  cam->SetCaptureData(true);

  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(!FindRedColor(cam));

  gazebo::rendering::VisualPtr model02Vis = scene->GetVisual(model02Name);
  QVERIFY(model02Vis != NULL);
  gazebo::rendering::VisualPtr model03Vis = scene->GetVisual(model03Name);
  QVERIFY(model03Vis != NULL);

  auto glWidget = mainWindow->findChild<gazebo::gui::GLWidget *>("GLWidget");
  QVERIFY(glWidget != NULL);

  QVERIFY(gazebo::gui::g_snapAct != NULL);
  QVERIFY(gazebo::gui::g_arrowAct != NULL);

  gazebo::gui::ModelSnap::Instance()->Init();
  gazebo::gui::ModelSnap::Instance()->Reset();

  // start with arrow mode
  gazebo::gui::g_arrowAct->trigger();

  // trigger snap
  gazebo::gui::g_snapAct->trigger();

  // select any triangle on the sphere
  ignition::math::Vector2i spherePt =
      cam->Project(model03Vis->GetWorldPose().pos.Ign() +
      ignition::math::Vector3d(0.5, 0, 0));

  QTest::mouseRelease(glWidget, Qt::LeftButton, 0,
      QPoint(spherePt.X(), spherePt.Y()));

  this->ProcessEventsAndDraw(mainWindow);

  // verify that a triangle is highlighted by checking for red pixels.
  QVERIFY(FindRedColor(cam));

  // try the box model next
  // verify we can repeatedly pick triangles on the same model.
  for (unsigned int k = 0; k < 2; ++k)
  {
    // cancel and reset snap
    gazebo::gui::g_arrowAct->trigger();

    this->ProcessEventsAndDraw(mainWindow);

    // verify that no triangles are highlighted
    QVERIFY(!FindRedColor(cam));

    // trigger snap again
    gazebo::gui::g_snapAct->trigger();

    // select the front face of the box
    ignition::math::Vector2i boxPt =
        cam->Project(model02Vis->GetWorldPose().pos.Ign() +
        ignition::math::Vector3d(0.5, 0, 0));

    QTest::mouseRelease(glWidget, Qt::LeftButton, 0,
         QPoint(boxPt.X(), boxPt.Y()));

    this->ProcessEventsAndDraw(mainWindow);

    // verify that a triangle is highlighted
    QVERIFY(FindRedColor(cam));
  }

  gazebo::gui::ModelSnap::Instance()->Fini();

  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void ModelSnap_TEST::Snap()
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

  gazebo::rendering::Events::createScene("default");

  this->ProcessEventsAndDraw(mainWindow);

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
  ignition::math::Vector2i srcPt =
      cam->Project(model03Vis->GetWorldPose().pos.Ign() +
      ignition::math::Vector3d(0.5, 0, 0));
  ignition::math::Vector3d intersect;
  ignition::math::Triangle3d triangleSrc;
  rayQuery.SelectMeshTriangle(srcPt.X(), srcPt.Y(), model03Vis, intersect,
      triangleSrc);

  // select the front face of the box
  ignition::math::Vector2i destPt =
      cam->Project(model02Vis->GetWorldPose().pos.Ign() +
      ignition::math::Vector3d(0.5, 0, 0));
  ignition::math::Triangle3d triangleDest;
  rayQuery.SelectMeshTriangle(destPt.X(), destPt.Y(), model02Vis, intersect,
      triangleDest);

  this->ProcessEventsAndDraw(mainWindow);

  // Snap the sphere to the front face of the box.
  gazebo::gui::ModelSnap::Instance()->Snap(
      triangleSrc, triangleDest, model03Vis);

  this->ProcessEventsAndDraw(mainWindow);

  double xDiff = 0;
  double yDiff = 0;
  double zDiff = 0;

  // The sphere should now be in front of the box
  // Given that they are both unit shapes, the x value sphere will roughly be
  // within the box's x + 1.0.
  // The tolerance is higher as we did not select a triangle
  // that lies exactly [0.5, 0 , 0] from the center of the sphere.
  QVERIFY(ignition::math::equal(model03Vis->GetWorldPose().pos.x,
      model02Vis->GetWorldPose().pos.x + 1.0, 1e-2));

  // The y and z pos of the sphere should be within the y and z bounds of the
  // box.
  yDiff = model03Vis->GetWorldPose().pos.y - model02Vis->GetWorldPose().pos.y;
  zDiff = model03Vis->GetWorldPose().pos.z - model02Vis->GetWorldPose().pos.z;
  QVERIFY(fabs(yDiff) <= 0.5);
  QVERIFY(fabs(zDiff) <= 0.5);

  gazebo::gui::ModelSnap::Instance()->Reset();

  // select the spherical face of the cylinder
  ignition::math::Vector2i srcPt2 =
      cam->Project(model01Vis->GetWorldPose().pos.Ign() +
      ignition::math::Vector3d(0.5, 0, 0.0));
  triangleSrc.Set(ignition::math::Vector3d::Zero,
                  ignition::math::Vector3d::Zero,
                  ignition::math::Vector3d::Zero);
  rayQuery.SelectMeshTriangle(srcPt2.X(), srcPt2.Y(), model01Vis, intersect,
      triangleSrc);

  // select the top face of the box
  ignition::math::Vector2i destPt2 =
      cam->Project(model02Vis->GetWorldPose().pos.Ign() +
      ignition::math::Vector3d(0.0, 0, 0.5));
  triangleDest.Set(ignition::math::Vector3d::Zero,
                  ignition::math::Vector3d::Zero,
                  ignition::math::Vector3d::Zero);
  rayQuery.SelectMeshTriangle(destPt2.X(), destPt2.Y(), model02Vis, intersect,
      triangleDest);

  // Snap the cylinder to the top of the box.
  gazebo::gui::ModelSnap::Instance()->Snap(
      triangleSrc, triangleDest, model01Vis);

  // The cylinder should now be on top of the box
  // Given that they are both unit shapes, the height of the cylinder will now
  // be 1.0 + 0.5 = 1.5.
  QVERIFY(ignition::math::equal(model01Vis->GetWorldPose().pos.z, 1.5));

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
