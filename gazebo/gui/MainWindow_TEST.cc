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
#include <boost/filesystem.hpp>

#include <ignition/math/Helpers.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector2.hh>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/GLWidget.hh"
#include "gazebo/gui/MainWindow_TEST.hh"

#include "test_config.h"

bool g_gotSetWireframe = false;
void OnRequest(ConstRequestPtr &_msg)
{
  if (_msg->request() == "set_wireframe")
    g_gotSetWireframe = true;
}


/////////////////////////////////////////////////
void MainWindow_TEST::MinimizeMaximize()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, false);

  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  // Create the main window.
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  // repeat minimize and maximize a couple of times
  this->ProcessEventsAndDraw(mainWindow);
  mainWindow->showMinimized();
  this->ProcessEventsAndDraw(mainWindow);
  mainWindow->showMaximized();
  this->ProcessEventsAndDraw(mainWindow);

  mainWindow->showMinimized();
  this->ProcessEventsAndDraw(mainWindow);
  mainWindow->showMaximized();
  this->ProcessEventsAndDraw(mainWindow);

  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void MainWindow_TEST::StepState()
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

  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(gazebo::gui::g_stepAct != NULL);
  QVERIFY(!gazebo::gui::g_stepAct->isEnabled());
  QVERIFY(!mainWindow->IsPaused());


  // toggle pause and play step and check if the step action is properly
  // enabled / disabled.
  mainWindow->Pause();

  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(mainWindow->IsPaused());
  QVERIFY(gazebo::gui::g_stepAct->isEnabled());

  mainWindow->Play();

  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(!mainWindow->IsPaused());
  QVERIFY(!gazebo::gui::g_stepAct->isEnabled());

  mainWindow->Pause();

  this->ProcessEventsAndDraw(mainWindow);

  QVERIFY(mainWindow->IsPaused());
  QVERIFY(gazebo::gui::g_stepAct->isEnabled());

  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void MainWindow_TEST::Selection()
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

  this->ProcessEventsAndDraw(mainWindow);

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);

  gazebo::gui::GLWidget *glWidget =
    mainWindow->findChild<gazebo::gui::GLWidget *>("GLWidget");
  QVERIFY(glWidget != NULL);

  ignition::math::Vector2i glWidgetCenter(
      glWidget->width()*0.5, glWidget->height()*0.5);

  // get model at center of window - should get the box
  gazebo::rendering::VisualPtr vis =
      cam->GetVisual(glWidgetCenter);
  QVERIFY(vis != NULL);
  QVERIFY(vis->GetRootVisual()->GetName() == "box");

  // move camera to look at the box
  ignition::math::Pose3d cameraPose(ignition::math::Vector3d(-1, 0, 0.5),
      ignition::math::Quaterniond(0, 0, 0));
  cam->SetWorldPose(cameraPose);
  QVERIFY(cam->WorldPose() == cameraPose);

  // verify we get a box
  gazebo::rendering::VisualPtr vis2 =
      cam->GetVisual(ignition::math::Vector2i(0, 0));
  QVERIFY(vis2 != NULL);
  QVERIFY(vis2->GetRootVisual()->GetName() == "box");

  // look upwards
  ignition::math::Quaterniond pitch90(ignition::math::Vector3d(0, -1.57, 0));
  cam->SetWorldRotation(pitch90);
  QVERIFY(cam->WorldRotation() == pitch90);

  // verify there is nothing in the middle of the window
  gazebo::rendering::VisualPtr vis3 = cam->GetVisual(glWidgetCenter);
  QVERIFY(vis3 == NULL);

  // reset orientation
  ignition::math::Quaterniond identityRot(ignition::math::Vector3d(0, 0, 0));
  cam->SetWorldRotation(identityRot);
  QVERIFY(cam->WorldRotation() == identityRot);

  // verify we can still get the box
  gazebo::rendering::VisualPtr vis4 =
      cam->GetVisual(ignition::math::Vector2i(0, 0));
  QVERIFY(vis4 != NULL);
  QVERIFY(vis4->GetRootVisual()->GetName() == "box");

  // hide the box
  vis4->SetVisible(false);
  gazebo::rendering::VisualPtr vis5 = cam->GetVisual(glWidgetCenter);

  // verify we don't get anything now
  QVERIFY(vis5 == NULL);

  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void MainWindow_TEST::SceneDestruction()
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

  this->ProcessEventsAndDraw(mainWindow);

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != NULL);

  cam->Fini();
  mainWindow->close();
  delete mainWindow;

  // verify that this test case has the only scene shared pointer remaining.
  QVERIFY(scene.use_count() == 1u);
}

/////////////////////////////////////////////////
void MainWindow_TEST::UserCameraFPS()
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

  this->ProcessEventsAndDraw(mainWindow);

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);

  // some machines are unable to hit the target FPS
  // sample update time and determine whether to skip FPS lower bound check
  bool skipFPSTest = false;
  gazebo::common::Time t = gazebo::common::Time::GetWallTime();
  QCoreApplication::processEvents();
  double dt = (gazebo::common::Time::GetWallTime()-t).Double();
  if (dt >= 0.01)
  {
    std::cerr << "Skipping lower bound FPS check" << std::endl;
    skipFPSTest = true;
  }
  unsigned int iterations = skipFPSTest ? 50 : 5000;
  double lowerFPSBound = skipFPSTest ? 0 : 45;

  // Wait a little bit for the average FPS to even out.
  this->ProcessEventsAndDraw(NULL, iterations, 1);

  std::cerr << "\nFPS[" << cam->AvgFPS() << "]\n" << std::endl;

  QVERIFY(cam->AvgFPS() > lowerFPSBound);
  QVERIFY(cam->AvgFPS() < 75.0);

  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void MainWindow_TEST::CopyPaste()
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

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);
  gazebo::rendering::ScenePtr scene = cam->GetScene();
  QVERIFY(scene != NULL);

  // Get GLWidget
  gazebo::gui::GLWidget *glWidget =
    mainWindow->findChild<gazebo::gui::GLWidget *>("GLWidget");
  QVERIFY(glWidget != NULL);

  // Test model copy
  {
    std::string modelName = "cylinder";

    // trigger selection to initialize wirebox's vertex buffer creation first.
    // Otherwise test segfaults later when selecting a model due to making
    // this call outside the rendering thread.
    gazebo::event::Events::setSelectedEntity(modelName, "normal");

    this->ProcessEventsAndDraw(mainWindow);

    gazebo::rendering::VisualPtr modelVis = scene->GetVisual(modelName);
    QVERIFY(modelVis != NULL);

    // Select the model
    gazebo::event::Events::setSelectedEntity(modelName, "normal");

    // Wait until the model is selected
    int sleep = 0;
    int maxSleep = 100;
    while (!modelVis->GetHighlighted() && sleep < maxSleep)
    {
      gazebo::common::Time::MSleep(30);
      sleep++;
    }
    QVERIFY(modelVis->GetHighlighted());

    this->ProcessEventsAndDraw(mainWindow);

    QVERIFY(gazebo::gui::g_copyAct != NULL);
    QVERIFY(gazebo::gui::g_pasteAct != NULL);

    // Copy the model
    QTest::keyClick(glWidget, Qt::Key_C, Qt::ControlModifier, 100);

    // Move to center of the screen
    QPoint moveTo(glWidget->width()/2, glWidget->height()/2);
    QTest::mouseMove(glWidget, moveTo, 100);

    // Paste the model
    QTest::keyClick(glWidget, Qt::Key_V, Qt::ControlModifier, 100);

    // Release and spawn the model
    QTest::mouseClick(glWidget, Qt::LeftButton, Qt::NoModifier, moveTo, 100);
    QCoreApplication::processEvents();

    // Verify there is a clone of the model
    gazebo::rendering::VisualPtr modelVisClone;
    sleep = 0;
    maxSleep = 100;
    while (!modelVisClone && sleep < maxSleep)
    {
      modelVisClone = scene->GetVisual(modelName + "_clone");
      QTest::qWait(100);
      sleep++;
    }
    QVERIFY(modelVisClone != NULL);
  }

  // Test light copy
  {
    std::string lightName = "sun";
    // Select the light
    gazebo::event::Events::setSelectedEntity(lightName, "normal");

    gazebo::rendering::VisualPtr lightVis = scene->GetVisual(lightName);
    QVERIFY(lightVis != NULL);

    // Wait until the light is selected
    int sleep = 0;
    int maxSleep = 100;
    while (!lightVis->GetHighlighted() && sleep < maxSleep)
    {
      gazebo::common::Time::MSleep(30);
      sleep++;
    }
    QVERIFY(lightVis->GetHighlighted());

    // Copy the light
    QTest::keyClick(glWidget, Qt::Key_C, Qt::ControlModifier, 500);
    QCoreApplication::processEvents();

    // Move to center of the screen
    QPoint moveTo(glWidget->width()/2, glWidget->height()/2);
    QTest::mouseMove(glWidget, moveTo, 500);
    QCoreApplication::processEvents();

    // Paste the light
    QTest::keyClick(glWidget, Qt::Key_V, Qt::ControlModifier, 500);
    QCoreApplication::processEvents();

    // Release and spawn the model
    QTest::mouseClick(glWidget, Qt::LeftButton, Qt::NoModifier, moveTo, 500);
    QCoreApplication::processEvents();

    // Verify there is a clone of the light
    gazebo::rendering::LightPtr lightClone;
    sleep = 0;
    maxSleep = 100;
    while (!lightClone && sleep < maxSleep)
    {
      lightClone = scene->GetLight(lightName + "_clone");
      QTest::qWait(30);
      sleep++;
    }
    QVERIFY(lightClone != NULL);

    lightClone.reset();
  }

  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void MainWindow_TEST::Wireframe()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  boost::filesystem::path path = TEST_PATH;
  path = path / "worlds" / "empty_dark_plane.world";
  this->Load(path.string(), false, false, false);
  gazebo::transport::NodePtr node;
  gazebo::transport::SubscriberPtr sub;

  node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init();
  sub = node->Subscribe("~/request", &OnRequest, true);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get the user camera, and tell it to save frames
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  if (!cam)
    return;

  cam->SetCaptureData(true);

  this->ProcessEventsAndDraw(mainWindow);

  // Get the image data
  const unsigned char *image = cam->ImageData();
  unsigned int height = cam->ImageHeight();
  unsigned int width = cam->ImageWidth();
  unsigned int depth = 3;

  // Calculate the average color.
  unsigned int sum = 0;
  for (unsigned int y = 0; y < height; ++y)
  {
    for (unsigned int x = 0; x < width*depth; ++x)
    {
      unsigned int a = image[(y*width*depth)+x];
      sum += a;
    }
  }
  double avgPreWireframe = static_cast<double>(sum) / (height*width*depth);

  // Trigger the wireframe request.
  gazebo::gui::g_viewWireframeAct->trigger();

  double avgPostWireframe = avgPreWireframe;

  // Redraw the screen
  for (unsigned int i = 0; i < 100 &&
      ignition::math::equal(avgPostWireframe, avgPreWireframe, 1e-3); ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();

    // Get the new image data, and calculate the new average color
    image = cam->ImageData();
    sum = 0;
    for (unsigned int y = 0; y < height; ++y)
    {
      for (unsigned int x = 0; x < width*depth; ++x)
      {
        unsigned int a = image[(y*width*depth)+x];
        sum += a;
      }
    }
    avgPostWireframe = static_cast<double>(sum) / (height*width*depth);
  }

  // Make sure the request was set.
  QVERIFY(g_gotSetWireframe);

  gzdbg << "AvgPrewireframe [" << avgPreWireframe
        << "] AvgPostWireframe[" << avgPostWireframe << "]\n";

  // Removing the grey ground plane should change the image.
  QVERIFY(!ignition::math::equal(avgPreWireframe, avgPostWireframe));

  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void MainWindow_TEST::NonDefaultWorld()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  boost::filesystem::path path = TEST_PATH;
  path = path / "worlds" / "empty_different_name.world";
  this->Load(path.string(), false, false, false);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get the user camera, and tell it to save frames
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();

  QVERIFY(cam != nullptr);

  cam->SetCaptureData(true);

  this->ProcessEventsAndDraw(mainWindow);

  // Get the image data
  const unsigned char *image = cam->ImageData();
  unsigned int height = cam->ImageHeight();
  unsigned int width = cam->ImageWidth();
  unsigned int depth = 3;

  unsigned int sum = 0;
  for (unsigned int y = 0; y < height; ++y)
  {
    for (unsigned int x = 0; x < width*depth; ++x)
    {
      unsigned int a = image[(y*width*depth)+x];
      sum += a;
    }
  }

  QVERIFY(sum > 0);

  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void MainWindow_TEST::UserCameraJoystick()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  // Create the main window.
  mainWindow->Load();

  gazebo::rendering::create_scene(
      gazebo::physics::get_world()->GetName(), false);

  mainWindow->Init();
  mainWindow->show();

  this->ProcessEventsAndDraw(mainWindow);

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);

  ignition::math::Pose3d startPose = cam->WorldPose();
  QVERIFY(startPose == ignition::math::Pose3d(5, -5, 2, 0, 0.275643, 2.35619));

  gazebo::transport::NodePtr node = gazebo::transport::NodePtr(
      new gazebo::transport::Node());
  node->Init();

  gazebo::transport::PublisherPtr joyPub =
    node->Advertise<gazebo::msgs::Joystick>("~/user_camera/joy_twist");

  // Test with just translation
  {
    gazebo::msgs::Joystick joystickMsg;

    joystickMsg.mutable_translation()->set_x(0.1);
    joystickMsg.mutable_translation()->set_y(0.2);
    joystickMsg.mutable_translation()->set_z(0.3);

    joyPub->Publish(joystickMsg);

    this->ProcessEventsAndDraw(mainWindow);

    ignition::math::Pose3d endPose = cam->WorldPose();
    QVERIFY(endPose == ignition::math::Pose3d(4.98664, -5.00091, 2.01306,
                                              0, 0.275643, 2.35619));
  }

  // Test with just rotation
  {
    gazebo::msgs::Joystick joystickMsg;

    joystickMsg.mutable_rotation()->set_x(0.0);
    joystickMsg.mutable_rotation()->set_y(0.1);
    joystickMsg.mutable_rotation()->set_z(0.2);

    joyPub->Publish(joystickMsg);

    this->ProcessEventsAndDraw(mainWindow);

    ignition::math::Pose3d endPose = cam->WorldPose();
    QVERIFY(endPose == ignition::math::Pose3d(4.98664, -5.00091, 2.01306,
                                              0, 0.276643, 2.36619));
  }

  // Test with both translation and  rotation
  {
    gazebo::msgs::Joystick joystickMsg;

    joystickMsg.mutable_translation()->set_x(1.0);
    joystickMsg.mutable_translation()->set_y(2.1);
    joystickMsg.mutable_translation()->set_z(3.2);

    joystickMsg.mutable_rotation()->set_x(1.0);
    joystickMsg.mutable_rotation()->set_y(2.1);
    joystickMsg.mutable_rotation()->set_z(3.2);

    joyPub->Publish(joystickMsg);

    this->ProcessEventsAndDraw(mainWindow);

    ignition::math::Pose3d endPose = cam->WorldPose();
    QVERIFY(endPose == ignition::math::Pose3d(4.84758, -5.01151, 2.15333,
                                              0, 0.297643, 2.52619));
  }

  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void MainWindow_TEST::ActionCreationDestruction()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, false);

  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);
  // Create the main window.
  mainWindow->Load();
  mainWindow->Init();

  QVERIFY(gazebo::gui::g_topicVisAct);

  QVERIFY(gazebo::gui::g_openAct);

  QVERIFY(gazebo::gui::g_saveAct);

  QVERIFY(gazebo::gui::g_saveAsAct);

  QVERIFY(gazebo::gui::g_saveCfgAct);

  QVERIFY(gazebo::gui::g_cloneAct);

  QVERIFY(gazebo::gui::g_aboutAct);

  QVERIFY(gazebo::gui::g_hotkeyChartAct);

  QVERIFY(gazebo::gui::g_quitAct);

  QVERIFY(gazebo::gui::g_resetModelsAct);

  QVERIFY(gazebo::gui::g_resetWorldAct);

  QVERIFY(gazebo::gui::g_editBuildingAct);

  QVERIFY(gazebo::gui::g_editTerrainAct);

  QVERIFY(gazebo::gui::g_editModelAct);

  QVERIFY(gazebo::gui::g_stepAct);

  QVERIFY(gazebo::gui::g_playAct);

  QVERIFY(gazebo::gui::g_pauseAct);

  QVERIFY(gazebo::gui::g_arrowAct);

  QVERIFY(gazebo::gui::g_translateAct);

  QVERIFY(gazebo::gui::g_rotateAct);

  QVERIFY(gazebo::gui::g_scaleAct);

  QVERIFY(gazebo::gui::g_boxCreateAct);

  QVERIFY(gazebo::gui::g_sphereCreateAct);

  QVERIFY(gazebo::gui::g_cylinderCreateAct);

  QVERIFY(gazebo::gui::g_pointLghtCreateAct);

  QVERIFY(gazebo::gui::g_spotLghtCreateAct);

  QVERIFY(gazebo::gui::g_dirLghtCreateAct);

  QVERIFY(gazebo::gui::g_resetAct);

  QVERIFY(gazebo::gui::g_showCollisionsAct);

  QVERIFY(gazebo::gui::g_showGridAct);

  QVERIFY(gazebo::gui::g_showOriginAct);

  QVERIFY(gazebo::gui::g_showLinkFrameAct);

  QVERIFY(gazebo::gui::g_showSkeletonAct);

  QVERIFY(gazebo::gui::g_transparentAct);

  QVERIFY(gazebo::gui::g_viewWireframeAct);

  QVERIFY(gazebo::gui::g_showCOMAct);

  QVERIFY(gazebo::gui::g_showInertiaAct);

  QVERIFY(gazebo::gui::g_showContactsAct);

  QVERIFY(gazebo::gui::g_showJointsAct);

  QVERIFY(gazebo::gui::g_showToolbarsAct);

  QVERIFY(gazebo::gui::g_fullScreenAct);

  QVERIFY(gazebo::gui::g_fpsAct);

  QVERIFY(gazebo::gui::g_orbitAct);

  QVERIFY(gazebo::gui::g_overlayAct);

  QVERIFY(gazebo::gui::g_viewOculusAct);

  QVERIFY(gazebo::gui::g_dataLoggerAct);

  QVERIFY(gazebo::gui::g_screenshotAct);

  QVERIFY(gazebo::gui::g_copyAct);

  QVERIFY(gazebo::gui::g_pasteAct);

  QVERIFY(gazebo::gui::g_snapAct);

  QVERIFY(gazebo::gui::g_alignAct);

  QVERIFY(gazebo::gui::g_viewAngleAct);

  QVERIFY(gazebo::gui::g_cameraOrthoAct);

  QVERIFY(gazebo::gui::g_cameraPerspectiveAct);

  QVERIFY(gazebo::gui::g_undoAct);

  QVERIFY(gazebo::gui::g_undoHistoryAct);

  QVERIFY(gazebo::gui::g_redoAct);

  QVERIFY(gazebo::gui::g_plotAct);

  QVERIFY(gazebo::gui::g_redoHistoryAct);

  mainWindow->close();
  delete mainWindow;

  QVERIFY(!gazebo::gui::g_topicVisAct);

  QVERIFY(!gazebo::gui::g_openAct);

  QVERIFY(!gazebo::gui::g_saveAct);

  QVERIFY(!gazebo::gui::g_saveAsAct);

  QVERIFY(!gazebo::gui::g_saveCfgAct);

  QVERIFY(!gazebo::gui::g_cloneAct);

  QVERIFY(!gazebo::gui::g_aboutAct);

  QVERIFY(!gazebo::gui::g_hotkeyChartAct);

  QVERIFY(!gazebo::gui::g_quitAct);

  QVERIFY(!gazebo::gui::g_resetModelsAct);

  QVERIFY(!gazebo::gui::g_resetWorldAct);

  QVERIFY(!gazebo::gui::g_editBuildingAct);

  QVERIFY(!gazebo::gui::g_editTerrainAct);

  QVERIFY(!gazebo::gui::g_editModelAct);

  QVERIFY(!gazebo::gui::g_stepAct);

  QVERIFY(!gazebo::gui::g_playAct);

  QVERIFY(!gazebo::gui::g_pauseAct);

  QVERIFY(!gazebo::gui::g_arrowAct);

  QVERIFY(!gazebo::gui::g_translateAct);

  QVERIFY(!gazebo::gui::g_rotateAct);

  QVERIFY(!gazebo::gui::g_scaleAct);

  QVERIFY(!gazebo::gui::g_boxCreateAct);

  QVERIFY(!gazebo::gui::g_sphereCreateAct);

  QVERIFY(!gazebo::gui::g_cylinderCreateAct);

  QVERIFY(!gazebo::gui::g_pointLghtCreateAct);

  QVERIFY(!gazebo::gui::g_spotLghtCreateAct);

  QVERIFY(!gazebo::gui::g_dirLghtCreateAct);

  QVERIFY(!gazebo::gui::g_resetAct);

  QVERIFY(!gazebo::gui::g_showCollisionsAct);

  QVERIFY(!gazebo::gui::g_showGridAct);

  QVERIFY(!gazebo::gui::g_showOriginAct);

  QVERIFY(!gazebo::gui::g_showLinkFrameAct);

  QVERIFY(!gazebo::gui::g_showSkeletonAct);

  QVERIFY(!gazebo::gui::g_transparentAct);

  QVERIFY(!gazebo::gui::g_viewWireframeAct);

  QVERIFY(!gazebo::gui::g_showCOMAct);

  QVERIFY(!gazebo::gui::g_showInertiaAct);

  QVERIFY(!gazebo::gui::g_showContactsAct);

  QVERIFY(!gazebo::gui::g_showJointsAct);

  QVERIFY(!gazebo::gui::g_showToolbarsAct);

  QVERIFY(!gazebo::gui::g_fullScreenAct);

  QVERIFY(!gazebo::gui::g_fpsAct);

  QVERIFY(!gazebo::gui::g_orbitAct);

  QVERIFY(!gazebo::gui::g_overlayAct);

  QVERIFY(!gazebo::gui::g_viewOculusAct);

  QVERIFY(!gazebo::gui::g_dataLoggerAct);

  QVERIFY(!gazebo::gui::g_screenshotAct);

  QVERIFY(!gazebo::gui::g_copyAct);

  QVERIFY(!gazebo::gui::g_pasteAct);

  QVERIFY(!gazebo::gui::g_snapAct);

  QVERIFY(!gazebo::gui::g_alignAct);

  QVERIFY(!gazebo::gui::g_viewAngleAct);

  QVERIFY(!gazebo::gui::g_cameraOrthoAct);

  QVERIFY(!gazebo::gui::g_cameraPerspectiveAct);

  QVERIFY(!gazebo::gui::g_undoAct);

  QVERIFY(!gazebo::gui::g_undoHistoryAct);

  QVERIFY(!gazebo::gui::g_redoAct);

  QVERIFY(!gazebo::gui::g_redoHistoryAct);

  QVERIFY(!gazebo::gui::g_plotAct);
}

/////////////////////////////////////////////////
void MainWindow_TEST::SetUserCameraPoseSDF()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/usercamera_test.world", false, false, false);

  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);

  // Create the main window.
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  // Get the user camera and scene
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);

  cam->SetCaptureData(true);

  this->ProcessEventsAndDraw(mainWindow);

  const unsigned char *data = cam->ImageData();
  unsigned int width = cam->ImageWidth();
  unsigned int height = cam->ImageHeight();
  unsigned int depth = cam->ImageDepth();

  // Part 1 : The user camera should be positioned so that it sees only
  // a white box
  {
    int blackCount = 0;

    // Get the number of black pixels
    for (unsigned int y = 0; y < height; ++y)
    {
      for (unsigned int x = 0; x < width*depth; ++x)
      {
        if (data[y*(width*depth) + x] <= 10)
          blackCount++;
      }
    }

    // Make sure the black count is zero. This means the camera is
    // positioned correctly
    QVERIFY(blackCount == 0);
  }

  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void MainWindow_TEST::MenuBar()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, false);

  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);

  // Create the main window.
  mainWindow->Load();
  mainWindow->Init();
  mainWindow->show();

  // Get the user camera
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);

  QList<QMenuBar *> menuBars  = mainWindow->findChildren<QMenuBar *>();
  QVERIFY(!menuBars.empty());

  std::set<std::string> mainMenus;
  mainMenus.insert("&File");
  mainMenus.insert("&Edit");
  mainMenus.insert("&Camera");
  mainMenus.insert("&View");
  mainMenus.insert("&Window");
  mainMenus.insert("&Help");

  // verify all menus are created in the menu bar.
  std::set<std::string> mainMenusCopy = mainMenus;
  QMenuBar *menuBar = menuBars[0];
  QList<QMenu *> menus  = menuBar->findChildren<QMenu *>();
  for (auto &m : menus)
  {
    auto it = mainMenusCopy.find(m->title().toStdString());
    QVERIFY(it != mainMenus.end());
    mainMenusCopy.erase(it);
  }

  // test adding a new menu to the menu bar
  QMenu newMenu(tr("&TEST"));
  mainWindow->AddMenu(&newMenu);

  QList<QMenu *> newMenus  = menuBar->findChildren<QMenu *>();
  mainMenusCopy = mainMenus;
  mainMenusCopy.insert("&TEST");
  for (auto &m : menus)
  {
    std::string title = m->title().toStdString();
    auto it = mainMenusCopy.find(title);
    QVERIFY(it != mainMenus.end());
    mainMenusCopy.erase(it);
  }

  // test calling ShowMenuBar and verify all menus remain the same
  mainWindow->ShowMenuBar();

  menus  = menuBar->findChildren<QMenu *>();
  mainMenusCopy = mainMenus;
  mainMenusCopy.insert("TEST");
  for (auto &m : menus)
  {
    std::string title = m->title().toStdString();
    auto it = mainMenusCopy.find(title);
    QVERIFY(it != mainMenus.end());
    mainMenusCopy.erase(title);
  }

  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void MainWindow_TEST::WindowModes()
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

  this->ProcessEventsAndDraw(mainWindow);

  // Check edit actions are visible
  QVERIFY(gazebo::gui::g_resetModelsAct->isVisible());
  QVERIFY(gazebo::gui::g_resetWorldAct->isVisible());
  QVERIFY(gazebo::gui::g_editBuildingAct->isVisible());
  QVERIFY(gazebo::gui::g_editModelAct->isVisible());

  // Change to Model Editor mode
  gazebo::gui::Events::windowMode("ModelEditor");

  // Check edit actions are not visible
  QVERIFY(!gazebo::gui::g_resetModelsAct->isVisible());
  QVERIFY(!gazebo::gui::g_resetWorldAct->isVisible());
  QVERIFY(!gazebo::gui::g_editBuildingAct->isVisible());
  QVERIFY(!gazebo::gui::g_editModelAct->isVisible());

  // Terminate
  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
void MainWindow_TEST::MinimumSize()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, false);

  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);

  // Create the main window.
  mainWindow->Load();
  mainWindow->Init();

  // Check that minimum size is smaller then a predefined size
  // This desired values are arbitrary, but increasing the minimum
  // size could create problems on small screens (such as laptop's).
  // See https://bitbucket.org/osrf/gazebo/issues/1706 for more info.
  int desiredMinimumWidth  = 700;
  int desiredMinimumHeight = 710;
  QVERIFY(mainWindow->minimumSize().width() <= desiredMinimumWidth);
  QVERIFY(mainWindow->minimumSize().height() <= desiredMinimumHeight);

  // Check that resizing to a small window (10x10) actually result
  // in a size that is smaller then desiredMinimum*
  mainWindow->resize(10, 10);

  QVERIFY(mainWindow->width() <= desiredMinimumWidth);
  QVERIFY(mainWindow->height() <= desiredMinimumHeight);

  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(MainWindow_TEST)
