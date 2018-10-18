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
#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GLWidget.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/ViewAngleWidget.hh"

#include "view_angle.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void ViewAngleTest::OrthoProjection()
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

  // Get camera
  gazebo::rendering::UserCameraPtr cam = gazebo::gui::get_active_camera();
  QVERIFY(cam != NULL);

  // Get the view angle widget
  gazebo::gui::ViewAngleWidget *viewAngleWidget =
      mainWindow->findChild<gazebo::gui::ViewAngleWidget *>("viewAngleWidget");
  QVERIFY(viewAngleWidget != NULL);

  // Get the combobox
  QList<QComboBox *> comboBoxes =
      viewAngleWidget->findChildren<QComboBox *>();
  QVERIFY(comboBoxes.size() == 1u);

  QApplication::postEvent(viewAngleWidget, new QShowEvent());

  this->ProcessEventsAndDraw(mainWindow);

  // Check that it is in perspective projection
  QVERIFY(gazebo::gui::g_cameraPerspectiveAct->isChecked());
  QVERIFY(!gazebo::gui::g_cameraOrthoAct->isChecked());
  QVERIFY(comboBoxes[0]->currentText() == "Perspective");

  // Trigger ortho and see it changed
  gazebo::gui::g_cameraOrthoAct->trigger();

  this->ProcessEventsAndDraw(mainWindow);

  // Check that it is in orthographic projection
  QVERIFY(!gazebo::gui::g_cameraPerspectiveAct->isChecked());
  QVERIFY(gazebo::gui::g_cameraOrthoAct->isChecked());
  QVERIFY(comboBoxes[0]->currentText() == "Orthographic");

  // Get buttons
  QList<QToolButton *> buttons =
      viewAngleWidget->findChildren<QToolButton *>();
  QVERIFY(buttons.size() == 7u);

  // Trigger the top view button twice
  // gives more accurate results
  for (unsigned int j = 0; j < 2; ++j)
  {
    buttons[0]->click();

    this->ProcessEventsAndDraw(mainWindow, 50);
  }

  // initial projection matrix at top view.
  ignition::math::Matrix4d m1= cam->ProjectionMatrix();

  // change zoom using mouse
  auto glWidget = mainWindow->findChild<gazebo::gui::GLWidget *>("GLWidget");
  QVERIFY(glWidget != NULL);

  // focus
  QTest::mouseClick(glWidget, Qt::LeftButton, 0,
      QPoint(glWidget->width()*0.5, glWidget->height()*0.5));

  this->ProcessEventsAndDraw(mainWindow);

  // There seem to be a problem simulating mouse drag using QTest
  // so use raw QMouseEvent.
  QPoint midPt(glWidget->width()*0.5, glWidget->height()*0.5);
  QMouseEvent *pressEvent = new QMouseEvent(QEvent::MouseButtonPress, midPt,
      glWidget->mapToGlobal(midPt), Qt::RightButton, Qt::RightButton,
      Qt::NoModifier);
  QApplication::postEvent(glWidget, pressEvent);

  for (unsigned int i = 0; i < 10; ++i)
  {
    double dy = 0.5 - (0.25 * 0.1 * (i+1));

    QPoint movePt(glWidget->width()*0.5, glWidget->height()*dy);
    QMouseEvent *moveEvent = new QMouseEvent(QEvent::MouseMove, movePt,
        glWidget->mapToGlobal(movePt), Qt::RightButton, Qt::RightButton,
        Qt::NoModifier);
    QApplication::postEvent(glWidget, moveEvent);

    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  QPoint releasePt(glWidget->width()*0.5, glWidget->height()*0.25);
  QMouseEvent *releaseEvent = new QMouseEvent(QEvent::MouseButtonRelease,
      releasePt, glWidget->mapToGlobal(releasePt), Qt::RightButton,
      Qt::RightButton, Qt::NoModifier);
  QApplication::postEvent(glWidget, releaseEvent);

  this->ProcessEventsAndDraw(mainWindow, 50);

  // projection matrix after zoom.
  ignition::math::Matrix4d m2= cam->ProjectionMatrix();

  // Trigger the top view button twice
  // gives more accurate results
  for (unsigned int j = 0; j < 2; ++j)
  {
    buttons[0]->click();

    this->ProcessEventsAndDraw(mainWindow, 50);
  }

  // projection matrix after triggering the top view option again at different
  // zoom level.
  ignition::math::Matrix4d m3= cam->ProjectionMatrix();

  // check that the state of camera should be the same after triggering
  // the top view option the second time.
  QVERIFY(m3 == m2);
  QVERIFY(m3 != m1);

  // Clean up
  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(ViewAngleTest)
