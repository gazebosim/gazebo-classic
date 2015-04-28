/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
#include "gazebo/common/KeyEvent.hh"
#include "gazebo/math/Helpers.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/KeyEventHandler.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/GLWidget.hh"
#include "gazebo/gui/GLWidget_TEST.hh"

#include "test_config.h"


bool g_gotBoxSelection = false;
void OnSelection(ConstSelectionPtr &_msg)
{
  if (_msg->name() == "box" && _msg->has_selected() && _msg->selected())
    g_gotBoxSelection = true;
}

/////////////////////////////////////////////////
void GLWidget_TEST::SelectObject()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/shapes.world", false, false, false);

  gazebo::transport::NodePtr node;
  gazebo::transport::SubscriberPtr sub;
  node = gazebo::transport::NodePtr(new gazebo::transport::Node());
  node->Init();
  sub = node->Subscribe("~/selection", &OnSelection, this);

  // Create the main window.
  gazebo::gui::MainWindow *mainWindow = new gazebo::gui::MainWindow();
  QVERIFY(mainWindow != NULL);

  mainWindow->Load();

  gazebo::rendering::create_scene(
      gazebo::physics::get_world()->GetName(), false);

  mainWindow->Init();
  mainWindow->show();

  gazebo::rendering::Events::createScene("default");

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Get GLWidget
  gazebo::gui::GLWidget *glWidget =
      mainWindow->findChild<gazebo::gui::GLWidget *>("GLWidget");
  QVERIFY(glWidget != NULL);

  // Click in the center of the screen
  QPoint moveTo(glWidget->width()/2, glWidget->height()/2);
  QTest::mouseClick(glWidget, Qt::LeftButton, Qt::NoModifier, moveTo);
  QTest::qWait(500);

  // Verify the box was selected
  QVERIFY(g_gotBoxSelection);

  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(GLWidget_TEST)
