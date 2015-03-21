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
#include "gazebo/gui/GLWidget_TEST2.hh"

#include "test_config.h"

gazebo::common::KeyEvent lastKeyEvent;
bool SetKey(const gazebo::common::KeyEvent &_event)
{
  lastKeyEvent = _event;
  return true;
}

/////////////////////////////////////////////////
void GLWidget_TEST2::KeyPresses()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/blank.world", false, false, false);

  // Create the main window.
  gazebo::gui::MainWindow mainWindow;

  mainWindow.Load();

  gazebo::rendering::create_scene(
      gazebo::physics::get_world()->GetName(), false);

  mainWindow.Init();
  mainWindow.show();

  gazebo::rendering::Events::createScene("default");

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow.repaint();
  }

  // Get GLWidget
  gazebo::gui::GLWidget *glWidget =
      mainWindow.findChild<gazebo::gui::GLWidget *>("GLWidget");
  QVERIFY(glWidget != NULL);

  // Set up a keypress filter
  gazebo::gui::KeyEventHandler::Instance()->
      AddPressFilter("glWidget_TEST_filter", SetKey);

  // Simulate keypresses separated by 100 millisecond intervals

  // Case: Lowercase 'a', no modifier
  QTest::keyClick(glWidget, 'a', Qt::NoModifier, 100);
  QVERIFY(lastKeyEvent.key == Qt::Key_A);
  QVERIFY(lastKeyEvent.text.compare("a") == 0);
  QVERIFY(!lastKeyEvent.control);
  QVERIFY(!lastKeyEvent.shift);
  QVERIFY(!lastKeyEvent.alt);

  // Case: Uppercase "A", no modifier
  QTest::keyClick(glWidget, 'A', Qt::NoModifier, 100);
  QVERIFY(lastKeyEvent.key == Qt::Key_A);
  QVERIFY(lastKeyEvent.text.compare("A") == 0);
  QVERIFY(!lastKeyEvent.control);
  QVERIFY(!lastKeyEvent.shift);
  QVERIFY(!lastKeyEvent.alt);

  // Case: Lowercase 'b', shift modifier
  QTest::keyClick(glWidget, 'b', Qt::ShiftModifier, 100);
  QVERIFY(lastKeyEvent.key == Qt::Key_B);
  QVERIFY(lastKeyEvent.text.compare("b") == 0);
  QVERIFY(!lastKeyEvent.control);
  QVERIFY(lastKeyEvent.shift);
  QVERIFY(!lastKeyEvent.alt);

  // Case: Uppercase 'b', shift modifier
  QTest::keyClick(glWidget, 'B', Qt::ShiftModifier, 100);
  QVERIFY(lastKeyEvent.key == Qt::Key_B);
  QVERIFY(lastKeyEvent.text.compare("B") == 0);
  QVERIFY(!lastKeyEvent.control);
  QVERIFY(lastKeyEvent.shift);
  QVERIFY(!lastKeyEvent.alt);

  // Case: lowercase 'c', control modifier
  QTest::keyClick(glWidget, 'c', Qt::ControlModifier, 100);
  QVERIFY(lastKeyEvent.key == Qt::Key_C);
  QVERIFY(lastKeyEvent.control);
  QVERIFY(!lastKeyEvent.shift);
  QVERIFY(!lastKeyEvent.alt);

  // Case: lowercase 'd', control modifier
  QTest::keyClick(glWidget, 'd', Qt::AltModifier, 100);
  QVERIFY(lastKeyEvent.key == Qt::Key_D);
  QVERIFY(!lastKeyEvent.control);
  QVERIFY(!lastKeyEvent.shift);
  QVERIFY(lastKeyEvent.alt);

  mainWindow.close();
}

// Generate a main function for the test
QTEST_MAIN(GLWidget_TEST2)
