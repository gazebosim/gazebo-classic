/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#include "gazebo/common/Time.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/gui/LaserVisualization_TEST.hh"
#include "test_config.h"

/////////////////////////////////////////////////
void LaserVisualization_TEST::Lines()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/laser_lines_test.world", false, false, false);

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

  // Process some events, and draw the screen
  for (unsigned int i = 0; i < 10; ++i)
  {
    gazebo::common::Time::MSleep(30);
    QCoreApplication::processEvents();
    mainWindow->repaint();
  }

  // Get camera data
  const unsigned char *data = cam->GetImageData();
  unsigned int width = cam->GetImageWidth();
  unsigned int height = cam->GetImageHeight();
  unsigned int depth = cam->GetImageDepth();

  // Make sure there are darker lines in the laser rendering
  int lightBlueCount = 0;
  int darkBlueTransition = 0;

  int rPrev = 0;
  int gPrev = 0;
  int bPrev = 0;

  // Read a horizontal line in the middle of the screen
  unsigned int y = height / 2;
  for (unsigned int x = 0; x < width*depth; x += depth)
  {
    int r = data[y*(width*depth) + x];
    int g = data[y*(width*depth) + x+1];
    int b = data[y*(width*depth) + x+2];

    if (r < 189 && g < 189 && b == 255 &&
        rPrev >= 204 && gPrev >= 204 && bPrev >= 255)
      ++darkBlueTransition;

    if (r >= 204 && g >= 204 && b == 255)
      ++lightBlueCount;

    rPrev = r;
    bPrev = b;
    gPrev = g;
  }

  // Make sure there are 2 dark blue lines.
  QVERIFY(darkBlueTransition == 2);

  // Make sure there is a bunch of light blue pixels
  QVERIFY(lightBlueCount > 500);

  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(LaserVisualization_TEST)
