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
#include "gazebo/gui/MainWindow.hh"
#include "collada_visualization.hh"
#include "test_config.h"

/////////////////////////////////////////////////
void ColladaVisualization::MultipleTextureCoordinates()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/multiple_texture_coordinates_test.world",
             false, false, false);

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

  // Get camera data
  const unsigned char *data = cam->ImageData();
  unsigned int width = cam->ImageWidth();
  unsigned int height = cam->ImageHeight();
  unsigned int depth = cam->ImageDepth();

  // The triangle should be all white.
  for (unsigned int y = 0; y < height; ++y)
  {
    for (unsigned int x = 0; x < width*depth; x += depth)
    {
      int r = data[y*(width*depth) + x];
      int g = data[y*(width*depth) + x+1];
      int b = data[y*(width*depth) + x+2];
      QVERIFY(r == 255);
      QVERIFY(g == 255);
      QVERIFY(b == 255);
    }
  }

  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(ColladaVisualization)
