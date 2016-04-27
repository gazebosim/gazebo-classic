/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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
#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/gui/LayersWidget_TEST.hh"
#include "test_config.h"


/////////////////////////////////////////////////
void LayersWidget_TEST::ToggleLayers()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/test_layers.world", false, false, false);

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

  // Part 1 : Make sure the box is rendered. The white box should fill the
  // entire field of view.
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

    // Make sure the black count is zero. This means the visual
    // is visible and compelty fills the camera field of view.
    std::cerr << "blackCount: " << blackCount
              << " (should be 0)" << std::endl;
    QVERIFY(blackCount == 0);
  }

  // PART 2: Disable the white box. Only a black background should be
  // visible
  {
    // Disable the visual. This should hide the visual.
    gazebo::rendering::Events::toggleLayer(0);

    this->ProcessEventsAndDraw(mainWindow);

    // Get the new image data
    data = cam->ImageData();

    int whiteCount = 0;

    // Get the number of white pixels
    for (unsigned int y = 0; y < height; ++y)
    {
      for (unsigned int x = 0; x < width; ++x)
      {
        unsigned int sum = 0;
        for (unsigned int i = 0; i < depth; ++i)
        {
          sum += data[y*(width*depth) + x + i];
        }

        if (sum >= 250*depth)
          whiteCount++;
      }
    }

    // Make sure the white count is zero. This means the visual is not
    // visible.
    std::cerr << "whiteCount: " << whiteCount
              << " (should be 0)" << std::endl;
    QVERIFY(whiteCount == 0);
  }

  // PART 3: Re-enable the white box, which again should fill the entire
  // field of view.
  {
    // Re-enable the visual. This should make the visual visible again.
    gazebo::rendering::Events::toggleLayer(0);

    this->ProcessEventsAndDraw(mainWindow);

    // Get the new image data
    data = cam->ImageData();

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

    // Make sure the black count is zero. This means the visual
    // is visible and completely fills the camera field of view.
    std::cerr << "blackCount: " << blackCount
              << " (should be 0)" << std::endl;
    QVERIFY(blackCount == 0);
  }

  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(LayersWidget_TEST)
