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

  this->ProcessEventsAndDraw(mainWindow);

  // Get camera data
  const unsigned char *data = cam->ImageData();
  unsigned int width = cam->ImageWidth();
  unsigned int height = cam->ImageHeight();
  unsigned int depth = cam->ImageDepth();

  // Make sure there are darker lines in the laser rendering
  int lightBlueCount = 0;
  int darkBlueTransition = 0;

  int rPrev = 0;
  int gPrev = 0;
  int bPrev = 0;

  // Read a horizontal line in the middle of the screen
  unsigned int y = height / 2;
  bool transitioning = false;
  for (unsigned int x = 0; x < width*depth; x += depth)
  {
    int r = data[y*(width*depth) + x];
    int g = data[y*(width*depth) + x+1];
    int b = data[y*(width*depth) + x+2];

    // check if current pixel is blue, prev pixel is also blue
    // and the two shades of blue are different
    if (!transitioning && r == g && r < b && rPrev == gPrev && rPrev < bPrev &&
        r < rPrev)
    {
      ++darkBlueTransition;
      transitioning = true;
    }

    // check if current and prev pixels are the same blue color
    if (r == g && r < b && r == rPrev && g == gPrev && b == bPrev)
    {
      ++lightBlueCount;
      transitioning = false;
    }

    rPrev = r;
    bPrev = b;
    gPrev = g;
  }

  // Make sure there are 2 dark blue lines.
  QVERIFY(darkBlueTransition == 2);

  // Make sure there is a bunch of light blue pixels
  QVERIFY(lightBlueCount > static_cast<int>(width * 0.95));

  mainWindow->close();
  delete mainWindow;
}

/////////////////////////////////////////////////
// This test pulls out a single row of pixels from the user camera. The row
// of pixels should pass through the three different zones of a laser
// visualization.
//
//   1. no-hit : Light blue region where the rays do not hit an object
//   2. hit : Dark blue region where the rays have hit an object
//   3. deadzone : Grey region between the sensor origin and minimum ray
//   distance
//
// The pixel row should follow this pattern :
//
// NNNNNNN__HHDDDDDDHH__NNNNNN
//
// where N = no-hit, _ = white area (no rays), H = hit, D = deadzone.
void LaserVisualization_TEST::Areas()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/laser_areas_test.world", false, false, false);

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
  unsigned int depth = cam->ImageDepth();

  int rPrev = 0;
  int gPrev = 0;
  int bPrev = 0;

  unsigned int whiteBand1Start = 0;
  unsigned int whiteBand1End = 0;

  unsigned int whiteBand2Start = 0;
  unsigned int whiteBand2End = 0;

  unsigned int greyBandStart = 0;
  unsigned int greyBandEnd = 0;

  unsigned int rSum = 0;
  unsigned int gSum = 0;
  unsigned int bSum = 0;
  unsigned int count = 0;

  float avgLightBlueBand1R = 0;
  float avgLightBlueBand1G = 0;
  float avgLightBlueBand1B = 0;

  float avgLightBlueBand2R = 0;
  float avgLightBlueBand2G = 0;
  float avgLightBlueBand2B = 0;

  float avgDarkBlueBand1R = 0;
  float avgDarkBlueBand1G = 0;
  float avgDarkBlueBand1B = 0;

  float avgDarkBlueBand2R = 0;
  float avgDarkBlueBand2G = 0;
  float avgDarkBlueBand2B = 0;

  // Read a horizontal line that intersects all three areas (no-hit, hit,
  // deadzone).
  unsigned int y = 350;

  for (unsigned int x = 0; x < width*depth; x += depth)
  {
    int r = data[y*(width*depth) + x];
    int g = data[y*(width*depth) + x+1];
    int b = data[y*(width*depth) + x+2];

    rSum += r;
    gSum += g;
    bSum += b;
    count++;

    // A white pixel is found
    if (r == 255 && g == 255 && b == 255)
    {
      // This is the start of the first white band. This is also the
      // end of the first light blue band
      if (whiteBand1Start == 0)
      {
        whiteBand1Start = x / depth;
        avgLightBlueBand1R = rSum / static_cast<float>(count);
        avgLightBlueBand1G = gSum / static_cast<float>(count);
        avgLightBlueBand1B = bSum / static_cast<float>(count);

        rSum = 0;
        gSum = 0;
        bSum = 0;
        count = 0;
      }
      // This is the start of the second white band. This is also the
      // end of the second dark blue band
      else if (whiteBand1End != 0 && whiteBand2Start == 0)
      {
        whiteBand2Start = x / depth;

        avgDarkBlueBand2R = rSum / static_cast<float>(count);
        avgDarkBlueBand2G = gSum / static_cast<float>(count);
        avgDarkBlueBand2B = bSum / static_cast<float>(count);

        rSum = 0;
        gSum = 0;
        bSum = 0;
        count = 0;
      }
    }

    // Transition from white to not white
    if (rPrev == 255 && gPrev == 255 && bPrev == 255 &&
        (r != 255 || g != 255 || b != 255))
    {
      if (whiteBand1End == 0)
        whiteBand1End = (x-depth) / depth;
      else
        whiteBand2End = (x-depth) / depth;

      rSum = 0;
      gSum = 0;
      bSum = 0;
      count = 0;
    }

    // Start of the grey band. This is also the end of the first
    // dark blue band
    if (r == 128 && g == 128 && b == 128 && greyBandStart == 0)
    {
      greyBandStart = x / depth;

      avgDarkBlueBand1R = rSum / static_cast<float>(count);
      avgDarkBlueBand1G = gSum / static_cast<float>(count);
      avgDarkBlueBand1B = bSum / static_cast<float>(count);

      rSum = 0;
      gSum = 0;
      bSum = 0;
      count = 0;
    }
    else if (rPrev == 128 && gPrev == 128 && bPrev == 128 &&
             (r != 128 || g != 128 || b != 128))
    {
      greyBandEnd = (x-depth) / depth;

      rSum = 0;
      gSum = 0;
      bSum = 0;
      count = 0;
    }

    rPrev = r;
    bPrev = b;
    gPrev = g;
  }

  // The end of the second light blue band is the end of the pixel row.
  avgLightBlueBand2R = rSum / static_cast<float>(count);
  avgLightBlueBand2G = gSum / static_cast<float>(count);
  avgLightBlueBand2B = bSum / static_cast<float>(count);

  std::cout << "White band 1[" << whiteBand1Start << " "
            << whiteBand1End << "]\n";

  std::cout << "White band 2[" << whiteBand2Start << " "
            << whiteBand2End << "]\n";

  std::cout << "Grey band[" << greyBandStart << " "
            << greyBandEnd << "]\n";

  std::cout << "Avg Light Blue Band 1["
    << avgLightBlueBand1R << " "
    << avgLightBlueBand1G << " "
    << avgLightBlueBand1B << "]\n";
  std::cout << "Avg Light Blue Band 2["
    << avgLightBlueBand2R << " "
    << avgLightBlueBand2G << " "
    << avgLightBlueBand2B << "]\n";

  std::cout << "Avg Dark Blue Band 1["
    << avgDarkBlueBand1R << " "
    << avgDarkBlueBand1G << " "
    << avgDarkBlueBand1B << "]\n";
  std::cout << "Avg Dark Blue Band 2["
    << avgDarkBlueBand2R << " "
    << avgDarkBlueBand2G << " "
    << avgDarkBlueBand2B << "]\n";

  // Verify the avg color of the first light blue band
  QVERIFY(avgLightBlueBand1R > 190 && avgLightBlueBand1R < 194);
  QVERIFY(avgLightBlueBand1G > 190 && avgLightBlueBand1G < 194);
  QVERIFY(static_cast<unsigned int>(avgLightBlueBand1B) == 255);

  // Verify the avg color of the second light blue band
  QVERIFY(avgLightBlueBand2R > 190 && avgLightBlueBand2R < 194);
  QVERIFY(avgLightBlueBand2G > 190 && avgLightBlueBand2G < 194);
  QVERIFY(static_cast<unsigned int>(avgLightBlueBand2B) == 255);

  // Verify the avg color of the first dark blue band
  QVERIFY(avgDarkBlueBand1R > 109 && avgDarkBlueBand1R < 114);
  QVERIFY(avgDarkBlueBand1G > 109 && avgDarkBlueBand1G < 114);
  QVERIFY(avgDarkBlueBand1B > 249 && avgDarkBlueBand1B <= 255);

  // Verify the avg color of the second dark blue band
  QVERIFY(avgDarkBlueBand2R > 109 && avgDarkBlueBand2R < 114);
  QVERIFY(avgDarkBlueBand2G > 109 && avgDarkBlueBand2G < 114);
  QVERIFY(avgDarkBlueBand2B > 249 && avgDarkBlueBand2B <= 255);

  // Verify the bands are in the correct order
  QVERIFY(whiteBand1Start > 0 && whiteBand1Start < whiteBand1End);
  QVERIFY(greyBandStart > whiteBand1End && greyBandStart < greyBandEnd);
  QVERIFY(whiteBand2Start > greyBandEnd && whiteBand2Start < whiteBand2End);
  QVERIFY(whiteBand2End < width);

  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(LaserVisualization_TEST)
