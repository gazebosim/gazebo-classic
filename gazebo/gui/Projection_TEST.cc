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
#include "gazebo/gui/Projection_TEST.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void Projection_TEST::Projection()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/ortho_box.world", false, false, false);

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
  std::cerr << "width: " << width << std::endl;
  std::cerr << "height: " << height << std::endl;
  std::cerr << "depth: " << depth << std::endl;

  int topWidth = 0;
  int bottomWidth = 0;

  // Get the width of the white box at the top and bottom of the image
  for (unsigned int x = 0; x < width*depth; ++x)
  {
    if ( data[x] >= 250)
      topWidth++;

    if ( data[((height-1)*width*depth)+x] >= 250)
      bottomWidth++;
  }

  // Make sure the widths are roughly the same
  std::cerr << "topWidth: " << topWidth << std::endl;
  std::cerr << "bottomWidth: " << bottomWidth << std::endl;
  std::cerr << "std::abs(topWidth - bottomWidth): "
            << std::abs(topWidth - bottomWidth)
            << " (should be less than 5)"
            << std::endl;
  QVERIFY(std::abs(topWidth - bottomWidth) < 5);

  // Now change to Perspective projection
  cam->SetProjectionType("perspective");

  this->ProcessEventsAndDraw(mainWindow);

  topWidth = 0;
  bottomWidth = 0;

  // Get the width of the white box at the top and bottom of the image
  for (unsigned int x = 0; x < width*depth; ++x)
  {
    if (data[x] >= 250)
      topWidth++;

    if (data[((height-1)*width*depth)+x] >= 250)
      bottomWidth++;
  }

  // Make sure the top and bottom width difference is large enough to
  // indicate perspective rendering is used
  std::cerr << "topWidth: " << topWidth << std::endl;
  std::cerr << "bottomWidth: " << bottomWidth << std::endl;
  double widthRatio = topWidth - bottomWidth;
  widthRatio /= height;
  std::cerr << "(topWidth - bottomWidth) / height: "
            << widthRatio
            << " (should be less than -0.2)"
            << std::endl;
  QVERIFY(widthRatio < -0.2);

  cam->Fini();
  mainWindow->close();
  delete mainWindow;
}

// Generate a main function for the test
QTEST_MAIN(Projection_TEST)
