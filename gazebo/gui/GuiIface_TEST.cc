/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include <iostream>

#include "gazebo/gui/GuiIface_TEST.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/common/ModelDatabase.hh"
#include "test_config.h"

/////////////////////////////////////////////////
void GuiIface_TEST::noINIFile()
{
  // Get a temp directory
  boost::filesystem::path path =
    boost::filesystem::temp_directory_path() / "gazebo";
  boost::filesystem::create_directories(path);

  // Set the gui.ini filename
  path /= "gui.ini";

  // Make sure the INI file doesn't exist.
  if (boost::filesystem::exists(path))
    boost::filesystem::remove(path);

  // This should create the INI file.
  QVERIFY(gazebo::gui::loadINI(path));

  int xPos = gazebo::gui::getINIProperty<int>("geometry.x", -10);
  int yPos = gazebo::gui::getINIProperty<int>("geometry.y", -20);

  int width = gazebo::gui::getINIProperty<int>("geometry.width", 100);
  int height = gazebo::gui::getINIProperty<int>("geometry.height", 200);

  // X and Y pos should return zeros.
  QVERIFY(xPos == 0);
  QVERIFY(yPos == 0);

  // Width and height should return the default values.
  QVERIFY(width == 100);
  QVERIFY(height == 200);

  boost::filesystem::remove(path);
  gazebo::gui::stop();
}

/////////////////////////////////////////////////
void GuiIface_TEST::GUIINIPATHEnvVariable()
{
  // Get a temp directory
  boost::filesystem::path path =
    boost::filesystem::temp_directory_path() / "gazebo";
  boost::filesystem::create_directories(path);

  // Set the gui.ini filename
  path /= "foo.ini";

  // Make sure the INI file doesn't exist.
  if (boost::filesystem::exists(path))
    boost::filesystem::remove(path);

  const char *filepath = path.string().c_str();

  setenv("GAZEBO_GUI_INI_FILE", filepath, 1);

  // False if the path didn't exist
  QVERIFY(gazebo::gui::loadINI() == false);

  // Write invalid content for an INI file
  std::ofstream iniFile;
  iniFile.open(filepath);
  iniFile << "Invalid content\n";
  iniFile.close();

  // Only false when trying to read invalid content
  QVERIFY(gazebo::gui::loadINI() == false);

  // Need to clean up the variable for the next tests
  unsetenv("GAZEBO_GUI_INI_FILE");
  boost::filesystem::remove(path);

  gazebo::gui::stop();
}

/////////////////////////////////////////////////
void GuiIface_TEST::setINIProperties()
{
  // Get a temp directory
  boost::filesystem::path path =
    boost::filesystem::temp_directory_path() / "gazebo";
  boost::filesystem::create_directories(path);

  // Set the gui.ini filename
  path /= "gui.ini";

  // This will create the INI file if it doesn't exist.
  QVERIFY(gazebo::gui::loadINI(path));

  // Get the original values.
  int xPosOrig = gazebo::gui::getINIProperty<int>("geometry.x", 0);
  int yPosOrig = gazebo::gui::getINIProperty<int>("geometry.y", 0);
  int widthOrig = gazebo::gui::getINIProperty<int>("geometry.width", 0);
  int heightOrig = gazebo::gui::getINIProperty<int>("geometry.height", 0);

  // Set new values
  gazebo::gui::setINIProperty("geometry.x", 100);
  gazebo::gui::setINIProperty("geometry.y", 50);
  gazebo::gui::setINIProperty("geometry.width", 800);
  gazebo::gui::setINIProperty("geometry.height", 600);

  // Get the new values
  int xPos = gazebo::gui::getINIProperty<int>("geometry.x", 0);
  int yPos = gazebo::gui::getINIProperty<int>("geometry.y", 0);
  int width = gazebo::gui::getINIProperty<int>("geometry.width", 0);
  int height = gazebo::gui::getINIProperty<int>("geometry.height", 0);

  // Verify the new values are correct
  QVERIFY(xPos == 100);
  QVERIFY(yPos == 50);
  QVERIFY(width == 800);
  QVERIFY(height == 600);

  // Reload the INI file, and read back the values.
  QVERIFY(gazebo::gui::loadINI(path));
  xPos = gazebo::gui::getINIProperty<int>("geometry.x", 0);
  yPos = gazebo::gui::getINIProperty<int>("geometry.y", 0);
  width = gazebo::gui::getINIProperty<int>("geometry.width", 0);
  height = gazebo::gui::getINIProperty<int>("geometry.height", 0);

  QVERIFY(xPos == xPosOrig);
  QVERIFY(yPos == yPosOrig);
  QVERIFY(width == widthOrig);
  QVERIFY(height == heightOrig);

  boost::filesystem::remove(path);
  gazebo::gui::stop();
}

/////////////////////////////////////////////////
void GuiIface_TEST::saveINIProperties()
{
  // Get a temp directory
  boost::filesystem::path path =
    boost::filesystem::temp_directory_path() / "gazebo";
  boost::filesystem::create_directories(path);

  // Set the gui.ini filename
  path /= "gui.ini";

  // This will create the INI file if it doesn't exist.
  QVERIFY(gazebo::gui::loadINI(path));

  // Set new values
  gazebo::gui::setINIProperty("geometry.x", 100);
  gazebo::gui::setINIProperty("geometry.y", 50);
  gazebo::gui::setINIProperty("geometry.width", 800);
  gazebo::gui::setINIProperty("geometry.height", 600);

  // Get the new values
  int xPos = gazebo::gui::getINIProperty<int>("geometry.x", 0);
  int yPos = gazebo::gui::getINIProperty<int>("geometry.y", 0);
  int width = gazebo::gui::getINIProperty<int>("geometry.width", 0);
  int height = gazebo::gui::getINIProperty<int>("geometry.height", 0);

  // Verify the new values are correct
  QVERIFY(xPos == 100);
  QVERIFY(yPos == 50);
  QVERIFY(width == 800);
  QVERIFY(height == 600);

  // Save the values to disk
  QVERIFY(gazebo::gui::saveINI(path));

  // Reload the INI file, and read back the values.
  QVERIFY(gazebo::gui::loadINI(path));
  int xPosNew = gazebo::gui::getINIProperty<int>("geometry.x", 0);
  int yPosNew = gazebo::gui::getINIProperty<int>("geometry.y", 0);
  int widthNew = gazebo::gui::getINIProperty<int>("geometry.width", 0);
  int heightNew = gazebo::gui::getINIProperty<int>("geometry.height", 0);

  QVERIFY(xPos == xPosNew);
  QVERIFY(yPos == yPosNew);
  QVERIFY(width == widthNew);
  QVERIFY(height == heightNew);

  boost::filesystem::remove(path);
  gazebo::gui::stop();
}

// Generate a main function for the test
QTEST_MAIN(GuiIface_TEST)
