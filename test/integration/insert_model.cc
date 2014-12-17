/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "gazebo/common/SystemPaths.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/InsertModelWidget.hh"
#include "insert_model.hh"

#include "test_config.h"

/////////////////////////////////////////////////
void InsertModelTest::ReadPermissions()
{
  this->resMaxPercentChange = 5.0;
  this->shareMaxPercentChange = 2.0;

  this->Load("worlds/empty.world", false, false, true);

  gazebo::gui::MainWindow mainWindow;

  mainWindow.Load();
  mainWindow.Init();

  // wait a bit for the event to fire (?)
  gazebo::gui::InsertModelWidget *insertModelWidget =
      mainWindow.findChild<gazebo::gui::InsertModelWidget *>("insertModel");

  // Create files in a temporary directory and set permissions accordingly
  boost::filesystem::path testDir = boost::filesystem::temp_directory_path() /
                                      "InsertModelTest";
  if (!boost::filesystem::exists(testDir))
  {
    boost::filesystem::create_directories(testDir);
  }

  // Case 1: add a restricted access folder to GAZEBO_MODEL_PATHS
  {
    boost::filesystem::path testFolder = testDir / "case1";
    if (!boost::filesystem::exists(testFolder))
    {
      boost::filesystem::create_directories(testFolder);
    }
    boost::filesystem::permissions(testFolder, boost::filesystem::no_perms);

    try
    {
      // Try to add the folder to the model path
      gazebo::common::SystemPaths::Instance()->
        AddModelPathsUpdate(testFolder.string());
    }
    catch(boost::filesystem::filesystem_error & e)
    {
      QFAIL("Failed to catch filesystem error!\n");
    }

    boost::filesystem::permissions(testFolder, boost::filesystem::all_all);
  }

  // Case 2: Add parent of a restricted access folder to GAZEBO_MODEL_PATHS
  {
    boost::filesystem::path testFolder = testDir / "case2";
    boost::filesystem::path childFolder = testFolder / "child";
    if (!boost::filesystem::exists(childFolder))
    {
      boost::filesystem::create_directories(childFolder);
    }
    boost::filesystem::permissions(childFolder, boost::filesystem::no_perms);

    try
    {
      // Try to add the folder to the model path
      gazebo::common::SystemPaths::Instance()->
        AddModelPathsUpdate(testFolder.string());
    }
    catch(boost::filesystem::filesystem_error & e)
    {
      QFAIL("Failed to catch filesystem error!\n");
    }

    // Check if it is in InsertModelWidget
    QVERIFY(insertModelWidget->LocalPathInFileWidget(testFolder.string()));

    boost::filesystem::permissions(childFolder, boost::filesystem::all_all);
  }

  // Case 3: Add parent of a folder containing a restricted access model.config
  {
    boost::filesystem::path testFolder = testDir / "case3";
    boost::filesystem::path childFolder = testFolder / "child";
    if (!boost::filesystem::exists(childFolder))
    {
      boost::filesystem::create_directories(childFolder);
    }
    boost::filesystem::path modelConfig = childFolder / "model.config";
    std::ofstream savefile;
    savefile.open(modelConfig.string().c_str());
    savefile << "asdf";
    savefile.close();

    boost::filesystem::permissions(modelConfig, boost::filesystem::no_perms);

    try
    {
      // Try to add the folder to the model path
      gazebo::common::SystemPaths::Instance()->
        AddModelPathsUpdate(testFolder.string());
    }
    catch(boost::filesystem::filesystem_error & e)
    {
      QFAIL("Failed to catch filesystem error!\n");
    }

    // Check if it is in InsertModelWidget
    QVERIFY(insertModelWidget->LocalPathInFileWidget(testFolder.string()));

    boost::filesystem::permissions(modelConfig, boost::filesystem::all_all);
  }

  mainWindow.close();

  // Delete all test files
  boost::filesystem::remove_all(testDir);
}

// Generate a main function for the test
QTEST_MAIN(InsertModelTest)
