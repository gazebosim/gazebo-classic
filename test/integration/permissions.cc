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
#include "ServerFixture.hh"

using namespace gazebo;

class PermissionsTest : public ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(PermissionsTest, ModelPathRestricted)
{
  Load("worlds/empty.world");

  gui::MainWindow mainWindow;

  mainWindow.Load();
  mainWindow.Init();

  // Create files in /tmp and set permissions accordingly

  boost::filesystem::path testDir("/tmp/InsertModelWidget_TEST");
  if (!boost::filesystem::exists(testDir))
  {
    boost::filesystem::create_directories(testDir);
  }

  // Case 1: add a restricted access folder to GAZEBO_MODEL_PATHS
  boost::filesystem::path testFolder = testDir / "case1";
  if (!boost::filesystem::exists(testFolder))
  {
    boost::filesystem::create_directories(testFolder);
  }
  boost::filesystem::permissions(testFolder, boost::filesystem::no_perms);

  // Try to add the folder to the model path
  ASSERT_NO_THROW(common::SystemPaths::Instance()->
                    AddModelPathsUpdate(testFolder.string()));

  boost::filesystem::permissions(testFolder, boost::filesystem::all_all);

  mainWindow.close();

  // Delete all test files
  boost::filesystem::remove_all(testDir);
}

/////////////////////////////////////////////////
TEST_F(PermissionsTest, ModelPathChildRestricted)
{
  Load("worlds/empty.world");

  gui::MainWindow mainWindow;

  mainWindow.Load();
  mainWindow.Init();

  gazebo::gui::InsertModelWidget *insertModelWidget =
      mainWindow.findChild<gazebo::gui::InsertModelWidget *>("insertModel");

  // Create files in /tmp and set permissions accordingly

  boost::filesystem::path testDir("/tmp/InsertModelWidget_TEST");
  if (!boost::filesystem::exists(testDir))
  {
    boost::filesystem::create_directories(testDir);
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

    // Try to add the folder to the model path
    ASSERT_NO_THROW(common::SystemPaths::Instance()->
                      AddModelPathsUpdate(testFolder.string()));

    // Check if it is in InsertModelWidget
    ASSERT_TRUE(insertModelWidget->LocalPathInFileWidget(testFolder.string()));

    boost::filesystem::permissions(childFolder, boost::filesystem::all_all);
  }
  mainWindow.close();

  // Delete all test files
  boost::filesystem::remove_all(testDir);
}

/////////////////////////////////////////////////
TEST_F(PermissionsTest, ModelPathChildFileRestricted)
{
  Load("worlds/empty.world");

  gui::MainWindow mainWindow;

  mainWindow.Load();
  mainWindow.Init();

  gazebo::gui::InsertModelWidget *insertModelWidget =
      mainWindow.findChild<gazebo::gui::InsertModelWidget *>("insertModel");

  // Create files in /tmp and set permissions accordingly

  boost::filesystem::path testDir("/tmp/InsertModelWidget_TEST");
  if (!boost::filesystem::exists(testDir))
  {
    boost::filesystem::create_directories(testDir);
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

    // Try to add the folder to the model path
    ASSERT_NO_THROW(common::SystemPaths::Instance()->
                      AddModelPathsUpdate(testFolder.string()));

    // Check if it is in InsertModelWidget
    ASSERT_TRUE(insertModelWidget->LocalPathInFileWidget(testFolder.string()));

    boost::filesystem::permissions(modelConfig, boost::filesystem::all_all);
  }

  mainWindow.close();

  // Delete all test files
  boost::filesystem::remove_all(testDir);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
