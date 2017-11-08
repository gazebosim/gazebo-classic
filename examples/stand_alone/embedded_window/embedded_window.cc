/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#include <gazebo/gazebo_client.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/gui/MainWindow.hh>

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Create an app
  auto app = new QApplication(_argc, _argv);

  // Make sure the model database has started
  gazebo::common::ModelDatabase::Instance()->Start();

  // Setup a client with the passed arguments
  if (!gazebo::client::setup(_argc, _argv))
    return false;

  // Perform several startup actions, such as:
  // * loading ini file
  // * initializing rendering
  // * setting stylesheet and custom message handlers
  // * create and load a Gazebo main window
  // * it also initializes an app which we're not going to execute
  if (!gazebo::gui::load())
    return false;

  // Initialize the window created above
  gazebo::gui::init();

  // Get Gazebo's main window
  auto gazeboWindow = gazebo::gui::get_main_window();

  // Create a custom layout for our app and embed Gazebo's window inside it
  auto layout = new QHBoxLayout();
  layout->addWidget(new QLabel("My custom widget"));
  layout->addWidget(gazeboWindow);

  // Create the central widget
  auto mainWidget = new QWidget();
  mainWidget->setLayout(layout);

  // Create our custom window and show it
  auto mainWindow = new QMainWindow();
  mainWindow->setCentralWidget(mainWidget);
  mainWindow->show();

  // Execute our app
  app->exec();

  // After the app is closed, shutdown
  gazebo::client::shutdown();
  return true;
}
