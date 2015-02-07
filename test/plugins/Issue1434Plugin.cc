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
#include <gazebo/gui/GuiIface.hh>
#include "Issue1434Plugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(Issue1434Plugin)

Issue1434Plugin::Issue1434Plugin() : GUIPlugin()
{
  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QHBoxLayout *frameLayout = new QHBoxLayout();

  QLabel *label = new QLabel(tr("I am a test widget"));

  // Add the label to the frame's layout
  frameLayout->addWidget(label);

  // Add frameLayout to the frame
  mainFrame->setLayout(frameLayout);

  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(4, 4, 4, 4);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Position and resize this widget
  this->move(200, 10);
  this->resize(200, 30);

  gzdbg << "World: " << gazebo::gui::get_world() << std::endl;
  gazebo::gui::get_active_camera();
}

void Issue1434Plugin::Load(sdf::ElementPtr /*_sdf*/)
{
  gzdbg << "World: " << gazebo::gui::get_world() << std::endl;
  gazebo::gui::get_active_camera();
}
