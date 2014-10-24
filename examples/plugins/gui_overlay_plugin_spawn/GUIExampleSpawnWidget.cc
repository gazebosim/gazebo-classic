/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include <sstream>
#include <gazebo/msgs/msgs.hh>
#include "GUIExampleSpawnWidget.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GUIExampleSpawnWidget)

/////////////////////////////////////////////////
GUIExampleSpawnWidget::GUIExampleSpawnWidget()
  : GUIPlugin()
{
  this->counter = 0;

  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QVBoxLayout *frameLayout = new QVBoxLayout();

  // Create a push button, and connect it to the OnButton function
  QPushButton *button = new QPushButton(tr("Spawn Sphere"));
  connect(button, SIGNAL(clicked()), this, SLOT(OnButton()));

  // Add the button to the frame's layout
  frameLayout->addWidget(button);

  // Add frameLayout to the frame
  mainFrame->setLayout(frameLayout);

  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Position and resize this widget
  this->move(10, 10);
  this->resize(120, 30);

  // Create a node for transportation
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");
}

/////////////////////////////////////////////////
GUIExampleSpawnWidget::~GUIExampleSpawnWidget()
{
}

/////////////////////////////////////////////////
void GUIExampleSpawnWidget::OnButton()
{
  std::ostringstream newModelStr;
  newModelStr << "<sdf version ='" << SDF_VERSION << "'>"
    << "<model name='plugin_unit_sphere_" << this->counter++ << "'>"
    << "  <pose>0 0 1.5 0 0 0</pose>"
    << "  <link name='link'>"
    << "    <inertial><mass>1.0</mass></inertial>"
    << "    <collision name='collision'>"
    << "      <geometry>"
    << "        <sphere><radius>0.5</radius></sphere>"
    << "      </geometry>"
    << "    </collision>"
    << "    <visual name ='visual'>"
    << "      <geometry>"
    << "        <sphere><radius>0.5</radius></sphere>"
    << "      </geometry>"
    << "      <material>"
    << "        <script>"
    << "          <uri>file://media/materials/scripts/gazebo.material</uri>"
    << "          <name>Gazebo/Grey</name>"
    << "        </script>"
    << "      </material>"
    << "    </visual>"
    << "  </link>"
    << "  </model>"
    << "</sdf>";

  // Send the model to the gazebo server
  msgs::Factory msg;
  msg.set_sdf(newModelStr.str());
  this->factoryPub->Publish(msg);
}
