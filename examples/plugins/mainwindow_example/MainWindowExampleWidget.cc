/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
#include <vector>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/gui/gui.hh>
#include <gazebo/gui/MainWindow.hh>

#include "MainWindowExampleWidget.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(MainWindowExampleWidget)

void MainWindowExampleWidget::PauseWorld()
{
  gui::get_main_window()->Pause();
  std::cout << "Got main window" << std::endl;
}

/////////////////////////////////////////////////
MainWindowExampleWidget::MainWindowExampleWidget()
  : GUIPlugin()
{
  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QHBoxLayout *frameLayout = new QHBoxLayout();

  QLabel *label = new QLabel(tr("Sim Time:"));

  // Create a time label
  QLabel *timeLabel = new QLabel(tr("00:00:00.00"));

  // Add the label to the frame's layout
  frameLayout->addWidget(label);
  frameLayout->addWidget(timeLabel);
  connect(this, SIGNAL(SetSimTime(QString)),
      timeLabel, SLOT(setText(QString)), Qt::QueuedConnection);

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

  // Create a node for transportation
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init("default");
  this->statsSub = this->node->Subscribe("~/world_stats",
      &MainWindowExampleWidget::OnStats, this);

  this->connections.push_back(gui::Events::ConnectMainWindowReady(
              boost::bind(&MainWindowExampleWidget::PauseWorld, this)));
}

/////////////////////////////////////////////////
MainWindowExampleWidget::~MainWindowExampleWidget()
{
}

/////////////////////////////////////////////////
void MainWindowExampleWidget::OnStats(ConstWorldStatisticsPtr &_msg)
{
  this->SetSimTime(QString::fromStdString(
        this->FormatTime(_msg->sim_time())));
}

/////////////////////////////////////////////////
std::string MainWindowExampleWidget::FormatTime(const msgs::Time &_msg) const
{
  std::ostringstream stream;
  unsigned int day, hour, min, sec, msec;

  stream.str("");

  sec = _msg.sec();

  day = sec / 86400;
  sec -= day * 86400;

  hour = sec / 3600;
  sec -= hour * 3600;

  min = sec / 60;
  sec -= min * 60;

  msec = rint(_msg.nsec() * 1e-6);

  stream << std::setw(2) << std::setfill('0') << day << " ";
  stream << std::setw(2) << std::setfill('0') << hour << ":";
  stream << std::setw(2) << std::setfill('0') << min << ":";
  stream << std::setw(2) << std::setfill('0') << sec << ".";
  stream << std::setw(3) << std::setfill('0') << msec;

  return stream.str();
}
