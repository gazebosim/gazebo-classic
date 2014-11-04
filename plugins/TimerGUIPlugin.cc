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
#include "TimerGUIPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(TimerGUIPlugin)

/////////////////////////////////////////////////
TimerGUIPlugin::TimerGUIPlugin()
  : GUIPlugin()
{
  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame {"
        "background-color : rgba(255, 255, 255, 255);"
        "color : black;"
        "font-size: 24px;"
      "}");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QHBoxLayout *frameLayout = new QHBoxLayout();

  // Create a time label
  QLabel *timeLabel = new QLabel(tr("00:00:00.00"));

  // Add the label to the frame's layout
  frameLayout->addWidget(timeLabel);
  connect(this, SIGNAL(SetTime(QString)),
      timeLabel, SLOT(setText(QString)), Qt::QueuedConnection);

  // Add frameLayout to the frame
  mainFrame->setLayout(frameLayout);

  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(4, 4, 4, 4);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Connect to the PreRender Gazebo signal
  this->connections.push_back(event::Events::ConnectPreRender(
                              boost::bind(&TimerGUIPlugin::PreRender, this)));
}

/////////////////////////////////////////////////
TimerGUIPlugin::~TimerGUIPlugin()
{
}

/////////////////////////////////////////////////
void TimerGUIPlugin::Load(sdf::ElementPtr _elem)
{
  // Size this widget
  if (_elem->HasElement("size"))
  {
    math::Vector2d s = _elem->Get<math::Vector2d>("size");
    this->resize(s.x, s.y);
  }
  else
  {
    this->resize(200, 30);
  }

  // Position this widget
  if (_elem->HasElement("pos"))
  {
    math::Vector2d p = _elem->Get<math::Vector2d>("pos");
    this->move(p.x, p.y);
  }
  else
  {
    int xPos, yPos;
    if (parent())
       xPos = static_cast<QWidget*>(parent())->width() - this->width() - 10;
    else
      xPos = 600;

    yPos = 10;
    this->move(xPos, yPos);
  }

  // Create a node for transportation
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  // Subscribe to the control topic
  if (_elem->HasElement("topic"))
  {
    this->ctrlSub = this->node->Subscribe(_elem->Get<std::string>("topic"),
        &TimerGUIPlugin::OnTimerCtrl, this);
  }
  else
  {
    this->ctrlSub = this->node->Subscribe("~/timer_control",
        &TimerGUIPlugin::OnTimerCtrl, this);
  }
}

/////////////////////////////////////////////////
void TimerGUIPlugin::PreRender()
{
  this->SetTime(QString::fromStdString(
        this->FormatTime(this->timer.GetElapsed())));
}

/////////////////////////////////////////////////
void TimerGUIPlugin::OnTimerCtrl(ConstGzStringPtr &_msg)
{
  if (_msg->data() == "start")
    this->timer.Start();
  else if (_msg->data() == "stop")
    this->timer.Stop();
  else if (_msg->data() == "reset")
    this->timer.Reset();
  else
    gzwarn << "Unable to process command[" << _msg->data() << "]\n";
}

/////////////////////////////////////////////////
std::string TimerGUIPlugin::FormatTime(const common::Time &_time) const
{
  std::ostringstream stream;
  unsigned int day, hour, min, sec, msec;

  stream.str("");

  sec = _time.sec;

  day = sec / 86400;
  sec -= day * 86400;

  hour = sec / 3600;
  sec -= hour * 3600;

  min = sec / 60;
  sec -= min * 60;

  msec = rint(_time.nsec * 1e-6);

  // \todo Add in ability to specify time format in SDF.
  // stream << std::setw(2) << std::setfill('0') << day << " ";

  stream << std::setw(2) << std::setfill('0') << hour << ":";
  stream << std::setw(2) << std::setfill('0') << min << ":";
  stream << std::setw(2) << std::setfill('0') << sec << ".";
  stream << std::setw(3) << std::setfill('0') << msec;

  return stream.str();
}
