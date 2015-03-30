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
#include <sstream>

#include "gazebo/transport/Node.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/TimeWidget.hh"
#include "gazebo/gui/LogPlayWidget.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/gui/TimePanel.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
TimePanel::TimePanel(QWidget *_parent)
  : QWidget(_parent)
{
  this->setObjectName("timePanel");

  this->timeWidget = new TimeWidget(this);
  connect(this, SIGNAL(SetTimeWidgetVisible(bool)), this->timeWidget,
      SLOT(setVisible(bool)));
  this->logPlayWidget = new LogPlayWidget(this);
  connect(this, SIGNAL(SetLogPlayWidgetVisible(bool)), this->logPlayWidget,
      SLOT(setVisible(bool)));

  QHBoxLayout *mainLayout = new QHBoxLayout;
  mainLayout->addWidget(this->timeWidget);
  mainLayout->addWidget(this->logPlayWidget);
  this->setLayout(mainLayout);

  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  this->layout()->setContentsMargins(0, 0, 0, 0);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->statsSub =
    this->node->Subscribe("~/world_stats", &TimePanel::OnStats, this);
  this->worldControlPub =
    this->node->Advertise<msgs::WorldControl>("~/world_control");

  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(Update()));
  timer->start(33);

  this->connections.push_back(
      gui::Events::ConnectFullScreen(
        boost::bind(&TimePanel::OnFullScreen, this, _1)));

  this->show();
}

/////////////////////////////////////////////////
void TimePanel::OnFullScreen(bool & /*_value*/)
{
  /*if (_value)
    this->hide();
  else
    this->show();
    */
}

/////////////////////////////////////////////////
TimePanel::~TimePanel()
{
  this->node.reset();
}

/////////////////////////////////////////////////
void TimePanel::ShowRealTimeFactor(bool _show)
{
  if (this->timeWidget->isVisible())
    this->timeWidget->ShowRealTimeFactor(_show);
  else
    gzwarn << "Time widget not visible" << std::endl;
}

/////////////////////////////////////////////////
void TimePanel::ShowRealTime(bool _show)
{
  if (this->timeWidget->isVisible())
    this->timeWidget->ShowRealTime(_show);
  else
    gzwarn << "Time widget not visible" << std::endl;
}

/////////////////////////////////////////////////
void TimePanel::ShowSimTime(bool _show)
{
  if (this->timeWidget->isVisible())
    this->timeWidget->ShowSimTime(_show);
  else
    gzwarn << "Time widget not visible" << std::endl;
}

/////////////////////////////////////////////////
void TimePanel::ShowIterations(bool _show)
{
  if (this->timeWidget->isVisible())
    this->timeWidget->ShowIterations(_show);
  else
    gzwarn << "Time widget not visible" << std::endl;
}

/////////////////////////////////////////////////
void TimePanel::ShowFPS(bool _show)
{
  if (this->timeWidget->isVisible())
    this->timeWidget->ShowFPS(_show);
  else
    gzwarn << "Time widget not visible" << std::endl;
}

/////////////////////////////////////////////////
void TimePanel::ShowStepWidget(bool _show)
{
  if (this->timeWidget->isVisible())
    this->timeWidget->ShowStepWidget(_show);
  else
    gzwarn << "Time widget not visible" << std::endl;
}

/////////////////////////////////////////////////
bool TimePanel::IsPaused() const
{
  return this->paused;
}

/////////////////////////////////////////////////
void TimePanel::SetPaused(bool _paused)
{
  this->paused = _paused;

  if (this->timeWidget->isVisible())
    this->timeWidget->SetPaused(_paused);
  else if (this->logPlayWidget->isVisible())
    this->logPlayWidget->SetPaused(_paused);
  else
    gzwarn << "No widget visible" << std::endl;
}

/////////////////////////////////////////////////
void TimePanel::OnStats(ConstWorldStatisticsPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);

  this->simTimes.push_back(msgs::Convert(_msg->sim_time()));
  if (this->simTimes.size() > 20)
    this->simTimes.pop_front();

  this->realTimes.push_back(msgs::Convert(_msg->real_time()));
  if (this->realTimes.size() > 20)
    this->realTimes.pop_front();

  if (_msg->has_paused())
    this->SetPaused(_msg->paused());

  if (_msg->has_log_playback())
  {
    if (_msg->log_playback())
    {
      this->SetTimeWidgetVisible(false);
      this->SetLogPlayWidgetVisible(true);
    }
    else
    {
      this->SetTimeWidgetVisible(true);
      this->SetLogPlayWidgetVisible(false);
    }
  }

  if (this->timeWidget->isVisible())
  {
    // Set simulation time
    this->timeWidget->EmitSetSimTime(QString::fromStdString(FormatTime(_msg->sim_time())));

    // Set real time
    this->timeWidget->EmitSetRealTime(QString::fromStdString(FormatTime(_msg->real_time())));

    // Set the iterations
    this->timeWidget->EmitSetIterations(QString::fromStdString(
          boost::lexical_cast<std::string>(_msg->iterations())));
  }
  else if (this->logPlayWidget->isVisible())
  {
    // Set simulation time
    this->logPlayWidget->EmitSetSimTime(QString::fromStdString(FormatTime(_msg->sim_time())));

    // Set the iterations
    this->logPlayWidget->EmitSetIterations(QString::fromStdString(
          boost::lexical_cast<std::string>(_msg->iterations())));
  }
}

/////////////////////////////////////////////////
std::string TimePanel::FormatTime(const msgs::Time &_msg)
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


/////////////////////////////////////////////////
void TimePanel::Update()
{
  boost::mutex::scoped_lock lock(this->mutex);

  std::ostringstream percent;

  common::Time simAvg, realAvg;
  std::list<common::Time>::iterator simIter, realIter;

  simIter = ++(this->simTimes.begin());
  realIter = ++(this->realTimes.begin());
  while (simIter != this->simTimes.end() && realIter != this->realTimes.end())
  {
    simAvg += ((*simIter) - this->simTimes.front());
    realAvg += ((*realIter) - this->realTimes.front());
    ++simIter;
    ++realIter;
  }
  if (realAvg == 0)
    simAvg = 0;
  else
    simAvg = simAvg / realAvg;

  if (simAvg > 0)
    percent << std::fixed << std::setprecision(2) << simAvg.Double();
  else
    percent << "0";

  if (this->timeWidget->isVisible())
    this->timeWidget->SetPercentRealTimeEdit(percent.str().c_str());

  rendering::UserCameraPtr cam = gui::get_active_camera();
  if (cam)
  {
    std::ostringstream avgFPS;
    avgFPS << cam->GetAvgFPS();

    if (this->timeWidget->isVisible())
    {
      // Set the avg fps
      this->timeWidget->EmitSetFPS(QString::fromStdString(
            boost::lexical_cast<std::string>(avgFPS.str().c_str())));
    }
  }
}

/////////////////////////////////////////////////
void TimePanel::OnTimeReset()
{
  msgs::WorldControl msg;
  msg.mutable_reset()->set_all(false);
  msg.mutable_reset()->set_time_only(true);
  this->worldControlPub->Publish(msg);
}

/////////////////////////////////////////////////
void TimePanel::OnStepValueChanged(int _value)
{
  emit gui::Events::inputStepSize(_value);
}
