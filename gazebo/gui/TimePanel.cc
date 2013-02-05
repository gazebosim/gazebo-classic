/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "transport/Node.hh"

#include "gui/Actions.hh"
#include "gui/GuiEvents.hh"
#include "gui/TimePanel.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
TimePanel::TimePanel(QWidget *_parent)
  : QWidget(_parent)
{
  this->setObjectName("timePanel");

  QHBoxLayout *mainLayout = new QHBoxLayout;

  QFrame *frame = new QFrame;
  QHBoxLayout *frameLayout = new QHBoxLayout;

  this->percentRealTimeEdit = new QLineEdit;
  this->percentRealTimeEdit->setObjectName("timePanelPercentRealTime");
  this->percentRealTimeEdit->setReadOnly(true);
  this->percentRealTimeEdit->setFixedWidth(90);

  this->simTimeEdit = new QLineEdit;
  this->simTimeEdit->setObjectName("timePanelSimTime");
  this->simTimeEdit->setReadOnly(true);
  this->simTimeEdit->setFixedWidth(110);

  this->realTimeEdit = new QLineEdit;
  this->realTimeEdit->setObjectName("timePanelRealTime");
  this->realTimeEdit->setReadOnly(true);
  this->realTimeEdit->setFixedWidth(110);

  QLabel *percentRealTimeLabel = new QLabel(tr("Real Time Factor:"));
  QLabel *simTimeLabel = new QLabel(tr("Sim Time:"));
  QLabel *realTimeLabel = new QLabel(tr("Real Time:"));

  QPushButton *timeResetButton = new QPushButton("Reset");
  timeResetButton->setFocusPolicy(Qt::NoFocus);
  connect(timeResetButton, SIGNAL(clicked()),
          this, SLOT(OnTimeReset()));

  frameLayout->addWidget(percentRealTimeLabel);
  frameLayout->addWidget(this->percentRealTimeEdit);

  frameLayout->addWidget(simTimeLabel);
  frameLayout->addWidget(this->simTimeEdit);

  frameLayout->addWidget(realTimeLabel);
  frameLayout->addWidget(this->realTimeEdit);

  frameLayout->addWidget(timeResetButton);

  frame->setLayout(frameLayout);
  frame->layout()->setContentsMargins(0, 0, 0, 0);

  mainLayout->addWidget(frame);
  this->setLayout(mainLayout);

  this->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
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

  this->simTime.Set(0);
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
void TimePanel::OnStats(ConstWorldStatisticsPtr &_msg)
{
  this->simTimes.push_back(msgs::Convert(_msg->sim_time()));
  if (this->simTimes.size() > 20)
    this->simTimes.pop_front();

  this->realTimes.push_back(msgs::Convert(_msg->real_time()));
  if (this->realTimes.size() > 20)
    this->realTimes.pop_front();

  this->simTime = msgs::Convert(_msg->sim_time());
  this->realTime = msgs::Convert(_msg->real_time());

  if (_msg->paused() && (g_pauseAct && !g_pauseAct->isChecked()))
  {
    g_pauseAct->setChecked(true);
    g_playAct->setChecked(false);
  }
  else if (!_msg->paused() && (g_playAct && !g_playAct->isChecked()))
  {
    g_pauseAct->setChecked(false);
    g_playAct->setChecked(true);
  }
}

/////////////////////////////////////////////////
void TimePanel::Update()
{
  std::ostringstream percent;
  std::ostringstream sim;
  std::ostringstream real;
  std::ostringstream pause;

  double simDbl = this->simTime.Double();
  if (simDbl > 31536000)
    sim << std::fixed << std::setprecision(2) << simDbl/31536000 << " dys";
  else if (simDbl > 86400)
    sim << std::fixed << std::setprecision(2) << simDbl / 86400 << " dys";
  else if (simDbl > 3600)
    sim << std::fixed << std::setprecision(2) << simDbl/3600 << " hrs";
  else if (simDbl > 999)
    sim << std::fixed << std::setprecision(2) << simDbl/60 << " min";
  else
    sim << std::fixed << std::setprecision(2) << simDbl << " sec";

  double realDbl = this->realTime.Double();
  if (realDbl > 31536000)
    real << std::fixed << std::setprecision(2) << realDbl/31536000 << " dys";
  else if (realDbl > 86400)
    real << std::fixed << std::setprecision(2) << realDbl/86400 << " dys";
  else if (realDbl > 3600)
    real << std::fixed << std::setprecision(2) << realDbl/3600 << " hrs";
  else if (realDbl > 999)
    real << std::fixed << std::setprecision(2) << realDbl/60 << " min";
  else
    real << std::fixed << std::setprecision(2) << realDbl << " sec";

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

  this->percentRealTimeEdit->setText(tr(percent.str().c_str()));

  this->simTimeEdit->setText(tr(sim.str().c_str()));
  this->realTimeEdit->setText(tr(real.str().c_str()));
}

/////////////////////////////////////////////////
void TimePanel::OnTimeReset()
{
  msgs::WorldControl msg;
  msg.mutable_reset()->set_all(false);
  msg.mutable_reset()->set_time_only(true);
  this->worldControlPub->Publish(msg);
}
