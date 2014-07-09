/*
 * Copyright 2014 Open Source Robotics Foundation
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

#include "gazebo/transport/transport.hh"
#include "gazebo/util/LogPlay.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/LogPlayWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
LogPlayWidget::LogPlayWidget(QWidget *_parent)
  : QWidget(_parent)
{
  this->sliderPressed = false;

  QHBoxLayout *mainLayout = new QHBoxLayout;

  // The play button allows the user to start and pause data playback
  /*this->playButton = new QToolButton(this);
  this->playButton->setIcon(QPixmap(":/images/play.svg"));
  this->playButton->setStatusTip(tr("Play log"));
  this->playButton->setCheckable(true);
  this->playButton->setChecked(false);
  this->playButton->setIconSize(QSize(32, 32));
  this->playButton->setObjectName("logPlayButton");
  connect(this->playButton, SIGNAL(toggled(bool)),
          this, SLOT(OnPlay(bool)));

  // The step foward button allows the user to step forward in time
  this->stepForwardButton = new QToolButton(this);
  this->stepForwardButton->setIcon(QPixmap(":/images/step_forward.svg"));
  this->stepForwardButton->setStatusTip(tr("Step Forward"));
  this->stepForwardButton->setCheckable(true);
  this->stepForwardButton->setChecked(false);
  //this->stepForwardButton->setIconSize(QSize(32, 32));
  this->stepForwardButton->setObjectName("logStepButton");
  //connect(this->playButton, SIGNAL(toggled(bool)),
  //        this, SLOT(OnPlay(bool)));

  // The step backward button allows the user to step backward in time
  this->stepBackwardButton = new QToolButton(this);
  this->stepBackwardButton->setIcon(QPixmap(":/images/step_backward.svg"));
  this->stepBackwardButton->setStatusTip(tr("Step Forward"));
  this->stepBackwardButton->setCheckable(true);
  this->stepBackwardButton->setChecked(false);
  //this->stepBackwardButton->setIconSize(QSize(32, 32));
  this->stepBackwardButton->setObjectName("logStepButton");
  //connect(this->playButton, SIGNAL(toggled(bool)),
  //        this, SLOT(OnPlay(bool)));

  this->jumpBackwardButton = new QToolButton(this);
  this->jumpBackwardButton->setIcon(QPixmap(":/images/jump_backward.svg"));
  this->jumpBackwardButton->setStatusTip(tr("Jump to beginning"));
  this->jumpBackwardButton->setCheckable(true);
  this->jumpBackwardButton->setChecked(false);
  //this->jumpBackwardButton->setIconSize(QSize(32, 32));
  this->jumpBackwardButton->setObjectName("logStepButton");

  this->jumpForwardButton = new QToolButton(this);
  this->jumpForwardButton->setIcon(QPixmap(":/images/jump_forward.svg"));
  this->jumpForwardButton->setStatusTip(tr("Jump to end"));
  this->jumpForwardButton->setCheckable(true);
  this->jumpForwardButton->setChecked(false);
  //this->jumpForwardButton->setIconSize(QSize(32, 32));
  this->jumpForwardButton->setObjectName("logStepButton");
  */

  this->scrubber = new TimeLine(this);
  this->scrubber->setTickInterval(1);
  this->scrubber->setOrientation(Qt::Horizontal);
  this->scrubber->setValue(0);
  this->scrubber->setPageStep(50);
  connect(this->scrubber, SIGNAL(valueChanged(int)),
      this, SLOT(OnScrubber(int)));
  connect(this->scrubber, SIGNAL(sliderPressed()),
      this, SLOT(OnScrubberPressed()));
  connect(this->scrubber, SIGNAL(sliderReleased()),
      this, SLOT(OnScrubberReleased()));

  QToolBar *playToolbar = new QToolBar;

  playToolbar->addAction(g_stepBackwardAct);
  playToolbar->addAction(g_playAct);
  playToolbar->addAction(g_pauseAct);
  playToolbar->addAction(g_stepForwardAct);

  QHBoxLayout *controlsLayout = new QHBoxLayout;
  controlsLayout->addStretch(0);
  controlsLayout->addWidget(playToolbar);
  /*controlsLayout->addWidget(this->jumpBackwardButton, 1);
  controlsLayout->addWidget(this->stepBackwardButton, 1);
  controlsLayout->addWidget(this->playButton, 1);
  controlsLayout->addWidget(this->stepForwardButton, 1);
  controlsLayout->addWidget(this->jumpForwardButton, 1);
  */
  controlsLayout->addStretch(0);

  QVBoxLayout *frameLayout = new QVBoxLayout;
  frameLayout->addLayout(controlsLayout);
  frameLayout->addWidget(this->scrubber);

  QFrame *frame = new QFrame;
  frame->setLayout(frameLayout);
  frame->layout()->setContentsMargins(0, 0, 0, 0);

  mainLayout->addWidget(frame);
  this->setLayout(mainLayout);

  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  this->layout()->setContentsMargins(0, 0, 0, 0);

  // Create a QueuedConnection. This is used for thread safety.
  connect(this, SIGNAL(SetRange(unsigned int)),
          this, SLOT(OnSetRange(unsigned int)), Qt::QueuedConnection);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->controlPub = this->node->Advertise<msgs::LogPlayControl>(
      "/gazebo/log/play/control");

  this->statusSub = this->node->Subscribe("/gazebo/log/play/status",
      &LogPlayWidget::OnStatusMsg, this, true);

  this->worldControlPub =
    this->node->Advertise<msgs::WorldControl>("~/world_control");

  this->statsSub = this->node->Subscribe("~/world_stats",
      &LogPlayWidget::OnStats, this);

  std::cout << "TopicNamespace[" << this->node->GetTopicNamespace() << "]\n";
}

/////////////////////////////////////////////////
void LogPlayWidget::Init()
{
  this->show();
}

/////////////////////////////////////////////////
LogPlayWidget::~LogPlayWidget()
{
}

/////////////////////////////////////////////////
void LogPlayWidget::OnSetRange(unsigned int _max)
{
  std::list<std::string> list;
  transport::get_topic_namespaces(list);

  std::cout << "====\n";
  for (std::list<std::string>::iterator iter = list.begin(); iter != list.end(); ++iter)
  {
    std::cout << *iter << "\n";
  }

  this->scrubber->setRange(0, _max);
}

/////////////////////////////////////////////////
void LogPlayWidget::OnScrubber(int _value)
{
  msgs::LogPlayControl msg;
  msg.set_target_step(_value);

  this->controlPub->Publish(msg);
}

/////////////////////////////////////////////////
void LogPlayWidget::OnStats(ConstWorldStatisticsPtr &_msg)
{
  printf("OnStats\n");
  this->scrubber->SetTime(msgs::Convert(_msg->sim_time()));
}

/////////////////////////////////////////////////
void LogPlayWidget::OnStatusMsg(ConstLogPlayStatusPtr &_msg)
{
  if (!this->sliderPressed)
  {
    if (static_cast<uint64_t>(this->scrubber->maximum()) != _msg->segments())
      this->SetRange(_msg->segments()-1);

    if (static_cast<uint64_t>(this->scrubber->value()) != _msg->step())
    {
      this->scrubber->blockSignals(true);
      this->scrubber->setValue(_msg->step());
      this->scrubber->blockSignals(false);
    }
  }
}

/////////////////////////////////////////////////
void LogPlayWidget::OnScrubberPressed()
{
  this->sliderPressed = true;
  msgs::LogPlayControl msg;
  msg.set_pause(true);
  this->controlPub->Publish(msg);
}

/////////////////////////////////////////////////
void LogPlayWidget::OnScrubberReleased()
{
  this->sliderPressed = false;
  msgs::LogPlayControl msg;
  msg.set_pause(false);
  this->controlPub->Publish(msg);
}

/////////////////////////////////////////////////
void LogPlayWidget::OnPlay(bool _toggle)
{
  msgs::WorldControl msg;

  if (_toggle)
  {
    // this->playButton->setIcon(QPixmap(":/images/pause.svg"));
    msg.set_pause(false);
  }
  else
  {
    // this->playButton->setIcon(QPixmap(":/images/play.svg"));
    msg.set_pause(true);
  }

  // g_pauseAct->setVisible(true);
  // g_playAct->setVisible(false);
  this->worldControlPub->Publish(msg);

  /*this->sliderPressed = false;
  msgs::LogPlayControl msg;
  msg.set_pause(false);
  msg.set_start(true);
  this->controlPub->Publish(msg);
  */
}
