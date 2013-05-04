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

#include "gazebo/transport/transport.hh"
#include "gazebo/util/LogPlay.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/LogPlayWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
LogPlayWidget::LogPlayWidget(QWidget *_parent)
  : QWidget(_parent)
{
  //this->setObjectName("dataPlayBack");

  QHBoxLayout *mainLayout = new QHBoxLayout;

  QFrame *frame = new QFrame;
  QHBoxLayout *frameLayout = new QHBoxLayout;

  this->scrubber = new QSlider(this);
  this->scrubber->setTickInterval(1);
  this->scrubber->setOrientation(Qt::Horizontal);
  this->scrubber->setValue(0);
  connect(this->scrubber, SIGNAL(valueChanged(int)),
      this, SLOT(OnScrubber(int)));
  frameLayout->addWidget(this->scrubber);

  frame->setLayout(frameLayout);
  frame->layout()->setContentsMargins(10, 10, 10, 10);

  mainLayout->addWidget(frame);
  this->setLayout(mainLayout);

  this->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  this->layout()->setContentsMargins(0, 0, 0, 0);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init("/gazebo");

  this->statusSub = this->node->Subscribe("/gazebo/log/play/status",
      &LogPlayWidget::OnStatusMsg, this);

  // Create a QueuedConnection. This is used for thread safety.
  connect(this, SIGNAL(SetRange(unsigned int)),
          this, SLOT(OnSetRange(unsigned int)), Qt::QueuedConnection);
}

/////////////////////////////////////////////////
LogPlayWidget::~LogPlayWidget()
{
}

/////////////////////////////////////////////////
void LogPlayWidget::OnSetRange(unsigned int _max)
{
  this->show();
  this->scrubber->setRange(0, _max);
}

/////////////////////////////////////////////////
void LogPlayWidget::OnScrubber(int _value)
{
  printf("On Scrubber[%d]\n", _value);
}

/////////////////////////////////////////////////
void LogPlayWidget::OnStatusMsg(ConstLogPlayStatusPtr &_msg)
{
  this->SetRange(_msg->segments());
}
