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

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/gui/DataLogger.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
DataLogger::DataLogger(QWidget *_parent)
  : QDialog(_parent)
{
  // This name is used in the qt style sheet
  this->setObjectName("dataLogger");
  this->setWindowIcon(QIcon(":/images/gazebo.svg"));
  this->setWindowTitle(tr("Gazebo: Data Logger"));

  // Create the main layout for this widget
  QVBoxLayout *mainLayout = new QVBoxLayout;

  QHBoxLayout *buttonLayout = new QHBoxLayout;

  this->recordButton = new QToolButton(this);
  this->recordButton->setIcon(QPixmap(":/images/record.png"));
  this->recordButton->setStatusTip(tr("Record a log file"));
  this->recordButton->setCheckable(true);
  this->recordButton->setChecked(false);
  this->recordButton->setIconSize(QSize(48, 48));
  this->recordButton->setObjectName("dataLoggerRecordButton");
  connect(this->recordButton, SIGNAL(toggled(bool)),
          this, SLOT(OnRecord(bool)));

  QVBoxLayout *stopButtonLayout = new QVBoxLayout;
  this->stopButton = new QToolButton(this);
  this->stopButton->setIcon(QIcon(":/images/stop.png"));
  this->stopButton->setStatusTip(tr("Stop recording of a log file"));
  this->stopButton->setCheckable(true);
  this->stopButton->setChecked(true);
  this->stopButton->setIconSize(QSize(24, 24));
  this->stopButton->setObjectName("dataLoggerStopButton");
  connect(this->stopButton, SIGNAL(clicked()), this, SLOT(OnStop()));

  stopButtonLayout->addStretch(1);
  stopButtonLayout->addWidget(this->stopButton, 0);

  this->timeLabel = new QLabel("hh:mm:ss");

  buttonLayout->addWidget(this->recordButton);
  buttonLayout->addLayout(stopButtonLayout);
  buttonLayout->addSpacing(10);
  buttonLayout->addStretch(1);
  buttonLayout->addWidget(this->timeLabel);
  buttonLayout->addSpacing(5);

  mainLayout->addLayout(buttonLayout);

  // Let the stylesheet handle the margin sizes
  mainLayout->setContentsMargins(4, 4, 4, 4);

  // Assign the mainlayout to this widget
  this->setLayout(mainLayout);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->pub = this->node->Advertise<msgs::LogControl>("~/log/control");
}

/////////////////////////////////////////////////
DataLogger::~DataLogger()
{
}

/////////////////////////////////////////////////
void DataLogger::OnRecord(bool _toggle)
{
  if (_toggle)
  {
    this->recordButton->setIcon(QPixmap(":/images/record_pause.png"));

    msgs::LogControl msg;
    msg.set_start(true);
    this->pub->Publish(msg);
  }
  else
  {
    this->recordButton->setIcon(QPixmap(":/images/record.png"));

    msgs::LogControl msg;
    msg.set_stop(true);
    this->pub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void DataLogger::OnStop()
{
  msgs::LogControl msg;
  msg.set_stop(true);
  this->pub->Publish(msg);
}
