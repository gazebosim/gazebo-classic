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

  // The record button allows the user to start and pause data recording
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

  // The stop button stops and closes an open log file.
  this->stopButton = new QToolButton(this);
  this->stopButton->setIcon(QIcon(":/images/stop.png"));
  this->stopButton->setStatusTip(tr("Stop recording of a log file"));
  this->stopButton->setCheckable(true);
  this->stopButton->setChecked(true);
  this->stopButton->setIconSize(QSize(24, 24));
  this->stopButton->setObjectName("dataLoggerStopButton");
  connect(this->stopButton, SIGNAL(clicked()), this, SLOT(OnStop()));

  // Position the stop button next to the record button
  stopButtonLayout->addStretch(1);
  stopButtonLayout->addWidget(this->stopButton, 0);

  this->timeLabel = new QLabel("hh:mm:ss");

  buttonLayout->addWidget(this->recordButton);
  buttonLayout->addLayout(stopButtonLayout);
  buttonLayout->addSpacing(10);
  buttonLayout->addStretch(1);
  buttonLayout->addWidget(this->timeLabel);
  buttonLayout->addSpacing(5);

  QFrame *statusFrame = new QFrame;

  mainLayout->addLayout(buttonLayout);
  mainLayout->addWidget(statusFrame);


  // Let the stylesheet handle the margin sizes
  mainLayout->setContentsMargins(4, 4, 4, 4);

  // Assign the mainlayout to this widget
  this->setLayout(mainLayout);
  this->layout()->setSizeConstraint(QLayout::SetFixedSize);

  // Create a node from communication.
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  // Advertise on the log control topic. The server listens to log control
  // messages.
  this->pub = this->node->Advertise<msgs::LogControl>("~/log/control");

  // Subscribe to the log status topic. The server publishes log status
  // messages.
  this->sub = this->node->Subscribe<msgs::LogStatus>("~/log/status",
      &DataLogger::OnStatus, this);
}

/////////////////////////////////////////////////
DataLogger::~DataLogger()
{
}

/////////////////////////////////////////////////
void DataLogger::OnRecord(bool _toggle)
{
  // If _toggle, then we should start logging data.
  if (_toggle)
  {
    // Switch the icon
    this->recordButton->setIcon(QPixmap(":/images/record_pause.png"));

    // Tell the server to start data logging
    msgs::LogControl msg;
    msg.set_start(true);
    this->pub->Publish(msg);
  }
  // Otherwise pause data logging
  else
  {
    // Switch the icon
    this->recordButton->setIcon(QPixmap(":/images/record.png"));

    // Tell the server to stop data logging
    msgs::LogControl msg;
    msg.set_stop(true);
    this->pub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void DataLogger::OnStop()
{
  // Tell the server to stop data logging
  msgs::LogControl msg;
  msg.set_stop(true);
  this->pub->Publish(msg);
}

/////////////////////////////////////////////////
void DataLogger::OnStatus(ConstLogStatusPtr &_msg)
{
  // A new log status message has arrived, let's display the contents.
  common::Time time = msgs::Convert(_msg->start());
  std::ostringstream stream;

  int hours = time.sec / 3600;
  int min = (time.sec - hours * 3600) / 60;
  int sec = (time.sec - hours * 3600  - min * 60);
  ent msec = rint(time.nsec * 1e-6);

  stream << hours  << ":" << min << ":" << sec << "." << msec;

  this->timeLabel->setText(QString::fromStdString(stream.str()));
}
