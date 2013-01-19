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
  this->recordButton->setIconSize(QSize(30, 30));
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
  this->stopButton->setIconSize(QSize(16, 16));
  this->stopButton->setObjectName("dataLoggerStopButton");
  connect(this->stopButton, SIGNAL(clicked()), this, SLOT(OnStop()));

  // Position the stop button next to the record button
  stopButtonLayout->addStretch(1);
  stopButtonLayout->addWidget(this->stopButton, 0);

  QVBoxLayout *infoLayout = new QVBoxLayout;
  this->timeLabel = new QLabel("00:00:00.00");
  this->timeLabel->setObjectName("dataLoggerTimeLabel");

  this->sizeLabel = new QLabel("0 KB");
  this->sizeLabel->setObjectName("dataLoggerSizeLabel");

  infoLayout->addStretch(1);
  infoLayout->addWidget(this->timeLabel);
  infoLayout->addWidget(this->sizeLabel);
  infoLayout->addStretch(1);

  buttonLayout->addWidget(this->recordButton);
  buttonLayout->addLayout(stopButtonLayout);
  buttonLayout->addSpacing(10);
  buttonLayout->addStretch(1);
  buttonLayout->addLayout(infoLayout);
  buttonLayout->addSpacing(5);

  this->statusLabel = new QLabel;
  this->statusLabel->setObjectName("dataLoggerStatusLabel");
  this->statusLabel->setText("ready");

  mainLayout->addLayout(buttonLayout);
  mainLayout->addWidget(this->statusLabel);

  // Let the stylesheet handle the margin sizes
  mainLayout->setContentsMargins(0, 0, 0, 0);

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

  this->recording = false;
  this->paused = false;
}

/////////////////////////////////////////////////
DataLogger::~DataLogger()
{
}

/////////////////////////////////////////////////
void DataLogger::OnRecord(bool _toggle)
{
  // If _toggle, then we should start logging data.
  if (!this->recording || this->paused)
  {
    // Switch the icon
    this->recordButton->setIcon(QPixmap(":/images/record_pause.png"));

    // Tell the server to start data logging
    msgs::LogControl msg;

    if (!this->paused)
      msg.set_start(true);
    else
      msg.set_paused(false);

    this->pub->Publish(msg);
    this->statusLabel->setText("recording");
    this->recording = true;
    this->paused = false;
  }
  // Otherwise pause data logging
  else
  {
    // Switch the icon
    this->recordButton->setIcon(QPixmap(":/images/record.png"));

    // Tell the server to pause data logging
    msgs::LogControl msg;
    msg.set_paused(true);
    this->pub->Publish(msg);

    this->statusLabel->setText("paused");
    this->paused = true;
  }
}

/////////////////////////////////////////////////
void DataLogger::OnStop()
{
  this->recording = false;
  this->paused = false;

  // Switch the icon
  this->recordButton->setIcon(QPixmap(":/images/record.png"));

  // Tell the server to stop data logging
  msgs::LogControl msg;
  msg.set_stop(true);
  this->pub->Publish(msg);
  this->statusLabel->setText("ready");
}

/////////////////////////////////////////////////
void DataLogger::OnStatus(ConstLogStatusPtr &_msg)
{
  // A new log status message has arrived, let's display the contents.
  common::Time time = msgs::Convert(_msg->start());
  std::ostringstream stream;

  unsigned int hours = time.sec / 3600;
  unsigned int min = (time.sec - hours * 3600) / 60;
  unsigned int sec = (time.sec - hours * 3600  - min * 60);
  unsigned int msec = rint(time.nsec * 1e-6);

  stream << std::setw(2) << std::setfill('0') << hours  << ":";
  stream << std::setw(2) << std::setfill('0') << min << ":";
  stream << std::setw(2) << std::setfill('0') << sec << ".";
  stream << std::setw(2) << std::setfill('0') << msec;

  this->timeLabel->setText(QString::fromStdString(stream.str()));
}
