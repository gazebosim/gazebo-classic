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

  // Create the status frame, which contains the duration label, size
  // label, and a one word text message.
  // {
  QFrame *statusFrame = new QFrame;
  statusFrame->setObjectName("dataLoggerStatusFrame");

  this->statusLabel = new QLabel("Ready");
  this->statusLabel->setFixedWidth(70);

  this->timeLabel = new QLabel("00:00:00.000");
  this->timeLabel->setFixedWidth(85);

  this->sizeLabel = new QLabel("0 MB");

  // Create a QueuedConnection to set time. This is used for thread safety.
  connect(this, SIGNAL(SetTime(QString)),
          this->timeLabel, SLOT(setText(QString)), Qt::QueuedConnection);

  // Create a QueuedConnection to set size. This is used for thread safety.
  connect(this, SIGNAL(SetSize(QString)),
          this->sizeLabel, SLOT(setText(QString)), Qt::QueuedConnection);

  // Create a QueuedConnection to set filename. This is used for thread safety.
  connect(this, SIGNAL(SetFilename(QString)),
          this, SLOT(OnSetFilename(QString)), Qt::QueuedConnection);


  QHBoxLayout *timeLayout = new QHBoxLayout;
  timeLayout->addWidget(this->statusLabel);
  timeLayout->addSpacing(5);
  timeLayout->addWidget(this->timeLabel);

  QHBoxLayout *sizeLayout = new QHBoxLayout;
  sizeLayout->addStretch(1);
  sizeLayout->addWidget(this->sizeLabel);

  QVBoxLayout *statusFrameLayout = new QVBoxLayout;
  statusFrameLayout->addLayout(timeLayout);
  statusFrameLayout->addLayout(sizeLayout);

  statusFrame->setLayout(statusFrameLayout);
  // }

  // Create the settings frame, where the user can input a save-to location
  // {
  QFrame *settingFrame = new QFrame;
  settingFrame->setObjectName("dataLoggerSettingFrame");
  QVBoxLayout *settingFrameLayout = new QVBoxLayout;

  QHBoxLayout *filenameLayout = new QHBoxLayout;
  this->filenameEdit = new QLineEdit;
  filenameLayout->addWidget(this->filenameEdit);
  filenameLayout->addWidget(new QPushButton("..."));

  settingFrameLayout->addWidget(new QLabel("Settings"));
  settingFrameLayout->addLayout(filenameLayout);
  settingFrame->setLayout(settingFrameLayout);
  // }

  // Layout to position the record button vertically
  QVBoxLayout *buttonLayout = new QVBoxLayout;
  buttonLayout->addWidget(this->recordButton);
  buttonLayout->addStretch(4);

  // Layout to position the status information vertically
  QVBoxLayout *statusLayout = new QVBoxLayout;
  statusLayout->addSpacing(10);
  statusLayout->addWidget(statusFrame);

  // Horizontal layout for the record button and status information
  QHBoxLayout *topLayout = new QHBoxLayout;
  topLayout->addLayout(buttonLayout);
  topLayout->addSpacing(10);
  topLayout->addStretch(4);
  topLayout->addLayout(statusLayout);

  // Mainlayout for the whole widget
  // Create the main layout for this widget
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addLayout(topLayout);
  mainLayout->addWidget(settingFrame);

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

    this->statusLabel->setText("Recording");

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

    this->statusLabel->setText("Ready");

    // Tell the server to stop data logging
    msgs::LogControl msg;
    msg.set_stop(true);
    this->pub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void DataLogger::OnStatus(ConstLogStatusPtr &_msg)
{
  // A new log status message has arrived, let's display the contents.
  common::Time time = msgs::Convert(_msg->sim_time());
  std::ostringstream stream;

  unsigned int hours = time.sec / 3600;
  unsigned int min = (time.sec - hours * 3600) / 60;
  unsigned int sec = (time.sec - hours * 3600  - min * 60);
  unsigned int msec = rint(time.nsec * 1e-6);

  stream << std::setw(2) << std::setfill('0') << hours  << ":";
  stream << std::setw(2) << std::setfill('0') << min << ":";
  stream << std::setw(2) << std::setfill('0') << sec << ".";
  stream << std::setw(3) << std::setfill('0') << msec;

  this->SetTime(QString::fromStdString(stream.str()));


  stream.str("");

  if (_msg->has_log_file())
  {
    this->SetFilename(QString::fromStdString(_msg->log_file().base_path()));

    stream << std::fixed << std::setprecision(2) << _msg->log_file().size();
    if (_msg->log_file().size_units() == msgs::LogStatus::LogFile::BYTES)
      stream << "B";
    else if (_msg->log_file().size_units() == msgs::LogStatus::LogFile::K_BYTES)
      stream << "KB";
    else if (_msg->log_file().size_units() == msgs::LogStatus::LogFile::M_BYTES)
      stream << "MB";
    else
      stream << "GB";

    this->SetSize(QString::fromStdString(stream.str()));
  }
}

/////////////////////////////////////////////////
void DataLogger::OnSetFilename(QString _filename)
{

  std::string filename = _filename.toStdString();

  if (getenv("HOME"))
  {
    std::string homeDir = getenv("HOME");
    boost::replace_first(filename, homeDir, "~");
  }

  this->filenameEdit->setText(filename.c_str());
}
