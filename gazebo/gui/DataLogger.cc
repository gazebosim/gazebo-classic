/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
  // For _access()
  #include <io.h>
  #define access _access
#endif

#include <boost/filesystem.hpp>
#include <stdio.h>

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
  statusFrame->setFixedWidth(240);
  statusFrame->setObjectName("dataLoggerStatusFrame");

  // Textual status information
  this->statusLabel = new QLabel("Ready");
  this->statusLabel->setObjectName("dataLoggerStatusLabel");
  this->statusLabel->setFixedWidth(70);

  // Duration of logging
  this->timeLabel = new QLabel("00:00:00.000");
  this->timeLabel->setObjectName("dataLoggerTimeLabel");
  this->timeLabel->setFixedWidth(90);

  // Size of log file
  this->sizeLabel = new QLabel("0.00 B");
  this->sizeLabel->setObjectName("dataLoggerSizeLabel");

  QHBoxLayout *timeLayout = new QHBoxLayout;
  timeLayout->addStretch(1);
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
  QFrame *settingsMasterFrame = new QFrame;
  settingsMasterFrame->setObjectName("dataLoggerSettingFrame");

  this->logList = new QTextBrowser(this);
  this->logList->setObjectName("dataLoggerRecordingsList");

  QVBoxLayout *logListLayout = new QVBoxLayout;
  logListLayout->addWidget(this->logList);

  /*QHBoxLayout *filenameLayout = new QHBoxLayout;
  this->filenameEdit = new QLineEdit;
  this->filenameEdit->setText("~/.gazebo/log");
  filenameLayout->addWidget(new QLabel("Save to:"));
  filenameLayout->addWidget(this->filenameEdit);

  this->browseButton = new QPushButton("Browse");
  this->browseButton->setFixedHeight(23);
  this->browseButton->setFocusPolicy(Qt::NoFocus);
  connect(browseButton, SIGNAL(clicked()), this, SLOT(OnBrowse()));

  filenameLayout->addWidget(browseButton);
  */

  QHBoxLayout *settingExpandLayout = new QHBoxLayout;

  this->settingExpandButton = new QPushButton("Recordings");
  this->settingExpandButton->setObjectName("expandButton");
  this->settingExpandButton->setCheckable(true);
  this->settingExpandButton->setChecked(false);
  this->settingExpandButton->setFocusPolicy(Qt::NoFocus);
  connect(settingExpandButton, SIGNAL(toggled(bool)),
          this, SLOT(OnToggleSettings(bool)));

  settingExpandLayout->setContentsMargins(0, 0, 0, 0);
  settingExpandLayout->addWidget(this->settingExpandButton);
  settingExpandLayout->addStretch(1);

  /// Create the frame that can be hidden by toggling the setting's button
  this->settingsFrame = new QFrame;
  QVBoxLayout *settingsLayout = new QVBoxLayout;
  settingsLayout->setContentsMargins(2, 2, 2, 2);
  settingsLayout->addLayout(logListLayout);
  this->settingsFrame->setLayout(settingsLayout);
  this->settingsFrame->hide();

  QVBoxLayout *settingsMasterFrameLayout = new QVBoxLayout;
  settingsMasterFrameLayout->setContentsMargins(2, 2, 2, 2);

  settingsMasterFrameLayout->addLayout(settingExpandLayout);
  settingsMasterFrameLayout->addWidget(this->settingsFrame);
  settingsMasterFrame->setLayout(settingsMasterFrameLayout);
  // }

  // Layout to position the record button vertically
  QVBoxLayout *buttonLayout = new QVBoxLayout;
  buttonLayout->setContentsMargins(0, 0, 0, 0);
  buttonLayout->addWidget(this->recordButton);
  buttonLayout->addStretch(4);

  // Layout to position the status information vertically
  QVBoxLayout *statusLayout = new QVBoxLayout;
  statusLayout->setContentsMargins(0, 0, 0, 0);
  statusLayout->addWidget(statusFrame);

  // Horizontal layout for the record button and status information
  QHBoxLayout *topLayout = new QHBoxLayout;
  topLayout->setContentsMargins(0, 0, 0, 0);
  topLayout->addLayout(buttonLayout);
  topLayout->addStretch(4);
  topLayout->addLayout(statusLayout);

  QHBoxLayout *destPathLayout = new QHBoxLayout;
  this->destPath = new QLineEdit;
  this->destPath->setReadOnly(true);
  this->destPath->setObjectName("dataLoggerDestnationPathLabel");
  this->destPath->setStyleSheet(
      "QLineEdit {color: #aeaeae; font-size: 11px; "
      "background-color: transparent;}");

  QLabel *pathLabel = new QLabel("Path: ");
  pathLabel->setStyleSheet(
      "QLabel {color: #aeaeae; font-size: 11px; background: transparent}");

  destPathLayout->setContentsMargins(0, 0, 0, 0);
  destPathLayout->addSpacing(4);
  destPathLayout->addWidget(pathLabel);
  destPathLayout->addWidget(this->destPath);

  QHBoxLayout *destURILayout = new QHBoxLayout;
  this->destURI = new QLineEdit;
  this->destURI->setReadOnly(true);
  this->destURI->setObjectName("dataLoggerDestnationURILabel");
  this->destURI->setStyleSheet(
      "QLineEdit {color: #aeaeae; font-size: 11px; background: transparent}");
  QLabel *uriLabel = new QLabel("Address: ");
  uriLabel->setStyleSheet(
      "QLabel {color: #aeaeae; font-size: 11px; background: transparent}");
  destURILayout->setContentsMargins(0, 0, 0, 0);
  destURILayout->addSpacing(4);
  destURILayout->addWidget(uriLabel);
  destURILayout->addWidget(this->destURI);

  // Mainlayout for the whole widget
  // Create the main layout for this widget
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addLayout(topLayout);
  mainLayout->addLayout(destURILayout);
  mainLayout->addLayout(destPathLayout);
  mainLayout->addWidget(settingsMasterFrame);

  // Let the stylesheet handle the margin sizes
  mainLayout->setContentsMargins(2, 2, 2, 2);

  // Assign the mainlayout to this widget
  this->setLayout(mainLayout);
  this->layout()->setSizeConstraint(QLayout::SetFixedSize);

  // Create a QueuedConnection to set time. This is used for thread safety.
  connect(this, SIGNAL(SetTime(QString)),
          this->timeLabel, SLOT(setText(QString)), Qt::QueuedConnection);

  // Create a QueuedConnection to set size. This is used for thread safety.
  connect(this, SIGNAL(SetSize(QString)),
          this->sizeLabel, SLOT(setText(QString)), Qt::QueuedConnection);

  // Create a QueuedConnection to set destination path.
  // This is used for thread safety.
  connect(this, SIGNAL(SetDestinationPath(QString)),
          this, SLOT(OnSetDestinationPath(QString)), Qt::QueuedConnection);

  // Create a QueuedConnection to set destination URI.
  // This is used for thread safety.
  connect(this, SIGNAL(SetDestinationURI(QString)),
          this, SLOT(OnSetDestinationURI(QString)), Qt::QueuedConnection);

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
    this->recordButton->setIcon(QPixmap(":/images/record_stop.png"));

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

    this->logList->append(this->destPath->text());
  }
}

/////////////////////////////////////////////////
void DataLogger::OnStatus(ConstLogStatusPtr &_msg)
{
  // A new log status message has arrived, let's display the contents.
  common::Time time = msgs::Convert(_msg->sim_time());
  std::ostringstream stream;

  // Compute the hours, minutes, seconds, and milliseconds that logging has
  // been running.
  unsigned int hours = time.sec / 3600;
  unsigned int min = (time.sec - hours * 3600) / 60;
  unsigned int sec = (time.sec - hours * 3600  - min * 60);
  unsigned int msec = rint(time.nsec * 1e-6);

  // Display the time.
  stream << std::setw(2) << std::setfill('0') << hours  << ":";
  stream << std::setw(2) << std::setfill('0') << min << ":";
  stream << std::setw(2) << std::setfill('0') << sec << ".";
  stream << std::setw(3) << std::setfill('0') << msec;

  this->SetTime(QString::fromStdString(stream.str()));

  // Reset the stream
  stream.str("");

  // If there is log file information in the message...
  if (_msg->has_log_file())
  {
    // If there is file name information...
    if (_msg->log_file().has_base_path())
    {
      std::string basePath = _msg->log_file().base_path();

      // Display the leaf log filename
      if (_msg->log_file().has_full_path() && !basePath.empty())
      {
        std::string leaf = _msg->log_file().full_path();
        if (!leaf.empty())
          leaf = leaf.substr(basePath.size());
        this->SetDestinationPath(QString::fromStdString(leaf));
      }
    }

    // Display the URI
    if (_msg->log_file().has_uri())
      this->SetDestinationURI(QString::fromStdString(_msg->log_file().uri()));
    else
      this->SetDestinationURI(tr(""));

    // If there is log file size information...
    if (_msg->log_file().has_size() && _msg->log_file().has_size_units())
    {
      // Get the size of the log file.
      stream << std::fixed << std::setprecision(2) << _msg->log_file().size();


      // Get the size units.
      switch (_msg->log_file().size_units())
      {
        case msgs::LogStatus::LogFile::BYTES:
          stream << "B";
          break;
        case msgs::LogStatus::LogFile::K_BYTES:
          stream << "KB";
          break;
        case msgs::LogStatus::LogFile::M_BYTES:
          stream << "MB";
          break;
        default:
          stream << "GB";
          break;
      }

      this->SetSize(QString::fromStdString(stream.str()));
    }
    else
      this->SetSize("0.00 B");
  }
}

/////////////////////////////////////////////////
void DataLogger::OnSetDestinationPath(QString _filename)
{
  if (!_filename.isEmpty())
    this->destPath->setText(_filename);
  else
    this->destPath->setText("");
}

/////////////////////////////////////////////////
void DataLogger::OnSetDestinationURI(QString _uri)
{
  if (!_uri.isEmpty())
    this->destURI->setText(_uri);
  else
    this->destURI->setText("");
}

/////////////////////////////////////////////////
void DataLogger::OnToggleSettings(bool _checked)
{
  if (_checked)
    this->settingsFrame->show();
  else
    this->settingsFrame->hide();
}

/////////////////////////////////////////////////
void DataLogger::OnBrowse()
{
  boost::filesystem::path path = QFileDialog::getExistingDirectory(this,
      tr("Set log directory"), this->filenameEdit->text(),
      QFileDialog::ShowDirsOnly |
      QFileDialog::DontResolveSymlinks).toStdString();

  // Make sure the  directory exists
  if (!boost::filesystem::exists(path))
  {
    QMessageBox msgBox(this);
    std::ostringstream stream;
    stream << "Directory " << path << " does not exist.";
    msgBox.setText(stream.str().c_str());
    msgBox.exec();
    return;
  }

  // Make sure we have a directory
  if (!boost::filesystem::is_directory(path))
  {
    QMessageBox msgBox(this);
    std::ostringstream stream;
    stream << "Path " << path << " is not a directory. Please only specify a "
           << "directory for data logging.";
    msgBox.setText(stream.str().c_str());
    msgBox.exec();
    return;
  }

  // Make sure the path is writable.
  // Note: This is not cross-platform compatible.
#ifdef _WIN32
  // Check for write-only (2) and read-write (6)
  if ((access(path.string().c_str(), 2) != 0) &&
      (access(path.string().c_str(), 6) != 0))
#else
  if (access(path.string().c_str(), W_OK) != 0)
#endif
  {
    QMessageBox msgBox(this);
    std::ostringstream stream;
    stream << "You do no have permission to write into " << path;
    msgBox.setText(stream.str().c_str());
    msgBox.exec();
    return;
  }

  // Set the new base path
  msgs::LogControl msg;
  msg.set_base_path(path.string());
  this->pub->Publish(msg);

  this->SetFilename(QString::fromStdString(path.string()));
}
