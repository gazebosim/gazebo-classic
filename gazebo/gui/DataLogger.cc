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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
  // For _access()
  #include <io.h>
#endif

#include <boost/filesystem.hpp>
#include <stdio.h>

#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/SystemPaths.hh"

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/gui/DataLogger.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
DataLogger::DataLogger(QWidget *_parent)
  : QDialog(_parent)
{
  this->setWindowFlags(Qt::Window | Qt::WindowTitleHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  // This name is used in the qt style sheet
  this->setObjectName("dataLogger");
  this->setWindowIcon(QIcon(":/images/gazebo.svg"));
  this->setWindowTitle(tr("Gazebo: Data Logger"));
  this->setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

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

  // Textual status information
  this->statusLabel = new QLabel("Ready");
  this->statusLabel->setObjectName("dataLoggerStatusLabel");
  this->statusLabel->setStyleSheet(
      "QLabel{padding-left: 14px; padding-top: 9px}");

  // Duration of logging
  this->timeLabel = new QLabel("00:00:00.000");
  this->timeLabel->setObjectName("dataLoggerTimeLabel");
  this->timeLabel->setFixedWidth(90);

  // Size of log file
  this->sizeLabel = new QLabel("(0.00 B)");
  this->sizeLabel->setObjectName("dataLoggerSizeLabel");

  // Horizontal separator
  QFrame *separator = new QFrame();
  separator->setFrameShape(QFrame::HLine);
  separator->setLineWidth(1);
  separator->setFixedHeight(30);

  // Address label
  QLabel *uriLabel = new QLabel("Address: ");
  uriLabel->setStyleSheet(
      "QLabel {color: #aeaeae; font-size: 11px; background: transparent}");

  // Address URI Line Edit
  this->destURI = new QLineEdit;
  this->destURI->setReadOnly(true);
  this->destURI->setObjectName("dataLoggerDestnationURILabel");
  this->destURI->setStyleSheet(
      "QLineEdit {color: #aeaeae; font-size: 11px; background: transparent}");

  // "Save to" label
  QLabel *pathLabel = new QLabel("Save to: ");
  pathLabel->setStyleSheet(
      "QLabel {\
         color: #aeaeae;\
         font-size: 11px;\
         background: transparent;\
         padding-top: 4px;\
       }");

  // Destination path Text Edit
  this->destPath = new QPlainTextEdit();
  this->destPath->setObjectName("dataLoggerDestnationPathLabel");
  this->destPath->setMinimumWidth(200);
  this->destPath->setMaximumHeight(50);
  this->destPath->setReadOnly(true);
  this->destPath->setFrameStyle(QFrame::NoFrame);
  this->destPath->setLineWrapMode(QPlainTextEdit::WidgetWidth);
  this->destPath->setWordWrapMode(QTextOption::WrapAnywhere);
  this->destPath->setStyleSheet(
      "QPlainTextEdit {\
         color: #aeaeae;\
         font-size: 11px;\
         background: transparent;\
       }");

  // Browser button
  QPushButton *browseButton = new QPushButton("Browse");
  browseButton->setFixedWidth(100);
  browseButton->setFocusPolicy(Qt::NoFocus);
  connect(browseButton, SIGNAL(clicked()), this, SLOT(OnBrowse()));

  // Button which toggles recordings
  QRadioButton *recordingsButton = new QRadioButton();
  recordingsButton->setChecked(false);
  recordingsButton->setFocusPolicy(Qt::NoFocus);
  recordingsButton->setText("Recordings");
  recordingsButton->setStyleSheet(
     "QRadioButton {\
        color: #d0d0d0;\
      }\
      QRadioButton::indicator::unchecked {\
        image: url(:/images/right_arrow.png);\
      }\
      QRadioButton::indicator::checked {\
        image: url(:/images/down_arrow.png);\
      }");
  connect(recordingsButton, SIGNAL(toggled(bool)), this,
      SLOT(OnToggleSettings(bool)));

  // Insert widgets in the top layout
  QGridLayout *topLayout = new QGridLayout();
  topLayout->addWidget(this->recordButton, 0, 0, 2, 1);
  topLayout->addWidget(this->statusLabel, 0, 1, 2, 2);
  topLayout->addWidget(this->timeLabel, 0, 3);
  topLayout->addWidget(this->sizeLabel, 1, 3);
  topLayout->addWidget(separator, 2, 0, 1, 4);
  topLayout->addWidget(uriLabel, 3, 0);
  topLayout->addWidget(this->destURI, 3, 1, 1, 3);
  topLayout->addWidget(pathLabel, 4, 0);
  topLayout->addWidget(this->destPath, 4, 1, 1, 2);
  topLayout->addWidget(browseButton, 4, 3);
  topLayout->addWidget(recordingsButton, 5, 0, 1, 4);

  // Align widgets within layout
  topLayout->setAlignment(pathLabel, Qt::AlignTop | Qt::AlignRight);
  topLayout->setAlignment(browseButton, Qt::AlignTop);
  topLayout->setAlignment(this->statusLabel, Qt::AlignLeft);
  topLayout->setAlignment(this->timeLabel, Qt::AlignRight);
  topLayout->setAlignment(this->sizeLabel, Qt::AlignRight | Qt::AlignTop);
  topLayout->setAlignment(uriLabel, Qt::AlignRight);

  // Put the layout in a widget to be able to control size
  QWidget *topWidget = new QWidget();
  topWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
  topWidget->setLayout(topLayout);

  // List of recorded logs
  this->logList = new QTextBrowser(this);
  this->logList->setObjectName("dataLoggerRecordingsList");

  // Layout to hold the list
  QVBoxLayout *settingsLayout = new QVBoxLayout;
  settingsLayout->setContentsMargins(2, 2, 2, 2);
  settingsLayout->addWidget(logList);

  // Frame that can be hidden by toggling the expand button
  this->settingsFrame = new QFrame();
  this->settingsFrame->setObjectName("dataLoggerSettingFrame");
  this->settingsFrame->setLayout(settingsLayout);
  this->settingsFrame->setSizePolicy(QSizePolicy::Expanding,
      QSizePolicy::Expanding);
  this->settingsFrame->hide();

  // Mainlayout for the whole widget
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->setSizeConstraint(QLayout::SetMinAndMaxSize);
  mainLayout->addWidget(topWidget);
  mainLayout->addWidget(this->settingsFrame);

  // Assign the mainlayout to this widget
  this->setLayout(mainLayout);

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

  connect(this, SIGNAL(rejected()), this, SLOT(OnCancel()));

  // Timer used to blink the status label
  this->statusTimer = new QTimer();
  connect(this->statusTimer, SIGNAL(timeout()), this, SLOT(OnBlinkStatus()));
  this->statusTime = 0;

  // Timer used to hide the confirmation dialog
  this->confirmationDialog = NULL;
  this->confirmationTimer = new QTimer(this);
  connect(this->confirmationTimer, SIGNAL(timeout()), this,
      SLOT(OnConfirmationTimeout()));

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

  // Fill the path with the home folder - duplicated from util/LogRecord
#ifndef _WIN32
  const char *homePath = common::getEnv("HOME");
#else
  const char *homePath = common::getEnv("HOMEPATH");
#endif

  GZ_ASSERT(homePath, "HOME environment variable is missing");

  if (!homePath)
  {
    common::SystemPaths *paths = common::SystemPaths::Instance();
    this->basePath = QString::fromStdString(paths->GetTmpPath() + "/gazebo");
  }
  else
  {
    this->basePath =
        QString::fromStdString(boost::filesystem::path(homePath).string());
  }

  this->basePath = this->basePath + "/.gazebo/log/";
  this->SetDestinationPath(this->basePath);
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

    this->statusLabel->setText("Recording...");
    this->statusTimer->start(100);

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

    // Display confirmation
    this->confirmationTimer->start(2000);
    QLabel *confirmationLabel = new QLabel("Saved to \n" +
        this->destPath->toPlainText());
    confirmationLabel->setObjectName("dataLoggerConfirmationLabel");
    QHBoxLayout *confirmationLayout = new QHBoxLayout();
    confirmationLayout->addWidget(confirmationLabel);

    if (this->confirmationDialog)
      this->confirmationDialog->close();
    this->confirmationDialog = new QDialog(this, Qt::FramelessWindowHint);
    this->confirmationDialog->setObjectName("dataLoggerConfirmationDialog");
    this->confirmationDialog->setLayout(confirmationLayout);
    this->confirmationDialog->setStyleSheet(
        "QDialog {background-color: #eee}\
         QLabel {color: #111}");
    this->confirmationDialog->setModal(false);
    this->confirmationDialog->show();
    this->confirmationDialog->move(this->mapToGlobal(
        QPoint((this->width()-this->confirmationDialog->width())*0.5,
        this->height() + 10)));

    // Change the status
    this->statusLabel->setText("Ready");
    this->statusLabel->setStyleSheet(
        "QLabel {\
           color: #aeaeae;\
           padding-left: 14px;\
           padding-top: 9px;\
         }");
    this->statusTimer->stop();

    // Change the Save to box
    this->SetDestinationPath(this->basePath);

    // Tell the server to stop data logging
    msgs::LogControl msg;
    msg.set_stop(true);
    this->pub->Publish(msg);

    this->logList->append(this->destPath->toPlainText());
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
    // If there is file name information and we're recording...
    if (_msg->log_file().has_base_path() && this->recordButton->isChecked())
    {
      std::string logBasePath = _msg->log_file().base_path();

      // Display the log path
      if (_msg->log_file().has_full_path() && !logBasePath.empty())
      {
        std::string fullPath = _msg->log_file().full_path();
        if (!fullPath.empty())
          this->SetDestinationPath(QString::fromStdString(fullPath));
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
      stream << std::fixed << std::setprecision(2) << "(" <<
          _msg->log_file().size();

      // Get the size units.
      switch (_msg->log_file().size_units())
      {
        case msgs::LogStatus::LogFile::BYTES:
          stream << "B)";
          break;
        case msgs::LogStatus::LogFile::K_BYTES:
          stream << "KB)";
          break;
        case msgs::LogStatus::LogFile::M_BYTES:
          stream << "MB)";
          break;
        default:
          stream << "GB)";
          break;
      }

      this->SetSize(QString::fromStdString(stream.str()));
    }
    else
      this->SetSize("(0.00 B)");
  }
}

/////////////////////////////////////////////////
void DataLogger::OnSetDestinationPath(QString _filename)
{
  if (!_filename.isEmpty())
    this->destPath->setPlainText(_filename);
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
  QFileDialog fileDialog(this, tr("Set log directory"), QDir::homePath());
  fileDialog.setFileMode(QFileDialog::Directory);
  fileDialog.setFilter(QDir::AllDirs | QDir::Hidden);
  fileDialog.setOptions(QFileDialog::ShowDirsOnly
      | QFileDialog::DontResolveSymlinks);

  if (fileDialog.exec() != QDialog::Accepted)
    return;

  QStringList selected = fileDialog.selectedFiles();
  if (selected.empty())
    return;

  boost::filesystem::path path = selected[0].toStdString();

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
  if ((_access(path.string().c_str(), 2) != 0) &&
      (_access(path.string().c_str(), 6) != 0))
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

  this->basePath = QString::fromStdString(path.string());
  this->SetDestinationPath(this->basePath);
}

/////////////////////////////////////////////////
void DataLogger::OnBlinkStatus()
{
  this->statusTime += 1.0/10;

  if (this->statusTime >= 1)
    this->statusTime = 0;

  this->statusLabel->setStyleSheet(QString::fromStdString(
      "QLabel{color: rgb("+
          std::to_string(255+(128*(this->statusTime-1)))+", "+
          std::to_string(255+(128*(this->statusTime-1)))+", "+
          std::to_string(255+(128*(this->statusTime-1)))+
      "); "+
      "padding-left: 15px; padding-top: 9px}"));
}

/////////////////////////////////////////////////
void DataLogger::OnConfirmationTimeout()
{
  this->confirmationDialog->close();
  this->confirmationTimer->stop();
}

/////////////////////////////////////////////////
void DataLogger::OnCancel()
{
  if (this->recordButton->isChecked())
    this->recordButton->click();
}
