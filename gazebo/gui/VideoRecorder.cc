/*
 * Copyright 2016 Open Source Robotics Foundation
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

#include "gazebo/gazebo_config.h"
#include "gazebo/common/CommonIface.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/VideoRecorder.hh"

using namespace gazebo;
using namespace gui;

// Private data class
class gazebo::gui::VideoRecorderPrivate
{
  /// \brief Button to enable mp4 recording.
  public: QToolButton *mp4Button = nullptr;

  /// \brief Button to enable ogv recording.
  public: QToolButton *ogvButton = nullptr;

  /// \brief Button to stop recording.
  public: QPushButton *stopButton = nullptr;

#if defined(__linux__) && defined(HAVE_AVDEVICE)
  /// \brief Button to enable v4l recording.
  public: QToolButton *v4lButton = nullptr;
#endif

  /// \brief Format of the video.
  public: std::string format;
};

/////////////////////////////////////////////////
VideoRecorder::VideoRecorder(QWidget *_parent)
: QWidget(_parent), dataPtr(new VideoRecorderPrivate)
{
  // MP4 recording button
  this->dataPtr->mp4Button = new QToolButton(this);
  this->dataPtr->mp4Button->setToolTip(tr("Record a MP4 video"));
  QAction *mp4Action = new QAction(QIcon(":/images/mp4.svg"),
      tr("Record mp4 video"), this);
  this->dataPtr->mp4Button->setDefaultAction(mp4Action);

  // OGV recording button
  this->dataPtr->ogvButton = new QToolButton(this);
  this->dataPtr->ogvButton->setToolTip(tr("Record an OGV video"));
  QAction *ogvAction = new QAction(QIcon(":/images/ogv.svg"),
      tr("Record ogv video"), this);
  this->dataPtr->ogvButton->setDefaultAction(ogvAction);

  // Set icon size
  QSize iconSize(32, 32);
  this->dataPtr->mp4Button->setIconSize(iconSize);
  this->dataPtr->ogvButton->setIconSize(iconSize);

  // Stop button
  this->dataPtr->stopButton = new QPushButton("Stop");
  this->dataPtr->stopButton->setToolTip(tr("End video and save"));
  this->dataPtr->stopButton->hide();
  connect(this->dataPtr->stopButton, SIGNAL(pressed()), this,
          SLOT(OnRecordStop()));

  QGridLayout *mainLayout = new QGridLayout();
  mainLayout->addWidget(this->dataPtr->mp4Button, 0, 1);
  mainLayout->addWidget(this->dataPtr->ogvButton, 0, 2);
  mainLayout->addWidget(this->dataPtr->stopButton, 2, 0);
  mainLayout->setContentsMargins(2, 2, 2, 2);

  // Map each video record button to the OnRecordStart slot
  QSignalMapper *signalMapper = new QSignalMapper(this);
  connect(mp4Action, SIGNAL(triggered()), signalMapper, SLOT(map()));
  connect(ogvAction, SIGNAL(triggered()), signalMapper, SLOT(map()));

  signalMapper->setMapping(mp4Action, QString("mp4"));
  signalMapper->setMapping(ogvAction, QString("ogv"));

  // Only support video4linux on linux
#if defined(__linux__) && defined(HAVE_AVDEVICE)
  // V4L recording button
  this->dataPtr->v4lButton = new QToolButton(this);
  this->dataPtr->v4lButton->setToolTip(tr("Record to video4linux device."));
  QAction *v4lAction = new QAction(QIcon(":/images/v4l.svg"),
      tr("Record v4l video"), this);
  this->dataPtr->v4lButton->setDefaultAction(v4lAction);
  this->dataPtr->v4lButton->setIconSize(iconSize);
  mainLayout->addWidget(this->dataPtr->v4lButton, 0, 4);
  connect(v4lAction, SIGNAL(triggered()), signalMapper, SLOT(map()));
  signalMapper->setMapping(v4lAction, QString("v4l2"));
#endif

  connect(signalMapper, SIGNAL(mapped(QString)), this,
          SLOT(OnRecordStart(const QString &)));

  this->setLayout(mainLayout);
  this->setContentsMargins(0, 0, 0, 0);
}

/////////////////////////////////////////////////
VideoRecorder::~VideoRecorder()
{
}

/////////////////////////////////////////////////
void VideoRecorder::OnRecordStop()
{
  rendering::UserCameraPtr cam = gui::get_active_camera();

  // This should never happen...but just in case.
  if (!cam)
  {
    gzerr << "Unable to get pointer to user camera. "
      << "Can't stop video recording\n";
    return;
  }

  cam->StopVideo();

  // Inform listeners that we have stopped recording
  emit RecordingStopped();
  emit RecordingChanged(false);

  // Show the correct buttons
  this->dataPtr->mp4Button->show();
  this->dataPtr->ogvButton->show();
#if defined(__linux__) && defined(HAVE_AVDEVICE)
  this->dataPtr->v4lButton->show();
#endif
  this->dataPtr->stopButton->hide();

  bool saved = false;

  if (this->dataPtr->format != "v4l2")
  {
    std::string title = "Save Video (" + this->dataPtr->format + ")";
    // Create and open a file dialog box to save the video
    QFileDialog fileDialog(this, tr(title.c_str()), QDir::homePath());
    fileDialog.setObjectName("material");

    fileDialog.setDefaultSuffix(this->dataPtr->format.c_str());

    fileDialog.setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
        Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

    fileDialog.setAcceptMode(QFileDialog::AcceptSave);
    fileDialog.setFileMode(QFileDialog::AnyFile);

    if (fileDialog.exec() == QDialog::Accepted)
    {
      QStringList selected = fileDialog.selectedFiles();
      if (!selected.empty())
      {
        // Save the video
        saved = cam->SaveVideo(selected[0].toStdString());
      }
    }
  }

  if (!saved)
    cam->ResetVideo();
}

/////////////////////////////////////////////////
void VideoRecorder::OnRecordStart(const QString &_format)
{
  // Get the user camera, and start recording in the specified format
  rendering::UserCameraPtr cam = gui::get_active_camera();

  // This should never happen...but just in case.
  if (!cam)
  {
    gzerr << "Unable to get pointer to user camera. "
      << "Can't start video recording\n";
    return;
  }

  this->dataPtr->format = _format.toStdString();

  std::string filename;

  // Get the video4linux2 loopback device
  if (this->dataPtr->format == "v4l2")
  {
    std::string videoDevice;

    // Attempt to select an available video device.
    for (int i = 0; i < 4; ++i)
    {
      std::string tmp = "/dev/video" + std::to_string(i);
      if (common::exists(tmp))
        videoDevice = tmp;
    }

    // Display error message.
    if (videoDevice.empty())
    {
      QMessageBox msg(QMessageBox::Critical, "Unable to record video",
          "No video loopback device (/dev/video*) found."
          "Install v4l2loopback-utils, and 'sudo modprobe v4l2loopback'",
          QMessageBox::Close);
      msg.exec();
      return;
    }

    // Create and open a file dialog box. Only linux is supported, so
    // we can safely set the default directory to /dev.
    QFileDialog fileDialog(this, tr("Video Loopback Device"), "/dev");

    fileDialog.selectFile(videoDevice.c_str());
    fileDialog.setObjectName("material");

    fileDialog.setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
        Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

    fileDialog.setAcceptMode(QFileDialog::AcceptOpen);
    fileDialog.setFileMode(QFileDialog::ExistingFile);

    if (fileDialog.exec() == QDialog::Accepted)
    {
      QStringList selected = fileDialog.selectedFiles();
      if (!selected.empty())
      {
        filename = selected[0].toStdString();
      }
    }

    // Don't record if a device file was not selected.
    if (filename.empty())
      return;
  }

  if (cam->StartVideo(this->dataPtr->format, filename))
  {
    // Tell listeners that we started recording
    emit RecordingStarted();
    emit RecordingChanged(true);

    // Show the Stop button, and hide the record options
    this->dataPtr->mp4Button->hide();
    this->dataPtr->ogvButton->hide();
#if defined(__linux__) && defined(HAVE_AVDEVICE)
    this->dataPtr->v4lButton->hide();
#endif
    this->dataPtr->stopButton->show();
  }
}
