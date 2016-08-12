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

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/VideoRecorder.hh"

using namespace gazebo;
using namespace gui;

// Private data class
class gazebo::gui::VideoRecorderPrivate
{
  /// \brief Button to enable mp4 recording
  public: QToolButton *mp4Button = nullptr;

  /// \brief Button to enable ogv recording
  public: QToolButton *ogvButton = nullptr;

  /// \brief Button to enable avi recording
  public: QToolButton *aviButton = nullptr;

  /// \brief Button to stop recording
  public: QPushButton *stopButton = nullptr;
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

  // AVI recording button
  this->dataPtr->aviButton = new QToolButton(this);
  this->dataPtr->aviButton->setToolTip(tr("Record an AVI video"));
  QAction *aviAction = new QAction(QIcon(":/images/avi.svg"),
      tr("Record avi video"), this);
  this->dataPtr->aviButton->setDefaultAction(aviAction);

  // Set icon size
  QSize iconSize(32, 32);
  this->dataPtr->mp4Button->setIconSize(iconSize);
  this->dataPtr->ogvButton->setIconSize(iconSize);
  this->dataPtr->aviButton->setIconSize(iconSize);

  // Stop button
  this->dataPtr->stopButton = new QPushButton("Stop");
  this->dataPtr->stopButton->setToolTip(tr("End video and save"));
  this->dataPtr->stopButton->hide();
  connect(this->dataPtr->stopButton, SIGNAL(pressed()), this,
          SLOT(OnRecordStop()));

  QGridLayout *mainLayout = new QGridLayout();
  mainLayout->addWidget(this->dataPtr->mp4Button, 0, 1);
  mainLayout->addWidget(this->dataPtr->ogvButton, 0, 2);
  mainLayout->addWidget(this->dataPtr->aviButton, 0, 3);
  mainLayout->addWidget(this->dataPtr->stopButton, 1, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);
  this->setContentsMargins(0, 0, 0, 0);

  // Map each video record button to the OnRecordStart slot
  QSignalMapper *signalMapper = new QSignalMapper(this);
  connect(mp4Action, SIGNAL(triggered()), signalMapper, SLOT(map()));
  connect(aviAction, SIGNAL(triggered()), signalMapper, SLOT(map()));
  connect(ogvAction, SIGNAL(triggered()), signalMapper, SLOT(map()));

  signalMapper->setMapping(mp4Action, QString("mp4"));
  signalMapper->setMapping(aviAction, QString("avi"));
  signalMapper->setMapping(ogvAction, QString("ogv"));
  connect(signalMapper, SIGNAL(mapped(QString)), this,
          SLOT(OnRecordStart(const QString &)));
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
  emit recordingStopped();
  emit recordingChanged(false);

  // Show the correct buttons
  this->dataPtr->mp4Button->show();
  this->dataPtr->aviButton->show();
  this->dataPtr->ogvButton->show();
  this->dataPtr->stopButton->hide();

  // Create and open a file dialog box to save the video
  QFileDialog fileDialog(this, tr("Save Video"), QDir::homePath());
  fileDialog.setObjectName("material");

  fileDialog.setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
      Qt::WindowStaysOnTopHint | Qt::CustomizeWindowHint);

  fileDialog.setAcceptMode(QFileDialog::AcceptSave);
  fileDialog.setFileMode(QFileDialog::AnyFile);

  bool saved = false;
  if (fileDialog.exec() == QDialog::Accepted)
  {
    QStringList selected = fileDialog.selectedFiles();
    if (!selected.empty())
    {
      // Save the video
      saved = cam->SaveVideo(selected[0].toStdString());
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

  if (cam->StartVideo(_format.toStdString()))
  {
    // Tell listeners that we started recording
    emit recordingStarted();
    emit recordingChanged(true);

    // Show the Stop button, and hide the record options
    this->dataPtr->mp4Button->hide();
    this->dataPtr->aviButton->hide();
    this->dataPtr->ogvButton->hide();
    this->dataPtr->stopButton->show();
  }
}
