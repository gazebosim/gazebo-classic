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
#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/SaveDialog.hh"
#include "gazebo/gui/RenderWidget.hh"
#include "gazebo/gui/VideoRecorder.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
VideoRecorder::VideoRecorder(QWidget *_parent)
  : QWidget(_parent)
{
  this->recordVideoTimer = nullptr;
}

/////////////////////////////////////////////////
VideoRecorder::~VideoRecorder()
{
}

/////////////////////////////////////////////////
void VideoRecorder::CreateActions()
{
  connect(g_recordVideoAct, SIGNAL(triggered()), this,
      SLOT(RecordVideo()));

  QMenu *videoFormatSubmenu = new QMenu;
  std::vector<QAction *> formats;
  formats.push_back(videoFormatSubmenu->addAction("ogv"));
  formats.push_back(videoFormatSubmenu->addAction("avi"));
  formats.push_back(videoFormatSubmenu->addAction("mp4"));
  QActionGroup *formatActGroup = new QActionGroup(this);
  for (unsigned int i = 0; i < formats.size(); ++i)
  {
    formats[i]->setCheckable(true);
    formatActGroup->addAction(formats[i]);
  }
  formats[0]->setChecked(true);

  connect(videoFormatSubmenu, SIGNAL(triggered(QAction *)), this,
      SLOT(SetRecordVideoFormat(QAction *)));

  QToolButton *videoFormatButton = new QToolButton();
  videoFormatButton->setMenu(videoFormatSubmenu);
  g_recordVideoFormatAct->setDefaultWidget(videoFormatButton);
  videoFormatButton->setMaximumSize(18, videoFormatButton->height()/2);

  connect(videoFormatButton, SIGNAL(clicked()), this,
    SLOT(ShowVideoFormatMenu()));
}

/////////////////////////////////////////////////
void VideoRecorder::RecordVideo()
{
  rendering::UserCameraPtr cam = gui::get_active_camera();
  cam->SetEncodeVideo(g_recordVideoAct->isChecked());
  if (g_recordVideoAct->isChecked())
  {
    g_recordVideoAct->setIcon(QIcon(":/images/record_stop.png"));
    if (!this->recordVideoTimer)
    {
      this->recordVideoTimer = new QTimer(this);
      connect(this->recordVideoTimer, SIGNAL(timeout()), this,
          SLOT(DisplayRecordingMsg()));
    }
    this->recordVideoTimer->start(1000);
    QWidget *defaultWidget = g_recordVideoFormatAct->defaultWidget();
    qobject_cast<QToolButton *>(defaultWidget)->setEnabled(false);
  }
  else
  {
    this->recordVideoTimer->stop();
    g_recordVideoAct->setIcon(QIcon(":/images/record.png"));

    std::string friendlyName = cam->Name();
    boost::replace_all(friendlyName, "::", "_");
    std::string timestamp = common::Time::GetWallTimeAsISOString();
    boost::replace_all(timestamp, ":", "_");

    SaveDialog saveDialog;
    saveDialog.SetSaveName(friendlyName + "_" + timestamp);
    saveDialog.SetSaveLocation(QDir::homePath().toStdString());
    if (saveDialog.exec() == QDialog::Accepted)
    {
      std::string name = saveDialog.SaveName();
      std::string location = saveDialog.SaveLocation();
      cam->SaveVideo(location + "/" + name);
      emit MessageChanged(name + " saved in: " + location, 2000);
    }

    QWidget *defaultWidget = g_recordVideoFormatAct->defaultWidget();
    qobject_cast<QToolButton *>(defaultWidget)->setEnabled(true);
  }
}

/////////////////////////////////////////////////
void VideoRecorder::SetRecordVideoFormat(QAction *_action)
{
  rendering::UserCameraPtr cam = gui::get_active_camera();
  cam->SetEncodeVideoFormat(_action->text().toStdString());
  emit MessageChanged("Set video recording format to " +
      _action->text().toStdString(), 2000);
}

/////////////////////////////////////////////////
void VideoRecorder::ShowVideoFormatMenu()
{
  QWidget *defaultWidget = g_recordVideoFormatAct->defaultWidget();
  qobject_cast<QToolButton *>(defaultWidget)->showMenu();
}

/////////////////////////////////////////////////
void VideoRecorder::DisplayRecordingMsg()
{
  emit MessageChanged("Recording...",
      this->recordVideoTimer ? this->recordVideoTimer->interval()/2 : 500);
}
