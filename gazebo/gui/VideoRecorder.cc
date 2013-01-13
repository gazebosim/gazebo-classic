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

#include <boost/algorithm/string.hpp>

#include "gazebo/gui/Gui.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/gui/VideoRecorder.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
VideoRecorder::VideoRecorder(QWidget *_parent)
  : QDialog(_parent)
{
  // This name is used in the qt style sheet
  this->setObjectName("dataLogger");
  this->setWindowIcon(QIcon(":/images/gazebo.svg"));
  this->setWindowTitle(tr("Gazebo: Video Recorder"));

  // Create the main layout for this widget
  QVBoxLayout *mainLayout = new QVBoxLayout;

  this->poseList = new QListWidget();
  this->poseList->setVerticalScrollMode(
      QAbstractItemView::ScrollPerPixel);
  connect(this->poseList, SIGNAL(currentRowChanged(int)),
          this, SLOT(OnPoseSelect(int)));

  QHBoxLayout *poseEditLayout = new QHBoxLayout;
  this->xEdit = new QLineEdit;
  this->yEdit = new QLineEdit;
  this->zEdit = new QLineEdit;
  this->rollEdit = new QLineEdit;
  this->pitchEdit = new QLineEdit;
  this->yawEdit = new QLineEdit;
  this->xEdit->setFixedWidth(40);
  this->yEdit->setFixedWidth(40);
  this->zEdit->setFixedWidth(40);
  this->rollEdit->setFixedWidth(40);
  this->pitchEdit->setFixedWidth(40);
  this->yawEdit->setFixedWidth(40);

  QPushButton *setPoseButton = new QPushButton("Set");
  QPushButton *grabPoseButton = new QPushButton("Grab");

  poseEditLayout->addWidget(new QLabel("X"));
  poseEditLayout->addWidget(this->xEdit);
  poseEditLayout->addWidget(new QLabel("Y"));
  poseEditLayout->addWidget(this->yEdit);
  poseEditLayout->addWidget(new QLabel("Z"));
  poseEditLayout->addWidget(this->zEdit);
  poseEditLayout->addWidget(new QLabel("R"));
  poseEditLayout->addWidget(this->rollEdit);
  poseEditLayout->addWidget(new QLabel("P"));
  poseEditLayout->addWidget(this->pitchEdit);
  poseEditLayout->addWidget(new QLabel("Y"));
  poseEditLayout->addWidget(this->yawEdit);

  poseEditLayout->addWidget(setPoseButton);
  connect(setPoseButton, SIGNAL(clicked()), this, SLOT(OnSetPose()));

  poseEditLayout->addWidget(grabPoseButton);
  connect(grabPoseButton, SIGNAL(clicked()), this, SLOT(OnGrabPose()));

  QHBoxLayout *buttonLayout = new QHBoxLayout;

  QPushButton *previewButton = new QPushButton("Preview");
  connect(previewButton, SIGNAL(clicked()), this, SLOT(OnPreview()));

  QPushButton *recordButton = new QPushButton("Record");
  recordButton->setCheckable(true);
  connect(recordButton, SIGNAL(toggled(bool)), this, SLOT(OnRecord(bool)));

  buttonLayout->addWidget(previewButton);
  buttonLayout->addWidget(recordButton);

  mainLayout->addWidget(this->poseList);
  mainLayout->addLayout(poseEditLayout);
  mainLayout->addLayout(buttonLayout);

  // Let the stylesheet handle the margin sizes
  mainLayout->setContentsMargins(4, 4, 4, 4);

  // Assign the mainlayout to this widget
  this->setLayout(mainLayout);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->pub = this->node->Advertise<msgs::LogControl>("~/log/control");

  this->camera = gui::get_active_camera();
  this->activeRow = -1;
}

/////////////////////////////////////////////////
VideoRecorder::~VideoRecorder()
{
}

/////////////////////////////////////////////////
void VideoRecorder::OnRecord(bool _toggled)
{
  if (_toggled)
  {
    this->camera->EnableSaveFrame(true);
    this->camera->SetSaveFramePathname("/tmp/gazebo/frames");
  }
  else
    this->camera->EnableSaveFrame(false);
}

/////////////////////////////////////////////////
void VideoRecorder::OnPreview()
{
  std::vector<math::Pose> pts;

  // Get all the pose values.
  for (int i = 0; i < this->poseList->count(); ++i)
  {
    QListWidgetItem *item = this->poseList->item(i);
    std::vector<std::string> values;
    std::string text = item->text().toStdString();
    boost::split(values, text, boost::is_any_of(" "));
    math::Pose pose(
        boost::lexical_cast<double>(values[0]),
        boost::lexical_cast<double>(values[1]),
        boost::lexical_cast<double>(values[2]),
        boost::lexical_cast<double>(values[3]),
        boost::lexical_cast<double>(values[4]),
        boost::lexical_cast<double>(values[5]));
    pts.push_back(pose);
  }

  this->camera->MoveToPositions(pts, 5);
}

/////////////////////////////////////////////////
void VideoRecorder::OnSetPose()
{
  std::ostringstream stream;
  stream << this->xEdit->text().toStdString() << " "
         << this->yEdit->text().toStdString() << " "
         << this->zEdit->text().toStdString() << " "
         << this->rollEdit->text().toStdString() << " "
         << this->pitchEdit->text().toStdString() << " "
         << this->yawEdit->text().toStdString() << " ";

  if (this->activeRow < 0)
  {
    QListWidgetItem *item = new QListWidgetItem(
        QString::fromStdString(stream.str()));

    this->poseList->addItem(item);
  }
  else
  {
    QListWidgetItem *item = this->poseList->item(this->activeRow);
    item->setText(QString::fromStdString(stream.str()));
  }

  this->poseList->setCurrentRow(-1);
}

/////////////////////////////////////////////////
void VideoRecorder::OnGrabPose()
{
  math::Pose pose = this->camera->GetWorldPose();

  std::ostringstream stream;
  stream << pose.pos.x << " "
         << pose.pos.y << " "
         << pose.pos.z << " "
         << pose.rot.GetAsEuler().x << " "
         << pose.rot.GetAsEuler().y << " "
         << pose.rot.GetAsEuler().z << " ";

  if (this->activeRow < 0)
  {
    QListWidgetItem *item = new QListWidgetItem(
        QString::fromStdString(stream.str()));

    this->poseList->addItem(item);
  }
  else
  {
    QListWidgetItem *item = this->poseList->item(this->activeRow);
    item->setText(QString::fromStdString(stream.str()));
  }

  this->poseList->setCurrentRow(-1);
}

/////////////////////////////////////////////////
void VideoRecorder::OnPoseSelect(int _row)
{
  this->activeRow = _row;
  if (this->activeRow >= 0)
  {
    QListWidgetItem *item = this->poseList->item(this->activeRow);
    std::vector<std::string> values;
    std::string text = item->text().toStdString();
    boost::split(values, text, boost::is_any_of(" "));

    this->xEdit->setText(QString::fromStdString(values[0]));
    this->yEdit->setText(QString::fromStdString(values[1]));
    this->zEdit->setText(QString::fromStdString(values[2]));
    this->rollEdit->setText(QString::fromStdString(values[3]));
    this->pitchEdit->setText(QString::fromStdString(values[4]));
    this->yawEdit->setText(QString::fromStdString(values[5]));
  }
  else
  {
    this->xEdit->setText(tr(""));
    this->yEdit->setText(tr(""));
    this->zEdit->setText(tr(""));
    this->rollEdit->setText(tr(""));
    this->pitchEdit->setText(tr(""));
    this->yawEdit->setText(tr(""));
  }
}
