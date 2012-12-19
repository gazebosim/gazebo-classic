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
#include "gazebo/common/SystemPaths.hh"

#include "gazebo/gui/Gui.hh"
#include "gazebo/gui/GuiEvents.hh"

#include "gazebo/transport/Transport.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/gui/sensor_widgets/CameraSensorWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
CameraSensorWidget::CameraSensorWidget(QWidget *_parent)
: QWidget(_parent)
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->cameraSub = this->node->Subscribe("~/camera/link/camera/image",
      &CameraSensorWidget::OnImage, this);

  this->setWindowIcon(QIcon(":/images/gazebo.svg"));
  this->setWindowTitle(tr("Gazebo: Camera Sensor"));
  this->setObjectName("cameraSensor");

  QVBoxLayout *mainLayout = new QVBoxLayout;

  QFrame *frame = new QFrame;
  QVBoxLayout *frameLayout = new QVBoxLayout;

  QHBoxLayout *topicLayout = new QHBoxLayout;

  QLabel *topicLabel = new QLabel(tr("Topic: "));
  this->topicCombo = new QComboBox(this);
  this->topicCombo->setObjectName("comboList");
  this->topicCombo->setMinimumSize(300, 25);

  connect(this->topicCombo, SIGNAL(currentIndexChanged(int)),
          this, SLOT(OnTopicChanged(int)));

  msgs::ImageStamped msg;
  std::list<std::string> topics;
  topics = transport::getAdvertisedTopics(msg.GetTypeName());

  for (std::list<std::string>::iterator iter = topics.begin();
       iter != topics.end(); ++iter)
  {
    std::string topicName = this->node->EncodeTopicName(*iter);
    this->topicCombo->addItem(QString::fromStdString(topicName));
  }

  topicLayout->addSpacing(10);
  topicLayout->addWidget(topicLabel);
  topicLayout->addWidget(this->topicCombo);
  topicLayout->addSpacing(10);
  topicLayout->addStretch(4);

  this->pixmap = QPixmap(":/images/no_image.png");
  QPixmap image = (this->pixmap.scaled(320, 240, Qt::KeepAspectRatio,
                                 Qt::SmoothTransformation));
  this->imageLabel = new QLabel();
  this->imageLabel->setPixmap(image);
  this->imageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  this->imageLabel->setMinimumSize(320, 240);
  this->imageLabel->setScaledContents(true);

  frameLayout->addWidget(this->imageLabel);
  frame->setObjectName("blackBorderFrame");
  frame->setLayout(frameLayout);

  mainLayout->addLayout(topicLayout);
  mainLayout->addWidget(frame);
  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(4, 4, 4, 4);

  QTimer::singleShot(500, this, SLOT(Update()));
}

/////////////////////////////////////////////////
CameraSensorWidget::~CameraSensorWidget()
{
}

/////////////////////////////////////////////////
void CameraSensorWidget::Update()
{
  this->imageLabel->setPixmap(this->pixmap);

  QTimer::singleShot(500, this, SLOT(Update()));
}

/////////////////////////////////////////////////
void CameraSensorWidget::OnImage(ConstImageStampedPtr &_msg)
{
  QImage image(_msg->image().width(), _msg->image().height(),
               QImage::Format_RGB888);
  memcpy(image.bits(), _msg->image().data().c_str(),
         _msg->image().data().size());

  this->pixmap = QPixmap::fromImage(image);
}

/////////////////////////////////////////////////
void CameraSensorWidget::OnTopicChanged(int _index)
{
  this->SetTopic(this->topicCombo->itemText(_index).toStdString());
}

/////////////////////////////////////////////////
void CameraSensorWidget::SetTopic(const std::string &_topicName)
{
  this->cameraSub.reset();
  this->cameraSub = this->node->Subscribe(_topicName,
                                          &CameraSensorWidget::OnImage, this);
}
