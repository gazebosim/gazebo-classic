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

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/gui/sensor_widgets/CameraSensorWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
CameraSensorWidget::CameraSensorWidget(QWidget *_parent)
: QWidget(_parent)
{
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

  msgs::ImageStamped msg;
  std::list<std::string> topics;
  topics = transport::TopicManager::Instance()->GetUniqueAdvertisedTopics(
      msg.GetTypeName());

  for (std::list<std::string>::iterator iter = topics.begin();
       iter != topics.end(); ++iter)
  {
    std::cout << "Topic[" << *iter << "]\n";
    this->topicCombo->addItem(QString::fromStdString(*iter));
  }

  topicLayout->addSpacing(10);
  topicLayout->addWidget(topicLabel, 1);
  topicLayout->addWidget(this->topicCombo, 4);
  topicLayout->addSpacing(10);

  this->pixmap = QPixmap(":/images/no_image.png");
  QPixmap image = (this->pixmap.scaled(320, 240, Qt::KeepAspectRatio,
                                 Qt::SmoothTransformation));
  this->imageLabel = new QLabel();
  this->imageLabel->setPixmap(image);

  frameLayout->addWidget(this->imageLabel);
  frameLayout->setObjectName("blackBorderFrame");
  frame->setLayout(frameLayout);

  mainLayout->addLayout(topicLayout);
  mainLayout->addWidget(frame);
  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(4, 4, 4, 4);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->cameraSub = this->node->Subscribe("~/camera/link/camera/image",
      &CameraSensorWidget::OnImage, this);

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
