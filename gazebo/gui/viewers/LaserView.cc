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
#include "gazebo/transport/Transport.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/gui/viewers/ViewFactory.hh"
#include "gazebo/gui/viewers/LaserView.hh"

using namespace gazebo;
using namespace gui;

GZ_REGISTER_STATIC_VIEWER("gazebo.msgs.ImageStamped", LaserView)

/////////////////////////////////////////////////
LaserView::LaserView(QWidget *_parent)
: TopicView(_parent, "gazebo.msgs.ImageStamped", "laser")
{
  this->setWindowTitle(tr("Gazebo: Laser View"));

  // Create the image display
  // {
  QVBoxLayout *frameLayout = new QVBoxLayout;

  this->pixmap = QPixmap(":/images/no_image.png");
  QPixmap image = (this->pixmap.scaled(320, 240, Qt::KeepAspectRatio,
                                 Qt::SmoothTransformation));
  this->imageLabel = new QLabel();
  this->imageLabel->setPixmap(image);
  this->imageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  this->imageLabel->setMinimumSize(320, 240);
  this->imageLabel->setScaledContents(true);

  frameLayout->addWidget(this->imageLabel);
  this->frame->setObjectName("blackBorderFrame");
  this->frame->setLayout(frameLayout);
  // }
}

/////////////////////////////////////////////////
LaserView::~LaserView()
{
}

/////////////////////////////////////////////////
void LaserView::UpdateImpl()
{
}

/////////////////////////////////////////////////
void LaserView::SetTopic(const std::string &_topicName)
{
  TopicView::SetTopic(_topicName);

  // Subscribe to the new topic.
  this->sub.reset();
  this->sub = this->node->Subscribe(_topicName, &LaserView::OnScan, this);
}

/////////////////////////////////////////////////
void LaserView::OnScan(ConstLaserScanPtr &_msg)
{
  // Update the Hz and Bandwidth info
  this->OnMsg(msgs::Convert(_msg->time()), _msg->ByteSize());
}
