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
#include "gazebo/transport/TransportIface.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/common/Image.hh"

#include "gazebo/gui/viewers/ViewFactory.hh"
#include "gazebo/gui/viewers/ImageViewPrivate.hh"
#include "gazebo/gui/viewers/ImageView.hh"

using namespace gazebo;
using namespace gui;

GZ_REGISTER_STATIC_VIEWER("gazebo.msgs.ImageStamped", ImageView)

/////////////////////////////////////////////////
ImageView::ImageView(QWidget *_parent)
: TopicView(_parent, "gazebo.msgs.ImageStamped", "image", 60),
  dataPtr(new ImageViewPrivate())
{
  this->setWindowTitle(tr("Gazebo: Image View"));

  QVBoxLayout *frameLayout = new QVBoxLayout;

  this->dataPtr->imageFrame = new ImageFrame(this);
  this->dataPtr->imageFrame->setMinimumSize(320, 240);
  this->dataPtr->imageFrame->show();
  frameLayout->addWidget(this->dataPtr->imageFrame);

  this->frame->setObjectName("blackBorderFrame");
  this->frame->setLayout(frameLayout);
}

/////////////////////////////////////////////////
ImageView::~ImageView()
{
  delete this->dataPtr;
  this->dataPtr = NULL;

  this->sub.reset();
}

/////////////////////////////////////////////////
void ImageView::SetTopic(const std::string &_topicName)
{
  TopicView::SetTopic(_topicName);

  // Subscribe to the new topic.
  this->sub = this->node->Subscribe(_topicName, &ImageView::OnImage, this);
}

/////////////////////////////////////////////////
void ImageView::OnImage(ConstImageStampedPtr &_msg)
{
  // Update the Hz and Bandwidth info
  this->OnMsg(msgs::Convert(_msg->time()), _msg->image().data().size());

  this->dataPtr->imageFrame->OnImage(_msg->image());
}
