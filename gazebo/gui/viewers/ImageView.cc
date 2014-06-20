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
#include "gazebo/gui/viewers/ImageView.hh"

using namespace gazebo;
using namespace gui;

GZ_REGISTER_STATIC_VIEWER("gazebo.msgs.ImageStamped", ImageView)

/////////////////////////////////////////////////
ImageView::ImageView(QWidget *_parent)
: TopicView(_parent, "gazebo.msgs.ImageStamped", "image", 33)
{
  this->setWindowTitle(tr("Gazebo: Image View"));

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
ImageView::~ImageView()
{
  this->sub.reset();
}

/////////////////////////////////////////////////
void ImageView::UpdateImpl()
{
  // Update the image output
  this->imageLabel->setPixmap(this->pixmap);
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
  this->OnMsg(msgs::Convert(_msg->time()),
      _msg->image().data().size());

  unsigned char *rgbData = NULL;
  unsigned int rgbDataSize = 0;

  // Convert the image data to RGB
  common::Image img;
  img.SetFromData(
      (unsigned char *)(_msg->image().data().c_str()),
      _msg->image().width(),
      _msg->image().height(),
      (common::Image::PixelFormat)(_msg->image().pixel_format()));

  img.GetRGBData(&rgbData, rgbDataSize);

  // Get the image data in a QT friendly format
  QImage image(_msg->image().width(), _msg->image().height(),
               QImage::Format_RGB888);
  // Store the image data
  memcpy(image.bits(), rgbData, rgbDataSize);

  // Set the pixmap used by the image label.
  this->pixmap = QPixmap::fromImage(image);

  delete [] rgbData;
}
