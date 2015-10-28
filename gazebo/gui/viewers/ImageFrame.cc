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

#include "gazebo/common/Image.hh"
#include "gazebo/gui/viewers/ImageFramePrivate.hh"
#include "gazebo/gui/viewers/ImageFrame.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ImageFrame::ImageFrame(QWidget *_parent)
  : QFrame(_parent), dataPtr(new ImageFramePrivate())
{
}

/////////////////////////////////////////////////
ImageFrame::~ImageFrame()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void ImageFrame::paintEvent(QPaintEvent * /*_event*/)
{
  QPainter painter(this);
  boost::mutex::scoped_lock lock(this->dataPtr->mutex);

  if (!this->dataPtr->image.isNull())
  {
    painter.drawImage(this->contentsRect(), this->dataPtr->image);
  }
  else
  {
    // default image with gradient
    QLinearGradient gradient(0, 0, this->frameRect().width(),
        this->frameRect().height());
    gradient.setColorAt(0, Qt::white);
    gradient.setColorAt(1, Qt::black);
    painter.setBrush(gradient);
    painter.drawRect(0, 0, this->frameRect().width()
        + 1, this->frameRect().height() + 1);
  }
}

/////////////////////////////////////////////////
void ImageFrame::OnImage(const msgs::Image &_msg)
{
  unsigned char *rgbData = NULL;
  unsigned int rgbDataSize = 0;

  // Convert the image data to RGB
  common::Image img;
  img.SetFromData(
      (unsigned char *)(_msg.data().c_str()),
      _msg.width(), _msg.height(),
      (common::Image::PixelFormat)(_msg.pixel_format()));

  img.GetRGBData(&rgbData, rgbDataSize);

  if (_msg.width() != static_cast<unsigned int>(this->dataPtr->image.width()) ||
      _msg.height() != static_cast<unsigned int>(this->dataPtr->image.height()))
  {
    QImage qimage(_msg.width(), _msg.height(), QImage::Format_RGB888);
    this->dataPtr->image = qimage.copy();
  }

  // Store the image data
  memcpy(this->dataPtr->image.bits(), rgbData, rgbDataSize);
  boost::mutex::scoped_lock lock(this->dataPtr->mutex);

  this->update();
  delete [] rgbData;
}
