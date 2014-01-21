/*
 * Copyright (C) 2012-2013 Open Source Robotics Foundation
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
#include "gazebo/gui/viewers/ImageFrame.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ImageFrame::ImageFrame(QWidget *_parent)
  : QFrame(_parent)
{
}

/////////////////////////////////////////////////
ImageFrame::~ImageFrame()
{
}

/////////////////////////////////////////////////
void ImageFrame::paintEvent(QPaintEvent * /*_event*/)
{
  QPainter painter(this);
  boost::mutex::scoped_lock lock(this->mutex);

  if (!this->image.isNull())
  {
    painter.drawImage(this->contentsRect(), this->image);
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

  // Get the image data in a QT friendly format
  QImage qimage(_msg.width(), _msg.height(),
               QImage::Format_RGB888);
  // Store the image data
  memcpy(qimage.bits(), rgbData, rgbDataSize);

  boost::mutex::scoped_lock lock(this->mutex);
  this->image = qimage.copy();

  this->update();
  delete [] rgbData;
}
