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
  if (_msg.width() == 0 || _msg.height() == 0)
    return;


  common::Image img;
  QImage::Format qFormat;
  unsigned int imgSize = 0;
  bool isDepthImage = false;
  switch (_msg.pixel_format())
  {
    case common::Image::PixelFormat::L_INT8:
    case common::Image::PixelFormat::L_INT16:
    {
      qFormat = QImage::Format_Mono;
      imgSize = _msg.width() * _msg.height();
      break;
    }
    case common::Image::PixelFormat::R_FLOAT16:
    case common::Image::PixelFormat::R_FLOAT32:
    {
      qFormat = QImage::Format_RGB888;
      imgSize = _msg.width() * _msg.height() * 3;
      isDepthImage = true;
      break;
    }
    default:
    {
      qFormat = QImage::Format_RGB888;
      imgSize = _msg.width() * _msg.height() * 3;
      break;
    }
  }

  if (_msg.width() != static_cast<unsigned int>(this->dataPtr->image.width()) ||
      _msg.height() != static_cast<unsigned int>(this->dataPtr->image.height()))
  {
    QImage qimage(_msg.width(), _msg.height(), qFormat);
    this->dataPtr->image = qimage.copy();
  }

  // Convert the image data to RGB
  if (isDepthImage)
  {
    unsigned int depthSamples = _msg.width() * _msg.height();
    // cppchecker recommends sizeof(varname)
    float f;
    unsigned int depthBufferSize = depthSamples * sizeof(f);
    float *depthBuffer = new float[depthSamples];
    memcpy(depthBuffer, _msg.data().c_str(), depthBufferSize);

    float maxDepth = depthBuffer[0];
    for (unsigned int i = 1; i < _msg.height() * _msg.width(); ++i)
    {
      if (depthBuffer[i] > maxDepth)
        maxDepth = depthBuffer[i];
    }

    unsigned int idx = 0;
    for ( unsigned int j = 0; j < _msg.height(); j++)
    {
      for ( unsigned int i = 0; i < _msg.width(); i++)
      {
        float d = depthBuffer[idx++];
        d = 255 - (d / maxDepth * 255);
        QRgb value = qRgb(d, d, d);
        this->dataPtr->image.setPixel(i, j, value);
      }
    }

    delete [] depthBuffer;
  }
  else
  {
    memcpy(this->dataPtr->image.bits(), _msg.data().c_str(), imgSize);
  }
  boost::mutex::scoped_lock lock(this->dataPtr->mutex);

  this->update();
}
