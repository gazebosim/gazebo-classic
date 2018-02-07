/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  if (this->dataPtr->depthBuffer)
    delete [] this->dataPtr->depthBuffer;
  if (this->dataPtr->imageBuffer)
    delete [] this->dataPtr->imageBuffer;
}

/////////////////////////////////////////////////
void ImageFrame::paintEvent(QPaintEvent * /*_event*/)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  QPainter painter(this);

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

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  QImage::Format qFormat;
  bool isDepthImage = false;
  switch (_msg.pixel_format())
  {
    case common::Image::PixelFormat::L_INT8:
    {
      qFormat = QImage::Format_Grayscale8;
      break;
    }
    case common::Image::PixelFormat::R_FLOAT16:
    case common::Image::PixelFormat::R_FLOAT32:
    {
      qFormat = QImage::Format_RGB888;
      isDepthImage = true;
      break;
    }
    default:
    {
      qFormat = QImage::Format_RGB888;
      break;
    }
  }

  if (_msg.width() != static_cast<unsigned int>(this->dataPtr->image.width()) ||
      _msg.height() != static_cast<unsigned int>(this->dataPtr->image.height())
      || qFormat != this->dataPtr->image.format() || _msg.step() !=
      static_cast<unsigned int>(this->dataPtr->image.bytesPerLine()))
  {
    this->dataPtr->image = QImage(_msg.width(), _msg.height(), qFormat);
    delete [] this->dataPtr->imageBuffer;
    this->dataPtr->imageBuffer = nullptr;
    delete [] this->dataPtr->depthBuffer;
    this->dataPtr->depthBuffer = nullptr;
  }

  // Convert the image data to RGB
  if (isDepthImage)
  {
    unsigned int depthSamples = _msg.width() * _msg.height();
    float f;
    // cppchecker recommends using sizeof(varname)
    unsigned int depthBufferSize = depthSamples * sizeof(f);

    if (!this->dataPtr->depthBuffer)
      this->dataPtr->depthBuffer = new float[depthSamples];
    memcpy(this->dataPtr->depthBuffer, _msg.data().c_str(), depthBufferSize);

    float maxDepth = 0;
    for (unsigned int i = 0; i < _msg.height() * _msg.width(); ++i)
    {
      if (this->dataPtr->depthBuffer[i] > maxDepth &&
          !std::isinf(this->dataPtr->depthBuffer[i]))
      {
        maxDepth = this->dataPtr->depthBuffer[i];
      }
    }
    unsigned int idx = 0;
    double factor = 255 / maxDepth;
    for (unsigned int j = 0; j < _msg.height(); ++j)
    {
      for (unsigned int i = 0; i < _msg.width(); ++i)
      {
        float d = this->dataPtr->depthBuffer[idx++];
        d = 255 - (d * factor);
        QRgb value = qRgb(d, d, d);
        this->dataPtr->image.setPixel(i, j, value);
      }
    }
  }
  // convert 16 bit camera images to rgb image format for display
  else if (_msg.pixel_format() == common::Image::PixelFormat::L_INT16
      || _msg.pixel_format() == common::Image::PixelFormat::RGB_INT16)
  {
    uint16_t u;
    // cppchecker recommends using sizeof(varname)
    unsigned int channels = _msg.step() / _msg.width() / sizeof(u);
    unsigned int samples = _msg.width() * _msg.height() * channels;
    unsigned int bufferSize = samples * sizeof(u);

    if (!this->dataPtr->imageBuffer)
      this->dataPtr->imageBuffer = new unsigned char[bufferSize];
    memcpy(this->dataPtr->imageBuffer, _msg.data().c_str(), bufferSize);

    // factor to convert from 16bit to 8bit value
    double factor = 255 / (std::pow(2, 16) - 1);
    uint16_t *uint16Buffer = reinterpret_cast<uint16_t *>(
        this->dataPtr->imageBuffer);
    int rgb[3] = {0, 0, 0};
    unsigned int width = _msg.width()*channels;
    for (unsigned int j = 0; j < _msg.height(); ++j)
    {
      for (unsigned int i = 0; i < width; i+=channels)
      {
        u = uint16Buffer[j*width + i] * factor;
        // for L16 we set rgb to be the same value
        rgb[0] = rgb[1] = rgb[2] = u;
        // handle the case for rgb16
        for (unsigned int k = 1; k < channels; ++k)
        {
          rgb[k] = uint16Buffer[j*width + i + k] * factor;
        }
        QRgb value = qRgb(rgb[0], rgb[1], rgb[2]);
        this->dataPtr->image.setPixel(i/channels, j, value);
      }
    }
  }
  else
  {
    const char *buffer = _msg.data().c_str();
    for (int i = 0; i < this->dataPtr->image.height(); ++i)
    {
      memcpy(this->dataPtr->image.scanLine(i),
          buffer + i*this->dataPtr->image.bytesPerLine(),
          this->dataPtr->image.bytesPerLine());
    }
  }
  this->update();
}
