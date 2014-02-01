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
#include "gazebo/gui/viewers/ImagesView.hh"

using namespace gazebo;
using namespace gui;

GZ_REGISTER_STATIC_VIEWER("gazebo.msgs.ImagesStamped", ImagesView)

/////////////////////////////////////////////////
ImagesView::ImagesView(QWidget *_parent)
: TopicView(_parent, "gazebo.msgs.ImagesStamped", "images", 33)
{
  this->setWindowTitle(tr("Gazebo: Images View"));

  // Create the layout and frame for images
  // {
  this->frameLayout = new QGridLayout;
  this->frameLayout->setSizeConstraint(QLayout::SetMinimumSize);

  this->frame->setObjectName("blackBorderFrame");
  this->frame->setLayout(this->frameLayout);
  this->frame->setMinimumHeight(240);
  this->frame->setMinimumWidth(320);
  // }

  this->clearImages = false;
}

/////////////////////////////////////////////////
ImagesView::~ImagesView()
{
  this->sub.reset();
}

/////////////////////////////////////////////////
void ImagesView::UpdateImpl()
{
  boost::mutex::scoped_lock lock(this->mutex);
  std::vector<QLabel*>::iterator labelIter = this->imageLabels.begin();
  std::vector<QImage>::iterator imageIter = this->images.begin();

  // Clear out the images if the flag is set.
  if (this->clearImages)
  {
    // Remove all the images, and delete them
    for (; labelIter != this->imageLabels.end(); )
    {
      (*labelIter)->hide();
      this->frameLayout->removeWidget(*labelIter);
      delete *labelIter;
      labelIter = this->imageLabels.erase(labelIter);
    }

    // Clear the lists
    this->imageLabels.clear();
    this->images.clear();

    // Make sure to adjust the size of the widget
    this->frame->adjustSize();
    this->adjustSize();

    // Reset frame
    this->clearImages = false;
    return;
  }

  // Update the images if there are sizes
  if (!this->images.empty())
  {
    // Update the image output
    for (; labelIter != this->imageLabels.end(); ++labelIter, ++imageIter)
    {
      (*labelIter)->setPixmap(QPixmap::fromImage(*imageIter));
    }
  }

  // Create new images when necessary
  std::vector<std::pair<int, int> >::iterator iter;
  for (iter = this->addImage.begin(); iter != this->addImage.end(); ++iter)
  {
    this->AddImage((*iter).first, (*iter).second);
  }
  this->addImage.clear();
}

/////////////////////////////////////////////////
void ImagesView::SetTopic(const std::string &_topicName)
{
  boost::mutex::scoped_lock lock(this->mutex);

  // Tell the widget to clear the images
  this->clearImages = true;

  TopicView::SetTopic(_topicName);

  // Subscribe to the new topic.
  this->sub = this->node->Subscribe(_topicName, &ImagesView::OnImages, this);
}

/////////////////////////////////////////////////
void ImagesView::AddImage(int _width, int _height)
{
  QPixmap pixmap(":/images/no_image.png");
  QPixmap image = (pixmap.scaled(_width, _height, Qt::KeepAspectRatio,
        Qt::SmoothTransformation));

  QLabel *imageLabel = new QLabel();
  imageLabel->setPixmap(image);
  imageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  imageLabel->setMinimumSize(320, 240);
  imageLabel->setScaledContents(true);
  this->imageLabels.push_back(imageLabel);

  // Add the lable to the correct row and column
  this->frameLayout->addWidget(imageLabel, (this->imageLabels.size()-1) / 2,
      (this->imageLabels.size()-1) % 2);

  this->images.push_back(QImage(_width, _height, QImage::Format_RGB888));
}

/////////////////////////////////////////////////
void ImagesView::OnImages(ConstImagesStampedPtr &_msg)
{
  // Only use a try lock so that we don't block the node thread.
  boost::mutex::scoped_try_lock lock(this->mutex);
  if (!lock)
      return;

  unsigned char *rgbData = NULL;
  unsigned int rgbDataSize = 0;

  if (this->clearImages)
    return;

  int dataSize = 0;
  this->addImage.clear();

  for (int i = 0; i < _msg->image_size(); ++i)
  {
    rgbData = NULL;
    rgbDataSize = 0;

    dataSize += _msg->image(0).data().size();

    if (i >= static_cast<int>(this->images.size()))
    {
      this->addImage.push_back(std::make_pair(_msg->image(0).width(),
                                              _msg->image(0).height()));
      continue;
    }

    // Convert the image data to RGB
    common::Image img;
    img.SetFromData(
        (unsigned char *)(_msg->image(i).data().c_str()),
        _msg->image(i).width(),
        _msg->image(i).height(),
        (common::Image::PixelFormat)(_msg->image(i).pixel_format()));

    img.GetRGBData(&rgbData, rgbDataSize);

    // Store the image data
    memcpy(this->images[i].bits(), rgbData, rgbDataSize);

    delete [] rgbData;
  }

  // Update the Hz and Bandwidth info
  this->OnMsg(msgs::Convert(_msg->time()), dataSize);
}
