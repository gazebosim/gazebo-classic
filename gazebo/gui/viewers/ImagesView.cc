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

#include "gazebo/common/Image.hh"

#include "gazebo/gui/viewers/ViewFactory.hh"
#include "gazebo/gui/viewers/ImagesView.hh"

using namespace gazebo;
using namespace gui;

GZ_REGISTER_STATIC_VIEWER("gazebo.msgs.ImagesStamped", ImagesView)

/////////////////////////////////////////////////
ImagesView::ImagesView(QWidget *_parent)
: TopicView(_parent, "gazebo.msgs.ImagesStamped", "images")
{
  this->setWindowTitle(tr("Gazebo: Images View"));

  // Create the image display
  // {
  this->frameLayout = new QGridLayout;

  QPixmap pixmap(":/images/no_image.png");
  QPixmap image = (pixmap.scaled(320, 240, Qt::KeepAspectRatio,
        Qt::SmoothTransformation));

  QLabel *imageLabel = new QLabel();
  imageLabel->setPixmap(image);
  imageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  imageLabel->setMinimumSize(320, 240);
  imageLabel->setScaledContents(true);
  this->imageLabels.push_back(imageLabel);

  this->frameLayout->addWidget(imageLabel,0,0);

  /*imageLabel = new QLabel();
  imageLabel->setPixmap(image);
  imageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
  imageLabel->setMinimumSize(320, 240);
  imageLabel->setScaledContents(true);
  this->imageLabels.push_back(imageLabel);

  this->frameLayout->addWidget(imageLabel);
  */


  this->frame->setObjectName("blackBorderFrame");
  this->frame->setLayout(this->frameLayout);
  // }

  this->toAdd = 0;
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

  if (this->clearImages)
  {
    for (++labelIter; labelIter != this->imageLabels.end();)
    {
      printf("Remove 1\n");
      this->frameLayout->removeWidget(*labelIter);
      // delete *labelIter;
      this->imageLabels.erase(labelIter++);
    }
    this->images.clear();

    this->clearImages = false;
    return;
  }

  if (this->images.size() > 0)
  {
    // Update the image output
    for (; labelIter != this->imageLabels.end(); ++labelIter, ++imageIter)
    {
      (*labelIter)->setPixmap(QPixmap::fromImage(*imageIter));
    }
  }

  for (int i = 0; i < this->toAdd; ++i)
  {
    this->AddImage(1024, 544);
  }
  this->toAdd = 0;
}

/////////////////////////////////////////////////
void ImagesView::SetTopic(const std::string &_topicName)
{
  boost::mutex::scoped_lock lock(this->mutex);
  printf("SetTopic\n");
  this->clearImages = true;

  /*std::vector<QLabel*>::iterator labelIter = this->imageLabels.begin();

  for (++labelIter; labelIter != this->imageLabels.end();)
  {
    printf("Remove 1\n");
    this->frameLayout->removeWidget(*labelIter);
    delete *labelIter;
    this->imageLabels.erase(labelIter++);
  }
  std::cout << "Image LabelsSize[" << this->imageLabels.size() << "]\n";
  this->images.clear();
  */

  TopicView::SetTopic(_topicName);

  // Subscribe to the new topic.
  this->sub = this->node->Subscribe(_topicName, &ImagesView::OnImages, this);
}

/////////////////////////////////////////////////
void ImagesView::AddImage(int _width, int _height)
{
  if (this->images.size() > 0)
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
  }

  this->images.push_back(QImage(_width, _height, QImage::Format_RGB888));

}

/////////////////////////////////////////////////
void ImagesView::OnImages(ConstImagesStampedPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);

  unsigned char *rgbData = NULL;
  unsigned int rgbDataSize = 0;

  if (this->clearImages)
    return;

  int dataSize = 0;
  this->toAdd = 0;
  for (int i=0; i < _msg->image_size(); ++i)
  {
    rgbData = NULL;
    rgbDataSize = 0;

    dataSize += _msg->image(0).data().size();

    if (i >= static_cast<int>(this->images.size()))
    {
      this->toAdd++;
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

    // Get the image data in a QT friendly format
    // QImage image(_msg->image(i).width(), _msg->image(i).height(),
    //    QImage::Format_RGB888);

    // Store the image data
    memcpy(this->images[i].bits(), rgbData, rgbDataSize);

    // if (i > static_cast<int>(this->pixmaps.size()))
    //  this->AddImage();

    // Set the pixmap used by the image label.
    //this->pixmaps[i] = QPixmap::fromImage(image);

    delete [] rgbData;
  }

  // Update the Hz and Bandwidth info
  this->OnMsg(msgs::Convert(_msg->time()), dataSize);
}
