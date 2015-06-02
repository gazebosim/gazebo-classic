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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/transport/TransportIface.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/common/Image.hh"

#include "gazebo/gui/viewers/ViewFactory.hh"
#include "gazebo/gui/viewers/ImagesViewPrivate.hh"
#include "gazebo/gui/viewers/ImagesView.hh"

using namespace gazebo;
using namespace gui;

GZ_REGISTER_STATIC_VIEWER("gazebo.msgs.ImagesStamped", ImagesView)

/////////////////////////////////////////////////
ImagesView::ImagesView(QWidget *_parent)
: TopicView(_parent, "gazebo.msgs.ImagesStamped", "images", 60),
  dataPtr(new ImagesViewPrivate())
{
  this->setWindowTitle(tr("Gazebo: Images View"));

  // Create the layout and frame for images
  // {
  this->dataPtr->frameLayout = new QGridLayout;
  this->dataPtr->frameLayout->setSizeConstraint(QLayout::SetMinimumSize);

  this->frame->setObjectName("blackBorderFrame");
  this->frame->setLayout(this->dataPtr->frameLayout);
  this->frame->setMinimumHeight(240);
  this->frame->setMinimumWidth(320);
  // }

  this->dataPtr->clearImages = false;
}

/////////////////////////////////////////////////
ImagesView::~ImagesView()
{
  delete this->dataPtr;
  this->dataPtr = NULL;

  this->sub.reset();
}

/////////////////////////////////////////////////
void ImagesView::UpdateImpl()
{
  boost::mutex::scoped_lock lock(this->dataPtr->mutex);
  std::vector<ImageFrame *>::iterator imageIter = this->dataPtr->images.begin();

  // Clear out the images if the flag is set.
  if (this->dataPtr->clearImages)
  {
    // Remove all the images, and delete them
    for (; imageIter != this->dataPtr->images.end(); )
    {
      (*imageIter)->hide();
      this->dataPtr->frameLayout->removeWidget(*imageIter);
      delete *imageIter;
      imageIter = this->dataPtr->images.erase(imageIter);
    }

    // Clear the lists
    this->dataPtr->images.clear();

    // Make sure to adjust the size of the widget
    this->frame->adjustSize();
    this->adjustSize();

    // Reset frame
    this->dataPtr->clearImages = false;
    return;
  }

  // Create new images when necessary
  std::vector<std::pair<int, int> >::iterator iter;
  for (iter = this->dataPtr->addImage.begin();
       iter != this->dataPtr->addImage.end(); ++iter)
  {
    this->AddImage((*iter).first, (*iter).second);
  }
  this->dataPtr->addImage.clear();
}

/////////////////////////////////////////////////
void ImagesView::SetTopic(const std::string &_topicName)
{
  boost::mutex::scoped_lock lock(this->dataPtr->mutex);

  // Tell the widget to clear the images
  this->dataPtr->clearImages = true;

  TopicView::SetTopic(_topicName);

  // Subscribe to the new topic.
  this->sub = this->node->Subscribe(_topicName, &ImagesView::OnImages, this);
}

/////////////////////////////////////////////////
void ImagesView::AddImage(int _width, int _height)
{
  ImageFrame *imageFrame = new ImageFrame(this);
  imageFrame->setBaseSize(_width, _height);
  imageFrame->setMinimumSize(320, 240);
  imageFrame->show();
  this->dataPtr->images.push_back(imageFrame);

  // Add the lable to the correct row and column
  this->dataPtr->frameLayout->addWidget(imageFrame,
      (this->dataPtr->images.size()-1) / 2,
      (this->dataPtr->images.size()-1) % 2);
}

/////////////////////////////////////////////////
void ImagesView::OnImages(ConstImagesStampedPtr &_msg)
{
  // Only use a try lock so that we don't block the node thread.
  boost::mutex::scoped_try_lock lock(this->dataPtr->mutex);
  if (!lock)
      return;

  if (this->dataPtr->clearImages)
    return;

  int dataSize = 0;
  this->dataPtr->addImage.clear();

  for (int i = 0; i < _msg->image_size(); ++i)
  {
    dataSize += _msg->image(i).data().size();

    if (i >= static_cast<int>(this->dataPtr->images.size()))
    {
      this->dataPtr->addImage.push_back(std::make_pair(_msg->image(i).width(),
                                              _msg->image(i).height()));
      continue;
    }

    this->dataPtr->images[i]->OnImage(_msg->image(i));
  }

  // Update the Hz and Bandwidth info
  this->OnMsg(msgs::Convert(_msg->time()), dataSize);
}
