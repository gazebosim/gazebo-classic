/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include <memory>

#include "gazebo/transport/Node.hh"

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
  std::unique_ptr<QGridLayout> frameLayout(new QGridLayout);
  frameLayout->setSizeConstraint(QLayout::SetMinimumSize);

  this->frame->setObjectName("blackBorderFrame");
  this->frame->setLayout(frameLayout.release());
  this->frame->setMinimumHeight(240);
  this->frame->setMinimumWidth(320);

  this->dataPtr->clearImages = false;
}

/////////////////////////////////////////////////
ImagesView::~ImagesView()
{
  this->sub.reset();

  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void ImagesView::UpdateImpl()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Clear out the images if the flag is set.
  if (this->dataPtr->clearImages)
  {
    // Clear previous layout
    auto oldLayout = this->frame->layout();
    if (oldLayout)
    {
      // Let Qt delete the widgets. Give ownership of all widgets to an object
      // which will be out of scope.
      QWidget().setLayout(oldLayout);
    }

    // Create new layout
    std::unique_ptr<QGridLayout> newLayout(new QGridLayout);
    newLayout->setSizeConstraint(QLayout::SetMinimumSize);
    this->frame->setLayout(newLayout.release());

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
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  // Tell the widget to clear the images
  this->dataPtr->clearImages = true;

  TopicView::SetTopic(_topicName);

  // Subscribe to the new topic.
  if (this->node)
    this->sub = this->node->Subscribe(_topicName, &ImagesView::OnImages, this);
}

/////////////////////////////////////////////////
void ImagesView::AddImage(int _width, int _height)
{
  std::unique_ptr<ImageFrame> imageFrame(new ImageFrame(NULL));
  imageFrame->setBaseSize(_width, _height);
  imageFrame->setMinimumSize(320, 240);
  imageFrame->show();

  // Add the lable to the correct row and column
  auto frameLayout = qobject_cast<QGridLayout *>(this->frame->layout());
  if (!frameLayout)
    return;

  frameLayout->addWidget(imageFrame.release(),
      (frameLayout->count()) / 2,
      (frameLayout->count()) % 2);
}

/////////////////////////////////////////////////
void ImagesView::OnImages(ConstImagesStampedPtr &_msg)
{
  // Only use a try lock so that we don't block the node thread.
  std::unique_lock<std::mutex> lock(this->dataPtr->mutex, std::try_to_lock);
  if (!lock.owns_lock())
      return;

  if (this->dataPtr->clearImages)
    return;

  int dataSize = 0;
  this->dataPtr->addImage.clear();

  for (int i = 0; i < _msg->image_size(); ++i)
  {
    dataSize += _msg->image(i).data().size();

    if (i >= this->frame->layout()->count())
    {
      this->dataPtr->addImage.push_back(std::make_pair(_msg->image(i).width(),
                                              _msg->image(i).height()));
      continue;
    }

    auto frameLayout = qobject_cast<QGridLayout *>(this->frame->layout());
    if (!frameLayout)
      continue;

    auto imageFrame = qobject_cast<ImageFrame *>(
        frameLayout->itemAtPosition(i / 2, i % 2)->widget());

    if (imageFrame)
      imageFrame->OnImage(_msg->image(i));
  }

  // Update the Hz and Bandwidth info
  this->OnMsg(msgs::Convert(_msg->time()), dataSize);
}
