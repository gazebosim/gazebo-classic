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

#include "gazebo/gui/viewers/TextView.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
TextView::TextView(QWidget *_parent)
: TopicView("gazebo.msgs.GzString", _parent)
{
  this->setWindowTitle(tr("Gazebo: Text View"));

  // Create the text list
  // {
  QVBoxLayout *frameLayout = new QVBoxLayout;

  this->textView = new QListView();

  frameLayout->addWidget(this->textView);
  this->frame->setObjectName("blackBorderFrame");
  this->frame->setLayout(frameLayout);
  // }
}

/////////////////////////////////////////////////
TextView::~TextView()
{
}

/////////////////////////////////////////////////
void TextView::UpdateImpl()
{
  // Anything?
}

/////////////////////////////////////////////////
void TextView::SetTopic(const std::string &_topicName)
{
  // Subscribe to the new topic.
  this->sub.reset();
  this->sub = this->node->Subscribe(_topicName, &TextView::OnText, this);
}

/////////////////////////////////////////////////
void TextView::OnText(ConstGzStringPtr &_msg)
{
  // Update the Hz and Bandwidth info
  this->OnMsg(common::Time::GetWallTime(), _msg->image().data().size());

  // Add the new text to the output view.
}
