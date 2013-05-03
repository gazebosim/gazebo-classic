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
#include <sstream>

#include "transport/Node.hh"

#include "gui/Actions.hh"
#include "gui/GuiEvents.hh"
#include "gui/DataPlayback.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
DataPlayback::DataPlayback(QWidget *_parent)
  : QWidget(_parent)
{
  this->setObjectName("dataPlayBack");

  QHBoxLayout *mainLayout = new QHBoxLayout;

  QFrame *frame = new QFrame;
  QHBoxLayout *frameLayout = new QHBoxLayout;

  frame->setLayout(frameLayout);
  frame->layout()->setContentsMargins(0, 0, 0, 0);

  mainLayout->addWidget(frame);
  this->setLayout(mainLayout);

  this->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
  this->layout()->setContentsMargins(0, 0, 0, 0);

  this->show();
}

/////////////////////////////////////////////////
DataPlayback::~DataPlayback()
{
}
