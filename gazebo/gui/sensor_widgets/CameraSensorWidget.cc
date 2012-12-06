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
#include "gazebo/common/SystemPaths.hh"

#include "gazebo/gui/Gui.hh"
#include "gazebo/gui/GuiEvents.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/gui/sensor_widgets/CameraSensorWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
CameraSensorWidget::CameraSensorWidget(QWidget *_parent)
: QWidget(_parent)
{
  this->setObjectName("camera_sensor");

  QVBoxLayout *mainLayout = new QVBoxLayout;

  QFrame *frame = new QFrame;
  QVBoxLayout *frameLayout = new QVBoxLayout;

  frameLayout->setContentsMargins(0, 0, 0, 0);
  frame->setLayout(frameLayout);

  mainLayout->addWidget(frame);
  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0, 0, 0, 0);

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  // this->pub = this->node->Advertise<msgs::Sky>("~/sky");
}

/////////////////////////////////////////////////
CameraSensorWidget::~CameraSensorWidget()
{
}
