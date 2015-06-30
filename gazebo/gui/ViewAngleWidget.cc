/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gazebo/rendering/UserCamera.hh"

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/ViewAngleWidgetPrivate.hh"
#include "gazebo/gui/ViewAngleWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ViewAngleWidget::ViewAngleWidget(QWidget *_parent)
  : QWidget(_parent), dataPtr(new ViewAngleWidgetPrivate)
{
  // Lateral buttons
  this->dataPtr->topButton = new QToolButton(this);
  this->dataPtr->bottomButton = new QToolButton(this);
  this->dataPtr->frontButton = new QToolButton(this);
  this->dataPtr->backButton = new QToolButton(this);
  this->dataPtr->leftButton = new QToolButton(this);
  this->dataPtr->rightButton = new QToolButton(this);

  // Reset / home button
  this->dataPtr->resetButton = new QToolButton(this);

  // Main layout
  this->dataPtr->mainLayout = new QGridLayout();

  // Distance from world origin (zoom)
  this->dataPtr->dist = 40;

  this->setLayout(this->dataPtr->mainLayout);
}

/////////////////////////////////////////////////
ViewAngleWidget::~ViewAngleWidget()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void ViewAngleWidget::Add(Angle _angle, QAction *_action)
{
  if (_angle == TOP)
  {
    this->dataPtr->topButton->setDefaultAction(_action);
    this->dataPtr->mainLayout->addWidget(this->dataPtr->topButton, 0, 1);
    connect(_action, SIGNAL(triggered()), this, SLOT(OnTopView()));
  }
  else if (_angle == BOTTOM)
  {
    this->dataPtr->bottomButton->setDefaultAction(_action);
    this->dataPtr->mainLayout->addWidget(this->dataPtr->bottomButton, 2, 1);
    connect(_action, SIGNAL(triggered()), this, SLOT(OnBottomView()));
  }
  else if (_angle == FRONT)
  {
    this->dataPtr->frontButton->setDefaultAction(_action);
    this->dataPtr->mainLayout->addWidget(this->dataPtr->frontButton, 1, 1);
    connect(_action, SIGNAL(triggered()), this, SLOT(OnFrontView()));
  }
  else if (_angle == BACK)
  {
    this->dataPtr->backButton->setDefaultAction(_action);
    this->dataPtr->mainLayout->addWidget(this->dataPtr->backButton, 1, 3);
    connect(_action, SIGNAL(triggered()), this, SLOT(OnBackView()));
  }
  else if (_angle == LEFT)
  {
    this->dataPtr->leftButton->setDefaultAction(_action);
    this->dataPtr->mainLayout->addWidget(this->dataPtr->leftButton, 1, 0);
    connect(_action, SIGNAL(triggered()), this, SLOT(OnLeftView()));
  }
  else if (_angle == RIGHT)
  {
    this->dataPtr->rightButton->setDefaultAction(_action);
    this->dataPtr->mainLayout->addWidget(this->dataPtr->rightButton, 1, 2);
    connect(_action, SIGNAL(triggered()), this, SLOT(OnRightView()));
  }
  else if (_angle == RESET)
  {
    this->dataPtr->resetButton->setDefaultAction(_action);
    this->dataPtr->mainLayout->addWidget(this->dataPtr->resetButton, 0, 3);
    connect(_action, SIGNAL(triggered()), this, SLOT(OnResetView()));
  }
}

/////////////////////////////////////////////////
void ViewAngleWidget::MoveCamera(math::Vector3 _pos)
{
  rendering::UserCameraPtr cam = gui::get_active_camera();

  if (!cam)
    return;

  math::Vector3 lookAt = math::Vector3::Zero;
  math::Vector3 dir = lookAt - _pos;

  double roll = 0;
  double pitch = -atan2(dir.z, sqrt(pow(dir.x, 2) + pow(dir.y, 2)));
  double yaw = atan2(dir.y, dir.x);

  math::Quaternion quat =  math::Quaternion(roll, pitch, yaw);

  cam->MoveToPosition(math::Pose(_pos, quat), 1);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnTopView()
{
  this->MoveCamera(math::Vector3::UnitZ * this->dataPtr->dist);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnBottomView()
{
  this->MoveCamera(-math::Vector3::UnitZ * this->dataPtr->dist);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnFrontView()
{
  this->MoveCamera(math::Vector3::UnitX * this->dataPtr->dist);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnBackView()
{
  this->MoveCamera(-math::Vector3::UnitX * this->dataPtr->dist);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnLeftView()
{
  this->MoveCamera(math::Vector3::UnitY * this->dataPtr->dist);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnRightView()
{
  this->MoveCamera(-math::Vector3::UnitY * this->dataPtr->dist);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnResetView()
{
  this->MoveCamera(math::Vector3(5, -5, 2));
}

