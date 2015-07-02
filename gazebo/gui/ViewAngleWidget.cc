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
#include "gazebo/rendering/Scene.hh"

#include "gazebo/gui/Actions.hh"
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

  // Button size
  QSize iconSize(30, 30);
  this->dataPtr->topButton->setIconSize(iconSize);
  this->dataPtr->bottomButton->setIconSize(iconSize);
  this->dataPtr->frontButton->setIconSize(iconSize);
  this->dataPtr->backButton->setIconSize(iconSize);
  this->dataPtr->leftButton->setIconSize(iconSize);
  this->dataPtr->rightButton->setIconSize(iconSize);
  this->dataPtr->resetButton->setIconSize(iconSize);

  // Zoom
  this->dataPtr->zoomSlider = new QSlider(Qt::Horizontal, this);
  this->dataPtr->zoomSlider->setFocusPolicy(Qt::NoFocus);
  this->dataPtr->zoomSlider->setRange(1, 100);
  this->dataPtr->zoomSlider->setSliderPosition(40);

  // Dropdown
  this->dataPtr->projectionComboBox = new QComboBox(this);
  this->dataPtr->projectionComboBox->setMinimumWidth(150);
  this->dataPtr->projectionComboBox->addItem("Perspective", 0);
  this->dataPtr->projectionComboBox->addItem("Orthographic", 1);
  connect(this->dataPtr->projectionComboBox, SIGNAL(currentIndexChanged(int)),
      this, SLOT(OnProjection(int)));

  // Main layout
  this->dataPtr->mainLayout = new QGridLayout();
  this->dataPtr->mainLayout->addWidget(this->dataPtr->zoomSlider, 3, 0, 1, 4);
  this->dataPtr->mainLayout->addWidget(this->dataPtr->projectionComboBox,
      4, 0, 1, 4);
  this->setLayout(this->dataPtr->mainLayout);

  // Connect the ortho action
  connect(g_cameraOrthoAct, SIGNAL(triggered()), this, SLOT(OnOrtho()));

  // connect the perspective action
  connect(g_cameraPerspectiveAct, SIGNAL(triggered()), this,
      SLOT(OnPerspective()));
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
void ViewAngleWidget::MoveCamera(math::Vector3 _dir)
{
  rendering::UserCameraPtr cam = gui::get_active_camera();

  if (!cam)
    return;

  // Distance from look at point
  double dist = this->dataPtr->zoomSlider->sliderPosition();

  // Look at world origin
  math::Vector3 lookAt = math::Vector3::Zero;

  // Look at first contact point in the middle of the screen unless it's on
  // a plane
  math::Vector2i midScreen2D(cam->GetImageWidth()/2, cam->GetImageHeight()/2);

  rendering::VisualPtr vis = cam->GetScene()->GetVisualAt(cam, midScreen2D);

  if (vis && !vis->IsPlane())
    cam->GetScene()->GetFirstContact(cam, midScreen2D, lookAt);

  // Camera will be positioned with respect to that point
  math::Vector3 camPos = lookAt + _dir * dist;

  // Calculate camera orientation
  math::Vector3 negDir = -_dir;
  double roll = 0;
  double pitch = -atan2(negDir.z, sqrt(pow(negDir.x, 2) + pow(negDir.y, 2)));
  double yaw = atan2(negDir.y, negDir.x);

  math::Quaternion quat =  math::Quaternion(roll, pitch, yaw);

  // Move camera to that pose in 1s
  cam->MoveToPosition(math::Pose(camPos, quat), 1);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnTopView()
{
  this->MoveCamera(math::Vector3::UnitZ);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnBottomView()
{
  this->MoveCamera(-math::Vector3::UnitZ);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnFrontView()
{
  this->MoveCamera(math::Vector3::UnitX);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnBackView()
{
  this->MoveCamera(-math::Vector3::UnitX);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnLeftView()
{
  this->MoveCamera(math::Vector3::UnitY);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnRightView()
{
  this->MoveCamera(-math::Vector3::UnitY);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnResetView()
{
  rendering::UserCameraPtr cam = gui::get_active_camera();

  if (!cam)
    return;

  cam->MoveToPosition(cam->GetDefaultPose(), 1);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnPerspective()
{
  this->dataPtr->projectionComboBox->blockSignals(true);
  this->dataPtr->projectionComboBox->setCurrentIndex(0);
  this->dataPtr->projectionComboBox->blockSignals(false);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnOrtho()
{
  this->dataPtr->projectionComboBox->blockSignals(true);
  this->dataPtr->projectionComboBox->setCurrentIndex(1);
  this->dataPtr->projectionComboBox->blockSignals(false);
}

/////////////////////////////////////////////////
void ViewAngleWidget::OnProjection(int _index)
{
  if (_index == 0)
    g_cameraPerspectiveAct->trigger();
  else if (_index == 1)
    g_cameraOrthoAct->trigger();
}

/////////////////////////////////////////////////
void ViewAngleWidget::showEvent(QShowEvent */*_event*/)
{
  // Update slider with current zoom level the moment the widget is opened
  rendering::UserCameraPtr cam = gui::get_active_camera();

  if (!cam)
    return;

  math::Vector2i midScreen2D(cam->GetImageWidth()/2, cam->GetImageHeight()/2);

  math::Vector3 midScreen3D;
  cam->GetScene()->GetFirstContact(cam, midScreen2D, midScreen3D);

  double camDist  = fabs((cam->GetWorldPosition() - midScreen3D).GetLength());

  this->dataPtr->zoomSlider->setSliderPosition(camDist);
}

