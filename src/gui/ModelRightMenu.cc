/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#include "transport/transport.h"
#include "rendering/UserCamera.hh"
#include "rendering/Scene.hh"
#include "rendering/Visual.hh"
#include "gui/Gui.hh"
#include "gui/JointControlWidget.hh"
#include "gui/ModelRightMenu.hh"

using namespace gazebo;
using namespace gui;

ModelRightMenu::ModelRightMenu()
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->requestPub = this->node->Advertise<msgs::Request>("~/request", 5, true);

  this->snapBelowAction = new QAction(tr("Snap"), this);
  this->snapBelowAction->setStatusTip(tr("Snap to object below"));
  connect(this->snapBelowAction, SIGNAL(triggered()), this,
          SLOT(OnSnapBelow()));

  this->followAction = new QAction(tr("Follow"), this);
  this->followAction->setStatusTip(tr("Follow the selection"));
  connect(this->followAction, SIGNAL(triggered()), this, SLOT(OnFollow()));

  this->moveToAction = new QAction(tr("Move To"), this);
  this->moveToAction->setStatusTip(tr("Move camera to the selection"));
  connect(this->moveToAction, SIGNAL(triggered()), this, SLOT(OnMoveTo()));

  this->deleteAction = new QAction(tr("Delete"), this);
  this->deleteAction->setStatusTip(tr("Delete the selection"));
  connect(this->deleteAction, SIGNAL(triggered()), this, SLOT(OnDelete()));

  this->showCollisionAction = new QAction(tr("Show Collision"), this);
  this->showCollisionAction->setStatusTip(tr("Show Collision Entity"));
  this->showCollisionAction->setCheckable(true);
  connect(this->showCollisionAction, SIGNAL(triggered()), this,
          SLOT(OnShowCollision()));

  this->transparentAction = new QAction(tr("Transparent"), this);
  this->transparentAction->setStatusTip(tr("Make model transparent"));
  this->transparentAction->setCheckable(true);
  connect(this->transparentAction, SIGNAL(triggered()), this,
          SLOT(OnTransparent()));

  this->showJointsAction = new QAction(tr("Joints"), this);
  this->showJointsAction->setStatusTip(tr("Show joints"));
  this->showJointsAction->setCheckable(true);
  connect(this->showJointsAction, SIGNAL(triggered()), this,
          SLOT(OnShowJoints()));

  this->showCOMAction = new QAction(tr("Center of Mass"), this);
  this->showCOMAction->setStatusTip(tr("Show Center of Mass"));
  this->showCOMAction->setCheckable(true);
  connect(this->showCOMAction, SIGNAL(triggered()), this,
          SLOT(OnShowCOM()));

  this->jointControlAction = new QAction(tr("Control Joints"), this);
  this->jointControlAction->setStatusTip(tr("Control the model's Joints"));
  connect(this->jointControlAction, SIGNAL(triggered()), this,
          SLOT(OnJointControl()));
}

/////////////////////////////////////////////////
ModelRightMenu::~ModelRightMenu()
{
  this->node->Fini();
}

/////////////////////////////////////////////////
void ModelRightMenu::Run(const std::string &_modelName, const QPoint &_pt)
{
  this->modelName = _modelName.substr(0, _modelName.find("::"));

  QMenu menu;
  menu.addAction(this->snapBelowAction);
  menu.addAction(this->moveToAction);
  menu.addAction(this->followAction);
  menu.addAction(this->showCollisionAction);
  menu.addAction(this->showJointsAction);
  menu.addAction(this->showCOMAction);
  menu.addAction(this->transparentAction);
  menu.addAction(this->deleteAction);
  menu.addAction(this->jointControlAction);

  if (this->transparentActionState[this->modelName])
    this->transparentAction->setChecked(true);
  else
    this->transparentAction->setChecked(false);

  if (this->showCollisionsActionState[this->modelName])
    this->showCollisionAction->setChecked(true);
  else
    this->showCollisionAction->setChecked(false);

  menu.exec(_pt);
}

/////////////////////////////////////////////////
void ModelRightMenu::OnSnapBelow()
{
  rendering::UserCameraPtr cam = gui::get_active_camera();
  cam->GetScene()->SnapVisualToNearestBelow(this->modelName);
}

/////////////////////////////////////////////////
void ModelRightMenu::OnMoveTo()
{
  rendering::UserCameraPtr cam = gui::get_active_camera();
  cam->MoveToVisual(this->modelName);
}

/////////////////////////////////////////////////
void ModelRightMenu::OnShowCollision()
{
  this->showCollisionsActionState[this->modelName] =
    this->showCollisionAction->isChecked();

  if (this->showCollisionAction->isChecked())
    this->requestMsg = msgs::CreateRequest("show_collision", this->modelName);
  else
    this->requestMsg = msgs::CreateRequest("hide_collision", this->modelName);

  this->requestPub->Publish(*this->requestMsg);
}

/////////////////////////////////////////////////
void ModelRightMenu::OnShowJoints()
{
  this->showJointsActionState[this->modelName] =
    this->showJointsAction->isChecked();

  if (this->showJointsAction->isChecked())
    this->requestMsg = msgs::CreateRequest("show_joints", this->modelName);
  else
    this->requestMsg = msgs::CreateRequest("hide_joints", this->modelName);

  this->requestPub->Publish(*this->requestMsg);
}

/////////////////////////////////////////////////
void ModelRightMenu::OnShowCOM()
{
  this->showCOMActionState[this->modelName] =
    this->showCOMAction->isChecked();

  if (this->showCOMAction->isChecked())
    this->requestMsg = msgs::CreateRequest("show_com", this->modelName);
  else
    this->requestMsg = msgs::CreateRequest("hide_com", this->modelName);

  this->requestPub->Publish(*this->requestMsg);
}

/////////////////////////////////////////////////
void ModelRightMenu::OnTransparent()
{
  this->transparentActionState[this->modelName] =
    this->transparentAction->isChecked();

  if (this->transparentAction->isChecked())
  {
    this->requestMsg = msgs::CreateRequest("set_transparency", this->modelName);
    this->requestMsg->set_dbl_data(0.5);
  }
  else
  {
    this->requestMsg = msgs::CreateRequest("set_transparency", this->modelName);
    this->requestMsg->set_dbl_data(0.0);
  }

  this->requestPub->Publish(*this->requestMsg);
}

/////////////////////////////////////////////////
void ModelRightMenu::OnFollow()
{
  rendering::UserCameraPtr cam = gui::get_active_camera();
  cam->TrackVisual(this->modelName);
}

/////////////////////////////////////////////////
void ModelRightMenu::OnDelete()
{
  this->requestMsg = msgs::CreateRequest("entity_delete", this->modelName);
  this->requestPub->Publish(*this->requestMsg);
}

/////////////////////////////////////////////////
void ModelRightMenu::OnJointControl()
{
  // JointControlWidget *jc = new JointControlWidget(this->modelName);
  // jc->show();
}
