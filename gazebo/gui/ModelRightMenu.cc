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

#include "transport/transport.hh"
#include "rendering/UserCamera.hh"
#include "rendering/Scene.hh"
#include "rendering/Visual.hh"

#include "gui/Actions.hh"
#include "gui/Gui.hh"
#include "gui/ModelRightMenu.hh"

using namespace gazebo;
using namespace gui;

ModelRightMenu::ModelRightMenu()
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->requestPub = this->node->Advertise<msgs::Request>("~/request", 5, true);

  // this->snapBelowAction = new QAction(tr("Snap"), this);
  // this->snapBelowAction->setStatusTip(tr("Snap to object below"));
  // connect(this->snapBelowAction, SIGNAL(triggered()), this,
  //         SLOT(OnSnapBelow()));

  // this->followAction = new QAction(tr("Follow"), this);
  // this->followAction->setStatusTip(tr("Follow the selection"));
  // connect(this->followAction, SIGNAL(triggered()), this, SLOT(OnFollow()));

  this->moveToAct= new QAction(tr("Move To"), this);
  this->moveToAct->setStatusTip(tr("Move camera to the selection"));
  connect(this->moveToAct, SIGNAL(triggered()), this, SLOT(OnMoveTo()));

  this->transparentAct = new QAction(tr("Transparent"), this);
  this->transparentAct->setStatusTip(tr("Make model transparent"));
  this->transparentAct->setCheckable(true);
  connect(this->transparentAct, SIGNAL(triggered()), this,
          SLOT(OnTransparent()));

  this->showCollisionAct = new QAction(tr("Collisions"), this);
  this->showCollisionAct->setStatusTip(tr("Show collisions objects"));
  this->showCollisionAct->setCheckable(true);
  connect(this->showCollisionAct, SIGNAL(triggered()), this,
          SLOT(OnShowCollision()));

  this->showJointsAct = new QAction(tr("Joints"), this);
  this->showJointsAct->setStatusTip(tr("Show joints"));
  this->showJointsAct->setCheckable(true);
  connect(this->showJointsAct, SIGNAL(triggered()), this,
          SLOT(OnShowJoints()));

  this->showCOMAct = new QAction(tr("Center of Mass"), this);
  this->showCOMAct->setStatusTip(tr("Show Center of Mass"));
  this->showCOMAct->setCheckable(true);
  connect(this->showCOMAct, SIGNAL(triggered()), this,
          SLOT(OnShowCOM()));

  // Create the delete action
  g_deleteAct = new DeleteAction(tr("Delete"), this);
  g_deleteAct->setStatusTip(tr("Delete a model"));
  connect(g_deleteAct, SIGNAL(DeleteSignal(const std::string &)), this,
          SLOT(OnDelete(const std::string &)));
  connect(g_deleteAct, SIGNAL(triggered()), this,
          SLOT(OnDelete()));


  // this->skeletonAction = new QAction(tr("Skeleton"), this);
  // this->skeletonAction->setStatusTip(tr("Show model skeleton"));
  // this->skeletonAction->setCheckable(true);
  // connect(this->skeletonAction, SIGNAL(triggered()), this,
  //         SLOT(OnSkeleton()));


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
  // menu.addAction(this->snapBelowAction);
  menu.addAction(this->moveToAct);

  QMenu *viewMenu = menu.addMenu(tr("View"));
  viewMenu->addAction(this->transparentAct);
  viewMenu->addAction(this->showCollisionAct);
  viewMenu->addAction(this->showJointsAct);
  viewMenu->addAction(this->showCOMAct);

  menu.addSeparator();
  menu.addAction(g_deleteAct);

  // menu.addAction(this->followAction);
  // menu.addAction(this->showCOMAction);
  // menu.addAction(this->skeletonAction);

  if (this->transparentActionState[this->modelName])
    this->transparentAct->setChecked(true);
  else
    this->transparentAct->setChecked(false);

  if (this->showCollisionsActionState[this->modelName])
    this->showCollisionAct->setChecked(true);
  else
    this->showCollisionAct->setChecked(false);

  if (this->showCOMActionState[this->modelName])
    this->showCOMAct->setChecked(true);
  else
    this->showCOMAct->setChecked(false);

  if (this->showJointsActionState[this->modelName])
    this->showJointsAct->setChecked(true);
  else
    this->showJointsAct->setChecked(false);


  // if (this->skeletonActionState[this->modelName])
  //   this->skeletonAction->setChecked(true);
  // else
  //   this->skeletonAction->setChecked(false);

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
    this->showCollisionAct->isChecked();

  if (this->showCollisionAct->isChecked())
    this->requestMsg = msgs::CreateRequest("show_collision", this->modelName);
  else
    this->requestMsg = msgs::CreateRequest("hide_collision", this->modelName);

  this->requestPub->Publish(*this->requestMsg);
}

/////////////////////////////////////////////////
void ModelRightMenu::OnShowJoints()
{
  this->showJointsActionState[this->modelName] =
    this->showJointsAct->isChecked();

  if (this->showJointsAct->isChecked())
    this->requestMsg = msgs::CreateRequest("show_joints", this->modelName);
  else
    this->requestMsg = msgs::CreateRequest("hide_joints", this->modelName);

  this->requestPub->Publish(*this->requestMsg);
}

/////////////////////////////////////////////////
void ModelRightMenu::OnShowCOM()
{
  this->showCOMActionState[this->modelName] =
    this->showCOMAct->isChecked();

  if (this->showCOMAct->isChecked())
    this->requestMsg = msgs::CreateRequest("show_com", this->modelName);
  else
    this->requestMsg = msgs::CreateRequest("hide_com", this->modelName);

  this->requestPub->Publish(*this->requestMsg);
}

/////////////////////////////////////////////////
void ModelRightMenu::OnTransparent()
{
  this->transparentActionState[this->modelName] =
    this->transparentAct->isChecked();

  if (this->transparentAct->isChecked())
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
void ModelRightMenu::OnSkeleton()
{
  this->skeletonActionState[this->modelName] =
    this->skeletonAction->isChecked();

  if (this->skeletonAction->isChecked())
  {
    this->requestMsg = msgs::CreateRequest("show_skeleton", this->modelName);
    this->requestMsg->set_dbl_data(1.0);
  }
  else
  {
    this->requestMsg = msgs::CreateRequest("show_skeleton", this->modelName);
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
void ModelRightMenu::OnDelete(const std::string &_name)
{
  std::string name = _name;
  if (name.empty())
    name = this->modelName;

  if (!name.empty())
  {
    this->requestMsg = msgs::CreateRequest("entity_delete", name);
    this->requestPub->Publish(*this->requestMsg);
  }
}
