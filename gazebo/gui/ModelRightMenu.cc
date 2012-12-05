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
  this->showAllCollisions = false;
  this->showAllJoints = false;
  this->showAllCOM = false;
  this->allTransparent = false;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->requestSub = this->node->Subscribe("~/request",
      &ModelRightMenu::OnRequest, this);

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

  this->snapBelowAct = new QAction(tr("Snap"), this);
  this->snapBelowAct->setStatusTip(tr("Snap to object below"));
  connect(this->snapBelowAct, SIGNAL(triggered()), this,
           SLOT(OnSnapBelow()));

  // Create the delete action
  g_deleteAct = new DeleteAction(tr("Delete"), this);
  g_deleteAct->setStatusTip(tr("Delete a model"));
  connect(g_deleteAct, SIGNAL(DeleteSignal(const std::string &)), this,
          SLOT(OnDelete(const std::string &)));
  connect(g_deleteAct, SIGNAL(triggered()), this,
          SLOT(OnDelete()));

  // this->followAction = new QAction(tr("Follow"), this);
  // this->followAction->setStatusTip(tr("Follow the selection"));
  // connect(this->followAction, SIGNAL(triggered()), this, SLOT(OnFollow()));


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
  menu.addAction(this->moveToAct);
  menu.addAction(this->snapBelowAct);

  QMenu *viewMenu = menu.addMenu(tr("View"));
  viewMenu->addAction(this->transparentAct);
  viewMenu->addAction(this->showCollisionAct);
  viewMenu->addAction(this->showJointsAct);
  viewMenu->addAction(this->showCOMAct);

  menu.addSeparator();
  menu.addAction(g_deleteAct);

  // menu.addAction(this->followAction);
  // menu.addAction(this->skeletonAction);

  if (this->transparentActionState[this->modelName] || this->allTransparent)
    this->transparentAct->setChecked(true);
  else
    this->transparentAct->setChecked(false);

  if (this->showCollisionsActionState[this->modelName] ||
      this->showAllCollisions)
    this->showCollisionAct->setChecked(true);
  else
    this->showCollisionAct->setChecked(false);

  if (this->showCOMActionState[this->modelName] || this->showAllCOM)
    this->showCOMAct->setChecked(true);
  else
    this->showCOMAct->setChecked(false);

  if (this->showJointsActionState[this->modelName] || this->showAllJoints)
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
    transport::requestNoReply(this->node, "show_collision", this->modelName);
  else
    transport::requestNoReply(this->node, "hide_collision", this->modelName);
}

/////////////////////////////////////////////////
void ModelRightMenu::OnShowJoints()
{
  this->showJointsActionState[this->modelName] =
    this->showJointsAct->isChecked();

  if (this->showJointsAct->isChecked())
    transport::requestNoReply(this->node, "show_joints", this->modelName);
  else
    transport::requestNoReply(this->node, "hide_joints", this->modelName);
}

/////////////////////////////////////////////////
void ModelRightMenu::OnShowCOM()
{
  this->showCOMActionState[this->modelName] =
    this->showCOMAct->isChecked();

  if (this->showCOMAct->isChecked())
    transport::requestNoReply(this->node, "show_com", this->modelName);
  else
    transport::requestNoReply(this->node, "hide_com", this->modelName);
}

/////////////////////////////////////////////////
void ModelRightMenu::OnTransparent()
{
  this->transparentActionState[this->modelName] =
    this->transparentAct->isChecked();

  if (this->transparentAct->isChecked())
    transport::requestNoReply(this->node, "set_transparent", this->modelName);
  else
    transport::requestNoReply(this->node, "set_opaque", this->modelName);
}

/////////////////////////////////////////////////
void ModelRightMenu::OnSnapBelow()
{
  rendering::UserCameraPtr cam = gui::get_active_camera();
  cam->GetScene()->SnapVisualToNearestBelow(this->modelName);
}

/////////////////////////////////////////////////
// void ModelRightMenu::OnSkeleton()
// {
//   this->skeletonActionState[this->modelName] =
//     this->skeletonAction->isChecked();
//
//   if (this->skeletonAction->isChecked())
//   {
//     this->requestMsg = msgs::CreateRequest("show_skeleton", this->modelName);
//     this->requestMsg->set_dbl_data(1.0);
//   }
//   else
//   {
//     this->requestMsg = msgs::CreateRequest("show_skeleton", this->modelName);
//     this->requestMsg->set_dbl_data(0.0);
//   }
//
//   this->requestPub->Publish(*this->requestMsg);
// }

/////////////////////////////////////////////////
// void ModelRightMenu::OnFollow()
// {
//   rendering::UserCameraPtr cam = gui::get_active_camera();
//   cam->TrackVisual(this->modelName);
// }

/////////////////////////////////////////////////
void ModelRightMenu::OnDelete(const std::string &_name)
{
  std::string name = _name;
  if (name.empty())
    name = this->modelName;

  if (!name.empty())
  {
    transport::requestNoReply(this->node, "entity_delete", name);
  }
}

/////////////////////////////////////////////////
void ModelRightMenu::OnRequest(ConstRequestPtr &_msg)
{
  if (_msg->request() == "show_collision" ||
      _msg->request() == "hide_collision" )
  {
    bool value = _msg->request() == "show_collision" ? true : false;
    if (_msg->data() == "all")
    {
      this->SetMap(this->showCollisionsActionState, value);
      this->showAllCollisions = value;
    }
    else
      this->showCollisionsActionState[_msg->data()] = value;
  }
  else if (_msg->request() == "show_joints" ||
           _msg->request() == "hide_joints" )
  {
    bool value = _msg->request() == "show_joints" ? true : false;
    if (_msg->data() == "all")
    {
      this->SetMap(this->showJointsActionState, value);
      this->showAllJoints = value;
    }
    else
      this->showJointsActionState[_msg->data()] = value;
  }
  else if (_msg->request() == "show_com" ||
           _msg->request() == "hide_com" )
  {
    bool value = _msg->request() == "show_com" ? true : false;
    if (_msg->data() == "all")
    {
      this->SetMap(this->showCOMActionState, value);
      this->showAllCOM = value;
    }
    else
      this->showCOMActionState[_msg->data()] = value;
  }
  else if (_msg->request() == "set_transparent" ||
           _msg->request() == "set_opaque" )
  {
    bool value = _msg->request() == "set_transparent" ? true : false;
    if (_msg->data() == "all")
    {
      this->SetMap(this->transparentActionState, value);
      this->allTransparent = value;
    }
    else
      this->transparentActionState[_msg->data()] = value;
  }
}

/////////////////////////////////////////////////
void ModelRightMenu::SetMap(std::map<std::string, bool> &_map, bool _value)
{
  for (std::map<std::string, bool>::iterator iter = _map.begin();
       iter != _map.end(); ++iter)
  {
    iter->second = _value;
  }
}
