/*
 * Copyright 2011 Nate Koenig
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

#include <gazebo/transport/transport.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/gui/Gui.hh>
#include <gazebo/gui/JointControlWidget.hh>
#include <gazebo/gui/LightRightMenu.hh>

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
LightRightMenu::LightRightMenu()
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->requestPub = this->node->Advertise<msgs::Request>("~/request", 5, true);

  this->moveToAction = new QAction(tr("Move To"), this);
  this->moveToAction->setStatusTip(tr("Move camera to the selection"));
  connect(this->moveToAction, SIGNAL(triggered()), this, SLOT(OnMoveTo()));

  this->deleteAction = new QAction(tr("Delete"), this);
  this->deleteAction->setStatusTip(tr("Delete the selection"));
  connect(this->deleteAction, SIGNAL(triggered()), this, SLOT(OnDelete()));
}

/////////////////////////////////////////////////
LightRightMenu::~LightRightMenu()
{
  this->node->Fini();
}

/////////////////////////////////////////////////
void LightRightMenu::Run(const std::string &_lightName, const QPoint &_pt)
{
  this->lightName = _lightName.substr(0, _lightName.find("::"));

  QMenu menu;
  menu.addAction(this->moveToAction);
  menu.addAction(this->deleteAction);

  menu.exec(_pt);
}

/////////////////////////////////////////////////
void LightRightMenu::OnMoveTo()
{
  rendering::UserCameraPtr cam = gui::get_active_camera();
  cam->MoveToVisual(this->lightName);
}

/////////////////////////////////////////////////
void LightRightMenu::OnDelete()
{
  this->requestMsg = msgs::CreateRequest("entity_delete", this->lightName);
  this->requestPub->Publish(*this->requestMsg);
}
