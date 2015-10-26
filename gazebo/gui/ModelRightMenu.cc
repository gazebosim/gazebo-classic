/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include <boost/bind.hpp>

#include "gazebo/transport/transport.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"

#include "gazebo/gui/KeyEventHandler.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/ApplyWrenchDialog.hh"
#include "gazebo/gui/ModelRightMenu.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelRightMenu::ModelRightMenu()
{
  KeyEventHandler::Instance()->AddReleaseFilter("ModelRightMenu",
        boost::bind(&ModelRightMenu::OnKeyRelease, this, _1));

  this->moveToAct = new QAction(tr("Move To"), this);
  this->moveToAct->setStatusTip(tr("Move camera to the selection"));
  connect(this->moveToAct, SIGNAL(triggered()), this, SLOT(OnMoveTo()));

  this->followAct = new QAction(tr("Follow"), this);
  this->followAct->setStatusTip(tr("Follow the selection"));
  connect(this->followAct, SIGNAL(triggered()), this, SLOT(OnFollow()));

  this->editAct = new QAction(tr("Edit model"), this);
  this->editAct->setStatusTip(tr("Open on Model Editor"));
  connect(this->editAct, SIGNAL(triggered()), this, SLOT(OnEdit()));

  this->applyWrenchAct = new QAction(tr("Apply Force/Torque"), this);
  this->applyWrenchAct->setStatusTip(tr("Apply force and torque to a link"));
  connect(this->applyWrenchAct, SIGNAL(triggered()), this,
      SLOT(OnApplyWrench()));

  // \todo Reimplement
  // this->snapBelowAct = new QAction(tr("Snap"), this);
  // this->snapBelowAct->setStatusTip(tr("Snap to object below"));
  // connect(this->snapBelowAct, SIGNAL(triggered()), this,
  //          SLOT(OnSnapBelow()));

  // Create the delete action
  g_deleteAct = new DeleteAction(tr("Delete"), this);
  g_deleteAct->setStatusTip(tr("Delete a model"));
  connect(g_deleteAct, SIGNAL(DeleteSignal(const std::string &)), this,
          SLOT(OnDelete(const std::string &)));
  connect(g_deleteAct, SIGNAL(triggered()), this,
          SLOT(OnDelete()));

  ViewState *state = new ViewState(this, "set_transparent", "set_opaque");
  state->action = new QAction(tr("Transparent"), this);
  state->action->setStatusTip(tr("Make model transparent"));
  state->action->setCheckable(true);
  connect(state->action, SIGNAL(triggered()), state, SLOT(Callback()));
  this->viewStates.push_back(state);

  state = new ViewState(this, "set_wireframe", "set_solid");
  state->action = new QAction(tr("Wireframe"), this);
  state->action->setStatusTip(tr("Wireframe mode"));
  state->action->setCheckable(true);
  connect(state->action, SIGNAL(triggered()), state, SLOT(Callback()));
  this->viewStates.push_back(state);

  state = new ViewState(this, "show_collision", "hide_collision");
  state->action = new QAction(tr("Collisions"), this);
  state->action->setStatusTip(tr("Show collision objects"));
  state->action->setCheckable(true);
  connect(state->action, SIGNAL(triggered()), state, SLOT(Callback()));
  this->viewStates.push_back(state);

  state = new ViewState(this, "show_joints", "hide_joints");
  state->action = new QAction(tr("Joints"), this);
  state->action->setStatusTip(tr("Show joints"));
  state->action->setCheckable(true);
  connect(state->action, SIGNAL(triggered()), state, SLOT(Callback()));
  this->viewStates.push_back(state);

  state = new ViewState(this, "show_com", "hide_com");
  state->action = new QAction(tr("Center of mass"), this);
  state->action->setStatusTip(tr("Show center of mass"));
  state->action->setCheckable(true);
  connect(state->action, SIGNAL(triggered()), state, SLOT(Callback()));
  this->viewStates.push_back(state);

  state = new ViewState(this, "show_inertia", "hide_inertia");
  state->action = new QAction(tr("Inertia"), this);
  state->action->setStatusTip(tr("Show moments of inertia"));
  state->action->setCheckable(true);
  connect(state->action, SIGNAL(triggered()), state, SLOT(Callback()));
  this->viewStates.push_back(state);

  state = new ViewState(this, "show_link_frame", "hide_link_frame");
  state->action = new QAction(tr("Link Frames"), this);
  state->action->setStatusTip(tr("Show link frames"));
  state->action->setCheckable(true);
  connect(state->action, SIGNAL(triggered()), state, SLOT(Callback()));
  this->viewStates.push_back(state);

  // \todo Reimplement
  // this->skeletonAction = new QAction(tr("Skeleton"), this);
  // this->skeletonAction->setStatusTip(tr("Show model skeleton"));
  // this->skeletonAction->setCheckable(true);
  // connect(this->skeletonAction, SIGNAL(triggered()), this,
  //         SLOT(OnSkeleton()));

  // Window mode
  this->windowMode = "Simulation";

  // Event connections
  this->connections.push_back(
      gui::Events::ConnectWindowMode(
      boost::bind(&ModelRightMenu::OnWindowMode, this, _1)));
}

//////////////////////////////////////////////////
bool ModelRightMenu::Init()
{
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->requestSub = this->node->Subscribe("~/request",
      &ModelRightMenu::OnRequest, this);

  return true;
}

/////////////////////////////////////////////////
bool ModelRightMenu::OnKeyRelease(const common::KeyEvent &_event)
{
  if (_event.key == Qt::Key_Escape)
  {
    rendering::UserCameraPtr cam = gui::get_active_camera();
    cam->TrackVisual("");
    gui::Events::follow("");
  }

  return false;
}

/////////////////////////////////////////////////
ModelRightMenu::~ModelRightMenu()
{
  this->node->Fini();
}

/////////////////////////////////////////////////
void ModelRightMenu::Run(const std::string &_entityName, const QPoint &_pt,
    EntityTypes _type)
{
  // Find out the entity type
  if (_type == EntityTypes::MODEL || _type == EntityTypes::LIGHT)
  {
    this->entityName = _entityName.substr(0, _entityName.find("::"));
  }
  else if (_type == EntityTypes::LINK)
  {
    this->entityName = _entityName;
  }

  QMenu menu;

  // Move To
  menu.addAction(this->moveToAct);

  // Follow
  menu.addAction(this->followAct);

  // Apply Force/Torque
  if (this->windowMode == "Simulation" &&
      (_type == EntityTypes::MODEL || _type == EntityTypes::LINK))
    menu.addAction(this->applyWrenchAct);

  if (_type == EntityTypes::MODEL)
  {
    rendering::UserCameraPtr cam = gui::get_active_camera();
    rendering::ScenePtr scene = cam->GetScene();
    rendering::VisualPtr vis = scene->GetVisual(this->entityName);

    // Edit Model
    /// \todo Support editing planes
    if (vis && !vis->IsPlane() && this->windowMode == "Simulation")
    {
      menu.addSeparator();
      menu.addAction(this->editAct);
      menu.addSeparator();
    }

    // menu.addAction(this->snapBelowAct);

    // Create the view menu
    QMenu *viewMenu = menu.addMenu(tr("View"));
    for (std::vector<ViewState*>::iterator iter = this->viewStates.begin();
         iter != this->viewStates.end(); ++iter)
    {
      viewMenu->addAction((*iter)->action);

      std::map<std::string, bool>::iterator modelIter =
        (*iter)->modelStates.find(this->entityName);

      if (modelIter == (*iter)->modelStates.end())
        (*iter)->action->setChecked((*iter)->globalEnable);
      else
        (*iter)->action->setChecked(modelIter->second);
    }
  }

  if (this->windowMode == "Simulation" &&
      (_type == EntityTypes::MODEL || _type == EntityTypes::LIGHT))
  {
    if (g_copyAct && g_pasteAct)
    {
      menu.addSeparator();
      // Copy
      menu.addAction(g_copyAct);
      // Paste
      menu.addAction(g_pasteAct);
    }

    menu.addSeparator();
    // Delete
    menu.addAction(g_deleteAct);
  }

  /// \todo Reimplement these features.
  // menu.addAction(this->skeletonAction);

  menu.exec(_pt);
}

/////////////////////////////////////////////////
void ModelRightMenu::OnMoveTo()
{
  rendering::UserCameraPtr cam = gui::get_active_camera();
  cam->MoveToVisual(this->entityName);
}

/////////////////////////////////////////////////
void ModelRightMenu::OnFollow()
{
  rendering::UserCameraPtr cam = gui::get_active_camera();
  cam->TrackVisual(this->entityName);
  gui::Events::follow(this->entityName);
}

/////////////////////////////////////////////////
void ModelRightMenu::OnEdit()
{
  g_editModelAct->trigger();
  gui::Events::editModel(this->entityName);
}

/////////////////////////////////////////////////
void ModelRightMenu::OnApplyWrench()
{
  ApplyWrenchDialog *applyWrenchDialog = new ApplyWrenchDialog();

  rendering::VisualPtr vis = gui::get_active_camera()->GetScene()->
      GetVisual(this->entityName);

  if (!vis)
  {
    gzerr << "Can't find entity " << this->entityName << std::endl;
    return;
  }

  std::string modelName, linkName;
  if (vis == vis->GetRootVisual())
  {
    modelName = this->entityName;
    // If model selected just take the first link
    linkName = vis->GetChild(0)->GetName();
  }
  else
  {
    modelName = vis->GetRootVisual()->GetName();
    linkName = this->entityName;
  }

  applyWrenchDialog->Init(modelName, linkName);
}

/////////////////////////////////////////////////
// void ModelRightMenu::OnSnapBelow()
// {
//   rendering::UserCameraPtr cam = gui::get_active_camera();
//   if (!cam)
//     gzerr << "Invalid user camera\n";
//
//   if (!cam->GetScene())
//     gzerr << "Invalid user camera scene\n";
//
//   // cam->GetScene()->SnapVisualToNearestBelow(this->entityName);
// }

/////////////////////////////////////////////////
void ModelRightMenu::OnDelete(const std::string &_name)
{
  std::string name = _name;
  if (name.empty())
    name = this->entityName;

  // Delete the entity
  if (!name.empty())
    transport::requestNoReply(this->node, "entity_delete", name);
}

/////////////////////////////////////////////////
void ModelRightMenu::OnRequest(ConstRequestPtr &_msg)
{
  // Process the request by looking at all the view states.
  for (std::vector<ViewState*>::iterator iter = this->viewStates.begin();
       iter != this->viewStates.end(); ++iter)
  {
    // Only proceed if the request matches one of the check or uncheck
    // requests of the view state
    if (_msg->request() == (*iter)->checkRequest ||
        _msg->request() == (*iter)->uncheckRequest)
    {
      // Determine the value(state) of the view states
      bool value = _msg->request() == (*iter)->checkRequest ? true : false;

      // If the request is for all objects...
      if (_msg->data() == "all")
      {
        // Set all model states within the view state to the value.
        for (std::map<std::string, bool>::iterator modelIter =
            (*iter)->modelStates.begin();
            modelIter != (*iter)->modelStates.end(); ++modelIter)
        {
          modelIter->second = value;
        }

        // Use a globalEnable to handle the case when new models are added
        (*iter)->globalEnable = value;
      }
      // Otherwise the request is for a single model...
      else
      {
        // Set the state of the given model
        (*iter)->modelStates[_msg->data()] = value;
      }
    }
  }
}

/////////////////////////////////////////////////
ViewState::ViewState(ModelRightMenu *_parent,
                     const std::string &_checkRequest,
                     const std::string &_uncheckRequest)
  : QObject(_parent)
{
  this->globalEnable = false;
  this->action = NULL;
  this->parent = _parent;
  this->checkRequest = _checkRequest;
  this->uncheckRequest = _uncheckRequest;
}

/////////////////////////////////////////////////
void ViewState::Callback()
{
  // Store the check state for the model
  this->modelStates[this->parent->entityName] = this->action->isChecked();

  // Send a message with the new check state. The Scene listens to these
  // messages and updates the visualizations accordingly.
  if (this->action->isChecked())
  {
    transport::requestNoReply(this->parent->node, this->checkRequest,
                              this->parent->entityName);
  }
  else
  {
    transport::requestNoReply(this->parent->node, this->uncheckRequest,
                              this->parent->entityName);
  }
}

/////////////////////////////////////////////////
void ModelRightMenu::OnWindowMode(const std::string &_mode)
{
  this->windowMode = _mode;
}

/// \todo Reimplement these functions.
/////////////////////////////////////////////////
// void ModelRightMenu::OnSkeleton()
// {
//   this->skeletonActionState[this->entityName] =
//     this->skeletonAction->isChecked();
//
//   if (this->skeletonAction->isChecked())
//   {
//     this->requestMsg = msgs::CreateRequest("show_skeleton",
//         this->entityName);
//     this->requestMsg->set_dbl_data(1.0);
//   }
//   else
//   {
//     this->requestMsg = msgs::CreateRequest("show_skeleton",
//         this->entityName);
//     this->requestMsg->set_dbl_data(0.0);
//   }
//
//   this->requestPub->Publish(*this->requestMsg);
// }
