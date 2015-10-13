/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <string>
#include <vector>

#include "gazebo/common/MouseEvent.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/JointVisual.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/RenderTypes.hh"

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/KeyEventHandler.hh"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/MainWindow.hh"
#include "gazebo/gui/ModelAlign.hh"

#include "gazebo/gui/model/JointCreationDialog.hh"
#include "gazebo/gui/model/JointInspector.hh"
#include "gazebo/gui/model/JointData.hh"
#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/JointMaker.hh"

using namespace gazebo;
using namespace gui;

std::map<JointMaker::JointType, std::string> JointMaker::jointTypes;
std::map<JointMaker::JointType, std::string> JointMaker::jointMaterials;

/////////////////////////////////////////////////
JointMaker::JointMaker()
{
  this->unitVectors.push_back(ignition::math::Vector3d::UnitX);
  this->unitVectors.push_back(ignition::math::Vector3d::UnitY);
  this->unitVectors.push_back(ignition::math::Vector3d::UnitZ);

  this->jointBeingCreated = NULL;
  this->parentLinkVis = NULL;
  this->childLinkVis = NULL;
  this->modelSDF.reset();
  this->jointType = JointMaker::JOINT_NONE;
  this->jointCounter = 0;

  this->jointMaterials[JOINT_FIXED]     = "Gazebo/Red";
  this->jointMaterials[JOINT_HINGE]     = "Gazebo/Orange";
  this->jointMaterials[JOINT_HINGE2]    = "Gazebo/DarkYellow";
  this->jointMaterials[JOINT_SLIDER]    = "Gazebo/Green";
  this->jointMaterials[JOINT_SCREW]     = "Gazebo/DarkGrey";
  this->jointMaterials[JOINT_UNIVERSAL] = "Gazebo/Blue";
  this->jointMaterials[JOINT_BALL]      = "Gazebo/Purple";
  this->jointMaterials[JOINT_GEARBOX]   = "Gazebo/Indigo";

  this->jointTypes[JOINT_FIXED]     = "fixed";
  this->jointTypes[JOINT_HINGE]     = "revolute";
  this->jointTypes[JOINT_HINGE2]    = "revolute2";
  this->jointTypes[JOINT_SLIDER]    = "prismatic";
  this->jointTypes[JOINT_SCREW]     = "screw";
  this->jointTypes[JOINT_UNIVERSAL] = "universal";
  this->jointTypes[JOINT_BALL]      = "ball";
  this->jointTypes[JOINT_GEARBOX]   = "gearbox";
  this->jointTypes[JOINT_NONE]      = "none";

  this->jointCreationDialog = new JointCreationDialog(this);

  this->connections.push_back(
      event::Events::ConnectPreRender(
      boost::bind(&JointMaker::Update, this)));

  this->connections.push_back(
      gui::model::Events::ConnectOpenJointInspector(
      boost::bind(&JointMaker::OpenInspector, this, _1)));

  this->connections.push_back(
      gui::model::Events::ConnectShowJointContextMenu(
      boost::bind(&JointMaker::ShowContextMenu, this, _1)));

  this->connections.push_back(
      gui::model::Events::ConnectSetSelectedJoint(
      boost::bind(&JointMaker::OnSetSelectedJoint, this, _1, _2)));

  this->connections.push_back(
      event::Events::ConnectSetSelectedEntity(
      boost::bind(&JointMaker::OnSetSelectedEntity, this, _1, _2)));

  this->inspectAct = new QAction(tr("Open Joint Inspector"), this);
  connect(this->inspectAct, SIGNAL(triggered()), this, SLOT(OnOpenInspector()));

  this->updateMutex = new boost::recursive_mutex();

  // Gazebo event connections
  this->connections.push_back(
      gui::model::Events::ConnectLinkInserted(
      boost::bind(&JointMaker::OnLinkInserted, this, _1)));

  this->connections.push_back(
      gui::model::Events::ConnectLinkRemoved(
      boost::bind(&JointMaker::OnLinkRemoved, this, _1)));
}

/////////////////////////////////////////////////
JointMaker::~JointMaker()
{
  if (this->jointBeingCreated)
  {
    delete this->jointBeingCreated;
    this->jointBeingCreated = NULL;
  }

  {
    boost::recursive_mutex::scoped_lock lock(*this->updateMutex);
    while (this->joints.size() > 0)
    {
      std::string jointName = this->joints.begin()->first;
      this->RemoveJoint(jointName);
    }
    this->joints.clear();
  }

  delete this->updateMutex;
}

/////////////////////////////////////////////////
void JointMaker::Reset()
{
  boost::recursive_mutex::scoped_lock lock(*this->updateMutex);
  if (this->jointBeingCreated)
  {
    delete this->jointBeingCreated;
    this->jointBeingCreated = NULL;
  }

  // Reset child link
/*  if (this->childLinkVis)
  {
    this->childLinkVis->SetWorldPose(this->childLinkOriginalPose);
    this->SetHighlighted(this->childLinkVis, false);
  }*/

  this->jointType = JointMaker::JOINT_NONE;
  this->parentLinkVis.reset();
  this->childLinkVis.reset();
  this->hoverVis.reset();
  this->inspectName = "";
  this->selectedJoints.clear();

  this->scopedLinkedNames.clear();

  while (!this->joints.empty())
  {
    std::string jointId = this->joints.begin()->first;
    this->RemoveJoint(jointId);
  }
  this->joints.clear();
}

/////////////////////////////////////////////////
void JointMaker::EnableEventHandlers()
{
  MouseEventHandler::Instance()->AddDoubleClickFilter("model_joint",
    boost::bind(&JointMaker::OnMouseDoubleClick, this, _1));

  MouseEventHandler::Instance()->AddReleaseFilter("model_joint",
      boost::bind(&JointMaker::OnMouseRelease, this, _1));

  MouseEventHandler::Instance()->AddPressFilter("model_joint",
      boost::bind(&JointMaker::OnMousePress, this, _1));

  MouseEventHandler::Instance()->AddMoveFilter("model_joint",
      boost::bind(&JointMaker::OnMouseMove, this, _1));

  KeyEventHandler::Instance()->AddPressFilter("model_joint",
      boost::bind(&JointMaker::OnKeyPress, this, _1));
}

/////////////////////////////////////////////////
void JointMaker::DisableEventHandlers()
{
  MouseEventHandler::Instance()->RemoveDoubleClickFilter("model_joint");
  MouseEventHandler::Instance()->RemoveReleaseFilter("model_joint");
  MouseEventHandler::Instance()->RemovePressFilter("model_joint");
  MouseEventHandler::Instance()->RemoveMoveFilter("model_joint");
  KeyEventHandler::Instance()->RemovePressFilter("model_joint");
}

/////////////////////////////////////////////////
void JointMaker::RemoveJoint(const std::string &_jointId)
{
  boost::recursive_mutex::scoped_lock lock(*this->updateMutex);

  std::string jointId = _jointId;
  JointData *joint = NULL;

  // Existing joint
  if (this->joints.find(jointId) != this->joints.end())
  {
    joint = this->joints[jointId];
  }
  // Joint being created
  else if (this->jointBeingCreated)
  {
    joint = this->jointBeingCreated;
    // Already has hotspot
    if (joint->hotspot)
      jointId = joint->hotspot->GetName();
    // Still only line
    else
      jointId = "";
  }

  if (!joint)
    return;

  rendering::ScenePtr scene = rendering::get_scene();

  if (joint->handles)
  {
    scene->GetManager()->destroyBillboardSet(joint->handles);
    joint->handles = NULL;
  }

  if (joint->hotspot)
  {
    scene->RemoveVisual(joint->hotspot);
    joint->hotspot->Fini();
  }

  if (joint->visual)
  {
    scene->RemoveVisual(joint->visual);
    joint->visual->Fini();
  }

  if (joint->jointVisual)
  {
    rendering::JointVisualPtr parentAxisVis = joint->jointVisual
        ->GetParentAxisVisual();
    if (parentAxisVis)
    {
      parentAxisVis->GetParent()->DetachVisual(
          parentAxisVis->GetName());
      scene->RemoveVisual(parentAxisVis);
    }
    joint->jointVisual->GetParent()->DetachVisual(
        joint->jointVisual->GetName());
    scene->RemoveVisual(joint->jointVisual);
  }
  if (joint->inspector)
  {
    joint->inspector->hide();
    delete joint->inspector;
    joint->inspector = NULL;
  }
  if (this->jointBeingCreated && this->jointBeingCreated->hotspot)
    this->jointBeingCreated = NULL;
  joint->hotspot.reset();
  joint->visual.reset();
  joint->jointVisual.reset();
  joint->Parent().reset();
  joint->Child().reset();
  delete joint;
  gui::model::Events::modelChanged();
  if (jointId != "")
  {
    this->joints.erase(jointId);
    gui::model::Events::jointRemoved(jointId);
  }
}

/////////////////////////////////////////////////
void JointMaker::RemoveJointsByLink(const std::string &_linkName)
{
  this->Stop();

  std::vector<std::string> toDelete;
  for (auto it : this->joints)
  {
    JointData *joint = it.second;

    if (joint->Child()->GetName() == _linkName ||
        joint->Parent()->GetName() == _linkName)
    {
      toDelete.push_back(it.first);
    }
  }

  for (unsigned i = 0; i < toDelete.size(); ++i)
    this->RemoveJoint(toDelete[i]);

  toDelete.clear();
}

/////////////////////////////////////////////////
std::vector<JointData *> JointMaker::GetJointDataByLink(
    const std::string &_linkName) const
{
  std::vector<JointData *> linkJoints;
  for (auto jointIt : this->joints)
  {
    JointData *jointData = jointIt.second;

    if (jointData->Child()->GetName() == _linkName ||
        jointData->Parent()->GetName() == _linkName)
    {
      linkJoints.push_back(jointData);
    }
  }
  return linkJoints;
}

/////////////////////////////////////////////////
bool JointMaker::OnMousePress(const common::MouseEvent &_event)
{
  rendering::UserCameraPtr camera = gui::get_active_camera();
  if (_event.Button() == common::MouseEvent::MIDDLE)
  {
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
    camera->HandleMouseEvent(_event);
    return true;
  }
  else if (_event.Button() != common::MouseEvent::LEFT)
    return false;

//  if (!this->mouseMoveEnabled)
  //  return false;

  // intercept mouse press events when user clicks on the joint hotspot visual
  rendering::VisualPtr vis = camera->GetVisual(_event.Pos());
  if (vis)
  {
    if (this->joints.find(vis->GetName()) != this->joints.end())
    {
      // stop event propagation as we don't want users to manipulate the
      // hotspot
      return true;
    }
  }
  return false;
}

/////////////////////////////////////////////////
bool JointMaker::OnMouseRelease(const common::MouseEvent &_event)
{
  rendering::UserCameraPtr camera = gui::get_active_camera();

  // Not in the process of creating a joint
  // or already chose parent and child
  if (!this->mouseMoveEnabled || (this->parentLinkVis && this->childLinkVis))
  {
    rendering::VisualPtr vis = camera->GetVisual(_event.Pos());
    if (vis)
    {
      if (this->joints.find(vis->GetName()) != this->joints.end())
      {
        // trigger joint inspector on right click
        if (_event.Button() == common::MouseEvent::RIGHT)
        {
          this->inspectName = vis->GetName();
          this->ShowContextMenu(this->inspectName);
          return true;
        }
        else if (_event.Button() == common::MouseEvent::LEFT)
        {
          // Not in multi-selection mode.
          if (!(QApplication::keyboardModifiers() & Qt::ControlModifier))
          {
            this->DeselectAll();
            this->SetSelected(vis, true);
          }
          // Multi-selection mode
          else
          {
            auto it = std::find(this->selectedJoints.begin(),
                this->selectedJoints.end(), vis);
            // Highlight and select clicked joint if not already selected
            // Otherwise deselect if already selected
            this->SetSelected(vis, it == this->selectedJoints.end());
          }
        }
      }
      else
        this->DeselectAll();
      return false;
    }
  }
  // In the process of creating a joint - still selecting parent/child
  else
  {
    if (_event.Button() == common::MouseEvent::LEFT)
    {
      if (this->hoverVis)
      {
        if (this->hoverVis->IsPlane())
        {
          QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
          camera->HandleMouseEvent(_event);
          return true;
        }

        // Pressed parent link
        if (!this->parentLinkVis)
        {
          this->NewParentLink(this->hoverVis);

          if (this->parentLinkVis)
          {
            this->jointCreationDialog->NewParent(
                this->parentLinkVis->GetName());
          }
        }
        // Pressed child link
        else if (this->parentLinkVis != this->hoverVis)
        {
          this->NewChildLink(this->hoverVis);

          if (this->childLinkVis)
          {
            this->jointCreationDialog->NewChild(
                this->childLinkVis->GetName());
          }
        }
      }
    }

    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
    camera->HandleMouseEvent(_event);
    return true;
  }
  return false;
}

/////////////////////////////////////////////////
JointData *JointMaker::CreateJointLine(const std::string &_name,
    rendering::VisualPtr _parent)
{
  // Joint visual
  rendering::VisualPtr jointVis(
      new rendering::Visual(_name, _parent->GetParent()));
  jointVis->Load();

  // Line
  rendering::DynamicLines *jointLine =
      jointVis->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  math::Vector3 origin = _parent->GetWorldPose().pos
      - _parent->GetParent()->GetWorldPose().pos;
  jointLine->AddPoint(origin);
  jointLine->AddPoint(origin + math::Vector3(0, 0, 0.1));
  jointVis->GetSceneNode()->setInheritScale(false);
  jointVis->GetSceneNode()->setInheritOrientation(false);

  // Get leaf name
  std::string jointVisName = jointVis->GetName();
  std::string leafName = jointVisName;
  size_t pIdx = jointVisName.rfind("::");
  if (pIdx != std::string::npos)
    leafName = jointVisName.substr(pIdx+2);

  // Joint data
  JointData *jointData = new JointData();
  jointData->dirty = false;
  jointData->name = leafName;
  jointData->visual = jointVis;
  jointData->SetParent(_parent);
  jointData->line = jointLine;
  jointData->SetType(this->jointType);
  jointData->line->setMaterial(this->jointMaterials[jointData->Type()]);

  return jointData;
}

/////////////////////////////////////////////////
JointData *JointMaker::CreateJoint(rendering::VisualPtr _parent,
    rendering::VisualPtr _child)
{
  std::stringstream ss;
  ss << _parent->GetName() << "_JOINT_" << this->jointCounter++;

  JointData *jointData = this->CreateJointLine(ss.str(), _parent);

  jointData->jointMsg.reset(new msgs::Joint);
  jointData->jointMsg->CopyFrom(JointMaker::SetupDefaultJointMsg(
      jointData->Type()));
  jointData->SetChild(_child);
  jointData->SetParent(_parent);

  // Inspector
  jointData->inspector = new JointInspector(this);
  jointData->inspector->setModal(false);
  connect(jointData->inspector, SIGNAL(Applied()), jointData, SLOT(OnApply()));
  connect(this, SIGNAL(EmitLinkRemoved(std::string)), jointData->inspector,
      SLOT(OnLinkRemoved(std::string)));
  connect(this, SIGNAL(EmitLinkInserted(std::string)), jointData->inspector,
      SLOT(OnLinkInserted(std::string)));

  MainWindow *mainWindow = gui::get_main_window();
  if (mainWindow)
  {
    connect(gui::get_main_window(), SIGNAL(Close()), jointData->inspector,
        SLOT(close()));
  }
  jointData->inspector->Update(jointData->jointMsg);

  return jointData;
}

/////////////////////////////////////////////////
msgs::Joint JointMaker::SetupDefaultJointMsg(JointType _type)
{
  std::string type = JointMaker::GetTypeAsString(_type);

  msgs::Joint msg;

 // msg.reset(new msgs::Joint);
  msgs::Set(msg.mutable_pose(),
      ignition::math::Pose3d::Zero);

  msg.set_type(msgs::ConvertJointType(type));

  unsigned int axisCount = JointMaker::GetJointAxisCount(_type);
  for (unsigned int i = 0; i < axisCount; ++i)
  {
    msgs::Axis *axisMsg;
    if (i == 0u)
    {
      axisMsg = msg.mutable_axis1();
    }
    else if (i == 1u)
    {
      axisMsg = msg.mutable_axis2();
    }
    else
    {
      gzerr << "Invalid axis index["
            << i
            << "]"
            << std::endl;
      continue;
    }
  //  if (_type == JointMaker::JOINT_GEARBOX)
      msgs::Set(axisMsg->mutable_xyz(), ignition::math::Vector3d::UnitZ);
  //   else
  //   {
  //     msgs::Set(axisMsg->mutable_xyz(),
  //         JointMaker::unitVectors[i%JointMaker::unitVectors.size()]);
  //   }

    axisMsg->set_use_parent_model_frame(false);
    axisMsg->set_limit_lower(-GZ_DBL_MAX);
    axisMsg->set_limit_upper(GZ_DBL_MAX);
    axisMsg->set_limit_effort(-1);
    axisMsg->set_limit_velocity(-1);
    axisMsg->set_damping(0);

    // Add angle field after we've checked that index i is valid
    msg.add_angle(0);
  }
  msg.set_limit_erp(0.2);
  msg.set_suspension_erp(0.2);

  return msg;
}

/////////////////////////////////////////////////
JointMaker::JointType JointMaker::ConvertJointType(const std::string &_type)
{
  for (auto iter : jointTypes)
    if (iter.second == _type)
      return iter.first;

  return JOINT_NONE;
}

/////////////////////////////////////////////////
std::string JointMaker::GetJointMaterial(const std::string &_type)
{
  auto it = jointMaterials.find(ConvertJointType(_type));
  if (it != jointMaterials.end())
    return it->second;
  else
    return "";
}

/////////////////////////////////////////////////
void JointMaker::AddJoint(const std::string &_type)
{
  this->AddJoint(this->ConvertJointType(_type));
}

/////////////////////////////////////////////////
void JointMaker::AddJoint(JointMaker::JointType _type)
{
  this->jointType = _type;
  // Start joint creation
  if (_type != JointMaker::JOINT_NONE)
  {
    this->jointCreationDialog->Open(_type);
    this->mouseMoveEnabled = true;
    this->creatingJoint = true;
  }
  // End joint creation
  else
  {
    this->creatingJoint = false;
    this->mouseMoveEnabled = false;
  }
}

/////////////////////////////////////////////////
void JointMaker::Stop()
{
  boost::recursive_mutex::scoped_lock lock(*this->updateMutex);
  // if (this->jointType != JointMaker::JOINT_NONE)
  {
    // Cancel joint creation
    this->RemoveJoint("");

    // Reset links
    if (this->parentLinkVis)
    {
      this->parentLinkVis->SetWorldPose(this->parentLinkOriginalPose);
      this->SetHighlighted(this->parentLinkVis, false);
    }
    if (this->childLinkVis)
    {
      this->childLinkVis->SetWorldPose(this->childLinkOriginalPose);
      this->SetHighlighted(this->childLinkVis, false);
    }

    if (this->jointCreationDialog)
      this->jointCreationDialog->close();
    //this->jointBeingCreated = NULL;
    this->creatingJoint = false;
    this->jointType == JointMaker::JOINT_NONE;
    // Notify ModelEditor to uncheck tool button
    this->JointAdded();
    // this->AddJoint(JointMaker::JOINT_NONE);
    this->mouseMoveEnabled = false;
    if (this->hoverVis)
      this->hoverVis->SetEmissive(common::Color(0, 0, 0));
    this->parentLinkVis.reset();
    this->childLinkVis.reset();
    this->hoverVis.reset();
  }
}

/////////////////////////////////////////////////
bool JointMaker::OnMouseMove(const common::MouseEvent &_event)
{
  if (!this->mouseMoveEnabled)
    return false;

  // Get the active camera.
  rendering::UserCameraPtr camera = gui::get_active_camera();

  if (_event.Dragging())
  {
    // this enables the joint maker to pan while connecting joints
    // QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
    // camera->HandleMouseEvent(_event);
    return false;
  }

  rendering::VisualPtr vis = camera->GetVisual(_event.Pos());

  // Highlight visual on hover
  if (vis)
  {
    if (this->hoverVis && this->hoverVis != this->parentLinkVis)
      this->hoverVis->SetEmissive(common::Color(0.0, 0.0, 0.0));

    // only highlight editor links by making sure it's not an item in the
    // gui tree widget or a joint hotspot.
    rendering::VisualPtr rootVis = vis->GetRootVisual();
    if (rootVis->IsPlane())
      this->hoverVis = vis->GetParent();
    else if (!gui::get_entity_id(rootVis->GetName()) &&
        vis->GetName().find("_UNIQUE_ID_") == std::string::npos)
    {
      this->hoverVis = vis->GetParent();
      // Parent hasn't been defined yet
      if (!this->parentLinkVis ||
         // Child hasn't been defined and hovered vis is different from parent
          (!this->childLinkVis &&
          this->parentLinkVis && this->hoverVis != this->parentLinkVis))
      {
        this->hoverVis->SetEmissive(common::Color(0.5, 0.5, 0.5));
      }
    }
  }

  // Case when a parent link is already selected and currently
  // extending the joint line to a child link
  if (this->parentLinkVis && !this->childLinkVis && this->hoverVis
      && this->jointBeingCreated && this->jointBeingCreated->line &&
      this->jointBeingCreated->Parent())
  {
    math::Vector3 parentPos;

    // Set end point to center of child link
    if (!this->hoverVis->IsPlane())
    {
      parentPos =  this->GetLinkWorldCentroid(
          this->jointBeingCreated->Parent()) -
          this->jointBeingCreated->line->GetPoint(0);
    }
    // Set end point to mouse plane intersection
    else
    {
      math::Vector3 pt;
      camera->GetWorldPointOnPlane(_event.Pos().X(), _event.Pos().Y(),
          math::Plane(math::Vector3(0, 0, 1)), pt);

      parentPos = this->GetLinkWorldCentroid(this->jointBeingCreated->Parent())
	  - this->jointBeingCreated->line->GetPoint(0) - pt;
    }

    this->jointBeingCreated->line->SetPoint(1,
	this->GetLinkWorldCentroid(this->hoverVis) - parentPos);
  }
  return true;
}

/////////////////////////////////////////////////
void JointMaker::OnOpenInspector()
{
  if (this->inspectName.empty())
    return;

  this->OpenInspector(this->inspectName);
  this->inspectName = "";
}

/////////////////////////////////////////////////
void JointMaker::OpenInspector(const std::string &_jointId)
{
  if (this->joints.find(_jointId) == this->joints.end())
  {
    gzerr << "Joint [" << _jointId << "] not found." << std::endl;
    return;
  }
  JointData *joint = this->joints[_jointId];
  joint->OpenInspector();
}

/////////////////////////////////////////////////
bool JointMaker::OnMouseDoubleClick(const common::MouseEvent &_event)
{
  rendering::UserCameraPtr camera = gui::get_active_camera();
  rendering::VisualPtr vis = camera->GetVisual(_event.Pos());

  if (vis)
  {
    if (this->joints.find(vis->GetName()) != this->joints.end())
    {
      this->OpenInspector(vis->GetName());
      return true;
    }
  }

  return false;
}

/////////////////////////////////////////////////
bool JointMaker::OnKeyPress(const common::KeyEvent &_event)
{
  if (_event.key == Qt::Key_Delete)
  {
    if (!this->selectedJoints.empty())
    {
      for (auto jointVis : this->selectedJoints)
      {
        this->RemoveJoint(jointVis->GetName());
      }
      this->DeselectAll();
      return true;
    }
  }

  return false;
}

/////////////////////////////////////////////////
void JointMaker::OnDelete()
{
  if (this->inspectName.empty())
    return;

  this->RemoveJoint(this->inspectName);
  this->inspectName = "";
}

/////////////////////////////////////////////////
std::string JointMaker::CreateHotSpot(JointData *_joint)
{
  if (!_joint)
    return "";

  rendering::UserCameraPtr camera = gui::get_active_camera();

  // Joint hotspot visual name is the JointId for easy access when clicking
  std::string jointId = _joint->visual->GetName() + "_UNIQUE_ID_";
  rendering::VisualPtr hotspotVisual(
      new rendering::Visual(jointId, _joint->visual, false));

  // create a cylinder to represent the joint
  hotspotVisual->InsertMesh("unit_cylinder");
  Ogre::MovableObject *hotspotObj =
      (Ogre::MovableObject*)(camera->GetScene()->GetManager()->createEntity(
      _joint->visual->GetName(), "unit_cylinder"));
  hotspotObj->getUserObjectBindings().setUserAny(Ogre::Any(jointId));
  hotspotVisual->GetSceneNode()->attachObject(hotspotObj);
  hotspotVisual->SetMaterial(this->jointMaterials[_joint->Type()]);
  hotspotVisual->SetTransparency(0.7);

  // create a handle at the parent end
  Ogre::BillboardSet *handleSet =
      camera->GetScene()->GetManager()->createBillboardSet(1);
  handleSet->setAutoUpdate(true);
  handleSet->setMaterialName("Gazebo/PointHandle");
  Ogre::MaterialPtr mat =
      Ogre::MaterialManager::getSingleton().getByName(
      this->jointMaterials[_joint->Type()]);
  Ogre::ColourValue color = mat->getTechnique(0)->getPass(0)->getDiffuse();
  color.a = 0.5;

  double linkSize = std::min(0.1,
      _joint->Parent()->GetBoundingBox().GetSize().GetLength()*0.05);
  linkSize = std::max(linkSize, 0.01);

  double dimension = linkSize;
  handleSet->setDefaultDimensions(dimension, dimension);
  Ogre::Billboard *parentHandle = handleSet->createBillboard(0, 0, 0);
  parentHandle->setColour(color);
  Ogre::SceneNode *handleNode =
      hotspotVisual->GetSceneNode()->createChildSceneNode();
  handleNode->attachObject(handleSet);
  handleNode->setInheritScale(false);
  handleNode->setInheritOrientation(false);
  _joint->handles = handleSet;

  hotspotVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI |
      GZ_VISIBILITY_SELECTABLE);
  hotspotVisual->GetSceneNode()->setInheritScale(false);

  this->joints[jointId] = _joint;
  camera->GetScene()->AddVisual(hotspotVisual);

  _joint->hotspot = hotspotVisual;
  _joint->inspector->SetJointId(_joint->hotspot->GetName());

  return jointId;
}

/////////////////////////////////////////////////
void JointMaker::Update()
{
  boost::recursive_mutex::scoped_lock lock(*this->updateMutex);

  // Update each joint
  for (auto it : this->joints)
  {
    JointData *joint = it.second;
    if (joint->hotspot)
    {
      if (joint->Child() && joint->Parent())
      {
        bool poseUpdate = false;
        if (joint->parentPose != joint->Parent()->GetWorldPose() ||
            joint->childPose != joint->Child()->GetWorldPose() ||
            joint->childScale != joint->Child()->GetScale())
         {
           joint->parentPose = joint->Parent()->GetWorldPose();
           joint->childPose = joint->Child()->GetWorldPose();
           joint->childScale = joint->Child()->GetScale();
           poseUpdate = true;

           // Highlight links connected to joint being created if they have
           // been moved to another position
           if (joint == this->jointBeingCreated)
           {
             // Parent
             if (joint->Parent() == this->parentLinkVis &&
                 joint->Parent()->GetWorldPose().Ign() !=
                 this->parentLinkOriginalPose)
             {
               this->SetHighlighted(this->parentLinkVis, true);
             }
             else
             {
               this->SetHighlighted(this->parentLinkVis, false);
             }

             // Child
             if (joint->Child() == this->childLinkVis &&
                 joint->Child()->GetWorldPose().Ign() !=
                 this->childLinkOriginalPose)
             {
               this->SetHighlighted(this->childLinkVis, true);
             }
             else
             {
               this->SetHighlighted(this->childLinkVis, false);
             }
           }
         }

        // Create / update joint visual
        if (joint->dirty || poseUpdate)
        {
          joint->Update();

          if (joint == this->jointBeingCreated)
          {
            if (!this->jointCreationDialog ||
                !this->jointCreationDialog->isVisible())
            {
              gzerr << "Joint creation dialog not found" << std::endl;
            }
            else
            {
              // Get poses as homogeneous transforms
              ignition::math::Matrix4d parent_world(
                  joint->Parent()->GetWorldPose().Ign());
              ignition::math::Matrix4d child_world(
                  joint->Child()->GetWorldPose().Ign());

              // w_T_c = w_T_p * p_T_c
              // w_T_p^-1 * w_T_c = p_T_c
              ignition::math::Matrix4d child_parent = parent_world.Inverse() *
                  child_world;

              this->jointCreationDialog->UpdateRelativePose(
                  child_parent.Pose());
            }
          }
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void JointMaker::AddScopedLinkName(const std::string &_name)
{
  this->scopedLinkedNames.push_back(_name);
}

/////////////////////////////////////////////////
std::string JointMaker::GetScopedLinkName(const std::string &_name)
{
  for (unsigned int i = 0; i < this->scopedLinkedNames.size(); ++i)
  {
    std::string scopedName = this->scopedLinkedNames[i];
    size_t idx = scopedName.find("::" + _name);
    if (idx != std::string::npos)
      return scopedName;
  }
  return _name;
}

/////////////////////////////////////////////////
void JointMaker::GenerateSDF()
{
  this->modelSDF.reset(new sdf::Element);
  sdf::initFile("model.sdf", this->modelSDF);
  this->modelSDF->ClearElements();

  // loop through all joints
  for (auto jointsIt : this->joints)
  {
    JointData *joint = jointsIt.second;
    sdf::ElementPtr jointElem = this->modelSDF->AddElement("joint");

    msgs::JointPtr jointMsg = joint->jointMsg;
    unsigned int axisCount = GetJointAxisCount(joint->Type());
    for (unsigned int i = axisCount; i < 2u; ++i)
    {
      if (i == 0u)
        jointMsg->clear_axis1();
      else if (i == 1u)
        jointMsg->clear_axis2();
    }
    jointElem = msgs::JointToSDF(*jointMsg.get(), jointElem);

    sdf::ElementPtr parentElem = jointElem->GetElement("parent");
    std::string parentName = joint->Parent()->GetName();
    size_t pIdx = parentName.find("::");
    if (pIdx != std::string::npos)
      parentName = parentName.substr(pIdx+2);
    parentElem->Set(parentName);

    sdf::ElementPtr childElem = jointElem->GetElement("child");
    std::string childName = joint->Child()->GetName();
    size_t cIdx = childName.find("::");
    if (cIdx != std::string::npos)
      childName = childName.substr(cIdx+2);
    childElem->Set(childName);
  }
}

/////////////////////////////////////////////////
sdf::ElementPtr JointMaker::GetSDF() const
{
  return this->modelSDF;
}

/////////////////////////////////////////////////
std::string JointMaker::GetTypeAsString(JointMaker::JointType _type)
{
  std::string jointTypeStr = "";

  auto iter = jointTypes.find(_type);
  if (iter != jointTypes.end())
    jointTypeStr = iter->second;

  return jointTypeStr;
}

/////////////////////////////////////////////////
unsigned int JointMaker::GetJointAxisCount(JointMaker::JointType _type)
{
  if (_type == JOINT_FIXED)
  {
    return 0;
  }
  else if (_type == JOINT_HINGE)
  {
    return 1;
  }
  else if (_type == JOINT_HINGE2)
  {
    return 2;
  }
  else if (_type == JOINT_SLIDER)
  {
    return 1;
  }
  else if (_type == JOINT_SCREW)
  {
    return 1;
  }
  else if (_type == JOINT_UNIVERSAL)
  {
    return 2;
  }
  else if (_type == JOINT_BALL)
  {
    return 0;
  }
  else if (_type == JOINT_GEARBOX)
  {
    return 2;
  }

  return 0;
}

/////////////////////////////////////////////////
JointMaker::JointType JointMaker::GetState() const
{
  return this->jointType;
}

/////////////////////////////////////////////////
math::Vector3 JointMaker::GetLinkWorldCentroid(
    const rendering::VisualPtr _visual)
{
  math::Vector3 centroid;
  int count = 0;
  for (unsigned int i = 0; i < _visual->GetChildCount(); ++i)
  {
    if (_visual->GetChild(i)->GetName().find("_JOINT_VISUAL_") ==
        std::string::npos)
    {
      centroid += _visual->GetChild(i)->GetWorldPose().pos;
      count++;
    }
  }
  centroid /= count;
  return centroid;
}

/////////////////////////////////////////////////
unsigned int JointMaker::GetJointCount()
{
  return this->joints.size();
}

/////////////////////////////////////////////////
void JointMaker::ShowContextMenu(const std::string &_name)
{
  auto it = this->joints.find(_name);
  if (it == this->joints.end())
    return;

  QMenu menu;
  if (this->inspectAct)
    menu.addAction(this->inspectAct);

  this->inspectName = _name;
  QAction *deleteAct = new QAction(tr("Delete"), this);
  connect(deleteAct, SIGNAL(triggered()), this, SLOT(OnDelete()));
  menu.addAction(deleteAct);

  menu.exec(QCursor::pos());
}

/////////////////////////////////////////////////
void JointMaker::OnSetSelectedEntity(const std::string &/*_name*/,
    const std::string &/*_mode*/)
{
  this->DeselectAll();
}

/////////////////////////////////////////////////
void JointMaker::OnSetSelectedJoint(const std::string &_name,
    const bool _selected)
{
  this->SetSelected(_name, _selected);
}

/////////////////////////////////////////////////
void JointMaker::SetSelected(const std::string &_name,
    const bool _selected)
{
  auto it = this->joints.find(_name);
  if (it == this->joints.end())
    return;

  this->SetSelected((*it).second->hotspot, _selected);
}

/////////////////////////////////////////////////
void JointMaker::SetSelected(rendering::VisualPtr _jointVis,
    const bool _selected)
{
  if (!_jointVis)
    return;

  _jointVis->SetHighlighted(_selected);
  auto it = std::find(this->selectedJoints.begin(),
      this->selectedJoints.end(), _jointVis);
  if (_selected)
  {
    if (it == this->selectedJoints.end())
    {
      this->selectedJoints.push_back(_jointVis);
      model::Events::setSelectedJoint(_jointVis->GetName(), _selected);
    }
  }
  else
  {
    if (it != this->selectedJoints.end())
    {
      this->selectedJoints.erase(it);
      model::Events::setSelectedJoint(_jointVis->GetName(), _selected);
    }
  }
}

/////////////////////////////////////////////////
void JointMaker::DeselectAll()
{
  while (!this->selectedJoints.empty())
  {
    rendering::VisualPtr vis = this->selectedJoints[0];
    vis->SetHighlighted(false);
    this->selectedJoints.erase(this->selectedJoints.begin());
    model::Events::setSelectedJoint(vis->GetName(), false);
  }
}

/////////////////////////////////////////////////
void JointMaker::CreateJointFromSDF(sdf::ElementPtr _jointElem,
    const std::string &_modelName)
{
  msgs::Joint jointMsg = msgs::JointFromSDF(_jointElem);

  // Parent
  std::string parentName = _modelName + "::" + jointMsg.parent();
  rendering::VisualPtr parentVis =
      gui::get_active_camera()->GetScene()->GetVisual(parentName);

  // Child
  std::string childName = _modelName + "::" + jointMsg.child();
  rendering::VisualPtr childVis =
      gui::get_active_camera()->GetScene()->GetVisual(childName);

  if (!parentVis || !childVis)
  {
    gzerr << "Unable to load joint. Joint child / parent not found"
        << std::endl;
    return;
  }

  JointData *joint = new JointData();
  joint->name = jointMsg.name();
  joint->SetParent(parentVis);
  joint->SetChild(childVis);
  joint->SetType(this->ConvertJointType(
      msgs::ConvertJointType(jointMsg.type())));
  std::string jointVisName = _modelName + "::" + joint->name;

  joint->jointMsg.reset(new msgs::Joint);
  joint->jointMsg->CopyFrom(jointMsg);
  joint->jointMsg->set_parent_id(joint->Parent()->GetId());
  joint->jointMsg->set_child_id(joint->Child()->GetId());

  // Inspector
  joint->inspector = new JointInspector(this);
  joint->inspector->Update(joint->jointMsg);
  joint->inspector->setModal(false);
  connect(joint->inspector, SIGNAL(Applied()), joint, SLOT(OnApply()));
  connect(this, SIGNAL(EmitLinkRemoved(std::string)), joint->inspector,
      SLOT(OnLinkRemoved(std::string)));
  connect(this, SIGNAL(EmitLinkInserted(std::string)), joint->inspector,
      SLOT(OnLinkInserted(std::string)));

  // Visuals
  rendering::VisualPtr jointVis(
      new rendering::Visual(jointVisName, parentVis->GetParent()));
  jointVis->Load();
  rendering::DynamicLines *jointLine =
      jointVis->CreateDynamicLine(rendering::RENDERING_LINE_LIST);

  math::Vector3 origin = parentVis->GetWorldPose().pos
      - parentVis->GetParent()->GetWorldPose().pos;
  jointLine->AddPoint(origin);
  jointLine->AddPoint(origin + math::Vector3(0, 0, 0.1));

  jointVis->GetSceneNode()->setInheritScale(false);
  jointVis->GetSceneNode()->setInheritOrientation(false);
  joint->visual = jointVis;
  joint->line = jointLine;
  joint->dirty = true;

  std::string jointId = this->CreateHotSpot(joint);

  // Notify other widgets
  if (!jointId.empty())
  {
    gui::model::Events::jointInserted(jointId, joint->name,
        jointTypes[joint->Type()], joint->Parent()->GetName(),
        joint->Child()->GetName());
  }
}

/////////////////////////////////////////////////
void JointMaker::OnLinkInserted(const std::string &_linkName)
{
  std::string leafName = _linkName;
  size_t idx = _linkName.rfind("::");
  if (idx != std::string::npos)
    leafName = _linkName.substr(idx+2);

  this->linkList[_linkName] = leafName;

  this->EmitLinkInserted(_linkName);
}

/////////////////////////////////////////////////
void JointMaker::OnLinkRemoved(const std::string &_linkName)
{
  auto it = this->linkList.find(_linkName);
  if (it != this->linkList.end())
  {
    this->linkList.erase(_linkName);
    this->EmitLinkRemoved(_linkName);
  }
}

/////////////////////////////////////////////////
std::map<std::string, std::string> JointMaker::LinkList() const
{
  return this->linkList;
}

/////////////////////////////////////////////////
void JointMaker::ShowJoints(bool _show)
{
  for (auto iter : this->joints)
  {
    rendering::VisualPtr vis = iter.second->hotspot;
    if (vis)
    {
      vis->SetVisible(_show);
      vis->SetHighlighted(false);
    }
    if (iter.second->jointVisual)
      iter.second->jointVisual->SetVisible(_show);
  }
  this->DeselectAll();
}

/////////////////////////////////////////////////
void JointMaker::NewParentLink(rendering::VisualPtr _parentLink)
{
  if (!_parentLink)
  {
    gzerr << "Parent link is null" << std::endl;
    return;
  }

  if (this->hoverVis)
  {
    this->hoverVis->SetEmissive(common::Color(0, 0, 0));
    this->hoverVis.reset();
  }

  // Choosing parent for the first time
  if (!this->jointBeingCreated)
  {
    this->jointBeingCreated = this->CreateJointLine("JOINT_LINE",
        _parentLink);
  }
  // Update parent of joint being created
  else
  {
    // Reset previous parent
    this->parentLinkVis->SetWorldPose(this->parentLinkOriginalPose);
    this->SetHighlighted(this->parentLinkVis, false);

    this->jointBeingCreated->SetParent(_parentLink);

    // If joint already has parent and child
    if (this->jointBeingCreated->hotspot)
      this->jointBeingCreated->Update();
    // If joint has only parent
    else
      this->jointBeingCreated->UpdateJointLine();
  }

  this->parentLinkVis = _parentLink;
  this->parentLinkOriginalPose = _parentLink->GetWorldPose().Ign();
}

/////////////////////////////////////////////////
void JointMaker::NewChildLink(rendering::VisualPtr _childLink)
{
  if (!_childLink)
  {
    gzerr << "Child link can't be null" << std::endl;
    return;
  }

  if (this->hoverVis)
    this->hoverVis->SetEmissive(common::Color(0, 0, 0));

  if (this->jointBeingCreated)
  {
    boost::recursive_mutex::scoped_lock lock(*this->updateMutex);

    // Choosing child for the first time
    if (!this->childLinkVis)
    {
      this->RemoveJoint("");

      // Create new joint with parent and child
      this->jointBeingCreated = this->CreateJoint(this->parentLinkVis,
          _childLink);

      // Create hotspot visual
      this->CreateHotSpot(this->jointBeingCreated);
    }
    // Update child
    else
    {
      this->jointBeingCreated->SetChild(_childLink);
      this->jointBeingCreated->dirty = true;
      this->jointBeingCreated->Update();
      _childLink->AttachVisual(this->jointBeingCreated->jointVisual);

      // Reset previous child
      this->childLinkVis->SetWorldPose(this->childLinkOriginalPose);
      this->SetHighlighted(this->childLinkVis, false);
    }
  }

  this->childLinkVis = _childLink;
  this->childLinkOriginalPose = _childLink->GetWorldPose().Ign();

  this->mouseMoveEnabled = false;
}

/////////////////////////////////////////////////
void JointMaker::NewType(const int _typeInt)
{
  this->jointType = static_cast<JointMaker::JointType>(_typeInt);

  if (this->jointBeingCreated)
  {
    this->jointBeingCreated->SetType(this->jointType);
    this->jointBeingCreated->Update();
  }
}

/////////////////////////////////////////////////
void JointMaker::NewParentLink(const std::string &_name)
{
  // Get scoped name
  std::string scopedName;
  for (auto link : this->linkList)
  {
    if (link.second == _name || link.first == _name)
    {
      scopedName = link.first;
      break;
    }
  }

  if (scopedName.empty())
    return;

  // Get visual
  rendering::ScenePtr scene = rendering::get_scene();

  if (!scene)
    return;

  rendering::VisualPtr vis = scene->GetVisual(scopedName);

  if (vis)
    this->NewParentLink(vis);
}

/////////////////////////////////////////////////
void JointMaker::NewChildLink(const std::string &_name)
{
  // Get scoped name
  std::string scopedName;
  for (auto link : this->linkList)
  {
    if (link.second == _name || link.first == _name)
    {
      scopedName = link.first;
      break;
    }
  }

  if (scopedName.empty())
    return;

  // Get visual
  rendering::ScenePtr scene = rendering::get_scene();

  if (!scene)
    return;

  rendering::VisualPtr vis = scene->GetVisual(scopedName);

  if (vis)
    this->NewChildLink(vis);
}

/////////////////////////////////////////////////
void JointMaker::NewPose(const ignition::math::Pose3d &_pose,
    bool reset)
{
  if (this->parentLinkVis && this->childLinkVis)
  {
    ignition::math::Pose3d newChildPose;

    if (reset)
    {
      newChildPose = this->childLinkOriginalPose;
      this->parentLinkVis->SetWorldPose(this->parentLinkOriginalPose);
    }
    else
    {
      // Get poses as homogeneous transforms
      ignition::math::Matrix4d parent_world(
          parentLinkVis->GetWorldPose().Ign());
      ignition::math::Matrix4d child_parent(_pose);

      // w_T_c = w_T_p * p_T_c
      ignition::math::Matrix4d child_world =
          parent_world * child_parent;

      newChildPose = child_world.Pose();
    }

    this->childLinkVis->SetWorldPose(newChildPose);
  }
}

/////////////////////////////////////////////////
void JointMaker::AlignLinks(const bool _childToParent,
    const std::string &_axis, const std::string &_config)
{
  if (!this->parentLinkVis || !this->childLinkVis)
    return;

  std::vector<rendering::VisualPtr> links;
  links.push_back(this->parentLinkVis);
  links.push_back(this->childLinkVis);

  std::string target = _childToParent ? "first" : "last";

  ModelAlign::Instance()->AlignVisuals(links, _axis, _config,
      target, true);
}

/////////////////////////////////////////////////
void JointMaker::CreationComplete()
{
  gui::model::Events::modelChanged();
  this->jointType = JointMaker::JOINT_NONE;

  // Notify schematic view and palette list
  if (this->jointBeingCreated &&
      this->jointBeingCreated->hotspot &&
      this->jointBeingCreated->Child() &&
      this->jointBeingCreated->Parent())
  {
    gui::model::Events::jointInserted(
        this->jointBeingCreated->hotspot->GetName(),
        this->jointBeingCreated->name,
        this->jointTypes[this->jointBeingCreated->Type()],
        this->jointBeingCreated->Parent()->GetName(),
        this->jointBeingCreated->Child()->GetName());
  }

  // Reset visuals
  if (this->parentLinkVis)
  {
    this->SetHighlighted(this->parentLinkVis, false);
    this->parentLinkVis = NULL;
  }
  if (this->childLinkVis)
  {
    this->SetHighlighted(this->childLinkVis, false);
    this->childLinkVis = NULL;
  }
  this->jointBeingCreated = NULL;
  this->creatingJoint = false;

  // Notify ModelEditor to uncheck tool button
  this->JointAdded();

  this->Stop();
}

/////////////////////////////////////////////////
void JointMaker::SetHighlighted(rendering::VisualPtr _vis, bool _highlight)
{
  // Taken from ModelAlign
  if (_vis->GetChildCount() != 0)
  {
    for (unsigned int j = 0; j < _vis->GetChildCount(); ++j)
    {
      this->SetHighlighted(_vis->GetChild(j), _highlight);
    }
  }
  else
  {
    // Highlighting increases transparency for opaque visuals (0 < t < 0.3) and
    // decreases transparency for semi-transparent visuals (0.3 < t < 1).
    // A visual will never become fully transparent (t = 1) when highlighted.
    if (_highlight)
    {
      _vis->SetEmissive(common::Color(0, 0, 1, 1));
      //_vis->SetTransparency((1.0 - _vis->GetTransparency()) * 0.5);
    }
    // The inverse operation restores the original transparency value.
    else
    {
      _vis->SetEmissive(common::Color(0, 0, 0, 1));
      //_vis->SetTransparency(std::abs(_vis->GetTransparency()*2.0-1.0));
    }
  }
}

