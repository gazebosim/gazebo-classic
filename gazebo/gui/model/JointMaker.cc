/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
#include "gazebo/gui/model/MEUserCmdManager.hh"
#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/JointMaker.hh"
#include "gazebo/gui/model/JointMakerPrivate.hh"

using namespace gazebo;
using namespace gui;

std::map<JointMaker::JointType, std::string> JointMaker::jointTypes;
std::map<JointMaker::JointType, std::string> JointMaker::jointMaterials;
std::vector<ignition::math::Vector3d> JointMaker::unitVectors;

/////////////////////////////////////////////////
JointMaker::JointMaker() : dataPtr(new JointMakerPrivate())
{
  this->unitVectors.push_back(ignition::math::Vector3d::UnitX);
  this->unitVectors.push_back(ignition::math::Vector3d::UnitY);
  this->unitVectors.push_back(ignition::math::Vector3d::UnitZ);

  this->dataPtr->newJoint = nullptr;
  this->dataPtr->modelSDF.reset();
  this->dataPtr->jointType = JointMaker::JOINT_NONE;
  this->dataPtr->jointCounter = 0;

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

  this->dataPtr->jointCreationDialog = nullptr;

  this->dataPtr->connections.push_back(
      event::Events::ConnectPreRender(
      std::bind(&JointMaker::Update, this)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectOpenJointInspector(
      std::bind(&JointMaker::OpenInspector, this, std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectShowJointContextMenu(
      std::bind(&JointMaker::ShowContextMenu, this, std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectSetSelectedJoint(
      std::bind(&JointMaker::OnSetSelectedJoint, this, std::placeholders::_1,
      std::placeholders::_2)));

  this->dataPtr->connections.push_back(
      event::Events::ConnectSetSelectedEntity(
      std::bind(&JointMaker::OnSetSelectedEntity, this, std::placeholders::_1,
      std::placeholders::_2)));

  this->dataPtr->inspectAct = new QAction(tr("Open Joint Inspector"), this);
  this->connect(this->dataPtr->inspectAct, SIGNAL(triggered()), this,
      SLOT(OnOpenInspector()));

  // Gazebo event connections
  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectLinkInserted(
      std::bind(&JointMaker::OnLinkInserted, this, std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectLinkRemoved(
      std::bind(&JointMaker::OnLinkRemoved, this, std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectRequestJointInsertion(
      std::bind(&JointMaker::CreateJointFromSDF, this,
      std::placeholders::_1, std::placeholders::_2)));

  this->dataPtr->connections.push_back(
      gui::model::Events::ConnectRequestJointRemoval(
      std::bind(&JointMaker::RemoveJoint, this,
      std::placeholders::_1)));
}

/////////////////////////////////////////////////
JointMaker::~JointMaker()
{
  if (this->dataPtr->newJoint)
  {
    delete this->dataPtr->newJoint;
    this->dataPtr->newJoint = nullptr;
  }

  {
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->updateMutex);
    while (this->dataPtr->joints.size() > 0)
    {
      std::string jointName = this->dataPtr->joints.begin()->first;
      this->RemoveJoint(jointName);
    }
    this->dataPtr->joints.clear();
  }

  if (this->dataPtr->jointCreationDialog)
    delete this->dataPtr->jointCreationDialog;
}

/////////////////////////////////////////////////
void JointMaker::Reset()
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->updateMutex);
  if (this->dataPtr->newJoint)
  {
    delete this->dataPtr->newJoint;
    this->dataPtr->newJoint = nullptr;
  }

  this->dataPtr->jointType = JointMaker::JOINT_NONE;
  this->dataPtr->hoverVis.reset();
  this->dataPtr->inspectName = "";
  this->dataPtr->selectedJoints.clear();

  this->dataPtr->scopedLinkedNames.clear();

  while (!this->dataPtr->joints.empty())
  {
    std::string jointId = this->dataPtr->joints.begin()->first;
    this->RemoveJoint(jointId);
  }
  this->dataPtr->joints.clear();
}

/////////////////////////////////////////////////
void JointMaker::EnableEventHandlers()
{
  MouseEventHandler::Instance()->AddDoubleClickFilter("model_joint",
      std::bind(&JointMaker::OnMouseDoubleClick, this, std::placeholders::_1));

  MouseEventHandler::Instance()->AddReleaseFilter("model_joint",
      std::bind(&JointMaker::OnMouseRelease, this, std::placeholders::_1));

  MouseEventHandler::Instance()->AddPressFilter("model_joint",
      std::bind(&JointMaker::OnMousePress, this, std::placeholders::_1));

  MouseEventHandler::Instance()->AddMoveFilter("model_joint",
      std::bind(&JointMaker::OnMouseMove, this, std::placeholders::_1));

  KeyEventHandler::Instance()->AddPressFilter("model_joint",
      std::bind(&JointMaker::OnKeyPress, this, std::placeholders::_1));
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
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->updateMutex);

  std::string jointId = _jointId;
  JointData *joint = nullptr;

  auto jointIt = this->dataPtr->joints.find(_jointId);

  // Existing joint
  if (jointIt != this->dataPtr->joints.end())
  {
    joint = jointIt->second;
  }
  // Joint being created
  else if (this->dataPtr->newJoint)
  {
    joint = this->dataPtr->newJoint;
    // Already has hotspot
    if (joint->hotspot)
      jointId = joint->hotspot->GetName();
    // Still only line
    else
      jointId = "";
  }

  if (!joint)
    return;

  if (jointId != "")
  {
    this->dataPtr->joints.erase(jointId);
    gui::model::Events::jointRemoved(jointId);
  }

  auto scene = rendering::get_scene();
  if (!scene)
    return;

  if (joint->handles)
  {
    scene->OgreSceneManager()->destroyBillboardSet(joint->handles);
    joint->handles = nullptr;
  }

  if (joint->hotspot)
  {
    auto camera = gui::get_active_camera();
    if (camera)
    {
      // FIXME: Ogre object destruction should be handled in rendering::Visual
      camera->GetScene()->OgreSceneManager()->destroyEntity(
          joint->visual->GetName());
    }

    scene->RemoveVisual(joint->hotspot);
    joint->hotspot->Fini();
  }

  if (joint->visual)
  {
    joint->visual->Fini();
  }

  if (joint->jointVisual)
  {
    joint->jointVisual->Fini();
  }

  if (joint->inspector)
  {
    joint->inspector->hide();
    delete joint->inspector;
    joint->inspector = nullptr;
  }

  this->dataPtr->newJoint = nullptr;
  joint->hotspot.reset();
  joint->visual.reset();
  joint->jointVisual.reset();
  joint->parent.reset();
  joint->child.reset();
  delete joint->inspector;
  delete joint;
  gui::model::Events::modelChanged();
}

/////////////////////////////////////////////////
void JointMaker::RemoveJointsByLink(const std::string &_linkName)
{
  std::vector<std::string> toDelete;
  for (auto it : this->dataPtr->joints)
  {
    JointData *joint = it.second;

    if (joint->child->GetName() == _linkName ||
        joint->parent->GetName() == _linkName)
    {
      toDelete.push_back(it.first);
    }
  }

  for (unsigned i = 0; i < toDelete.size(); ++i)
    this->RemoveJointByUser(toDelete[i]);

  toDelete.clear();
}

/////////////////////////////////////////////////
std::vector<JointData *> JointMaker::JointDataByLink(
    const std::string &_linkName) const
{
  std::vector<JointData *> linkJoints;
  for (auto jointIt : this->dataPtr->joints)
  {
    JointData *jointData = jointIt.second;

    if (jointData->child->GetName() == _linkName ||
        jointData->parent->GetName() == _linkName)
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

  if (this->dataPtr->jointType != JointMaker::JOINT_NONE)
    return false;

  // intercept mouse press events when user clicks on the joint hotspot visual
  rendering::VisualPtr vis = camera->GetVisual(_event.Pos());
  if (vis)
  {
    if (this->dataPtr->joints.find(vis->GetName()) !=
        this->dataPtr->joints.end())
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

  // Not in the process of selecting joint links with mouse
  // Handle joint selection
  if (this->dataPtr->jointType == JointMaker::JOINT_NONE ||
      (this->dataPtr->newJoint && this->dataPtr->newJoint->parent &&
       this->dataPtr->newJoint->child))
  {
    rendering::VisualPtr vis = camera->GetVisual(_event.Pos());
    if (vis)
    {
      if (this->dataPtr->joints.find(vis->GetName()) !=
          this->dataPtr->joints.end())
      {
        // trigger joint inspector on right click
        if (_event.Button() == common::MouseEvent::RIGHT)
        {
          this->dataPtr->inspectName = vis->GetName();
          this->ShowContextMenu(this->dataPtr->inspectName);
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
            auto it = std::find(this->dataPtr->selectedJoints.begin(),
                this->dataPtr->selectedJoints.end(), vis);
            // Highlight and select clicked joint if not already selected
            // Otherwise deselect if already selected
            this->SetSelected(vis, it == this->dataPtr->selectedJoints.end());
          }
        }
      }
      else
        this->DeselectAll();
      return false;
    }
  }
  // Still selecting parent/child during new joint creation
  else
  {
    if (_event.Button() == common::MouseEvent::LEFT)
    {
      if (this->dataPtr->hoverVis)
      {
        if (this->dataPtr->hoverVis->IsPlane())
        {
          QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
          camera->HandleMouseEvent(_event);
          return true;
        }

        // Pressed parent link
        if (!this->dataPtr->newJoint)
        {
          if (!this->SetParentLink(this->dataPtr->hoverVis))
            return false;

          if (this->dataPtr->jointCreationDialog)
          {
            this->dataPtr->jointCreationDialog->SetParent(
                this->dataPtr->newJoint->parent->GetName());
          }
        }
        // Pressed child link
        else if (this->dataPtr->newJoint &&
            this->dataPtr->newJoint->parent != this->dataPtr->hoverVis)
        {
          if (!this->SetChildLink(this->dataPtr->hoverVis))
            return false;

          if (this->dataPtr->jointCreationDialog)
          {
            this->dataPtr->jointCreationDialog->SetChild(
                this->dataPtr->newJoint->child->GetName());
          }
        }

        if (this->dataPtr->hoverVis)
        {
          this->dataPtr->hoverVis->SetEmissive(common::Color(0, 0, 0));
          this->dataPtr->hoverVis.reset();
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
    const rendering::VisualPtr &_parent)
{
  if (this->dataPtr->jointType == JOINT_NONE)
  {
    gzwarn << "Can't create joint line of type JOINT_NONE" << std::endl;
    return nullptr;
  }

  rendering::VisualPtr jointVis(
      new rendering::Visual(_name, _parent->GetRootVisual(), false));
  jointVis->Load();
  rendering::DynamicLines *jointLine =
      jointVis->CreateDynamicLine(rendering::RENDERING_LINE_LIST);

  auto parentOriginInRootFrame = _parent->GetWorldPose().Ign().Pos()
      - _parent->GetRootVisual()->GetWorldPose().Ign().Pos();
  jointLine->AddPoint(parentOriginInRootFrame);
  jointLine->AddPoint(
      parentOriginInRootFrame + ignition::math::Vector3d(0, 0, 0.1));

  jointVis->GetSceneNode()->setInheritScale(false);
  jointVis->GetSceneNode()->setInheritOrientation(false);

  std::string jointVisName = jointVis->GetName();
  std::string leafName = jointVisName;
  size_t pIdx = jointVisName.rfind("::");
  if (pIdx != std::string::npos)
    leafName = jointVisName.substr(pIdx+2);

  JointData *jointData = new JointData();
  jointData->dirty = false;
  jointData->name = leafName;
  jointData->visual = jointVis;
  jointData->parent = _parent;
  jointData->line = jointLine;
  jointData->type = this->dataPtr->jointType;
  jointData->line->setMaterial(this->jointMaterials[jointData->type]);

  return jointData;
}

/////////////////////////////////////////////////
JointData *JointMaker::CreateJoint(const rendering::VisualPtr &_parent,
    const rendering::VisualPtr &_child)
{
  std::stringstream ss;
  ss << _parent->GetName() << "_JOINT_" << this->dataPtr->jointCounter++;

  JointData *jointData = this->CreateJointLine(ss.str(), _parent);
  jointData->child = _child;

  // Inspector
  jointData->inspector = new JointInspector(this);
  jointData->inspector->setModal(false);
  connect(jointData->inspector, SIGNAL(Applied()), jointData, SLOT(OnApply()));

  MainWindow *mainWindow = gui::get_main_window();
  if (mainWindow)
  {
    connect(gui::get_main_window(), SIGNAL(Close()), jointData->inspector,
        SLOT(close()));
  }

  // setup the joint msg
  jointData->UpdateMsg();

  jointData->inspector->Update(jointData->jointMsg);
  return jointData;
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
std::string JointMaker::JointMaterial(const std::string &_type)
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
void JointMaker::AddJoint(const JointMaker::JointType _type)
{
  this->dataPtr->jointType = _type;
  // Start joint creation
  if (_type != JointMaker::JOINT_NONE)
  {
    if (!this->dataPtr->jointCreationDialog)
    {
      auto mainWindow = gui::get_main_window();
      this->dataPtr->jointCreationDialog =
          new JointCreationDialog(this, mainWindow);
    }
    this->dataPtr->jointCreationDialog->Open(_type);
  }
  // End joint creation
  else
  {
    // signal the end of a joint action.
    emit JointAdded();
  }
}

/////////////////////////////////////////////////
void JointMaker::Stop()
{
  // Reset links
  if (this->dataPtr->newJoint)
  {
    if (this->dataPtr->newJoint->parent)
    {
      this->dataPtr->newJoint->parent->SetWorldPose(
          this->dataPtr->parentLinkOriginalPose);
      this->SetVisualMoved(this->dataPtr->newJoint->parent, false);
    }
    if (this->dataPtr->newJoint->child)
    {
      this->dataPtr->newJoint->child->SetWorldPose(
          this->dataPtr->childLinkOriginalPose);
      this->SetVisualMoved(this->dataPtr->newJoint->child, false);
    }
  }

  this->RemoveJoint("");

  this->AddJoint(JointMaker::JOINT_NONE);
  if (this->dataPtr->hoverVis)
    this->dataPtr->hoverVis->SetEmissive(common::Color(0, 0, 0));
  this->dataPtr->hoverVis.reset();

  if (this->dataPtr->jointCreationDialog &&
      this->dataPtr->jointCreationDialog->isVisible())
  {
    this->dataPtr->jointCreationDialog->reject();
  }
}

/////////////////////////////////////////////////
bool JointMaker::OnMouseMove(const common::MouseEvent &_event)
{
  // Only handle mouse move events during joint creation
  if (this->dataPtr->jointType == JointMaker::JOINT_NONE ||
      (this->dataPtr->newJoint && this->dataPtr->newJoint->child))
  {
    return false;
  }
  QApplication::setOverrideCursor(Qt::CrossCursor);

  // Get the active camera and scene.
  rendering::UserCameraPtr camera = gui::get_active_camera();

  if (_event.Dragging())
  {
    // this enables the joint maker to pan while connecting joints
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
    camera->HandleMouseEvent(_event);
    return true;
  }

  rendering::VisualPtr vis = camera->GetVisual(_event.Pos());

  // Highlight visual on hover
  if (vis)
  {
    if (this->dataPtr->hoverVis)
      this->dataPtr->hoverVis->SetEmissive(common::Color(0.0, 0.0, 0.0));

    // only highlight editor links by making sure it's not an item in the
    // gui tree widget or a joint hotspot.
    rendering::VisualPtr rootVis = vis->GetRootVisual();
    if (rootVis->IsPlane())
      this->dataPtr->hoverVis = vis->GetParent();
    else if (!gui::get_entity_id(rootVis->GetName()) &&
        vis->GetName().find("_UNIQUE_ID_") == std::string::npos)
    {
      this->dataPtr->hoverVis = vis->GetParent();
      if (!this->dataPtr->newJoint || (this->dataPtr->newJoint->parent &&
           this->dataPtr->hoverVis != this->dataPtr->newJoint->parent))
      {
        this->dataPtr->hoverVis->SetEmissive(common::Color(0.5, 0.5, 0.5));
      }
    }
  }

  // Case when a parent link is already selected and currently
  // extending the joint line to a child link
  if (this->dataPtr->newJoint && this->dataPtr->newJoint->parent &&
      this->dataPtr->hoverVis && this->dataPtr->newJoint->line)
  {
    ignition::math::Vector3d posWorld;

    // Set end point to origin of child link
    if (!this->dataPtr->hoverVis->IsPlane())
    {
      posWorld = this->dataPtr->hoverVis->GetWorldPose().Ign().Pos();
    }
    // Set end point to mouse plane intersection
    else
    {
      camera->WorldPointOnPlane(_event.Pos().X(), _event.Pos().Y(),
          ignition::math::Planed(ignition::math::Vector3d(0, 0, 1)), posWorld);
    }

    // Set point in root frame
    this->dataPtr->newJoint->line->SetPoint(1, posWorld -
        this->dataPtr->newJoint->parent->GetRootVisual()->
        GetWorldPose().Ign().Pos());
  }
  return true;
}

/////////////////////////////////////////////////
void JointMaker::OnOpenInspector()
{
  if (this->dataPtr->inspectName.empty())
    return;

  this->OpenInspector(this->dataPtr->inspectName);
  this->dataPtr->inspectName = "";
}

/////////////////////////////////////////////////
void JointMaker::OpenInspector(const std::string &_jointId)
{
  JointData *joint = this->dataPtr->joints[_jointId];
  if (!joint)
  {
    gzerr << "Joint [" << _jointId << "] not found." << std::endl;
    return;
  }
  joint->OpenInspector();
}

/////////////////////////////////////////////////
bool JointMaker::OnMouseDoubleClick(const common::MouseEvent &_event)
{
  rendering::UserCameraPtr camera = gui::get_active_camera();
  rendering::VisualPtr vis = camera->GetVisual(_event.Pos());

  if (vis)
  {
    if (this->dataPtr->joints.find(vis->GetName()) !=
        this->dataPtr->joints.end())
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
    if (!this->dataPtr->selectedJoints.empty())
    {
      for (auto jointVis : this->dataPtr->selectedJoints)
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
  if (this->dataPtr->inspectName.empty())
    return;

  this->RemoveJointByUser(this->dataPtr->inspectName);
  this->dataPtr->inspectName = "";
}

/////////////////////////////////////////////////
void JointMaker::RemoveJointByUser(const std::string &_name)
{
  // Register user cmd
  if (this->dataPtr->userCmdManager)
  {
    auto jointIt = this->dataPtr->joints.find(_name);
    if (jointIt != this->dataPtr->joints.end())
    {
      auto joint = jointIt->second;
      auto cmd = this->dataPtr->userCmdManager->NewCmd(
          "Deleted [" + joint->name + "]", MEUserCmd::DELETING_JOINT);
      cmd->SetSDF(msgs::JointToSDF(*joint->jointMsg));
      cmd->SetScopedName(joint->visual->GetName());
      cmd->SetJointId(joint->hotspot->GetName());
    }
  }

  this->RemoveJoint(_name);
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
  hotspotVisual->Load();

  // create a cylinder to represent the joint
  hotspotVisual->InsertMesh("unit_cylinder");
  // FIXME: Ogre object creation should be handled in rendering::Visual
  Ogre::MovableObject *hotspotObj =
      (Ogre::MovableObject*)(
      camera->GetScene()->OgreSceneManager()->createEntity(
      _joint->visual->GetName(), "unit_cylinder"));
  hotspotObj->getUserObjectBindings().setUserAny(Ogre::Any(jointId));
  hotspotVisual->GetSceneNode()->attachObject(hotspotObj);
  hotspotVisual->SetMaterial(this->jointMaterials[_joint->type]);
  hotspotVisual->SetTransparency(0.7);

  // create a handle at the parent end
  Ogre::BillboardSet *handleSet =
      camera->GetScene()->OgreSceneManager()->createBillboardSet(1);
  handleSet->setAutoUpdate(true);
  handleSet->setMaterialName("Gazebo/PointHandle");
  Ogre::MaterialPtr mat =
      Ogre::MaterialManager::getSingleton().getByName(
      this->jointMaterials[_joint->type]);
  Ogre::ColourValue color = mat->getTechnique(0)->getPass(0)->getDiffuse();
  color.a = 0.5;

  double linkSize = std::min(0.1,
      _joint->parent->GetBoundingBox().GetSize().GetLength()*0.05);
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

  this->dataPtr->joints[jointId] = _joint;

  _joint->hotspot = hotspotVisual;
  _joint->inspector->SetJointId(_joint->hotspot->GetName());

  _joint->dirty = true;

  return jointId;
}

/////////////////////////////////////////////////
void JointMaker::Update()
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->updateMutex);
  // Update each joint
  for (auto it : this->dataPtr->joints)
  {
    JointData *joint = it.second;
    if (joint->hotspot)
    {
      if (joint->child && joint->parent)
      {
        bool poseUpdate = false;
        if (joint->parentPose != joint->parent->GetWorldPose().Ign() ||
            joint->childPose != joint->child->GetWorldPose().Ign() ||
            joint->childScale != joint->child->GetScale().Ign())
         {
           joint->parentPose = joint->parent->GetWorldPose().Ign();
           joint->childPose = joint->child->GetWorldPose().Ign();
           joint->childScale = joint->child->GetScale().Ign();
           poseUpdate = true;

           // Highlight links connected to joint being created if they have
           // been moved to another position
           if (joint == this->dataPtr->newJoint)
           {
             // Parent
             this->SetVisualMoved(joint->parent,
                 joint->parent->GetWorldPose().Ign() !=
                 this->dataPtr->parentLinkOriginalPose);

             // Child
             this->SetVisualMoved(joint->child,
                 joint->child->GetWorldPose().Ign() !=
                 this->dataPtr->childLinkOriginalPose);
           }
         }

        // Create / update joint visual
        if (joint->dirty || poseUpdate)
        {
          joint->Update();

          if (joint == this->dataPtr->newJoint &&
              this->dataPtr->newJoint->parent && this->dataPtr->newJoint->child
              && this->dataPtr->jointCreationDialog &&
              this->dataPtr->jointCreationDialog->isVisible())
          {
            // Get poses as homogeneous transforms
            ignition::math::Matrix4d parentWorld(
                this->dataPtr->newJoint->parent->GetWorldPose().Ign());
            ignition::math::Matrix4d childWorld(
                this->dataPtr->newJoint->child->GetWorldPose().Ign());

            // w_T_c = w_T_p * p_T_c
            // w_T_p^-1 * w_T_c = p_T_c
            ignition::math::Matrix4d childParent = parentWorld.Inverse() *
                childWorld;

            this->dataPtr->jointCreationDialog->UpdateRelativePose(
                childParent.Pose());
          }
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void JointMaker::AddScopedLinkName(const std::string &_name)
{
  this->dataPtr->scopedLinkedNames.push_back(_name);
}

/////////////////////////////////////////////////
std::string JointMaker::ScopedLinkName(const std::string &_name)
{
  for (unsigned int i = 0; i < this->dataPtr->scopedLinkedNames.size(); ++i)
  {
    std::string scopedName = this->dataPtr->scopedLinkedNames[i];
    size_t idx = scopedName.find("::" + _name);
    if (idx != std::string::npos)
      return scopedName;
  }
  return _name;
}

/////////////////////////////////////////////////
void JointMaker::GenerateSDF()
{
  this->dataPtr->modelSDF.reset(new sdf::Element);
  sdf::initFile("model.sdf", this->dataPtr->modelSDF);
  this->dataPtr->modelSDF->ClearElements();

  // update joint visuals as the model pose may have changed when
  // generating model sdf
  for (auto jointsIt : this->dataPtr->joints)
  {
    JointData *joint = jointsIt.second;
    joint->dirty = true;
    this->Update();
  }

  // loop through all joints
  for (auto jointsIt : this->dataPtr->joints)
  {
    JointData *joint = jointsIt.second;
    sdf::ElementPtr jointElem = this->dataPtr->modelSDF->AddElement("joint");

    msgs::JointPtr jointMsg = joint->jointMsg;
    unsigned int axisCount = this->JointAxisCount(joint->type);
    for (unsigned int i = axisCount; i < 2u; ++i)
    {
      if (i == 0u)
        jointMsg->clear_axis1();
      else if (i == 1u)
        jointMsg->clear_axis2();
    }
    jointElem = msgs::JointToSDF(*jointMsg.get(), jointElem);

    sdf::ElementPtr parentElem = jointElem->GetElement("parent");
    std::string parentName = joint->parent->GetName();
    size_t pIdx = parentName.find("::");
    if (pIdx != std::string::npos)
      parentName = parentName.substr(pIdx+2);
    parentElem->Set(parentName);

    sdf::ElementPtr childElem = jointElem->GetElement("child");
    std::string childName = joint->child->GetName();
    size_t cIdx = childName.find("::");
    if (cIdx != std::string::npos)
      childName = childName.substr(cIdx+2);
    childElem->Set(childName);
  }
}

/////////////////////////////////////////////////
sdf::ElementPtr JointMaker::SDF() const
{
  return this->dataPtr->modelSDF;
}

/////////////////////////////////////////////////
std::string JointMaker::TypeAsString(const JointMaker::JointType _type)
{
  std::string jointTypeStr = "";

  auto iter = jointTypes.find(_type);
  if (iter != jointTypes.end())
    jointTypeStr = iter->second;

  return jointTypeStr;
}

/////////////////////////////////////////////////
unsigned int JointMaker::JointAxisCount(const JointMaker::JointType _type)
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
JointMaker::JointType JointMaker::State() const
{
  return this->dataPtr->jointType;
}

/////////////////////////////////////////////////
unsigned int JointMaker::JointCount()
{
  return this->dataPtr->joints.size();
}

/////////////////////////////////////////////////
void JointData::OnApply()
{
  // Get data from inspector
  msgs::Joint *inspectorMsg = this->inspector->Data();
  if (!inspectorMsg)
    return;

  this->jointMsg->CopyFrom(*inspectorMsg);

  // Name
  if (this->name != this->jointMsg->name())
    gui::model::Events::jointNameChanged(this->hotspot->GetName(),
        this->jointMsg->name());
  this->name = this->jointMsg->name();

  // Type
  this->type = JointMaker::ConvertJointType(
      msgs::ConvertJointType(this->jointMsg->type()));

  // Get scoped names
  std::string parentOldName = this->parent->GetName();
  std::string parentScope = parentOldName;
  size_t parentIdx = parentOldName.find("::");
  if (parentIdx != std::string::npos)
    parentScope = parentOldName.substr(0, parentIdx+2);
  std::string childOldName = this->child->GetName();
  std::string childScope = childOldName;
  size_t childIdx = childOldName.find("::");
  if (childIdx != std::string::npos)
    childScope = childOldName.substr(0, childIdx+2);

  std::string parentName = parentScope + this->jointMsg->parent();
  std::string childName = childScope + this->jointMsg->child();

  // Parent
  if (parentName != this->jointMsg->parent())
  {
    rendering::VisualPtr parentVis = gui::get_active_camera()->GetScene()
        ->GetVisual(parentName);
    if (parentVis)
      this->parent = parentVis;
    else
      gzwarn << "Invalid parent, keeping old parent" << std::endl;
  }

  // Child
  if (childName != this->jointMsg->child())
  {
    rendering::VisualPtr childVis = gui::get_active_camera()->GetScene()
        ->GetVisual(childName);
    if (childVis)
    {
      this->child = childVis;
      if (this->jointVisual)
        childVis->AttachVisual(this->jointVisual);
    }
    else
      gzwarn << "Invalid child, keeping old child" << std::endl;
  }

  this->dirty = true;
  gui::model::Events::modelChanged();
}

/////////////////////////////////////////////////
void JointData::OnOpenInspector()
{
  this->OpenInspector();
}

/////////////////////////////////////////////////
void JointData::OpenInspector()
{
  this->inspector->Update(this->jointMsg);
  this->inspector->Open();
}

/////////////////////////////////////////////////
void JointData::Update()
{
  // Material
  std::string material = JointMaker::jointMaterials[this->type];

  // Hotspot and parent handle
  if (this->parent && this->child && this->hotspot && this->handles)
  {
    auto parentOrigin = this->parent->GetWorldPose().Ign().Pos();
    auto childOrigin = this->child->GetWorldPose().Ign().Pos();

    // Hotspot position
    auto dPos = childOrigin - parentOrigin;
    auto center = dPos * 0.5;
    double length = std::max(dPos.Length(), 0.001);
    this->hotspot->SetScale(ignition::math::Vector3d(0.008, 0.008, length));
    this->hotspot->SetWorldPosition(parentOrigin + center);

    // Hotspot orientation
    auto u = dPos.Normalize();
    auto v = ignition::math::Vector3d::UnitZ;
    double cosTheta = v.Dot(u);
    double angle = acos(cosTheta);
    auto w = (v.Cross(u)).Normalize();
    ignition::math::Quaterniond q;
    q.Axis(w, angle);
    this->hotspot->SetWorldRotation(q);

    // Parent handle position
    this->handles->getBillboard(0)->setPosition(
        rendering::Conversions::Convert(parentOrigin -
        this->hotspot->GetWorldPose().Ign().Pos()));
    this->handles->_updateBounds();

    // set new material if joint type has changed
    if (this->hotspot->GetMaterialName() != material)
    {
      // Note: issue setting material when there is a billboard child,
      // seems to hang so detach before setting and re-attach later.
      Ogre::SceneNode *handleNode = this->handles->getParentSceneNode();
      this->handles->detachFromParent();
      this->hotspot->SetMaterial(material);
      this->hotspot->SetTransparency(0.7);
      handleNode->attachObject(this->handles);
      Ogre::MaterialPtr mat =
          Ogre::MaterialManager::getSingleton().getByName(material);
      Ogre::ColourValue color = mat->getTechnique(0)->getPass(0)->getDiffuse();
      color.a = 0.5;
      this->handles->getBillboard(0)->setColour(color);
    }
  }

  // Joint message and joint visual
  if (this->jointMsg)
  {
    this->UpdateMsg();

    // Update existing visual
    if (this->jointVisual)
    {
      this->jointVisual->UpdateFromMsg(this->jointMsg);
    }
    // Create joint visual
    else if (this->child)
    {
      std::string childName = this->child->GetName();
      std::string jointVisName = childName;
      size_t idx = childName.find("::");
      if (idx != std::string::npos)
        jointVisName = childName.substr(0, idx+2);
      jointVisName += "_JOINT_VISUAL_";

      gazebo::rendering::JointVisualPtr jointVis(
          new gazebo::rendering::JointVisual(jointVisName, this->child));
      jointVis->Load(this->jointMsg);
      this->jointVisual = jointVis;
    }
  }

  // Line
  if (this->line)
  {
    this->line->setMaterial(material);

    // Parent - child
    if (this->child && this->jointVisual)
    {
      this->line->SetPoint(0, (this->child->GetWorldPose().pos
          - this->child->GetParent()->GetWorldPose().pos).Ign());
      this->line->SetPoint(1,
          (this->jointVisual->GetWorldPose().pos
          - this->child->GetParent()->GetWorldPose().pos).Ign());
    }
    // Parent - mouse
    else if (this->parent && this->parent->GetParent())
    {
      this->line->SetPoint(0, this->parent->GetWorldPose().Ign().Pos());
    }
  }

  // Notify joint changes
  if (this->parent && this->child && this->hotspot)
  {
    std::string parentName = this->parent->GetName();
    std::string childName = this->child->GetName();
    gui::model::Events::jointChanged(this->hotspot->GetName(), this->name,
        JointMaker::jointTypes[this->type], parentName, childName);
  }

  this->dirty = false;
}

/////////////////////////////////////////////////
void JointData::UpdateMsg()
{
  // Some values are only stored in the msg, so we keep those
  msgs::JointPtr oldMsg(new msgs::Joint);
  if (this->jointMsg)
  {
    oldMsg->CopyFrom(*this->jointMsg);
  }

  // Reset
  this->jointMsg.reset(new msgs::Joint);

  // Name
  this->jointMsg->set_name(this->name);

  // Parent
  if (this->parent)
  {
    std::string jointParentName = this->parent->GetName();
    std::string unscopedName = jointParentName;
    size_t pIdx = jointParentName.find("::");
    if (pIdx != std::string::npos)
      unscopedName = jointParentName.substr(pIdx+2);

    this->jointMsg->set_parent(unscopedName);
    this->jointMsg->set_parent_id(this->parent->GetId());
  }

  // Child
  if (this->child)
  {
    std::string jointChildName = this->child->GetName();
    std::string unscopedName = jointChildName;
    size_t pIdx = jointChildName.find("::");
    if (pIdx != std::string::npos)
      unscopedName = jointChildName.substr(pIdx+2);

    this->jointMsg->set_child(unscopedName);
    this->jointMsg->set_child_id(this->child->GetId());
  }

  // Pose
  if (oldMsg && oldMsg->has_pose())
  {
    this->jointMsg->mutable_pose()->CopyFrom(*(oldMsg->mutable_pose()));
  }
  else
  {
    msgs::Set(this->jointMsg->mutable_pose(), ignition::math::Pose3d::Zero);
  }

  // Type
  this->jointMsg->set_type(
      msgs::ConvertJointType(JointMaker::TypeAsString(this->type)));

  // Axes
  unsigned int axisCount = JointMaker::JointAxisCount(this->type);
  for (unsigned int i = 0; i < axisCount; ++i)
  {
    msgs::Axis *axisMsg;
    msgs::Axis *oldAxisMsg = nullptr;
    if (i == 0u)
    {
      axisMsg = this->jointMsg->mutable_axis1();
      if (oldMsg && oldMsg->has_axis1())
        oldAxisMsg = oldMsg->mutable_axis1();
    }
    else if (i == 1u)
    {
      axisMsg = this->jointMsg->mutable_axis2();
      if (oldMsg && oldMsg->has_axis2())
        oldAxisMsg = oldMsg->mutable_axis2();
    }
    else
    {
      gzerr << "Invalid axis index["
            << i
            << "]"
            << std::endl;
      continue;
    }
    // Keep axis from previous msg if possible
    if (oldAxisMsg)
    {
      axisMsg->CopyFrom(*oldAxisMsg);
    }
    else
    {
      if (this->type == JointMaker::JOINT_GEARBOX)
      {
        msgs::Set(axisMsg->mutable_xyz(), ignition::math::Vector3d::UnitZ);
      }
      else
      {
        if (this->axes.size() < i+1)
        {
          this->axes.push_back(
              JointMaker::unitVectors[i%JointMaker::unitVectors.size()]);
        }
        msgs::Set(axisMsg->mutable_xyz(), this->axes[i]);
      }
      axisMsg->set_use_parent_model_frame(false);
      axisMsg->set_limit_lower(-GZ_DBL_MAX);
      axisMsg->set_limit_upper(GZ_DBL_MAX);
      axisMsg->set_limit_effort(-1);
      axisMsg->set_limit_velocity(-1);
      axisMsg->set_damping(0);
    }

    // Add angle field after we've checked that index i is valid
    this->jointMsg->add_angle(0);
  }

  // Others
  if (oldMsg && oldMsg->has_limit_erp())
  {
    this->jointMsg->set_limit_erp(oldMsg->limit_erp());
  }
  else
    this->jointMsg->set_limit_erp(0.2);

  if (oldMsg && oldMsg->has_suspension_erp())
  {
    this->jointMsg->set_suspension_erp(oldMsg->suspension_erp());
  }
  else
    this->jointMsg->set_suspension_erp(0.2);
}

/////////////////////////////////////////////////
void JointMaker::ShowContextMenu(const std::string &_name)
{
  auto it = this->dataPtr->joints.find(_name);
  if (it == this->dataPtr->joints.end())
    return;

  QMenu menu;
  if (this->dataPtr->inspectAct)
    menu.addAction(this->dataPtr->inspectAct);

  this->dataPtr->inspectName = _name;
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
  auto it = this->dataPtr->joints.find(_name);
  if (it == this->dataPtr->joints.end())
    return;

  this->SetSelected((*it).second->hotspot, _selected);
}

/////////////////////////////////////////////////
void JointMaker::SetSelected(const rendering::VisualPtr &_jointVis,
    const bool _selected)
{
  if (!_jointVis)
    return;

  _jointVis->SetHighlighted(_selected);
  auto it = std::find(this->dataPtr->selectedJoints.begin(),
      this->dataPtr->selectedJoints.end(), _jointVis);
  if (_selected)
  {
    if (it == this->dataPtr->selectedJoints.end())
    {
      this->dataPtr->selectedJoints.push_back(_jointVis);
      model::Events::setSelectedJoint(_jointVis->GetName(), _selected);
    }
  }
  else
  {
    if (it != this->dataPtr->selectedJoints.end())
    {
      this->dataPtr->selectedJoints.erase(it);
      model::Events::setSelectedJoint(_jointVis->GetName(), _selected);
    }
  }
}

/////////////////////////////////////////////////
void JointMaker::DeselectAll()
{
  while (!this->dataPtr->selectedJoints.empty())
  {
    rendering::VisualPtr vis = this->dataPtr->selectedJoints[0];
    vis->SetHighlighted(false);
    this->dataPtr->selectedJoints.erase(this->dataPtr->selectedJoints.begin());
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
    gzerr << "Unable to load joint. Joint child [" << childName <<
        "] or parent [" << parentName << "] not found" << std::endl;
    return;
  }

  JointData *joint = new JointData();
  joint->name = jointMsg.name();
  joint->parent = parentVis;
  joint->child = childVis;
  joint->type = this->ConvertJointType(msgs::ConvertJointType(jointMsg.type()));
  std::string jointVisName = _modelName + "::" + joint->name;

  joint->jointMsg.reset(new msgs::Joint);
  joint->jointMsg->CopyFrom(jointMsg);
  joint->jointMsg->set_parent_id(joint->parent->GetId());
  joint->jointMsg->set_child_id(joint->child->GetId());

  // Inspector
  joint->inspector = new JointInspector(this);
  joint->inspector->Update(joint->jointMsg);
  joint->inspector->setModal(false);
  connect(joint->inspector, SIGNAL(Applied()), joint, SLOT(OnApply()));

  // Visuals
  rendering::VisualPtr jointVis(
      new rendering::Visual(jointVisName, parentVis->GetRootVisual(), false));
  jointVis->Load();
  rendering::DynamicLines *jointLine =
      jointVis->CreateDynamicLine(rendering::RENDERING_LINE_LIST);

  auto parentOriginInRootFrame = parentVis->GetWorldPose().Ign().Pos()
      - parentVis->GetRootVisual()->GetWorldPose().Ign().Pos();
  jointLine->AddPoint(parentOriginInRootFrame);
  jointLine->AddPoint(parentOriginInRootFrame +
                      ignition::math::Vector3d(0, 0, 0.1));

  jointVis->GetSceneNode()->setInheritScale(false);
  jointVis->GetSceneNode()->setInheritOrientation(false);
  joint->visual = jointVis;
  joint->line = jointLine;
  joint->dirty = true;

  auto jointId = this->CreateHotSpot(joint);

  // Notify other widgets
  if (!jointId.empty())
  {
    gui::model::Events::jointInserted(jointId, joint->name,
        jointTypes[joint->type], joint->parent->GetName(),
        joint->child->GetName());
  }
}

/////////////////////////////////////////////////
void JointMaker::OnLinkInserted(const std::string &_linkName)
{
  std::string unscopedName = _linkName;
  size_t idx = unscopedName.find("::");
  if (idx != std::string::npos)
    unscopedName = _linkName.substr(idx+2);

  this->dataPtr->linkList[_linkName] = unscopedName;

  this->EmitLinkInserted(_linkName);
}

/////////////////////////////////////////////////
void JointMaker::OnLinkRemoved(const std::string &_linkName)
{
  if (this->dataPtr->linkList.erase(_linkName))
    this->EmitLinkRemoved(_linkName);
}

/////////////////////////////////////////////////
std::map<std::string, std::string> JointMaker::LinkList() const
{
  return this->dataPtr->linkList;
}

/////////////////////////////////////////////////
void JointMaker::ShowJoints(bool _show)
{
  for (auto iter : this->dataPtr->joints)
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
bool JointMaker::SetParentLink(const rendering::VisualPtr &_parentLink)
{
  if (!_parentLink)
  {
    gzerr << "Parent link is null" << std::endl;
    return false;
  }

  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->updateMutex);

  // Choosing parent for the first time
  if (!this->dataPtr->newJoint)
  {
    // Create new line connecting parent to mouse
    this->dataPtr->newJoint = this->CreateJointLine("JOINT_LINE", _parentLink);
  }
  // Update parent of joint being created
  else if (this->dataPtr->newJoint->parent)
  {
    // Reset previous parent
    this->dataPtr->newJoint->parent->SetWorldPose(
        this->dataPtr->parentLinkOriginalPose);
    this->SetVisualMoved(this->dataPtr->newJoint->parent, false);

    this->dataPtr->newJoint->parent = _parentLink;
    this->dataPtr->newJoint->dirty = true;
  }
  else
  {
    gzerr << "There's a joint being created but the parent visual hasn't been "
        << "defined. This should never happen." << std::endl;
    return false;
  }

  this->dataPtr->parentLinkOriginalPose = _parentLink->GetWorldPose().Ign();
  return true;
}

/////////////////////////////////////////////////
bool JointMaker::SetChildLink(const rendering::VisualPtr &_childLink)
{
  if (!_childLink)
  {
    gzerr << "Child link can't be null" << std::endl;
    return false;
  }

  if (!this->dataPtr->newJoint || !this->dataPtr->newJoint->parent)
  {
    gzerr << "New joint must have a parent before a child" << std::endl;
    return false;
  }

  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->updateMutex);

  // Choosing child for the first time
  if (!this->dataPtr->newJoint->child)
  {
    rendering::VisualPtr parentVis = this->dataPtr->newJoint->parent;

    // Clear joint line connected to parent
    this->RemoveJoint("");

    // Create new joint with parent and child
    auto joint = this->CreateJoint(parentVis, _childLink);
    this->dataPtr->newJoint = joint;

    // Create hotspot visual
    this->CreateHotSpot(this->dataPtr->newJoint);
  }
  // Update child
  else
  {
    // Reset previous child
    this->dataPtr->newJoint->child->SetWorldPose(
        this->dataPtr->childLinkOriginalPose);
    this->SetVisualMoved(this->dataPtr->newJoint->child, false);

    this->dataPtr->newJoint->child = _childLink;
    this->dataPtr->newJoint->dirty = true;
    _childLink->AttachVisual(this->dataPtr->newJoint->jointVisual);
  }

  this->dataPtr->childLinkOriginalPose = _childLink->GetWorldPose().Ign();

  // Change state to not creating joint
  gui::Events::manipMode("select");
  this->dataPtr->jointType = JointMaker::JOINT_NONE;
  return true;
}

/////////////////////////////////////////////////
void JointMaker::OnType(const int _typeInt)
{
  this->dataPtr->jointType = static_cast<JointMaker::JointType>(_typeInt);

  if (this->dataPtr->newJoint && this->dataPtr->jointType != JOINT_NONE)
  {
    this->dataPtr->newJoint->type = this->dataPtr->jointType;
    this->dataPtr->newJoint->dirty = true;
  }
}

/////////////////////////////////////////////////
void JointMaker::SetAxis(const std::string &_axis,
      const ignition::math::Vector3d &_value)
{
  if (this->dataPtr->newJoint && this->dataPtr->newJoint->jointMsg)
  {
    if (_axis == "axis1" && this->dataPtr->newJoint->jointMsg->has_axis1())
    {
      msgs::Set(
          this->dataPtr->newJoint->jointMsg->mutable_axis1()->mutable_xyz(),
          _value);
      this->dataPtr->newJoint->axes[0] = _value;
    }
    else if (_axis == "axis2" &&
        this->dataPtr->newJoint->jointMsg->has_axis2())
    {
      msgs::Set(
          this->dataPtr->newJoint->jointMsg->mutable_axis2()->mutable_xyz(),
          _value);
      this->dataPtr->newJoint->axes[1] = _value;
    }
    this->dataPtr->newJoint->dirty = true;
  }
}

/////////////////////////////////////////////////
void JointMaker::SetJointPose(const ignition::math::Pose3d &_pose)
{
  if (this->dataPtr->newJoint && this->dataPtr->newJoint->jointMsg)
  {
    msgs::Set(this->dataPtr->newJoint->jointMsg->mutable_pose(), _pose);
    this->dataPtr->newJoint->dirty = true;
  }
}

/////////////////////////////////////////////////
void JointMaker::SetParentLink(const std::string &_name)
{
  auto vis = this->LinkVisualFromName(_name);

  if (vis)
    this->SetParentLink(vis);
}

/////////////////////////////////////////////////
void JointMaker::SetChildLink(const std::string &_name)
{
  auto vis = this->LinkVisualFromName(_name);

  if (vis)
    this->SetChildLink(vis);
}

/////////////////////////////////////////////////
rendering::VisualPtr JointMaker::LinkVisualFromName(const std::string &_name)
{
  // Get scoped name
  std::string scopedName;
  for (auto link : this->dataPtr->linkList)
  {
    if (link.second == _name || link.first == _name)
    {
      scopedName = link.first;
      break;
    }
  }

  if (scopedName.empty())
  {
    gzwarn << "No link found with name [" << _name << "]" << std::endl;
    return nullptr;
  }

  // Get visual
  rendering::ScenePtr scene = rendering::get_scene();

  if (!scene)
    return nullptr;

  return scene->GetVisual(scopedName);
}

/////////////////////////////////////////////////
void JointMaker::SetLinksRelativePose(const ignition::math::Pose3d &_pose,
    const bool _resetAll, const int _resetAxis)
{
  if (!this->dataPtr->newJoint || !this->dataPtr->newJoint->parent ||
      !this->dataPtr->newJoint->child)
  {
    return;
  }

  auto newChildPose = this->dataPtr->newJoint->child->GetWorldPose().Ign();

  if (_resetAll)
  {
    newChildPose = this->dataPtr->childLinkOriginalPose;
    this->dataPtr->newJoint->parent->SetWorldPose(
        this->dataPtr->parentLinkOriginalPose);
  }
  else if (_resetAxis == 0)
  {
    newChildPose.Pos().X(this->dataPtr->childLinkOriginalPose.Pos().X());
  }
  else if (_resetAxis == 1)
  {
    newChildPose.Pos().Y(this->dataPtr->childLinkOriginalPose.Pos().Y());
  }
  else if (_resetAxis == 2)
  {
    newChildPose.Pos().Z(this->dataPtr->childLinkOriginalPose.Pos().Z());
  }
  else
  {
    // Get poses as homogeneous transforms
    ignition::math::Matrix4d parent_world(
        this->dataPtr->newJoint->parent->GetWorldPose().Ign());
    ignition::math::Matrix4d child_parent(_pose);

    // w_T_c = w_T_p * p_T_c
    ignition::math::Matrix4d child_world =
        parent_world * child_parent;

    newChildPose = child_world.Pose();
  }

  this->dataPtr->newJoint->child->SetWorldPose(newChildPose);
}

/////////////////////////////////////////////////
void JointMaker::AlignLinks(const bool _childToParent,
    const std::string &_axis, const std::string &_config, const bool _reverse)
{
  if (!this->dataPtr->newJoint || !this->dataPtr->newJoint->parent ||
      !this->dataPtr->newJoint->child)
  {
    gzerr << "Couldn't find new joint's parent and child links to be aligned."
        << std::endl;
    return;
  }

  std::vector<rendering::VisualPtr> links;
  links.push_back(this->dataPtr->newJoint->parent);
  links.push_back(this->dataPtr->newJoint->child);

  std::string target = _childToParent ? "first" : "last";

  ModelAlign::Instance()->AlignVisuals(links, _axis, _config,
      target, true, _reverse);
}

/////////////////////////////////////////////////
void JointMaker::SetVisualMoved(const rendering::VisualPtr &_vis,
    const bool _moved)
{
  if (_vis->GetChildCount() != 0)
  {
    for (unsigned int j = 0; j < _vis->GetChildCount(); ++j)
    {
      this->SetVisualMoved(_vis->GetChild(j), _moved);
    }
  }
  else
  {
    if (_moved)
    {
      _vis->SetEmissive(common::Color(0, 0, 1, 1));
    }
    else
    {
      _vis->SetEmissive(common::Color(0, 0, 0, 1));
    }
  }
}

/////////////////////////////////////////////////
void JointMaker::FinalizeCreation()
{
  gui::model::Events::modelChanged();
  this->dataPtr->jointType = JointMaker::JOINT_NONE;

  // Notify schematic view and palette list
  if (this->dataPtr->newJoint && this->dataPtr->newJoint->hotspot &&
      this->dataPtr->newJoint->child && this->dataPtr->newJoint->parent)
  {
    gui::model::Events::jointInserted(
        this->dataPtr->newJoint->hotspot->GetName(),
        this->dataPtr->newJoint->name,
        this->jointTypes[this->dataPtr->newJoint->type],
        this->dataPtr->newJoint->parent->GetName(),
        this->dataPtr->newJoint->child->GetName());


    // Reset visuals
    this->SetVisualMoved(this->dataPtr->newJoint->parent, false);
    this->SetVisualMoved(this->dataPtr->newJoint->child, false);
  }

  // Register command
  if (this->dataPtr->userCmdManager)
  {
    auto cmd = this->dataPtr->userCmdManager->NewCmd(
        "Inserted [" + this->dataPtr->newJoint->name + "]",
        MEUserCmd::INSERTING_JOINT);
    cmd->SetSDF(msgs::JointToSDF(*this->dataPtr->newJoint->jointMsg));
    cmd->SetScopedName(this->dataPtr->newJoint->visual->GetName());
    cmd->SetJointId(this->dataPtr->newJoint->hotspot->GetName());
  }

  this->dataPtr->newJoint = nullptr;

  // Notify ModelEditor to uncheck tool button
  this->JointAdded();

  this->Stop();
}

/////////////////////////////////////////////////
void JointMaker::SetUserCmdManager(MEUserCmdManager *_manager)
{
  this->dataPtr->userCmdManager = _manager;
}

