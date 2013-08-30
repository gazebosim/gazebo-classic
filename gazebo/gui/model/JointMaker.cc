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

#include "gazebo/common/MouseEvent.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Scene.hh"

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/GuiEvents.hh"

#include "gazebo/gui/model/JointInspector.hh"
#include "gazebo/gui/model/JointMaker.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
JointMaker::JointMaker()
{
  this->connections.push_back(
      event::Events::ConnectPreRender(
        boost::bind(&JointMaker::Update, this)));

  this->newJointCreated = false;
  this->mouseJoint = NULL;
  this->modelSDF.reset();
  this->jointCounter = 0;

  this->jointMaterials[JOINT_FIXED] = "Gazebo/Red";
  this->jointMaterials[JOINT_HINGE] = "Gazebo/Orange";
  this->jointMaterials[JOINT_HINGE2] = "Gazebo/Yellow";
  this->jointMaterials[JOINT_SLIDER] = "Gazebo/Green";
  this->jointMaterials[JOINT_SCREW] = "Gazebo/Black";
  this->jointMaterials[JOINT_UNIVERSAL] = "Gazebo/Blue";
  this->jointMaterials[JOINT_BALL] = "Gazebo/Purple";

  MouseEventHandler::Instance()->AddDoubleClickFilter("model_joint",
    boost::bind(&JointMaker::OnMouseDoubleClick, this, _1));
}

/////////////////////////////////////////////////
JointMaker::~JointMaker()
{
  MouseEventHandler::Instance()->RemoveDoubleClickFilter("model_joint");
  this->hoverVis.reset();
}

/////////////////////////////////////////////////
void JointMaker::Reset()
{
 if (!gui::get_active_camera() || !gui::get_active_camera()->GetScene())
    return;

  this->newJointCreated = false;
  if (mouseJoint)
  {
    delete mouseJoint;
    mouseJoint = NULL;
  }

  this->jointType = JointMaker::JOINT_NONE;
  this->selectedVis.reset();
  this->hoverVis.reset();

  rendering::ScenePtr scene = gui::get_active_camera()->GetScene();

  while (this->joints.size() > 0)
    this->RemoveJoint(this->joints.begin()->first);
  this->joints.clear();

}

/////////////////////////////////////////////////
void JointMaker::RemoveJoint(const std::string &_jointName)
{
  JointData *joint = this->joints[_jointName];
  if (joint)
  {
    joint->visual.reset();
    rendering::ScenePtr scene = joint->hotspot->GetScene();
    scene->RemoveVisual(joint->hotspot);
    joint->hotspot.reset();
    joint->parent.reset();
    joint->child.reset();
    delete joint->line;
    delete joint->inspector;
    this->joints.erase(_jointName);
  }
}

/////////////////////////////////////////////////
bool JointMaker::OnMousePress(const common::MouseEvent &_event)
{
  if (_event.button != common::MouseEvent::LEFT)
    return false;

  // Get the active camera and scene.
  rendering::UserCameraPtr camera = gui::get_active_camera();
  rendering::ScenePtr scene = camera->GetScene();

  rendering::VisualPtr vis = camera->GetVisual(_event.pos);
//  if (vis)
//    vis = vis->GetRootVisual();

  if (this->hoverVis)
  {
    if (this->hoverVis->IsPlane())
      return false;

    // Pressed parent part
    if (!this->selectedVis)
    {
      if (this->mouseJoint)
        return false;

      this->selectedVis = this->hoverVis;
      this->hoverVis.reset();

      std::stringstream ss;
      ss << this->selectedVis->GetName() << "_JOINT_" << this->jointCounter++;
      rendering::VisualPtr jointVis(
          new rendering::Visual(ss.str(), this->selectedVis));
      jointVis->Load();
      rendering::DynamicLines *jointLine =
          jointVis->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
      jointLine->AddPoint(math::Vector3(0, 0, 0));
      jointLine->AddPoint(math::Vector3(0, 0, 0.01));
      jointVis->GetSceneNode()->setInheritScale(false);
      jointVis->GetSceneNode()->setInheritOrientation(false);


      JointData *jointData = new JointData;
      jointData->dirty = false;
      jointData->visual = jointVis;
      jointData->parent = this->selectedVis;
      jointData->line = jointLine;
      jointData->type = this->jointType;
      jointData->inspector = new JointInspector(JointMaker::JOINT_NONE);
      jointData->inspector->setModal(false);
      connect(jointData->inspector, SIGNAL(Applied()),
          jointData, SLOT(OnApply()));

      int axisCount = JointMaker::GetJointAxisCount(jointData->type);
      for (int i = 0; i < axisCount; ++i)
        jointData->axis[i] = math::Vector3::UnitX;
      jointData->anchor = math::Vector3::Zero;
      this->mouseJoint = jointData;
      jointData->line->setMaterial(this->jointMaterials[jointData->type]);
    }
    // Pressed child part
    else if (this->selectedVis != vis)
    {
      if (this->hoverVis)
        this->hoverVis->SetHighlighted(false);
      if (this->selectedVis)
        this->selectedVis->SetHighlighted(false);
      this->mouseJoint->child = vis;

      // reset variables.
      this->selectedVis.reset();
      this->hoverVis.reset();
      this->CreateJoint(JointMaker::JOINT_NONE);
      this->newJointCreated = true;
      emit JointAdded();
    }
  }
  return true;
}

/////////////////////////////////////////////////
void JointMaker::CreateJoint(JointMaker::JointType _type)
{
  this->jointType = _type;
  if (_type != JointMaker::JOINT_NONE)
  {
    // Add an event filter, which allows the TerrainEditor to capture
    // mouse events.
    MouseEventHandler::Instance()->AddPressFilter("model_joint",
        boost::bind(&JointMaker::OnMousePress, this, _1));

    MouseEventHandler::Instance()->AddMoveFilter("model_joint",
        boost::bind(&JointMaker::OnMouseMove, this, _1));
  }
  else
  {
    // Remove the event filters.
    MouseEventHandler::Instance()->RemovePressFilter("model_joint");
    MouseEventHandler::Instance()->RemoveMoveFilter("model_joint");
  }
}


/////////////////////////////////////////////////
bool JointMaker::OnMouseMove(const common::MouseEvent &_event)
{
  if (_event.button != common::MouseEvent::LEFT ||
      _event.dragging)
    return false;

  // Get the active camera and scene.
  rendering::UserCameraPtr camera = gui::get_active_camera();
  rendering::ScenePtr scene = camera->GetScene();

  rendering::VisualPtr vis = camera->GetVisual(_event.pos);

  // Highlight visual on hover
  if (vis)
  {
    if (this->hoverVis && this->hoverVis != this->selectedVis)
      this->hoverVis->SetHighlighted(false);

    // only highlight editor parts
    if (!gui::get_entity_id(vis->GetName()))
    {
      this->hoverVis = vis;
      if (!this->hoverVis->IsPlane())
      {
        this->hoverVis->SetHighlighted(true);
  //    event::Events::setSelectedEntity(vis->GetName(), "normal");
      }
    }
  }

  // Case when a parent part is already selected and currently
  // extending the joint line to a child part
  if (this->selectedVis && this->hoverVis
      && this->mouseJoint && this->mouseJoint->line)
  {
    math::Vector3 parentPos;
    // Set end point to center of child part
    if (!this->hoverVis->IsPlane())
    {
      if (this->mouseJoint->parent)
        parentPos = this->mouseJoint->parent->GetWorldPose().pos;
      this->mouseJoint->line->SetPoint(1,
          this->hoverVis->GetWorldPose().pos - parentPos);
    }
    else
    {
      // Set end point to mouse plane intersection
      math::Vector3 pt;
      camera->GetWorldPointOnPlane(_event.pos.x, _event.pos.y,
          math::Plane(math::Vector3(0, 0, 1)), pt);
      if (this->mouseJoint->parent)
        parentPos = this->mouseJoint->parent->GetWorldPose().pos;
      this->mouseJoint->line->SetPoint(1,
          this->hoverVis->GetWorldPose().pos - parentPos + pt);
    }
  }
  return true;
}

/////////////////////////////////////////////////
bool JointMaker::OnMouseDoubleClick(const common::MouseEvent &_event)
{
  rendering::UserCameraPtr camera = gui::get_active_camera();
  rendering::VisualPtr vis = camera->GetVisual(_event.pos);

  if (vis)
  {
    JointData *joint = this->joints[vis->GetName()];
    if (joint)
    {
      joint->inspector->SetType(joint->type);
      joint->inspector->SetName(joint->visual->GetName());
      joint->inspector->SetAnchor(0, joint->anchor);
      int axisCount = JointMaker::GetJointAxisCount(joint->type);
      for (int i = 0; i < axisCount; ++i)
        joint->inspector->SetAxis(i, joint->axis[i]);
    }
    joint->inspector->show();
    return true;
  }
  else
    return false;
}

/////////////////////////////////////////////////
void JointMaker::CreateHotSpot()
{
  if (!this->mouseJoint)
    return;

  JointData *joint = this->mouseJoint;

  rendering::UserCameraPtr camera = gui::get_active_camera();

  rendering::VisualPtr hotspotVisual(
      new rendering::Visual(joint->visual->GetName() + "_HOTSPOT_",
      joint->visual, false));

  hotspotVisual->InsertMesh("unit_sphere");

  Ogre::MovableObject *hotspotObj =
      (Ogre::MovableObject*)(camera->GetScene()->GetManager()->createEntity(
      "__HOTSPOT__" + joint->visual->GetName(), "unit_sphere"));
  hotspotObj->setUserAny(Ogre::Any(hotspotVisual->GetName()));

  hotspotVisual->GetSceneNode()->attachObject(hotspotObj);
  hotspotVisual->SetMaterial("Gazebo/RedTransparent");
  hotspotVisual->SetScale(math::Vector3(0.1, 0.1, 0.1));

  hotspotVisual->SetWorldPosition(
      joint->parent->GetWorldPose().pos +
      (joint->child->GetWorldPose().pos -
      joint->parent->GetWorldPose().pos)/2.0);

  hotspotVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  hotspotVisual->GetSceneNode()->setInheritScale(false);

  camera->GetScene()->AddVisual(hotspotVisual);
  joint->hotspot = hotspotVisual;

  this->joints[hotspotVisual->GetName()] = this->mouseJoint;
  this->mouseJoint = NULL;
  joint->dirty = true;
}

/////////////////////////////////////////////////
void JointMaker::Update()
{
  if (this->newJointCreated)
  {
    this->CreateHotSpot();
    this->newJointCreated = false;
  }

  boost::unordered_map<std::string, JointData *>::iterator it;
  for (it = this->joints.begin(); it != this->joints.end(); ++it)
  {
    JointData *joint = it->second;
    if (joint->dirty)
    {
      joint->line->SetPoint(1,
          joint->child->GetWorldPose().pos -
          joint->parent->GetWorldPose().pos);

      joint->hotspot->SetWorldPosition(
          joint->parent->GetWorldPose().pos +
          (joint->child->GetWorldPose().pos -
          joint->parent->GetWorldPose().pos)/2.0);
    }
  }
}

/////////////////////////////////////////////////
void JointMaker::GenerateSDF()
{
  sdf::ElementPtr jointElem;
  sdf::ElementPtr parentElem;
  sdf::ElementPtr childElem;
  sdf::ElementPtr axisElem[2];
  sdf::ElementPtr poseElem;

  this->modelSDF.reset(new sdf::Element);
  sdf::initFile("model.sdf", this->modelSDF);

  jointElem = this->modelSDF->GetElement("joint");
  jointElem = jointElem->Clone();
  this->modelSDF->ClearElements();

  boost::unordered_map<std::string, JointData *>::iterator jointsIt;
  // loop through all parts
  for (jointsIt = this->joints.begin(); jointsIt != this->joints.end();
      ++jointsIt)
  {
    JointData *joint = jointsIt->second;

    jointElem->GetAttribute("name")->Set(joint->visual->GetName());
    jointElem->GetAttribute("type")->Set(GetTypeAsString(joint->type));
    parentElem = jointElem->GetElement("parent");
    parentElem->Set(joint->parent->GetParent()->GetName());
    childElem = jointElem->GetElement("child");
    childElem->Set(joint->child->GetParent()->GetName());
    int axisCount = GetJointAxisCount(joint->type);
    for (int i = 0; i < axisCount; ++i)
    {
      std::stringstream ss;
      ss << "axis";
      if (i > 0)
        ss << (i+1);
      axisElem[i] = jointElem->GetElement(ss.str());
      axisElem[i]->Set(joint->axis[i]);
    }
    this->modelSDF->InsertElement(jointElem->Clone());
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

  if (_type == JointMaker::JOINT_FIXED)
  {
    jointTypeStr = "fixed";
  }
  else if (_type == JointMaker::JOINT_SLIDER)
  {
    jointTypeStr = "prismatic";
  }
  else if (_type == JointMaker::JOINT_HINGE)
  {
    jointTypeStr = "revolute";
  }
  else if (_type == JointMaker::JOINT_HINGE2)
  {
    jointTypeStr = "revolute2";
  }
  else if (_type == JointMaker::JOINT_SCREW)
  {
    jointTypeStr = "screw";
  }
  else if (_type == JointMaker::JOINT_UNIVERSAL)
  {
    jointTypeStr = "universal";
  }
  else if (_type == JointMaker::JOINT_BALL)
  {
    jointTypeStr = "ball";
  }

  return jointTypeStr;
}

/////////////////////////////////////////////////
int JointMaker::GetJointAxisCount(JointMaker::JointType _type)
{
  if (_type == JOINT_FIXED)
    return 0;
  else if (_type == JOINT_HINGE)
    return 1;
  else if (_type == JOINT_HINGE2)
    return 2;
  else if (_type == JOINT_SLIDER)
    return 1;
  else if (_type == JOINT_SCREW)
    return 1;
  else if (_type == JOINT_UNIVERSAL)
    return 2;
  else if (_type == JOINT_BALL)
    return 0;

  return 0;
}


/////////////////////////////////////////////////
void JointData::OnApply()
{
  this->anchor = this->inspector->GetAnchor(0);
  this->type = this->inspector->GetType();
  int axisCount = JointMaker::GetJointAxisCount(this->type);
  for (int i = 0; i < axisCount; ++i)
    this->axis[i] = this->inspector->GetAxis(i);

}
