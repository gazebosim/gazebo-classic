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

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/JointVisual.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Scene.hh"

#include "gazebo/gui/GuiIface.hh"

#include "gazebo/gui/model/JointInspector.hh"
#include "gazebo/gui/model/ModelEditorEvents.hh"
#include "gazebo/gui/model/JointData.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
void JointData::OnApply()
{
  // Get data from inspector
  msgs::Joint *inspectorMsg = this->inspector->GetData();
  if (!inspectorMsg)
    return;

  this->jointMsg->CopyFrom(*inspectorMsg);

  // Name
  if (this->name != this->jointMsg->name())
    gui::model::Events::jointNameChanged(this->hotspot->GetName(),
        this->jointMsg->name());
  this->name = this->jointMsg->name();

  // Type
  this->SetType(JointMaker::ConvertJointType(
      msgs::ConvertJointType(this->jointMsg->type())));

  // Parent
  if (this->jointMsg->parent() != this->Parent()->GetName())
  {
    // Get scoped name
    std::string oldName = this->Parent()->GetName();
    std::string scope = oldName;
    size_t idx = oldName.rfind("::");
    if (idx != std::string::npos)
      scope = oldName.substr(0, idx+2);

    rendering::VisualPtr parentVis = gui::get_active_camera()->GetScene()
        ->GetVisual(scope + this->jointMsg->parent());
    if (parentVis)
      this->SetParent(parentVis);
    else
      gzwarn << "Invalid parent, keeping old parent" << std::endl;
  }

  // Child
  if (this->jointMsg->child() != this->child->GetName())
  {
    // Get scoped name
    std::string oldName = this->child->GetName();
    std::string scope = oldName;
    size_t idx = oldName.rfind("::");
    if (idx != std::string::npos)
      scope = oldName.substr(0, idx+2);

    rendering::VisualPtr childVis = gui::get_active_camera()->GetScene()
        ->GetVisual(scope + this->jointMsg->child());
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
void JointData::SetType(const JointMaker::JointType _type)
{
  this->type = _type;

  if (this->jointMsg)
  {
    this->jointMsg->set_type(
        msgs::ConvertJointType(JointMaker::GetTypeAsString(_type)));
  }
  this->dirty = true;
}

/////////////////////////////////////////////////
JointMaker::JointType JointData::Type() const
{
  return this->type;
}

/////////////////////////////////////////////////
void JointData::SetChild(const rendering::VisualPtr &_vis)
{
  this->child = _vis;
  std::string jointChildName = this->child->GetName();
  std::string leafName = jointChildName;
  size_t pIdx = jointChildName.find_last_of("::");
  if (pIdx != std::string::npos)
    leafName = jointChildName.substr(pIdx+1);

  this->jointMsg->set_child(leafName);
  this->jointMsg->set_child_id(this->child->GetId());

  this->dirty = true;
}

/////////////////////////////////////////////////
rendering::VisualPtr JointData::Child() const
{
  return this->child;
}

/////////////////////////////////////////////////
void JointData::SetParent(const rendering::VisualPtr &_vis)
{
  this->parent = _vis;

  if (this->jointMsg)
  {
    std::string jointParentName = this->parent->GetName();
    std::string leafName = jointParentName;
    size_t pIdx = jointParentName.find_last_of("::");
    if (pIdx != std::string::npos)
      leafName = jointParentName.substr(pIdx+1);

    this->jointMsg->set_parent(leafName);
    this->jointMsg->set_parent_id(this->parent->GetId());
  }

  this->dirty = true;
}

/////////////////////////////////////////////////
rendering::VisualPtr JointData::Parent() const
{
  return this->parent;
}

/////////////////////////////////////////////////
void JointData::Update()
{
  // get origin of parent link visuals
  math::Vector3 parentOrigin = this->parent->GetWorldPose().pos;

  // get origin of child link visuals
  math::Vector3 childOrigin = this->child->GetWorldPose().pos;

  // set position of joint hotspot
  math::Vector3 dPos = (childOrigin - parentOrigin);
  math::Vector3 center = dPos * 0.5;
  double length = std::max(dPos.GetLength(), 0.001);
  this->hotspot->SetScale(
      math::Vector3(0.008, 0.008, length));
  this->hotspot->SetWorldPosition(parentOrigin + center);

  // set orientation of joint hotspot
  math::Vector3 u = dPos.Normalize();
  math::Vector3 v = math::Vector3::UnitZ;
  double cosTheta = v.Dot(u);
  double angle = acos(cosTheta);
  math::Vector3 w = (v.Cross(u)).Normalize();
  math::Quaternion q;
  q.SetFromAxis(w, angle);
  this->hotspot->SetWorldRotation(q);

  // set new material if joint type has changed
  std::string material = JointMaker::jointMaterials[this->type];
  if (this->hotspot->GetMaterialName() != material)
  {
    // Note: issue setting material when there is a billboard child,
    // seems to hang so detach before setting and re-attach later.
    Ogre::SceneNode *handleNode = this->handles->getParentSceneNode();
    this->handles->detachFromParent();
    this->hotspot->SetMaterial(material, true, false);
    this->hotspot->SetTransparency(0.7, false);
    handleNode->attachObject(this->handles);
    Ogre::MaterialPtr mat =
        Ogre::MaterialManager::getSingleton().getByName(material);
    Ogre::ColourValue color =
        mat->getTechnique(0)->getPass(0)->getDiffuse();
    color.a = 0.5;
    this->handles->getBillboard(0)->setColour(color);

    // notify joint changes
    std::string parentName = this->parent->GetName();
    std::string childName = this->child->GetName();
    gui::model::Events::jointChanged(this->hotspot->GetName(), this->name,
        JointMaker::jointTypes[this->type], parentName, childName);
  }

  // set pos of joint handle
  this->handles->getBillboard(0)->setPosition(
      rendering::Conversions::Convert(parentOrigin -
      this->hotspot->GetWorldPose().pos));
  this->handles->_updateBounds();

  // Update msg
  msgs::JointPtr jointUpdateMsg = this->jointMsg;
  msgs::Joint defaultMsg = JointMaker::SetupDefaultJointMsg(this->type);
  unsigned int axisCount = JointMaker::GetJointAxisCount(this->type);

  // Remove axis2
  if (axisCount < 2u && jointUpdateMsg->has_axis2())
    jointUpdateMsg->clear_axis2();
  // Remove axis1
  if (axisCount < 1u && jointUpdateMsg->has_axis1())
    jointUpdateMsg->clear_axis1();
  // Add axis1
  if (axisCount == 1u && !jointUpdateMsg->has_axis1())
  {
    jointUpdateMsg->mutable_axis1()->CopyFrom(*defaultMsg.mutable_axis1());
  }
  // Add axis2
  if (axisCount == 2u && !jointUpdateMsg->has_axis2())
  {
    jointUpdateMsg->mutable_axis2()->CopyFrom(*defaultMsg.mutable_axis2());
  }

  // Joint visual
  if (this->jointVisual)
  {
    this->jointVisual->UpdateFromMsg(jointUpdateMsg);
  }
  else
  {
    std::string childName = this->child->GetName();
    std::string jointVisName = childName;
    size_t idx = childName.find("::");
    if (idx != std::string::npos)
      jointVisName = childName.substr(0, idx+2);
    jointVisName += "_JOINT_VISUAL_";
    gazebo::rendering::JointVisualPtr jointVis(
        new gazebo::rendering::JointVisual(jointVisName, this->child));

    jointVis->Load(jointUpdateMsg);

    this->jointVisual = jointVis;
  }

  // Line now connects the child link to the joint frame
  this->line->SetPoint(0, this->child->GetWorldPose().pos
      - this->child->GetParent()->GetWorldPose().pos);
  this->line->SetPoint(1,
      this->jointVisual->GetWorldPose().pos
      - this->child->GetParent()->GetWorldPose().pos);
  this->line->setMaterial(JointMaker::jointMaterials[this->type]);
  this->dirty = false;
}

/////////////////////////////////////////////////
void JointData::UpdateJointLine()
{
  if (!this->parent)
    return;

  math::Vector3 origin = this->parent->GetWorldPose().pos
      - this->parent->GetParent()->GetWorldPose().pos;
  this->line->SetPoint(0, origin);
}

