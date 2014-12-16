/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include <boost/thread/recursive_mutex.hpp>
#include <string>
#include <vector>

#include "gazebo/common/MouseEvent.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Scene.hh"

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/KeyEventHandler.hh"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/GuiEvents.hh"

#include "gazebo/gui/model/JointInspector.hh"
#include "gazebo/gui/model/JointMaker.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
JointMaker::JointMaker()
{
  this->UnitVectors.push_back(math::Vector3::UnitX);
  this->UnitVectors.push_back(math::Vector3::UnitY);
  this->UnitVectors.push_back(math::Vector3::UnitZ);

  this->newJointCreated = false;
  this->mouseJoint = NULL;
  this->modelSDF.reset();
  this->jointType = JointMaker::JOINT_NONE;
  this->jointCounter = 0;

  this->jointMaterials[JOINT_FIXED]     = "Gazebo/Red";
  this->jointMaterials[JOINT_HINGE]     = "Gazebo/Orange";
  this->jointMaterials[JOINT_HINGE2]    = "Gazebo/Yellow";
  this->jointMaterials[JOINT_SLIDER]    = "Gazebo/Green";
  this->jointMaterials[JOINT_SCREW]     = "Gazebo/Black";
  this->jointMaterials[JOINT_UNIVERSAL] = "Gazebo/Blue";
  this->jointMaterials[JOINT_BALL]      = "Gazebo/Purple";

  this->connections.push_back(
      event::Events::ConnectPreRender(
        boost::bind(&JointMaker::Update, this)));

  this->inspectAct = new QAction(tr("Open Joint Inspector"), this);
  connect(this->inspectAct, SIGNAL(triggered()), this, SLOT(OnOpenInspector()));

  this->updateMutex = new boost::recursive_mutex();
}

/////////////////////////////////////////////////
JointMaker::~JointMaker()
{
  this->Reset();
}

/////////////////////////////////////////////////
void JointMaker::Reset()
{
  boost::recursive_mutex::scoped_lock lock(*this->updateMutex);
  this->newJointCreated = false;
  if (this->mouseJoint)
  {
    delete this->mouseJoint;
    this->mouseJoint = NULL;
  }

  this->jointType = JointMaker::JOINT_NONE;
  this->selectedVis.reset();
  this->hoverVis.reset();
  this->prevHoverVis.reset();
  this->inspectVis.reset();
  this->selectedJoint.reset();

  while (this->joints.size() > 0)
    this->RemoveJoint(this->joints.begin()->first);
  this->joints.clear();
}

/////////////////////////////////////////////////
void JointMaker::EnableEventHandlers()
{
  MouseEventHandler::Instance()->AddDoubleClickFilter("model_joint",
    boost::bind(&JointMaker::OnMouseDoubleClick, this, _1));

  MouseEventHandler::Instance()->AddReleaseFilter("model_joint",
      boost::bind(&JointMaker::OnMouseRelease, this, _1));

  KeyEventHandler::Instance()->AddPressFilter("model_joint",
      boost::bind(&JointMaker::OnKeyPress, this, _1));
}

/////////////////////////////////////////////////
void JointMaker::DisableEventHandlers()
{
  MouseEventHandler::Instance()->RemoveDoubleClickFilter("model_joint");
  MouseEventHandler::Instance()->RemoveReleaseFilter("model_joint");
  MouseEventHandler::Instance()->RemoveMoveFilter("model_joint");
  KeyEventHandler::Instance()->RemovePressFilter("model_joint");
}

/////////////////////////////////////////////////
void JointMaker::RemoveJoint(const std::string &_jointName)
{
  boost::recursive_mutex::scoped_lock lock(*this->updateMutex);
  if (this->joints.find(_jointName) != this->joints.end())
  {
    JointData *joint = this->joints[_jointName];
    rendering::ScenePtr scene = joint->hotspot->GetScene();
    scene->GetManager()->destroyBillboardSet(joint->handles);
    scene->RemoveVisual(joint->hotspot);
    scene->RemoveVisual(joint->visual);
    joint->hotspot.reset();
    joint->visual.reset();
    joint->parent.reset();
    joint->child.reset();
    joint->inspector->hide();
    delete joint->inspector;
    delete joint;
    this->joints.erase(_jointName);
  }
}

/////////////////////////////////////////////////
void JointMaker::RemoveJointsByPart(const std::string &_partName)
{
  std::vector<std::string> toDelete;
  boost::unordered_map<std::string, JointData *>::iterator it;
  for (it = this->joints.begin(); it != this->joints.end(); ++it)
  {
    JointData *joint = it->second;
    if (joint->child->GetName() == _partName ||
        joint->parent->GetName() == _partName)
    {
      toDelete.push_back(it->first);
    }
  }

  for (unsigned i = 0; i < toDelete.size(); ++i)
    this->RemoveJoint(toDelete[i]);

  toDelete.clear();
}

/////////////////////////////////////////////////
bool JointMaker::OnMousePress(const common::MouseEvent &_event)
{
  rendering::UserCameraPtr camera = gui::get_active_camera();
  if (_event.button == common::MouseEvent::MIDDLE)
  {
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
    camera->HandleMouseEvent(_event);
    return true;
  }
  else if (_event.button != common::MouseEvent::LEFT)
    return false;

  if (this->jointType != JointMaker::JOINT_NONE)
    return false;

  // intercept mouse press events when user clicks on the joint hotspot visual
  rendering::ScenePtr scene = camera->GetScene();
  rendering::VisualPtr vis = camera->GetVisual(_event.pos);
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
  if (this->jointType == JointMaker::JOINT_NONE)
  {
    rendering::VisualPtr vis = camera->GetVisual(_event.pos);
    if (vis)
    {
      if (this->selectedJoint)
        this->selectedJoint->SetHighlighted(false);
      this->selectedJoint.reset();
      if (this->joints.find(vis->GetName()) != this->joints.end())
      {
        // trigger joint inspector on right click
        if (_event.button == common::MouseEvent::RIGHT)
        {
          this->inspectVis = vis;
          QMenu menu;
          menu.addAction(this->inspectAct);
          menu.exec(QCursor::pos());
        }
        else if (_event.button == common::MouseEvent::LEFT)
        {
          this->selectedJoint = vis;
          this->selectedJoint->SetHighlighted(true);
        }
      }
      return false;
    }
  }
  else
  {
    if (_event.button == common::MouseEvent::LEFT)
    {
      if (this->hoverVis)
      {
        if (this->hoverVis->IsPlane())
        {
          QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
          camera->HandleMouseEvent(_event);
          return true;
        }

        // Pressed parent part
        if (!this->selectedVis)
        {
          if (this->mouseJoint)
            return false;

          this->hoverVis->SetEmissive(common::Color(0, 0, 0));
          this->selectedVis = this->hoverVis;
          this->hoverVis.reset();
          // Create joint data with selected visual as parent
          // the child will be set on the second mouse release.
          this->mouseJoint = this->CreateJoint(this->selectedVis,
              rendering::VisualPtr());
        }
        // Pressed child part
        else if (this->selectedVis != this->hoverVis)
        {
          if (this->hoverVis)
            this->hoverVis->SetEmissive(common::Color(0, 0, 0));
          if (this->selectedVis)
            this->selectedVis->SetEmissive(common::Color(0, 0, 0));
          this->mouseJoint->child = this->hoverVis;

          // reset variables.
          this->selectedVis.reset();
          this->hoverVis.reset();
          this->AddJoint(JointMaker::JOINT_NONE);

          this->newJointCreated = true;
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
JointData *JointMaker::CreateJoint(rendering::VisualPtr _parent,
    rendering::VisualPtr _child)
{
  std::stringstream ss;
  ss << _parent->GetName() << "_JOINT_" << this->jointCounter++;
  rendering::VisualPtr jointVis(
      new rendering::Visual(ss.str(), _parent->GetParent()));
  jointVis->Load();
  rendering::DynamicLines *jointLine =
      jointVis->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  math::Vector3 origin = this->GetPartWorldCentroid(_parent)
      - _parent->GetParent()->GetWorldPose().pos;
  jointLine->AddPoint(origin);
  jointLine->AddPoint(origin + math::Vector3(0, 0, 0.1));
  jointVis->GetSceneNode()->setInheritScale(false);
  jointVis->GetSceneNode()->setInheritOrientation(false);

  JointData *jointData = new JointData;
  jointData->name = jointVis->GetName();
  jointData->dirty = false;
  jointData->visual = jointVis;
  jointData->parent = _parent;
  jointData->child = _child;
  jointData->line = jointLine;
  jointData->type = this->jointType;
  jointData->inspector = new JointInspector(JointMaker::JOINT_NONE);
  jointData->inspector->setModal(false);
  connect(jointData->inspector, SIGNAL(Applied()),
      jointData, SLOT(OnApply()));

  int axisCount = JointMaker::GetJointAxisCount(jointData->type);
  for (int i = 0; i < axisCount; ++i)
  {
    if (i < static_cast<int>(this->UnitVectors.size()))
      jointData->axis[i] = this->UnitVectors[i];
    else
      jointData->axis[i] = this->UnitVectors[0];

    jointData->lowerLimit[i] = -3.14;
    jointData->upperLimit[i] = 3.14;
  }
  jointData->pose = math::Pose::Zero;
  jointData->line->setMaterial(this->jointMaterials[jointData->type]);

  return jointData;
}

/////////////////////////////////////////////////
void JointMaker::AddJoint(const std::string &_type)
{
  if (_type == "revolute")
  {
    this->AddJoint(JointMaker::JOINT_HINGE);
  }
  else if (_type == "revolute2")
  {
    this->AddJoint(JointMaker::JOINT_HINGE2);
  }
  else if (_type == "prismatic")
  {
    this->AddJoint(JointMaker::JOINT_SLIDER);
  }
  else if (_type == "ball")
  {
    this->AddJoint(JointMaker::JOINT_BALL);
  }
  else if (_type == "universal")
  {
    this->AddJoint(JointMaker::JOINT_UNIVERSAL);
  }
  else if (_type == "screw")
  {
    this->AddJoint(JointMaker::JOINT_SCREW);
  }
  else if (_type == "none")
  {
    this->AddJoint(JointMaker::JOINT_NONE);
  }
}

/////////////////////////////////////////////////
void JointMaker::AddJoint(JointMaker::JointType _type)
{
  this->jointType = _type;
  if (_type != JointMaker::JOINT_NONE)
  {
    // Add an event filter, which allows the JointMaker to capture mouse events.
    MouseEventHandler::Instance()->AddMoveFilter("model_joint",
        boost::bind(&JointMaker::OnMouseMove, this, _1));
  }
  else
  {
    // Remove the event filters.
    MouseEventHandler::Instance()->RemoveMoveFilter("model_joint");

    // signal the end of a joint action.
    emit JointAdded();
  }
}

/////////////////////////////////////////////////
void JointMaker::Stop()
{
  if (this->jointType != JointMaker::JOINT_NONE)
  {
    this->newJointCreated = false;
    if (this->mouseJoint)
    {
      this->mouseJoint->visual->DeleteDynamicLine(this->mouseJoint->line);
      rendering::ScenePtr scene = this->mouseJoint->visual->GetScene();
      scene->RemoveVisual(this->mouseJoint->visual);
      this->mouseJoint->visual.reset();
      delete this->mouseJoint->inspector;
      delete this->mouseJoint;
      this->mouseJoint = NULL;
    }
    this->AddJoint(JointMaker::JOINT_NONE);
    if (this->hoverVis)
      this->hoverVis->SetEmissive(common::Color(0, 0, 0));
    if (this->selectedVis)
      this->selectedVis->SetEmissive(common::Color(0, 0, 0));
    this->selectedVis.reset();
    this->hoverVis.reset();
  }
}

/////////////////////////////////////////////////
bool JointMaker::OnMouseMove(const common::MouseEvent &_event)
{
  // Get the active camera and scene.
  rendering::UserCameraPtr camera = gui::get_active_camera();

  if (_event.dragging)
  {
    // this enables the joint maker to pan while connecting joints
    QApplication::setOverrideCursor(QCursor(Qt::ArrowCursor));
    camera->HandleMouseEvent(_event);
    return true;
  }

  rendering::VisualPtr vis = camera->GetVisual(_event.pos);

  // Highlight visual on hover
  if (vis)
  {
    if (this->hoverVis && this->hoverVis != this->selectedVis)
      this->hoverVis->SetEmissive(common::Color(0.0, 0.0, 0.0));

    // only highlight editor parts by making sure it's not an item in the
    // gui tree widget or a joint hotspot.
    rendering::VisualPtr rootVis = vis->GetRootVisual();
    if (rootVis->IsPlane())
      this->hoverVis = vis->GetParent();
    else if (!gui::get_entity_id(rootVis->GetName()) &&
        vis->GetName().find("_HOTSPOT_") == std::string::npos)
    {
      this->hoverVis = vis->GetParent();
      if (!this->selectedVis ||
           (this->selectedVis && this->hoverVis != this->selectedVis))
        this->hoverVis->SetEmissive(common::Color(0.5, 0.5, 0.5));
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
      {
        parentPos =  this->GetPartWorldCentroid(this->mouseJoint->parent)
            - this->mouseJoint->line->GetPoint(0);
        this->mouseJoint->line->SetPoint(1,
            this->GetPartWorldCentroid(this->hoverVis) - parentPos);
      }
    }
    else
    {
      // Set end point to mouse plane intersection
      math::Vector3 pt;
      camera->GetWorldPointOnPlane(_event.pos.x, _event.pos.y,
          math::Plane(math::Vector3(0, 0, 1)), pt);
      if (this->mouseJoint->parent)
      {
        parentPos = this->GetPartWorldCentroid(this->mouseJoint->parent)
            - this->mouseJoint->line->GetPoint(0);
        this->mouseJoint->line->SetPoint(1,
            this->GetPartWorldCentroid(this->hoverVis) - parentPos + pt);
      }
    }
  }
  return true;
}

/////////////////////////////////////////////////
void JointMaker::OnOpenInspector()
{
  this->OpenInspector(this->inspectVis->GetName());
  this->inspectVis.reset();
}

/////////////////////////////////////////////////
void JointMaker::OpenInspector(const std::string &_name)
{
  JointData *joint = this->joints[_name];
  joint->inspector->SetType(joint->type);
  joint->inspector->SetName(joint->name);
  joint->inspector->SetParent(joint->parent->GetName());
  joint->inspector->SetChild(joint->child->GetName());
  joint->inspector->SetPose(joint->pose);
  int axisCount = JointMaker::GetJointAxisCount(joint->type);
  for (int i = 0; i < axisCount; ++i)
  {
    joint->inspector->SetAxis(i, joint->axis[i]);
    joint->inspector->SetAxis(i, joint->axis[i]);
    joint->inspector->SetLowerLimit(i, joint->lowerLimit[i]);
    joint->inspector->SetUpperLimit(i, joint->upperLimit[i]);
  }
  joint->inspector->move(QCursor::pos());
  joint->inspector->show();
}

/////////////////////////////////////////////////
bool JointMaker::OnMouseDoubleClick(const common::MouseEvent &_event)
{
  rendering::UserCameraPtr camera = gui::get_active_camera();
  rendering::VisualPtr vis = camera->GetVisual(_event.pos);

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
    if (this->selectedJoint)
    {
      this->RemoveJoint(this->selectedJoint->GetName());
      this->selectedJoint.reset();
    }
  }
  return false;
}

/////////////////////////////////////////////////
void JointMaker::CreateHotSpot(JointData *_joint)
{
  if (!_joint)
    return;

  rendering::UserCameraPtr camera = gui::get_active_camera();

  std::string hotSpotName = _joint->visual->GetName() + "_HOTSPOT_";
  rendering::VisualPtr hotspotVisual(
      new rendering::Visual(hotSpotName, _joint->visual, false));

  _joint->hotspot = hotspotVisual;

  // create a cylinder to represent the joint
  hotspotVisual->InsertMesh("unit_cylinder");
  Ogre::MovableObject *hotspotObj =
      (Ogre::MovableObject*)(camera->GetScene()->GetManager()->createEntity(
      "__HOTSPOT__" + _joint->visual->GetName(), "unit_cylinder"));
  hotspotObj->setUserAny(Ogre::Any(hotSpotName));
  hotspotVisual->GetSceneNode()->attachObject(hotspotObj);
  hotspotVisual->SetMaterial(this->jointMaterials[_joint->type]);
  hotspotVisual->SetTransparency(0.5);

  // create a handle at the parent end
  Ogre::BillboardSet *handleSet =
      camera->GetScene()->GetManager()->createBillboardSet(1);
  handleSet->setAutoUpdate(true);
  handleSet->setMaterialName("Gazebo/PointHandle");
  Ogre::MaterialPtr mat =
      Ogre::MaterialManager::getSingleton().getByName(
      this->jointMaterials[_joint->type]);
  Ogre::ColourValue color = mat->getTechnique(0)->getPass(0)->getDiffuse();
  color.a = 0.5;
  double dimension = 0.1;
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

  this->joints[hotSpotName] = _joint;
  camera->GetScene()->AddVisual(hotspotVisual);

  // remove line as we are using a cylinder hotspot visual to
  // represent the joint
  _joint->visual->DeleteDynamicLine(_joint->line);

  _joint->dirty = true;
}

/////////////////////////////////////////////////
void JointMaker::Update()
{
  boost::recursive_mutex::scoped_lock lock(*this->updateMutex);
  if (this->newJointCreated)
  {
    this->CreateHotSpot(this->mouseJoint);
    this->mouseJoint = NULL;
    this->newJointCreated = false;
  }

  // update joint line and hotspot position.
  boost::unordered_map<std::string, JointData *>::iterator it;
  for (it = this->joints.begin(); it != this->joints.end(); ++it)
  {
    JointData *joint = it->second;
    if (joint->dirty)
    {
      if (joint->child && joint->parent)
      {
        // get centroid of parent part visuals
        math::Vector3 parentCentroid =
            this->GetPartWorldCentroid(joint->parent);

        // get centroid of child part visuals
        math::Vector3 childCentroid =
            this->GetPartWorldCentroid(joint->child);

        // set orientation of joint hotspot
        math::Vector3 dPos = (childCentroid - parentCentroid);
        math::Vector3 center = dPos/2.0;
        joint->hotspot->SetScale(math::Vector3(0.02, 0.02, dPos.GetLength()));
        joint->hotspot->SetWorldPosition(parentCentroid + center);
        math::Vector3 u = dPos.Normalize();
        math::Vector3 v = math::Vector3::UnitZ;
        double cosTheta = v.Dot(u);
        double angle = acos(cosTheta);
        math::Vector3 w = (v.Cross(u)).Normalize();
        math::Quaternion q;
        q.SetFromAxis(w, angle);
        joint->hotspot->SetWorldRotation(q);

        // set new material if joint type has changed
        std::string material = this->jointMaterials[joint->type];
        if (joint->hotspot->GetMaterialName() != material)
        {
          // Note: issue setting material when there is a billboard child,
          // seems to hang so detach before setting and re-attach later.
          Ogre::SceneNode *handleNode = joint->handles->getParentSceneNode();
          joint->handles->detachFromParent();
          joint->hotspot->SetMaterial(material);
          handleNode->attachObject(joint->handles);
          Ogre::MaterialPtr mat =
              Ogre::MaterialManager::getSingleton().getByName(material);
          Ogre::ColourValue color =
              mat->getTechnique(0)->getPass(0)->getDiffuse();
          color.a = 0.5;
          joint->handles->getBillboard(0)->setColour(color);
        }

        // set pos of joint handle
        joint->handles->getBillboard(0)->setPosition(
            rendering::Conversions::Convert(parentCentroid -
            joint->hotspot->GetWorldPose().pos));
        joint->handles->_updateBounds();
      }
    }
  }
}

/////////////////////////////////////////////////
void JointMaker::GenerateSDF()
{
  this->modelSDF.reset(new sdf::Element);
  sdf::initFile("model.sdf", this->modelSDF);
  this->modelSDF->ClearElements();

  boost::unordered_map<std::string, JointData *>::iterator jointsIt;
  // loop through all parts
  for (jointsIt = this->joints.begin(); jointsIt != this->joints.end();
      ++jointsIt)
  {
    JointData *joint = jointsIt->second;
    sdf::ElementPtr jointElem = this->modelSDF->AddElement("joint");

    jointElem->GetAttribute("name")->Set(joint->name);
    jointElem->GetAttribute("type")->Set(GetTypeAsString(joint->type));
    sdf::ElementPtr parentElem = jointElem->GetElement("parent");
    parentElem->Set(joint->parent->GetName());
    sdf::ElementPtr childElem = jointElem->GetElement("child");
    childElem->Set(joint->child->GetName());
    sdf::ElementPtr poseElem = jointElem->GetElement("pose");
    poseElem->Set(joint->pose);
    int axisCount = GetJointAxisCount(joint->type);
    for (int i = 0; i < axisCount; ++i)
    {
      std::stringstream ss;
      ss << "axis";
      if (i > 0)
        ss << (i+1);
      sdf::ElementPtr axisElem = jointElem->GetElement(ss.str());
      axisElem->GetElement("xyz")->Set(joint->axis[i]);

      sdf::ElementPtr limitElem = axisElem->GetElement("limit");
      limitElem->GetElement("lower")->Set(joint->lowerLimit[i]);
      limitElem->GetElement("upper")->Set(joint->upperLimit[i]);
    }
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
  else if (_type == JointMaker::JOINT_NONE)
  {
    jointTypeStr = "none";
  }

  return jointTypeStr;
}

/////////////////////////////////////////////////
int JointMaker::GetJointAxisCount(JointMaker::JointType _type)
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

  return 0;
}

/////////////////////////////////////////////////
JointMaker::JointType JointMaker::GetState() const
{
  return this->jointType;
}

/////////////////////////////////////////////////
math::Vector3 JointMaker::GetPartWorldCentroid(
    const rendering::VisualPtr _visual)
{
  math::Vector3 centroid;
  for (unsigned int i = 0; i < _visual->GetChildCount(); ++i)
    centroid += _visual->GetChild(i)->GetWorldPose().pos;
  centroid /= _visual->GetChildCount();
  return centroid;
}

/////////////////////////////////////////////////
unsigned int JointMaker::GetJointCount()
{
  return this->joints.size();
}

/////////////////////////////////////////////////
void JointData::OnApply()
{
  this->pose = this->inspector->GetPose();
  this->type = this->inspector->GetType();
  this->name = this->inspector->GetName();

  int axisCount = JointMaker::GetJointAxisCount(this->type);
  for (int i = 0; i < axisCount; ++i)
  {
    this->axis[i] = this->inspector->GetAxis(i);
    this->lowerLimit[i] = this->inspector->GetLowerLimit(i);
    this->upperLimit[i] = this->inspector->GetUpperLimit(i);
  }
}
