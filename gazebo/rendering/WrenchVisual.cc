/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/common/MeshManager.hh"
#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/WrenchVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
WrenchVisual::WrenchVisual(const std::string &_name, VisualPtr _vis,
    const std::string &_topicName)
: Visual(_name, _vis)
{
  this->enabled = true;
  this->receivedMsg = false;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->scene->GetName());

  this->wrenchSub = this->node->Subscribe(_topicName,
      &WrenchVisual::OnMsg, this, true);

  // Make sure the meshes are in Ogre
  this->InsertMesh("unit_cone");
  Ogre::MovableObject *coneXObj =
    (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
          this->GetName()+"__WRENCH_X_CONE__", "unit_cone"));
  ((Ogre::Entity*)coneXObj)->setMaterialName("__GAZEBO_TRANS_RED_MATERIAL__");

  Ogre::MovableObject *coneYObj =
    (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
          this->GetName()+"__WRENCH_Y_CONE__", "unit_cone"));
  ((Ogre::Entity*)coneYObj)->setMaterialName("__GAZEBO_TRANS_GREEN_MATERIAL__");

  Ogre::MovableObject *coneZObj =
    (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
          this->GetName()+"__WRENCH_Z_CONE__", "unit_cone"));
  ((Ogre::Entity*)coneZObj)->setMaterialName("__GAZEBO_TRANS_BLUE_MATERIAL__");

  math::Quaternion q;

  this->coneXNode =
    this->sceneNode->createChildSceneNode(this->GetName() + "_WRENCH_X_CONE");
  this->coneXNode->attachObject(coneXObj);
  q.SetFromAxis(0, 1, 0, GZ_DTOR(-90));
  this->coneXNode->setOrientation(q.w, q.x, q.y, q.z);
  this->coneXNode->setScale(0.02, 0.02, 0.02);

  this->coneYNode =
    this->sceneNode->createChildSceneNode(this->GetName() + "_WRENCH_Y_CONE");
  this->coneYNode->attachObject(coneYObj);
  q.SetFromAxis(1, 0, 0, GZ_DTOR(90));
  this->coneYNode->setOrientation(q.w, q.x, q.y, q.z);
  this->coneYNode->setScale(0.02, 0.02, 0.02);

  this->coneZNode =
    this->sceneNode->createChildSceneNode(this->GetName() + "_WRENCH_Z_CONE");
  this->coneZNode->attachObject(coneZObj);
  q.SetFromAxis(1, 0, 0, GZ_DTOR(180));
  this->coneZNode->setOrientation(q.w, q.x, q.y, q.z);
  this->coneZNode->setScale(0.02, 0.02, 0.02);

  this->forceLine = new DynamicLines(RENDERING_LINE_LIST);
  this->forceLine->AddPoint(math::Vector3(0, 0, 0));
  this->forceLine->AddPoint(math::Vector3(0, 0, 0));
  this->forceLine->setMaterial("__GAZEBO_TRANS_PURPLE_MATERIAL__");

  this->forceNode = this->sceneNode->createChildSceneNode(this->GetName() +
      "_WRENCH_FORCE_NODE_");
  this->forceNode->attachObject(this->forceLine);

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);

  this->connections.push_back(
      event::Events::ConnectPreRender(
        boost::bind(&WrenchVisual::Update, this)));
}

/////////////////////////////////////////////////
WrenchVisual::~WrenchVisual()
{
  this->node.reset();
  this->connections.clear();

  delete this->forceLine;
  this->forceLine = NULL;

  delete this->forceNode;
  this->forceNode = NULL;
}

/////////////////////////////////////////////////
void WrenchVisual::Load(ConstJointPtr &_msg)
{
  Visual::Load();
  this->SetPosition(msgs::Convert(_msg->pose().position()));
  this->SetRotation(msgs::Convert(_msg->pose().orientation()));
}

/////////////////////////////////////////////////
void WrenchVisual::Update()
{
  boost::mutex::scoped_lock lock(this->mutex);

  if (!this->wrenchMsg || !this->receivedMsg)
    return;

  double magScale = 100;
  double vMax = 0.5;
  double vMin = 0.1;
  double vRange = vMax - vMin;
  double offset = vRange - vMin;

  // Scaling factor.
  double xScale = (2.0 * vRange) / (1 +
      exp(-this->wrenchMsg->wrench().torque().x() / magScale)) - offset;

  double yScale = (2.0 * vRange) / (1 +
      exp(-this->wrenchMsg->wrench().torque().y() / magScale)) - offset;

  double zScale = (2.0 * vRange) / (1 +
      exp(-this->wrenchMsg->wrench().torque().z() / magScale)) - offset;

  magScale = 50000;
  math::Vector3 force = msgs::Convert(this->wrenchMsg->wrench().force());
  double forceScale = (2.0 * vRange) / (1 +
      exp(force.GetSquaredLength() / magScale)) - offset;

  this->forceLine->SetPoint(1, force*forceScale);
  this->forceLine->Update();

  this->coneXNode->setScale(0.02, 0.02, xScale);
  this->coneXNode->setPosition(xScale * 0.5, 0, 0);

  this->coneYNode->setScale(0.02, 0.02, yScale);
  this->coneYNode->setPosition(0, yScale * 0.5, 0);

  this->coneZNode->setScale(0.02, 0.02, zScale);
  this->coneZNode->setPosition(0, 0, zScale * 0.5);
}

/////////////////////////////////////////////////
void WrenchVisual::OnMsg(ConstWrenchStampedPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);
  if (this->enabled)
  {
    this->wrenchMsg = _msg;
    this->receivedMsg = true;
  }
}
