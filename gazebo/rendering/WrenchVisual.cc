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
#include <boost/bind.hpp>

#include "gazebo/common/MeshManager.hh"
#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/WrenchVisualPrivate.hh"
#include "gazebo/rendering/WrenchVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
WrenchVisual::WrenchVisual(const std::string &_name, VisualPtr _vis,
    const std::string &_topicName)
  : Visual(*new WrenchVisualPrivate, _name, _vis)
{
  WrenchVisualPrivate *dPtr =
      reinterpret_cast<WrenchVisualPrivate *>(this->dataPtr);

  dPtr->enabled = true;
  dPtr->receivedMsg = false;

  dPtr->node = transport::NodePtr(new transport::Node());
  dPtr->node->Init(dPtr->scene->GetName());

  dPtr->wrenchSub = dPtr->node->Subscribe(_topicName,
      &WrenchVisual::OnMsg, this, true);

  // Make sure the meshes are in Ogre
  this->InsertMesh("unit_cone");
  Ogre::MovableObject *coneXObj =
    (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
          this->GetName()+"__WRENCH_X_CONE__", "unit_cone"));
  ((Ogre::Entity*)coneXObj)->setMaterialName("__GAZEBO_TRANS_RED_MATERIAL__");

  Ogre::MovableObject *coneYObj =
    (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
          this->GetName()+"__WRENCH_Y_CONE__", "unit_cone"));
  ((Ogre::Entity*)coneYObj)->setMaterialName("__GAZEBO_TRANS_GREEN_MATERIAL__");

  Ogre::MovableObject *coneZObj =
    (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
          this->GetName()+"__WRENCH_Z_CONE__", "unit_cone"));
  ((Ogre::Entity*)coneZObj)->setMaterialName("__GAZEBO_TRANS_BLUE_MATERIAL__");

  math::Quaternion q;

  dPtr->coneXNode =
      dPtr->sceneNode->createChildSceneNode(this->GetName() + "_WRENCH_X_CONE");
  dPtr->coneXNode->attachObject(coneXObj);
  q.SetFromAxis(0, 1, 0, GZ_DTOR(-90));
  dPtr->coneXNode->setOrientation(q.w, q.x, q.y, q.z);
  dPtr->coneXNode->setScale(0.02, 0.02, 0.02);

  dPtr->coneYNode =
      dPtr->sceneNode->createChildSceneNode(this->GetName() + "_WRENCH_Y_CONE");
  dPtr->coneYNode->attachObject(coneYObj);
  q.SetFromAxis(1, 0, 0, GZ_DTOR(90));
  dPtr->coneYNode->setOrientation(q.w, q.x, q.y, q.z);
  dPtr->coneYNode->setScale(0.02, 0.02, 0.02);

  dPtr->coneZNode =
    dPtr->sceneNode->createChildSceneNode(this->GetName() + "_WRENCH_Z_CONE");
  dPtr->coneZNode->attachObject(coneZObj);
  q.SetFromAxis(1, 0, 0, GZ_DTOR(180));
  dPtr->coneZNode->setOrientation(q.w, q.x, q.y, q.z);
  dPtr->coneZNode->setScale(0.02, 0.02, 0.02);

  dPtr->forceLine = new DynamicLines(RENDERING_LINE_LIST);
  dPtr->forceLine->AddPoint(math::Vector3(0, 0, 0));
  dPtr->forceLine->AddPoint(math::Vector3(0, 0, 0));
  dPtr->forceLine->setMaterial("__GAZEBO_TRANS_PURPLE_MATERIAL__");

  dPtr->forceNode = dPtr->sceneNode->createChildSceneNode(this->GetName() +
      "_WRENCH_FORCE_NODE_");
  dPtr->forceNode->attachObject(dPtr->forceLine);

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);

  dPtr->connections.push_back(
      event::Events::ConnectPreRender(
        boost::bind(&WrenchVisual::Update, this)));
}

/////////////////////////////////////////////////
WrenchVisual::~WrenchVisual()
{
  WrenchVisualPrivate *dPtr =
      reinterpret_cast<WrenchVisualPrivate *>(this->dataPtr);

  dPtr->node.reset();
  dPtr->connections.clear();

  delete dPtr->forceLine;
  dPtr->forceLine = NULL;

  delete dPtr->forceNode;
  dPtr->forceNode = NULL;
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
  WrenchVisualPrivate *dPtr =
      reinterpret_cast<WrenchVisualPrivate *>(this->dataPtr);

  boost::mutex::scoped_lock lock(dPtr->mutex);

  if (!dPtr->wrenchMsg || !dPtr->receivedMsg)
    return;

  double magScale = 100;
  double vMax = 0.5;
  double vMin = 0.1;
  double vRange = vMax - vMin;
  double offset = vRange - vMin;

  // Scaling factor.
  double xScale = (2.0 * vRange) / (1 +
      exp(-dPtr->wrenchMsg->wrench().torque().x() / magScale)) - offset;

  double yScale = (2.0 * vRange) / (1 +
      exp(-dPtr->wrenchMsg->wrench().torque().y() / magScale)) - offset;

  double zScale = (2.0 * vRange) / (1 +
      exp(-dPtr->wrenchMsg->wrench().torque().z() / magScale)) - offset;

  magScale = 50000;
  math::Vector3 force = msgs::Convert(dPtr->wrenchMsg->wrench().force());
  double forceScale = (2.0 * vRange) / (1 +
      exp(force.GetSquaredLength() / magScale)) - offset;

  dPtr->forceLine->SetPoint(1, force*forceScale);
  dPtr->forceLine->Update();

  dPtr->coneXNode->setScale(0.02, 0.02, xScale);
  dPtr->coneXNode->setPosition(xScale * 0.5, 0, 0);

  dPtr->coneYNode->setScale(0.02, 0.02, yScale);
  dPtr->coneYNode->setPosition(0, yScale * 0.5, 0);

  dPtr->coneZNode->setScale(0.02, 0.02, zScale);
  dPtr->coneZNode->setPosition(0, 0, zScale * 0.5);
}

/////////////////////////////////////////////////
void WrenchVisual::OnMsg(ConstWrenchStampedPtr &_msg)
{
  WrenchVisualPrivate *dPtr =
      reinterpret_cast<WrenchVisualPrivate *>(this->dataPtr);

  boost::mutex::scoped_lock lock(dPtr->mutex);
  if (dPtr->enabled)
  {
    dPtr->wrenchMsg = _msg;
    dPtr->receivedMsg = true;
  }
}
