/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <boost/bind/bind.hpp>

#include <ignition/common/Profiler.hh>

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

  dPtr->type = VT_PHYSICS;

  dPtr->enabled = true;
  dPtr->receivedMsg = false;

  // Transport
  dPtr->node = transport::NodePtr(new transport::Node());
  dPtr->node->Init(dPtr->scene->Name());

  dPtr->wrenchSub = dPtr->node->Subscribe(_topicName,
      &WrenchVisual::OnMsg, this, true);

  dPtr->connections.push_back(
      event::Events::ConnectPreRender(
      std::bind(&WrenchVisual::Update, this)));
}

/////////////////////////////////////////////////
WrenchVisual::~WrenchVisual()
{
  this->Fini();
}

/////////////////////////////////////////////////
void WrenchVisual::Fini()
{
  WrenchVisualPrivate *dPtr =
      reinterpret_cast<WrenchVisualPrivate *>(this->dataPtr);

  dPtr->wrenchSub.reset();
  if (dPtr->node)
    dPtr->node->Fini();
  dPtr->node.reset();
  dPtr->connections.clear();

  dPtr->wrenchMsg.reset();

  // Remove force visual and line
  if (dPtr->forceVisual && dPtr->forceLine)
    dPtr->forceVisual->DeleteDynamicLine(dPtr->forceLine);

  if (dPtr->scene && dPtr->forceVisual &&
      dPtr->scene->GetVisual(dPtr->forceVisual->Name()))
  {
    dPtr->scene->RemoveVisual(dPtr->forceVisual);
  }
  dPtr->forceVisual.reset();

  Visual::Fini();
}

/////////////////////////////////////////////////
void WrenchVisual::Load(ConstJointPtr &_msg)
{
  Visual::Load();

  WrenchVisualPrivate *dPtr =
      reinterpret_cast<WrenchVisualPrivate *>(this->dataPtr);

  // Make sure the meshes are in Ogre
  this->InsertMesh("unit_cone");

  // Initialize visuals on Load because we can't use shared_from_this in the
  // constructor

  // Torque visual
  dPtr->coneXVis.reset(
      new Visual(this->Name()+"__WRENCH_X_CONE__", shared_from_this(),
      false));
  dPtr->coneXVis->Load();
  dPtr->coneXVis->AttachMesh("unit_cone");
  dPtr->coneXVis->SetMaterial("__GAZEBO_TRANS_RED_MATERIAL__");

  dPtr->coneYVis.reset(
      new Visual(this->Name()+"__WRENCH_Y_CONE__", shared_from_this(),
      false));
  dPtr->coneYVis->Load();
  dPtr->coneYVis->AttachMesh("unit_cone");
  dPtr->coneYVis->SetMaterial("__GAZEBO_TRANS_GREEN_MATERIAL__");

  dPtr->coneZVis.reset(
      new Visual(this->Name()+"__WRENCH_Z_CONE__", shared_from_this(),
      false));
  dPtr->coneZVis->Load();
  dPtr->coneZVis->AttachMesh("unit_cone");
  dPtr->coneZVis->SetMaterial("__GAZEBO_TRANS_BLUE_MATERIAL__");

  ignition::math::Quaterniond q;
  q.Axis(0, 1, 0, -IGN_PI_2);
  dPtr->coneXVis->SetRotation(q);
  dPtr->coneXVis->SetScale(ignition::math::Vector3d(0.02, 0.02, 0.02));

  q.Axis(1, 0, 0, IGN_PI_2);
  dPtr->coneYVis->SetRotation(q);
  dPtr->coneYVis->SetScale(ignition::math::Vector3d(0.02, 0.02, 0.02));

  q.Axis(1, 0, 0, IGN_PI);
  dPtr->coneZVis->SetRotation(q);
  dPtr->coneZVis->SetScale(ignition::math::Vector3d(0.02, 0.02, 0.02));

  // Force visual
  dPtr->forceVisual.reset(new rendering::Visual(
      this->Name() + "_FORCE_VISUAL_", shared_from_this()));
  dPtr->forceVisual->Load();

  dPtr->forceLine = dPtr->forceVisual->CreateDynamicLine(RENDERING_LINE_LIST);
  dPtr->forceLine->AddPoint(ignition::math::Vector3d::Zero);
  dPtr->forceLine->AddPoint(ignition::math::Vector3d(0, 0, 0.1));
  GZ_OGRE_SET_MATERIAL_BY_NAME(dPtr->forceLine,
      "__GAZEBO_TRANS_PURPLE_MATERIAL__");

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);

  dPtr->connections.push_back(
      event::Events::ConnectPreRender(
        boost::bind(&WrenchVisual::Update, this)));

  this->SetPosition(msgs::ConvertIgn(_msg->pose().position()));
  this->SetRotation(msgs::ConvertIgn(_msg->pose().orientation()));
}

/////////////////////////////////////////////////
void WrenchVisual::Update()
{
  IGN_PROFILE("rendering::WrenchVisual::Update");
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
  ignition::math::Vector3d force =
    msgs::ConvertIgn(dPtr->wrenchMsg->wrench().force());
  double forceScale = (2.0 * vRange) / (1 +
      exp(force.SquaredLength() / magScale)) - offset;

  dPtr->forceLine->SetPoint(1, force*forceScale);
  dPtr->forceLine->Update();

  dPtr->coneXVis->SetScale(ignition::math::Vector3d(0.02, 0.02, xScale));
  dPtr->coneXVis->SetPosition(ignition::math::Vector3d(xScale * 0.5, 0, 0));

  dPtr->coneYVis->SetScale(ignition::math::Vector3d(0.02, 0.02, yScale));
  dPtr->coneYVis->SetPosition(ignition::math::Vector3d(0, yScale * 0.5, 0));

  dPtr->coneZVis->SetScale(ignition::math::Vector3d(0.02, 0.02, zScale));
  dPtr->coneZVis->SetPosition(ignition::math::Vector3d(0, 0, zScale * 0.5));
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
