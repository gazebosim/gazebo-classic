/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/common/MeshManager.hh"
#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/SonarVisualPrivate.hh"
#include "gazebo/rendering/SonarVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
SonarVisual::SonarVisual(const std::string &_name, VisualPtr _vis,
                         const std::string &_topicName)
  : Visual(*new SonarVisualPrivate, _name, _vis)
{
  SonarVisualPrivate *dPtr =
      reinterpret_cast<SonarVisualPrivate *>(this->dataPtr);

  dPtr->type = VT_SENSOR;

  dPtr->receivedMsg = false;

  dPtr->node = transport::NodePtr(new transport::Node());
  dPtr->node->Init(dPtr->scene->Name());

  dPtr->sonarSub = dPtr->node->Subscribe(_topicName,
      &SonarVisual::OnMsg, this, true);

  dPtr->sonarRay = NULL;

  dPtr->connections.push_back(
      event::Events::ConnectPreRender(
        boost::bind(&SonarVisual::Update, this)));
}

/////////////////////////////////////////////////
SonarVisual::~SonarVisual()
{
}

/////////////////////////////////////////////////
void SonarVisual::Load()
{
  Visual::Load();

  SonarVisualPrivate *dPtr =
      reinterpret_cast<SonarVisualPrivate *>(this->dataPtr);

  dPtr->sonarRay = this->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  dPtr->sonarRay->setMaterial("Gazebo/RedGlow");
  dPtr->sonarRay->AddPoint(0, 0, 0);
  dPtr->sonarRay->AddPoint(0, 0, 0);

  dPtr->coneVis.reset(
      new Visual(this->GetName() + "_SONAR_CONE_", shared_from_this(), false));
  dPtr->coneVis->Load();
  dPtr->coneVis->InsertMesh("unit_cone");
  dPtr->coneVis->AttachMesh("unit_cone");
  dPtr->coneVis->SetMaterial("Gazebo/BlueLaser");
  dPtr->coneVis->SetCastShadows(false);

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}

/////////////////////////////////////////////////
void SonarVisual::OnMsg(ConstSonarStampedPtr &_msg)
{
  SonarVisualPrivate *dPtr =
      reinterpret_cast<SonarVisualPrivate *>(this->dataPtr);

  boost::mutex::scoped_lock lock(dPtr->mutex);
  dPtr->sonarMsg = _msg;
  dPtr->receivedMsg = true;
}

/////////////////////////////////////////////////
void SonarVisual::Update()
{
  SonarVisualPrivate *dPtr =
      reinterpret_cast<SonarVisualPrivate *>(this->dataPtr);

  boost::mutex::scoped_lock lock(dPtr->mutex);

  if (!dPtr->sonarMsg || !dPtr->receivedMsg)
    return;

  // Skip the update if the user is moving the sonar.
  if (this->GetScene()->SelectedVisual() &&
      this->GetRootVisual()->GetName() ==
      this->GetScene()->SelectedVisual()->GetName())
  {
    return;
  }

  double rangeDelta = dPtr->sonarMsg->sonar().range_max()
      - dPtr->sonarMsg->sonar().range_min();
  double radiusScale = dPtr->sonarMsg->sonar().radius()*2.0;

  if (!ignition::math::equal(dPtr->coneVis->GetScale().z, rangeDelta) ||
      !ignition::math::equal(dPtr->coneVis->GetScale().x, radiusScale))
  {
    dPtr->coneVis->SetScale(
        ignition::math::Vector3d(radiusScale, radiusScale, rangeDelta));
    dPtr->sonarRay->SetPoint(0,
        ignition::math::Vector3d(0, 0, rangeDelta * 0.5));
  }

  ignition::math::Pose3d pose =
    msgs::ConvertIgn(dPtr->sonarMsg->sonar().world_pose());
  this->SetPose(pose);

  if (dPtr->sonarMsg->sonar().has_contact())
  {
    ignition::math::Vector3d pos =
      msgs::ConvertIgn(dPtr->sonarMsg->sonar().contact());
    dPtr->sonarRay->SetPoint(1, pos);
  }
  else
  {
    dPtr->sonarRay->SetPoint(1, ignition::math::Vector3d(0, 0,
          (rangeDelta * 0.5) - dPtr->sonarMsg->sonar().range()));
  }
  dPtr->receivedMsg = false;
}
