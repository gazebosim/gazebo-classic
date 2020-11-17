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
#include <ignition/common/Profiler.hh>

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

  dPtr->connections.push_back(
      event::Events::ConnectPreRender(
        std::bind(&SonarVisual::Update, this)));
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
  GZ_OGRE_SET_MATERIAL_BY_NAME(dPtr->sonarRay, "Gazebo/RedGlow");
  dPtr->sonarRay->AddPoint(0, 0, 0);
  dPtr->sonarRay->AddPoint(0, 0, 0);
  dPtr->sonarRay->SetPoint(0, ignition::math::Vector3d::Zero);

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
  IGN_PROFILE("rendering::SonarVisual::Update");
  SonarVisualPrivate *dPtr =
      reinterpret_cast<SonarVisualPrivate *>(this->dataPtr);

  boost::mutex::scoped_lock lock(dPtr->mutex);

  if (!dPtr->sonarMsg || !dPtr->receivedMsg)
    return;

  // Skip the update if the user is moving the sonar.
  if (this->GetScene()->SelectedVisual() &&
      this->GetRootVisual()->Name() ==
      this->GetScene()->SelectedVisual()->Name())
  {
    return;
  }

  if (!dPtr->shapeVis)
  {
    std::string geom = dPtr->sonarMsg->sonar().geometry();
    if (geom != "sphere" && geom != "cone")
    {
      gzwarn << "Unknown sonar geometry [" << geom << "], defaulting to [cone]"
             << std::endl;
      geom = "cone";
    }
    auto upperGeom = geom;
    std::transform(upperGeom.begin(), upperGeom.end(), upperGeom.begin(),
        ::toupper);
    dPtr->shapeVis = std::make_shared<Visual>(
        this->Name() + "_SONAR_" + upperGeom, shared_from_this(), false);
    dPtr->shapeVis->Load();
    dPtr->shapeVis->InsertMesh("unit_" + geom);
    dPtr->shapeVis->AttachMesh("unit_" + geom);
    dPtr->shapeVis->SetMaterial("Gazebo/BlueLaser");
    dPtr->shapeVis->SetCastShadows(false);
  }

  double rangeDelta = dPtr->sonarMsg->sonar().range_max()
      - dPtr->sonarMsg->sonar().range_min();

  if (dPtr->sonarMsg->sonar().geometry() == "sphere")
  {
    double rangeMax = dPtr->sonarMsg->sonar().range_max();
    if (!ignition::math::equal(dPtr->shapeVis->Scale().Z(), rangeMax) ||
        !ignition::math::equal(dPtr->shapeVis->Scale().X(), rangeMax))
    {
      dPtr->shapeVis->SetScale(
          ignition::math::Vector3d(rangeMax*2, rangeMax*2, rangeMax*2));
    }
  }
  else
  {
    dPtr->shapeVis->SetPosition(
        ignition::math::Vector3d(0, 0, -rangeDelta * 0.5));

    double radiusScale = dPtr->sonarMsg->sonar().radius() * 2.0;
    if (!ignition::math::equal(dPtr->shapeVis->Scale().Z(), rangeDelta) ||
        !ignition::math::equal(dPtr->shapeVis->Scale().X(), radiusScale))
    {
      dPtr->shapeVis->SetScale(
          ignition::math::Vector3d(radiusScale, radiusScale, rangeDelta));
    }
  }

  ignition::math::Pose3d pose =
      msgs::ConvertIgn(dPtr->sonarMsg->sonar().world_pose());
  this->SetWorldPose(pose);

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
