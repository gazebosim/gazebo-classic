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
#include "gazebo/rendering/SonarVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
SonarVisual::SonarVisual(const std::string &_name, VisualPtr _vis,
                         const std::string &_topicName)
: Visual(_name, _vis)
{
  this->receivedMsg = false;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->scene->GetName());

  this->sonarSub = this->node->Subscribe(_topicName,
      &SonarVisual::OnMsg, this, true);

  this->sonarRay = this->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  this->sonarRay->setMaterial("Gazebo/RedGlow");
  this->sonarRay->AddPoint(0, 0, 0);
  this->sonarRay->AddPoint(0, 0, 0);

  // Make sure the meshes are in Ogre
  this->InsertMesh("unit_cone");
  Ogre::MovableObject *coneObj =
    (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
          this->GetName()+"__SONAR_CONE__", "unit_cone"));
  ((Ogre::Entity*)coneObj)->setMaterialName("Gazebo/BlueLaser");

  this->coneNode =
    this->sceneNode->createChildSceneNode(this->GetName() + "_SONAR_CONE");
  this->coneNode->attachObject(coneObj);
  this->coneNode->setPosition(0, 0, 0);

  this->connections.push_back(
      event::Events::ConnectPreRender(
        boost::bind(&SonarVisual::Update, this)));
}

/////////////////////////////////////////////////
SonarVisual::~SonarVisual()
{
  delete this->sonarRay;
  this->sonarRay = NULL;
}

/////////////////////////////////////////////////
void SonarVisual::Load()
{
  Visual::Load();
  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  this->SetCastShadows(false);
}

/////////////////////////////////////////////////
void SonarVisual::OnMsg(ConstSonarStampedPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->mutex);
  this->sonarMsg = _msg;
  this->receivedMsg = true;
}

/////////////////////////////////////////////////
void SonarVisual::Update()
{
  boost::mutex::scoped_lock lock(this->mutex);

  if (!this->sonarMsg || !this->receivedMsg)
    return;

  // Skip the update if the user is moving the sonar.
  if (this->GetScene()->GetSelectedVisual() &&
      this->GetRootVisual()->GetName() ==
      this->GetScene()->GetSelectedVisual()->GetName())
  {
    return;
  }

  float rangeDelta = this->sonarMsg->sonar().range_max()
      - this->sonarMsg->sonar().range_min();
  float radiusScale = this->sonarMsg->sonar().radius()*2.0;

  if (!math::equal(this->coneNode->getScale().z, rangeDelta) ||
      !math::equal(this->coneNode->getScale().x, radiusScale))
  {
    this->coneNode->setScale(radiusScale, radiusScale, rangeDelta);
    this->sonarRay->SetPoint(0, math::Vector3(0, 0, rangeDelta * 0.5));
  }

  math::Pose pose = msgs::Convert(this->sonarMsg->sonar().world_pose());
  this->SetPose(pose);

  if (this->sonarMsg->sonar().has_contact())
  {
    math::Vector3 pos = msgs::Convert(this->sonarMsg->sonar().contact());
    this->sonarRay->SetPoint(1, pos);
  }
  else
  {
    this->sonarRay->SetPoint(1, math::Vector3(0, 0,
          (rangeDelta * 0.5) - this->sonarMsg->sonar().range()));
  }
  this->receivedMsg = false;
}
