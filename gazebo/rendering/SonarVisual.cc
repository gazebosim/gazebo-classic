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

  dPtr->receivedMsg = false;

  dPtr->node = transport::NodePtr(new transport::Node());
  dPtr->node->Init(dPtr->scene->GetName());

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
  SonarVisualPrivate *dPtr =
      reinterpret_cast<SonarVisualPrivate *>(this->dataPtr);

  delete dPtr->sonarRay;
  dPtr->sonarRay = NULL;
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

  // Make sure the meshes are in Ogre
  this->InsertMesh("unit_cone");
  Ogre::MovableObject *coneObj =
    (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
          this->GetName()+"__SONAR_CONE__", "unit_cone"));
  ((Ogre::Entity*)coneObj)->setMaterialName("Gazebo/BlueLaser");

  dPtr->coneNode =
      dPtr->sceneNode->createChildSceneNode(this->GetName() + "_SONAR_CONE");
  dPtr->coneNode->attachObject(coneObj);
  dPtr->coneNode->setPosition(0, 0, 0);

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  this->SetCastShadows(false);
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
  if (this->GetScene()->GetSelectedVisual() &&
      this->GetRootVisual()->GetName() ==
      this->GetScene()->GetSelectedVisual()->GetName())
  {
    return;
  }

  float rangeDelta = dPtr->sonarMsg->sonar().range_max()
      - dPtr->sonarMsg->sonar().range_min();
  float radiusScale = dPtr->sonarMsg->sonar().radius()*2.0;

  if (!math::equal(dPtr->coneNode->getScale().z, rangeDelta) ||
      !math::equal(dPtr->coneNode->getScale().x, radiusScale))
  {
    dPtr->coneNode->setScale(radiusScale, radiusScale, rangeDelta);
    dPtr->sonarRay->SetPoint(0, math::Vector3(0, 0, rangeDelta * 0.5));
  }

  math::Pose pose = msgs::Convert(dPtr->sonarMsg->sonar().world_pose());
  this->SetPose(pose);

  if (dPtr->sonarMsg->sonar().has_contact())
  {
    math::Vector3 pos = msgs::Convert(dPtr->sonarMsg->sonar().contact());
    dPtr->sonarRay->SetPoint(1, pos);
  }
  else
  {
    dPtr->sonarRay->SetPoint(1, math::Vector3(0, 0,
          (rangeDelta * 0.5) - dPtr->sonarMsg->sonar().range()));
  }
  dPtr->receivedMsg = false;
}
