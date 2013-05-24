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
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->scene->GetName());

  this->wrenchSub = this->node->Subscribe(_topicName,
      &WrenchVisual::OnMsg, this, true);

  // Make sure the meshes are in Ogre
  this->InsertMesh("unit_cone");
  Ogre::MovableObject *coneObj =
    (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
          this->GetName()+"__WRENCH_CONE__", "unit_cone"));
  ((Ogre::Entity*)coneObj)->setMaterialName("Gazebo/BlueLaser");

  this->coneXNode =
    this->sceneNode->createChildSceneNode(this->GetName() + "_WRENCH_X_CONE");
  this->coneXNode->attachObject(coneObj);
  math::Quaternion q(math::Vector3(0, 1, 0), GZ_DTOR(-90));
  this->coneXNode->setOrientation(q.w, q.x, q.y, q.z);
  this->coneXNode->setScale(0.2, 0.2, 0.2);

  // q.Set(math::Vector3(1, 0, 0), GZ_DTOR(-90));
  // this->coneYNode->setOrientation(q.w, q.x, q.y, q.z);

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}

/////////////////////////////////////////////////
WrenchVisual::~WrenchVisual()
{
}

/////////////////////////////////////////////////
void WrenchVisual::OnMsg(ConstWrenchStampedPtr &_msg)
{
  // Skip the update if the user is moving the attached visual.
  if (this->GetScene()->GetSelectedVisual() &&
      this->GetRootVisual()->GetName() ==
      this->GetScene()->GetSelectedVisual()->GetName())
  {
    return;
  }

  printf("Got message\n");
}
