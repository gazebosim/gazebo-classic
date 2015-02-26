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
/* Desc: Center of Mass Visualization Class
 * Author: Nate Koenig
 */

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Quaternion.hh"
#include "gazebo/math/Pose.hh"

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/COMVisualPrivate.hh"
#include "gazebo/rendering/COMVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
COMVisual::COMVisual(const std::string &_name, VisualPtr _vis)
  : Visual(*new COMVisualPrivate, _name, _vis, false)
{
}

/////////////////////////////////////////////////
COMVisual::~COMVisual()
{
}

/////////////////////////////////////////////////
void COMVisual::Load(sdf::ElementPtr _elem)
{
  Visual::Load();
  math::Pose pose = _elem->Get<math::Pose>("origin");
  this->Load(pose);
}

/////////////////////////////////////////////////
void COMVisual::Load(ConstLinkPtr &_msg)
{
  Visual::Load();

  math::Vector3 xyz(_msg->inertial().pose().position().x(),
                    _msg->inertial().pose().position().y(),
                    _msg->inertial().pose().position().z());
  math::Quaternion q(_msg->inertial().pose().orientation().w(),
                     _msg->inertial().pose().orientation().x(),
                     _msg->inertial().pose().orientation().y(),
                     _msg->inertial().pose().orientation().z());

  double mass = _msg->inertial().mass();
  if (mass < 0)
  {
    // Unrealistic mass, load with default mass
    gzlog << "The link " << _msg->name() << " has unrealistic mass, "
          << "unable to visualize sphere of equivalent mass." << std::endl;
    this->Load(math::Pose(xyz, q));
  }
  else
  {
    // Compute radius of sphere with density of lead and equivalent mass.
    double sphereRadius;
    double dLead = 11340;
    sphereRadius = cbrt((0.75 * mass) / (M_PI * dLead));

    // Get the link's bounding box
    std::string name = _msg->name();
    VisualPtr vis = this->GetScene()->GetVisual(name);
    math::Box box;

    if (vis)
      box = vis->GetBoundingBox();

    this->Load(math::Pose(xyz, q), sphereRadius, box);
  }
}

/////////////////////////////////////////////////
void COMVisual::Load(const math::Pose &_pose, double _radius, math::Box _box)
{
  COMVisualPrivate *dPtr =
      reinterpret_cast<COMVisualPrivate *>(this->dataPtr);

  // CoM position indicator
  math::Vector3 p1(0, 0, _box.min.z - _pose.pos.z);
  math::Vector3 p2(0, 0, _box.max.z - _pose.pos.z);
  math::Vector3 p3(0, _box.min.y - _pose.pos.y, 0);
  math::Vector3 p4(0, _box.max.y - _pose.pos.y, 0);
  math::Vector3 p5(_box.min.x - _pose.pos.x, 0, 0);
  math::Vector3 p6(_box.max.x - _pose.pos.x, 0, 0);
  p1 += _pose.pos;
  p2 += _pose.pos;
  p3 += _pose.pos;
  p4 += _pose.pos;
  p5 += _pose.pos;
  p6 += _pose.pos;

  dPtr->crossLines = this->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  dPtr->crossLines->setMaterial("Gazebo/Green");
  dPtr->crossLines->AddPoint(p1);
  dPtr->crossLines->AddPoint(p2);
  dPtr->crossLines->AddPoint(p3);
  dPtr->crossLines->AddPoint(p4);
  dPtr->crossLines->AddPoint(p5);
  dPtr->crossLines->AddPoint(p6);

  // Mass indicator: equivalent sphere with density of lead
  this->InsertMesh("unit_sphere");

  Ogre::MovableObject *sphereObj =
    (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
          this->GetName()+"__SPHERE__", "unit_sphere"));
  sphereObj->setVisibilityFlags(GZ_VISIBILITY_GUI);
  ((Ogre::Entity*)sphereObj)->setMaterialName("Gazebo/CoM");
  sphereObj->setCastShadows(false);

  dPtr->sphereNode =
      dPtr->sceneNode->createChildSceneNode(this->GetName() + "_SPHERE");

  dPtr->sphereNode->attachObject(sphereObj);
  dPtr->sphereNode->setScale(_radius*2, _radius*2, _radius*2);
  dPtr->sphereNode->setPosition(_pose.pos.x, _pose.pos.y, _pose.pos.z);
  dPtr->sphereNode->setOrientation(Ogre::Quaternion(_pose.rot.w, _pose.rot.x,
                                                    _pose.rot.y, _pose.rot.z));
  dPtr->sphereNode->setInheritScale(false);

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}
