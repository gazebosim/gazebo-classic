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
  COMVisualPrivate *dPtr =
      reinterpret_cast<COMVisualPrivate *>(this->dataPtr);
  dPtr->type = VT_PHYSICS;
}

/////////////////////////////////////////////////
COMVisual::~COMVisual()
{
}

/////////////////////////////////////////////////
void COMVisual::Load(sdf::ElementPtr _elem)
{
  Visual::Load();

  COMVisualPrivate *dPtr =
      reinterpret_cast<COMVisualPrivate *>(this->dataPtr);

  if (_elem->HasAttribute("name"))
    dPtr->linkName = _elem->Get<std::string>("name");

  if (_elem->HasElement("inertial"))
  {
    if (_elem->GetElement("inertial")->HasElement("pose"))
    {
      dPtr->inertiaPose =
          _elem->GetElement("inertial")->Get<math::Pose>("pose");
    }
    else if (_elem->GetElement("inertial")->HasElement("mass"))
    {
      dPtr->mass =
          _elem->GetElement("inertial")->Get<double>("mass");
    }
  }

  this->Load();
}

/////////////////////////////////////////////////
void COMVisual::Load(ConstLinkPtr &_msg)
{
  Visual::Load();

  COMVisualPrivate *dPtr =
      reinterpret_cast<COMVisualPrivate *>(this->dataPtr);

  math::Vector3 xyz(_msg->inertial().pose().position().x(),
                    _msg->inertial().pose().position().y(),
                    _msg->inertial().pose().position().z());
  math::Quaternion q(_msg->inertial().pose().orientation().w(),
                     _msg->inertial().pose().orientation().x(),
                     _msg->inertial().pose().orientation().y(),
                     _msg->inertial().pose().orientation().z());

  dPtr->inertiaPose = math::Pose(xyz, q);

  dPtr->mass = _msg->inertial().mass();
  dPtr->linkName = _msg->name();

  this->Load();
}

/////////////////////////////////////////////////
void COMVisual::Load()
{
  COMVisualPrivate *dPtr =
      reinterpret_cast<COMVisualPrivate *>(this->dataPtr);

  if (dPtr->mass < 0)
  {
    // Unrealistic mass, load with default mass
    gzlog << "The link " << dPtr->linkName << " has unrealistic mass, "
          << "unable to visualize sphere of equivalent mass." << std::endl;
    dPtr->mass = 1;
  }

  // Compute radius of sphere with density of lead and equivalent mass.
  double sphereRadius;
  double dLead = 11340;
  sphereRadius = cbrt((0.75 * dPtr->mass) / (M_PI * dLead));

  // Get the link's bounding box
  VisualPtr vis = this->GetScene()->GetVisual(dPtr->linkName);
  math::Box box;

  if (vis)
    box = vis->GetBoundingBox();

  VisualPtr sphereVis(
      new Visual(this->GetName()+"_SPHERE_", shared_from_this(), false));
  sphereVis->Load();

  // Mass indicator: equivalent sphere with density of lead
  sphereVis->InsertMesh("unit_sphere");
  sphereVis->AttachMesh("unit_sphere");

  sphereVis->SetScale(ignition::math::Vector3d(
      sphereRadius*2, sphereRadius*2, sphereRadius*2));
  sphereVis->SetPosition(dPtr->inertiaPose.pos.Ign());
  sphereVis->SetRotation(dPtr->inertiaPose.rot.Ign());

  Ogre::SceneNode *sphereNode = sphereVis->GetSceneNode();
  sphereNode->setInheritScale(false);

  sphereVis->SetMaterial("Gazebo/CoM");
  sphereVis->SetCastShadows(false);
  sphereVis->SetVisibilityFlags(GZ_VISIBILITY_GUI);

  // CoM position indicator
  ignition::math::Vector3d p1(0, 0, box.min.z - dPtr->inertiaPose.pos.z);
  ignition::math::Vector3d p2(0, 0, box.max.z - dPtr->inertiaPose.pos.z);
  ignition::math::Vector3d p3(0, box.min.y - dPtr->inertiaPose.pos.y, 0);
  ignition::math::Vector3d p4(0, box.max.y - dPtr->inertiaPose.pos.y, 0);
  ignition::math::Vector3d p5(box.min.x - dPtr->inertiaPose.pos.x, 0, 0);
  ignition::math::Vector3d p6(box.max.x - dPtr->inertiaPose.pos.x, 0, 0);
  p1 += dPtr->inertiaPose.pos.Ign();
  p2 += dPtr->inertiaPose.pos.Ign();
  p3 += dPtr->inertiaPose.pos.Ign();
  p4 += dPtr->inertiaPose.pos.Ign();
  p5 += dPtr->inertiaPose.pos.Ign();
  p6 += dPtr->inertiaPose.pos.Ign();

  dPtr->crossLines = this->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  dPtr->crossLines->setMaterial("Gazebo/Green");
  dPtr->crossLines->AddPoint(p1);
  dPtr->crossLines->AddPoint(p2);
  dPtr->crossLines->AddPoint(p3);
  dPtr->crossLines->AddPoint(p4);
  dPtr->crossLines->AddPoint(p5);
  dPtr->crossLines->AddPoint(p6);

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}

/////////////////////////////////////////////////
math::Pose COMVisual::GetInertiaPose() const
{
  COMVisualPrivate *dPtr =
      reinterpret_cast<COMVisualPrivate *>(this->dataPtr);

  return dPtr->inertiaPose;
}
