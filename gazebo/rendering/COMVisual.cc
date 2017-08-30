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
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/COMVisual.hh"
#include "gazebo/rendering/VisualPrivate.hh"

using namespace gazebo;
using namespace rendering;

namespace gazebo
{
  namespace rendering
  {
    /// \brief Private data for the COM Visual class
    class COMVisualPrivate : public VisualPrivate
    {
      /// \brief Lines that make the cross marking the center of mass.
      public: DynamicLines *crossLines;

      /// \brief Inertia pose in link frame.
      public: ignition::math::Pose3d inertiaPose;

      /// \brief Parent link name.
      public: std::string linkName;

      /// \brief Link mass.
      public: double mass;
    };
  }
}

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
          _elem->GetElement("inertial")->Get<ignition::math::Pose3d>("pose");
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

  dPtr->inertiaPose = msgs::ConvertIgn(_msg->inertial().pose());

  dPtr->mass = _msg->inertial().mass();
  dPtr->linkName = _msg->name();

  this->Load();
}

/////////////////////////////////////////////////
void COMVisual::Load()
{
  COMVisualPrivate *dPtr =
      reinterpret_cast<COMVisualPrivate *>(this->dataPtr);

  if (dPtr->mass <= 0)
  {
    // Unrealistic mass, load with default mass
    if (dPtr->mass < 0)
    {
      gzlog << "The link " << dPtr->linkName << " has unrealistic mass, "
            << "unable to visualize sphere of equivalent mass.\n";
    }
    else
    {
      gzlog << "The link " << dPtr->linkName << " is static or has mass of 0, "
            << "so a sphere of equivalent mass will not be shown.\n";
    }
    return;
  }

  // Compute radius of sphere with density of lead and equivalent mass.
  double sphereRadius;
  double dLead = 11340;
  sphereRadius = cbrt((0.75 * dPtr->mass) / (M_PI * dLead));

  // Get the link's bounding box
  VisualPtr vis = this->GetScene()->GetVisual(dPtr->linkName);
  ignition::math::Box box;

  if (vis)
    box = vis->BoundingBox();

  VisualPtr sphereVis(
      new Visual(this->Name()+"_SPHERE_", shared_from_this(), false));
  sphereVis->Load();

  // Mass indicator: equivalent sphere with density of lead
  sphereVis->InsertMesh("unit_sphere");
  sphereVis->AttachMesh("unit_sphere");

  sphereVis->SetScale(ignition::math::Vector3d(
      sphereRadius*2, sphereRadius*2, sphereRadius*2));
  sphereVis->SetPosition(dPtr->inertiaPose.Pos());
  sphereVis->SetRotation(dPtr->inertiaPose.Rot());

  Ogre::SceneNode *sphereNode = sphereVis->GetSceneNode();
  sphereNode->setInheritScale(false);

  sphereVis->SetMaterial("Gazebo/CoM");
  sphereVis->SetCastShadows(false);
  sphereVis->SetVisibilityFlags(GZ_VISIBILITY_GUI);

  // CoM position indicator
  ignition::math::Vector3d p1(0, 0,
      box.Min().Z() - dPtr->inertiaPose.Pos().Z());
  ignition::math::Vector3d p2(0, 0,
      box.Max().Z() - dPtr->inertiaPose.Pos().Z());

  ignition::math::Vector3d p3(0,
      box.Min().Y() - dPtr->inertiaPose.Pos().Y(), 0);
  ignition::math::Vector3d p4(0,
      box.Max().Y() - dPtr->inertiaPose.Pos().Y(), 0);

  ignition::math::Vector3d p5(
      box.Min().X() - dPtr->inertiaPose.Pos().X(), 0, 0);
  ignition::math::Vector3d p6(
      box.Max().X() - dPtr->inertiaPose.Pos().X(), 0, 0);

  p1 += dPtr->inertiaPose.Pos();
  p2 += dPtr->inertiaPose.Pos();
  p3 += dPtr->inertiaPose.Pos();
  p4 += dPtr->inertiaPose.Pos();
  p5 += dPtr->inertiaPose.Pos();
  p6 += dPtr->inertiaPose.Pos();

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
ignition::math::Pose3d COMVisual::InertiaPose() const
{
  COMVisualPrivate *dPtr =
      reinterpret_cast<COMVisualPrivate *>(this->dataPtr);

  return dPtr->inertiaPose;
}
