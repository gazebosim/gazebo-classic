/*
 * Copyright 2014 Open Source Robotics Foundation
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


#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Inertial.hh"

#include "gazebo/rendering/Scene.hh"

#include "gazebo/gui/model/PartVisualTab.hh"
#include "gazebo/gui/model/PartGeneralTab.hh"

#include "gazebo/gui/model/ModelData.hh"

using namespace gazebo;
using namespace gui;
/*
/////////////////////////////////////////////////
VisualData::VisualData()
{
  this->visualSDF.reset(new sdf::Element);
  sdf::initFile("visual.sdf", this->visualSDF);



}

/////////////////////////////////////////////////
void VisualData::SetName(const std::string &_name)
{
  this->visualSDF->GetElement("name")->Set(_name);
}


/////////////////////////////////////////////////
void VisualData::GetName(const std::string &_name)
{
  this->visualSDF->Get<std::string>("name")
}

/////////////////////////////////////////////////
void VisualData::SetMaterial(const std::string &_material)
{
  this->visualSDF->Get<std::string>("material")
}

/////////////////////////////////////////////////
void VisualData::SetMaterial(const std::string &_material)
{
  this->visualSDF->GetElement("material")-><std::string>("material")
}
*/


/////////////////////////////////////////////////
PartData::PartData()
{
  this->partSDF.reset(new sdf::Element);
  sdf::initFile("link.sdf", this->partSDF);
}

/////////////////////////////////////////////////
void PartData::OnApply()
{
  PartGeneralTab *generalTab = this->inspector->GetGeneral();

  this->partSDF->GetElement("pose")->Set(generalTab->GetPose());
  this->partSDF->GetElement("gravity")->Set(generalTab->GetGravity());
  this->partSDF->GetElement("self_collide")->Set(generalTab->GetSelfCollide());
  this->partSDF->GetElement("kinematic")->Set(generalTab->GetKinematic());

  sdf::ElementPtr inertialElem = this->partSDF->GetElement("inertial");
  inertialElem->GetElement("mass")->Set(generalTab->GetMass());
  inertialElem->GetElement("pose")->Set(generalTab->GetInertialPose());

  sdf::ElementPtr inertiaElem = inertialElem->GetElement("inertia");
  inertiaElem->GetElement("ixx")->Set(generalTab->GetInertiaIXX());
  inertiaElem->GetElement("iyy")->Set(generalTab->GetInertiaIYY());
  inertiaElem->GetElement("izz")->Set(generalTab->GetInertiaIZZ());
  inertiaElem->GetElement("ixy")->Set(generalTab->GetInertiaIXY());
  inertiaElem->GetElement("ixz")->Set(generalTab->GetInertiaIXZ());
  inertiaElem->GetElement("iyz")->Set(generalTab->GetInertiaIYZ());

  // set visual properties
  if (!this->visuals.empty())
  {
    this->partVisual->SetWorldPose(this->GetPose());

    PartVisualTab *visual = this->inspector->GetVisual();
    for (unsigned int i = 0; i < this->visuals.size(); ++i)
    {
      if (this->visuals[i]->GetMeshName() != visual->GetGeometry(i))
      {
        this->visuals[i]->DetachObjects();
        this->visuals[i]->AttachMesh(visual->GetGeometry(i));
      }
      if (this->visuals[i]->GetMaterialName() != visual->GetMaterial(i))
      {
        this->visuals[i]->SetMaterial(visual->GetMaterial(i), false);
      }

      this->visuals[i]->SetPose(visual->GetPose(i));
      this->visuals[i]->SetTransparency(visual->GetTransparency(i));
      this->visuals[i]->SetScale(visual->GetGeometryScale(i));
    }
  }
}

/////////////////////////////////////////////////
void PartData::OnAddVisual()
{
  // add a visual when the user adds a visual via the inspector's visual tab
  PartVisualTab *visualTab = this->inspector->GetVisual();
  if (this->visuals.size() != visualTab->GetVisualCount())
  {
    std::ostringstream visualName;
    visualName << this->partVisual->GetName() << "_visual_"
        << this->partVisual->GetChildCount();

    visualTab->SetName(visualTab->GetVisualCount()-1, visualName.str());

    // add a box for now
    rendering::VisualPtr visVisual(new rendering::Visual(visualName.str(),
        this->partVisual));
    visVisual->Load();
    this->partVisual->GetScene()->AddVisual(visVisual);
    visVisual->AttachMesh("unit_box");
    visVisual->SetMaterial("Gazebo/GreyTransparent");

    /*VisualData *visualData = new VisualData();
    visualData->SetName(visualName);
    visualData->SetPose(math::Pose::Zero);
    visualData->SetGeometry("box");
    visualData->SetMaterial("Gazebo/GreyTransparent");
    visualData->SetVisual(visVisual);
    this->visuals[visualName] = visualData;*/
  }
}

/////////////////////////////////////////////////
void PartData::OnRemoveVisual(const std::string &_name)
{
  // find and remove visual when the user removes it in the
  // inspector's visual tab
  for (unsigned int i = 0; i < this->visuals.size(); ++i)
  {
    if (_name == this->visuals[i]->GetName())
    {
      this->partVisual->DetachVisual(this->visuals[i]);
      this->partVisual->GetScene()->RemoveVisual(this->visuals[i]);
      this->visuals.erase(this->visuals.begin()+i);
      break;
    }
  }
}

/////////////////////////////////////////////////
std::string PartData::GetName() const
{
  return this->partSDF->Get<std::string>("name");
}

/////////////////////////////////////////////////
void PartData::SetName(const std::string &_name)
{
  this->partSDF->GetAttribute("name")->Set(_name);
}

/////////////////////////////////////////////////
bool PartData::GetGravity() const
{
  return this->partSDF->Get<bool>("gravity");
}

/////////////////////////////////////////////////
void PartData::SetGravity(bool _gravity)
{
  this->partSDF->GetElement("gravity")->Set(_gravity);
}

/////////////////////////////////////////////////
bool PartData::GetSelfCollide() const
{
  return this->partSDF->Get<bool>("self_collide");
}

/////////////////////////////////////////////////
void PartData::SetSelfCollide(bool _selfCollide)
{
  this->partSDF->GetElement("self_collide")->Set(_selfCollide);
}

/////////////////////////////////////////////////
bool PartData::GetKinematic() const
{
  return this->partSDF->Get<bool>("kinematic");
}

/////////////////////////////////////////////////
void PartData::SetKinematic(bool _kinematic)
{
  this->partSDF->GetElement("kinematic")->Set(_kinematic);
}

/////////////////////////////////////////////////
math::Pose PartData::GetPose() const
{
  return this->partSDF->Get<math::Pose>("pose");
}

/////////////////////////////////////////////////
void PartData::SetPose(const math::Pose &_pose)
{
  this->partSDF->GetElement("pose")->Set(_pose);
}

/////////////////////////////////////////////////
double PartData::GetMass() const
{
  return this->partSDF->GetElement("inertial")->Get<double>("mass");
}

/////////////////////////////////////////////////
void PartData::SetMass(double _mass)
{
  this->partSDF->GetElement("inertial")->GetElement("mass")->Set(_mass);
}


/////////////////////////////////////////////////
math::Pose PartData::GetInertialPose() const
{
  return this->partSDF->GetElement("inertial")->Get<math::Pose>("pose");
}

/////////////////////////////////////////////////
void PartData::SetInertialPose(const math::Pose &_pose)
{
  this->partSDF->GetElement("inertial")->GetElement("pose")->Set(_pose);
}

/////////////////////////////////////////////////
double PartData::GetInertiaIXX() const
{
  return this->partSDF->GetElement("inertial")->GetElement("inertia")
      ->Get<double>("ixx");
}

/////////////////////////////////////////////////
void PartData::SetInertiaIXX(double _ixx)
{
  this->partSDF->GetElement("inertial")->GetElement("inertia")
      ->GetElement("ixx")->Set(_ixx);
}

/////////////////////////////////////////////////
double PartData::GetInertiaIYY() const
{
  return this->partSDF->GetElement("inertial")->GetElement("inertia")
      ->Get<double>("iyy");
}

/////////////////////////////////////////////////
void PartData::SetInertiaIYY(double _iyy)
{
  this->partSDF->GetElement("inertial")->GetElement("inertia")
      ->GetElement("iyy")->Set(_iyy);
}

/////////////////////////////////////////////////
double PartData::GetInertiaIZZ() const
{
  return this->partSDF->GetElement("inertial")->GetElement("inertia")
      ->Get<double>("izz");
}

/////////////////////////////////////////////////
void PartData::SetInertiaIZZ(double _izz)
{
  this->partSDF->GetElement("inertial")->GetElement("inertia")
      ->GetElement("izz")->Set(_izz);
}

/////////////////////////////////////////////////
double PartData::GetInertiaIXY() const
{
  return this->partSDF->GetElement("inertial")->GetElement("inertia")
      ->Get<double>("ixy");
}

/////////////////////////////////////////////////
void PartData::SetInertiaIXY(double _ixy)
{
  this->partSDF->GetElement("inertial")->GetElement("inertia")
      ->GetElement("ixy")->Set(_ixy);
}

/////////////////////////////////////////////////
double PartData::GetInertiaIXZ() const
{
  return this->partSDF->GetElement("inertial")->GetElement("inertia")
      ->Get<double>("ixz");
}

/////////////////////////////////////////////////
void PartData::SetInertiaIXZ(double _ixz)
{
  this->partSDF->GetElement("inertial")->GetElement("inertia")
      ->GetElement("ixz")->Set(_ixz);
}

/////////////////////////////////////////////////
double PartData::GetInertiaIYZ() const
{
  return this->partSDF->GetElement("inertial")->GetElement("inertia")
      ->Get<double>("iyz");
}

/////////////////////////////////////////////////
void PartData::SetInertiaIYZ(double _iyz)
{
  this->partSDF->GetElement("inertial")->GetElement("inertia")
      ->GetElement("iyz")->Set(_iyz);
}
