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

/////////////////////////////////////////////////
void PartData::OnApply()
{
  PartGeneralTab *generalTab = this->inspector->GetGeneral();
  this->gravity = generalTab->GetGravity();
  this->selfCollide = generalTab->GetSelfCollide();
  this->kinematic = generalTab->GetKinematic();

  // set inertial properties
  this->inertial->SetMass(generalTab->GetMass());
  this->inertial->SetCoG(generalTab->GetInertialPose());
  this->inertial->SetInertiaMatrix(
      generalTab->GetInertiaIXX(), generalTab->GetInertiaIYY(),
      generalTab->GetInertiaIZZ(), generalTab->GetInertiaIXY(),
      generalTab->GetInertiaIXZ(), generalTab->GetInertiaIYZ());
  this->pose = generalTab->GetPose();

  // set visual properties
  if (!this->visuals.empty())
  {
    this->partVisual->SetWorldPose(this->pose);

    PartVisualTab *visual = this->inspector->GetVisual();
    for (unsigned int i = 0; i < this->visuals.size(); ++i)
    {
      if (this->visuals[i]->GetMeshName() != visual->GetGeometry(i))
      {
        this->visuals[i]->DetachObjects();
        this->visuals[i]->AttachMesh(visual->GetGeometry(i));
        this->visuals[i]->SetMaterial(visual->GetMaterial(i));
      }
      this->visuals[i]->SetPose(visual->GetPose(i));
      this->visuals[i]->SetTransparency(visual->GetTransparency(i));
      if (this->visuals[i]->GetMaterialName() != visual->GetMaterial(i))
      {
        this->visuals[i]->SetMaterial(visual->GetMaterial(i));
      }
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
    this->visuals.push_back(visVisual);
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
