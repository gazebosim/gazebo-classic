/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gazebo/common/Events.hh"
#include "plugins/BuoyancyPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(BuoyancyPlugin)

/////////////////////////////////////////////////
BuoyancyPlugin::BuoyancyPlugin() : fluidDensity(1000.0)
{

}

/////////////////////////////////////////////////
void BuoyancyPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model != NULL, "Received NULL model pointer" << std::endl;
  GZ_ASSERT(_sdf != NULL, "Received NULL SDF pointer" << std::endl;
  this->model = _model;
  this->sdf = _sdf;
  if (this->sdf->HasElement("fluid_density"))
  {
    this->fluidDensity = this->sdf->Get<double>("fluid_density");
  }

  // Get center of volume and density that were inputted in SDF
  for (ElementPtr linkElem = this->sdf->GetElement("link"); linkElem;
       linkElem = this->sdf->GetNextElement("link"))
  {
    int id = -1;
    if (linkElem->HasAttribute("name")
    {
      physics::LinkPtr link = this->model->GetLink(linkElem->GetAttribute("name"));
      if (!link)
      {
        continue;
      }
      id = link->GetId();
    }
    if (linkElem->HasElement("center_of_volume"))
    {
      math::Vector3 cov =
          linkElem->GetElement("center_of_volume")->Get<math::Vector3>();
      this->volPropsMap[id].cov = cov;
    }
    if (linkElem->HasElement("density"))
    {
      math::Vector3 density =
          linkElem->GetElement("density")->Get<math::Vector3>();
      this->volPropsMap[id].density = density;
    }
  }

  // For links the user didn't input, precompute the center of volume and
  // density of each link by approximating the collision volumes with
  // bounding boxes.

  for (auto link : this->model->GetLinks())
  {
    int id = link->GetId();
    if (this->volPropsMap.find(id) == this->volPropsMap.end())
    {
      double density = 0;
      for (auto collision : link->GetCollisions)
      {
        // density += collision->GetBoundingBox()
        switch (collision->GetShapeType())
        {
        }
      }
      this->volPropsMap[id].cov = ;
      this->volPropsMap[id].density = density;
    }
  }

  this->updateConnection = event::ConnectWorldUpdateBegin(
      std::bind(&BuoyancyPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
virtual void BuoyancyPlugin::Init()
{
}

/////////////////////////////////////////////////
virtual void BuoyancyPlugin::OnUpdate()
{
}
