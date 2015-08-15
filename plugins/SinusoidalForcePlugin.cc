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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Joint.hh"
#include "plugins/SinusoidalForcePlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(SinusoidalForcePlugin)

/////////////////////////////////////////////////
void SinusoidalForcePlugin::Load(physics::ModelPtr _model,
    sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "_model pointer is NULL");
  this->model = _model;
  this->sdf = _sdf;
  this->world = _model->GetWorld();
  GZ_ASSERT(world != NULL, "Model is in a NULL world");

  // Parse input parameters
  if (this->sdf->HasElement("joint"))
  {
    for (auto jointElem = this->sdf->GetElement("joint"); jointElem;
        jointElem = jointElem->GetNextElement("joint"))
    {
      std::string name;
      if (jointElem->HasAttribute("name"))
        name = jointElem->Get<std::string>("name");

      physics::JointPtr joint = this->model->GetJoint(name);
      if (name == "" || !joint)
      {
        gzerr << "Invalid joint [" << name << "]" << std::endl;
        continue;
      }

      double min = 0;
      if (jointElem->HasElement("min"))
        min = jointElem->Get<double>("min");

      double max = 1000;
      if (jointElem->HasElement("max"))
        max = jointElem->Get<double>("max");

      if (min == max)
      {
        gzerr << "Minimum and maximum can't be the same" << std::endl;
        continue;
      }

      double period = 1;
      if (jointElem->HasElement("period"))
        period = jointElem->Get<double>("period");

      if (period <= 0)
      {
        gzerr << "Invalid period [" << period << "] for joint [" << name <<
            "]" << std::endl;
        continue;
      }

      ForceData data;
      data.joint = joint;
      data.average = (min + max) * 0.5;
      data.amplitude = (max - min) * 0.5;
      data.period = period;
      this->dataList.push_back(data);
      gzmsg << "Registered joint [" << name << "], min [" << min << "], max ["
            << max << "], period [" << period << "]" << std::endl;
    }
  }
}

/////////////////////////////////////////////////
void SinusoidalForcePlugin::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&SinusoidalForcePlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void SinusoidalForcePlugin::OnUpdate()
{
  for (auto data : this->dataList)
  {
    double t = this->world->GetSimTime().Double();

    double force = data.average +
        data.amplitude * sin (2 * IGN_PI * t / data.period);

    data.joint->SetForce(0, force);
  }
}

