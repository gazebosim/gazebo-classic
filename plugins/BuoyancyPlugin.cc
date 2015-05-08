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
  GZ_ASSERT(_model != NULL, "Received NULL model pointer");
  this->model = _model;
  physics::WorldPtr world = _model->GetWorld();
  GZ_ASSERT(world != NULL, "Model is in a NULL world");
  this->physicsEngine = world->GetPhysicsEngine();
  GZ_ASSERT(this->physicsEngine != NULL, "Physics engine was NULL");

  GZ_ASSERT(_sdf != NULL, "Received NULL SDF pointer");
  this->sdf = _sdf;

  if (this->sdf->HasElement("fluid_density"))
  {
    this->fluidDensity = this->sdf->Get<double>("fluid_density");
  }

  // Get "center of volume" and "volume" that were inputted in SDF
  // SDF input is recommended for mesh or polylines collision shapes
  if (this->sdf->HasElement("link"))
  {
    for (sdf::ElementPtr linkElem = this->sdf->GetElement("link"); linkElem;
         linkElem = this->sdf->GetNextElement("link"))
    {
      int id = -1;
      if (linkElem->HasAttribute("name"))
      {
        std::string name = linkElem->Get<std::string>("name");
        physics::LinkPtr link =
            this->model->GetLink(name);
        if (!link)
        {
          gzwarn << "Specified link [" << name << "] not found." << std::endl;
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
      if (linkElem->HasElement("volume"))
      {
        double volume =
            linkElem->GetElement("volume")->Get<double>();
        this->volPropsMap[id].volume = volume;
      }
    }
  }

  // For links the user didn't input, precompute the center of volume and
  // density. This will be accurate for simple shapes.
  for (auto link : this->model->GetLinks())
  {
    int id = link->GetId();
    if (this->volPropsMap.find(id) == this->volPropsMap.end())
    {
      double volumeSum = 0;
      math::Vector3 weightedPosSum = math::Vector3::Zero;

      // The center of volume of the link is a weighted average over the pose
      // of each collision shape, where the weight is the volume of the shape
      for (auto collision : link->GetCollisions())
      {
        double volume = collision->GetShape()->ComputeVolume();
        volumeSum += volume;
        weightedPosSum += volume*collision->GetWorldPose().pos;
      }
      // Subtract the center of volume into the link frame.
      this->volPropsMap[id].cov =
          weightedPosSum/volumeSum - link->GetWorldPose().pos;
      this->volPropsMap[id].volume = volumeSum;
    }
  }
}

/////////////////////////////////////////////////
void BuoyancyPlugin::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&BuoyancyPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void BuoyancyPlugin::OnUpdate()
{
  for (auto link : this->model->GetLinks())
  {
    VolumeProperties volumeProperties = this->volPropsMap[link->GetId()];
    double volume = volumeProperties.volume;

    // By Archimedes' principle,
    // buoyancy = -(mass*gravity)*fluid_density/object_density
    // object_density = mass/volume, so the mass term cancels.
    // Therefore,
    math::Vector3 buoyancy =
        -this->fluidDensity * volume * this->physicsEngine->GetGravity();

    math::Pose linkFrame = link->GetWorldPose();
    // rotate buoyancy into the link frame before applying the force.
    math::Vector3 buoyancyLinkFrame =
        linkFrame.rot.GetInverse().RotateVector(buoyancy);

    link->AddLinkForce(buoyancyLinkFrame, volumeProperties.cov);
  }
}
