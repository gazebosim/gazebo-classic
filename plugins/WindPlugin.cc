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
#include "plugins/WindPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(WindPlugin)

/////////////////////////////////////////////////
WindPlugin::WindPlugin()
{
}

/////////////////////////////////////////////////
void WindPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/)
{
  GZ_ASSERT(_world, "WindPlugin world pointer is NULL");
  this->world = _world;

  physics::WindPtr wind = this->world->GetWind();

  wind->SetLinearVelFunc(std::bind(&WindPlugin::LinearVel, this,
        std::placeholders::_1, std::placeholders::_2));

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&WindPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
ignition::math::Vector3d WindPlugin::LinearVel(const physics::WindPtr &_wind,
                                               const physics::Entity *_entity)
{
  // Get wind linear velocity
  ignition::math::Vector3d wind = _wind->LinearVel();

  // Compute factor as a function of the distance from the origin
  double windFactor = sin(_entity->GetWorldPose().Ign().Pos().Length());

  // Apply factor on wind velocity
  wind.X(wind.X() * windFactor);
  wind.Y(wind.Y() * windFactor);

  return wind;
}

/////////////////////////////////////////////////
void WindPlugin::OnUpdate()
{
  // Get all the models
  physics::Model_V models = this->world->GetModels();

  // Process each model.
  for (auto const &model : models)
  {
    // Get all the links
    physics::Link_V links = model->GetLinks();

    // Process each link.
    for (auto const &link : links)
    {
      // Skip links for which the wind is disabled
      if (!link->GetWindMode())
        continue;

      // Add wind velocity as a force to the body
      link->AddRelativeForce(link->GetInertial()->GetMass() *
          (link->RelativeWindLinearVel() - link->GetRelativeLinearVel().Ign()));
    }
  }
}
