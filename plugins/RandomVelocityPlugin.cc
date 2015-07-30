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
#include <ignition/math/Rand.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include "RandomVelocityPluginPrivate.hh"
#include "RandomVelocityPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(RandomVelocityPlugin)

/////////////////////////////////////////////////
RandomVelocityPlugin::RandomVelocityPlugin()
  : dataPtr(new RandomVelocityPluginPrivate)
{
}

/////////////////////////////////////////////////
RandomVelocityPlugin::~RandomVelocityPlugin()
{
  delete this->dataPtr;
}

/////////////////////////////////////////////////
void RandomVelocityPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "Model pointer is null");

  // Make sure the link has been specified
  if (!_sdf->HasElement("link"))
  {
    gzerr << "<link> element missing from RandomVelocity plugin. "
      << "The plugin will not function.\n";
    return;
  }

  // Get the link;
  this->dataPtr->link = _model->GetLink(_sdf->Get<std::string>("link"));
  if (!this->dataPtr->link)
  {
    gzerr << "Unable to find link[" << _sdf->Get<std::string>("link") << "] "
      << "in model[" << _model->GetName() << "]. The RandomVelocity plugin "
      << "will not function.\n";
    return;
  }

  // Get x clamping values
  if (_sdf->HasElement("min_x"))
    this->dataPtr->xRange.X(_sdf->Get<double>("min_x"));
  if (_sdf->HasElement("max_x"))
    this->dataPtr->xRange.Y(_sdf->Get<double>("max_x"));

  // Make sure min <= max
  //
  // Analysis:
  //     min_x   max_x  | result_X  result_Y
  //    ----------------------------------
  // 0:   0       0     |   0         0
  // 1:   1       0     |   0         1
  // 2:   0       1     |   0         1
  //
  // Line 1 above is the error case, and can be handled different ways:
  //   a: Throw an exception
  //   b: Set both min and max values to the same value (either min_x or
  //   max_x)
  //   c: Fix it by swapping the values
  //   d: Do nothing, which would:
  //        i. First clamp the value to <= max_x
  //        ii. Second clamp the value to >= min_x
  //        iii. The result would always be the value of min_x
  //
  // We've opted for option c, which seems the most resonable.
  ignition::math::Vector2d tmp = this->dataPtr->xRange;
  this->dataPtr->xRange.X(std::min(tmp.X(), tmp.Y()));
  this->dataPtr->xRange.Y(std::max(tmp.X(), tmp.Y()));

  // Get y clamping values
  if (_sdf->HasElement("min_y"))
    this->dataPtr->yRange.X(_sdf->Get<double>("min_y"));
  if (_sdf->HasElement("max_y"))
    this->dataPtr->yRange.Y(_sdf->Get<double>("max_y"));

  // Make sure min <= max
  tmp = this->dataPtr->yRange;
  this->dataPtr->yRange.X(std::min(tmp.X(), tmp.Y()));
  this->dataPtr->yRange.Y(std::max(tmp.X(), tmp.Y()));

  // Get z clamping values
  if (_sdf->HasElement("min_z"))
    this->dataPtr->zRange.X(_sdf->Get<double>("min_z"));
  if (_sdf->HasElement("max_z"))
    this->dataPtr->zRange.Y(_sdf->Get<double>("max_z"));

  // Make sure min <= max
  tmp = this->dataPtr->yRange;
  this->dataPtr->zRange.X(std::min(tmp.X(), tmp.Y()));
  this->dataPtr->zRange.Y(std::max(tmp.X(), tmp.Y()));

  // Set the initial velocity, if present
  if (_sdf->HasElement("initial_velocity"))
  {
    this->dataPtr->velocity =
      _sdf->Get<ignition::math::Vector3d>("initial_velocity");
  }

  // Set the velocity factor
  if (_sdf->HasElement("velocity_factor"))
    this->dataPtr->velocityFactor = _sdf->Get<double>("velocity_factor");

  // Set the update period
  if (_sdf->HasElement("update_period"))
    this->dataPtr->updatePeriod = _sdf->Get<double>("update_period");

  // Connect to the world update signal
  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&RandomVelocityPlugin::Update, this, std::placeholders::_1));
}

/////////////////////////////////////////////////
void RandomVelocityPlugin::Update(const common::UpdateInfo &_info)
{
  GZ_ASSERT(this->dataPtr->link, "<link> in RandomVelocity plugin is null");

  // Short-circuit in case the link is invalid.
  if (!this->dataPtr->link)
    return;

  // Change direction when enough time has elapsed
  if (_info.simTime - this->dataPtr->prevUpdate > this->dataPtr->updatePeriod)
  {
    // Get a random velocity value.
    this->dataPtr->velocity.Set(
        ignition::math::Rand::DblUniform(-1, 1),
        ignition::math::Rand::DblUniform(-1, 1),
        ignition::math::Rand::DblUniform(-1, 1));

    // Apply scaling factor
    this->dataPtr->velocity.Normalize();
    this->dataPtr->velocity *= this->dataPtr->velocityFactor;

    // Clamp X value
    this->dataPtr->velocity.X(ignition::math::clamp(this->dataPtr->velocity.X(),
        this->dataPtr->xRange.X(), this->dataPtr->xRange.Y()));

    // Clamp Y value
    this->dataPtr->velocity.Y(ignition::math::clamp(this->dataPtr->velocity.Y(),
        this->dataPtr->yRange.X(), this->dataPtr->yRange.Y()));

    // Clamp Z value
    this->dataPtr->velocity.Z(ignition::math::clamp(this->dataPtr->velocity.Z(),
        this->dataPtr->zRange.X(), this->dataPtr->zRange.Y()));

    this->dataPtr->prevUpdate = _info.simTime;
  }

  // Apply velocity
  this->dataPtr->link->SetLinearVel(this->dataPtr->velocity);
}
