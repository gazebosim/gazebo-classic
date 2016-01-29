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
  : characteristicTimeForWindRise(1)
  , magnitudeSinAmplitudePercent(0)
  , magnitudeSinPeriod(1)
  , characteristicTimeForWindOrientationChange(1)
  , orientationSinAmplitude(0)
  , orientationSinPeriod(1)
  , verticalNoiseAmplitudePercent(0)
  , KMag(0)
  , KDir(0)
  , magnitudeMean(0)
  , directionMean(0)
{
}

/////////////////////////////////////////////////
void WindPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "WindPlugin world pointer is NULL");
  this->world = _world;

  physics::WindPtr wind = this->world->Wind();

  if(_sdf->HasElement("horizontal"))
  {
    sdf::ElementPtr sdfHoriz = _sdf->GetElement("horizontal");

    if(sdfHoriz->HasElement("magnitude"))
    {
      sdf::ElementPtr sdfMag = sdfHoriz->GetElement("magnitude");

      if(sdfMag->HasElement("time_for_rise"))
        this->characteristicTimeForWindRise =
          sdfMag->Get<double>("time_for_rise");

      if(sdfMag->HasElement("sin"))
      {
        sdf::ElementPtr sdfMagSin = sdfMag->GetElement("sin");

        if(sdfMagSin->HasElement("amplitude_percent"))
          this->magnitudeSinAmplitudePercent =
            sdfMagSin->Get<double>("amplitude_percent");

        if(sdfMagSin->HasElement("period"))
          this->magnitudeSinPeriod = sdfMagSin->Get<double>("period");
      }

      if(sdfMag->HasElement("noise"))
      {
        this->noiseMagnitude =
          sensors::NoiseFactory::NewNoiseModel(
              sdfMag->GetElement("noise"));
      }
    }

    if(sdfHoriz->HasElement("direction"))
    {
      sdf::ElementPtr sdfDir = sdfHoriz->GetElement("direction");

      if(sdfDir->HasElement("time_for_rise"))
        this->characteristicTimeForWindOrientationChange =
          sdfDir->Get<double>("time_for_rise");

      if(sdfDir->HasElement("sin"))
      {
        sdf::ElementPtr sdfDirSin = sdfDir->GetElement("sin");

        if(sdfDirSin->HasElement("amplitude"))
          this->orientationSinAmplitude =
            sdfDirSin->Get<double>("amplitude");

        if(sdfDirSin->HasElement("period"))
          this->orientationSinPeriod =
            sdfDirSin->Get<double>("period");
      }

      if(sdfDir->HasElement("noise"))
      {
        this->noiseDirection =
          sensors::NoiseFactory::NewNoiseModel(
              sdfDir->GetElement("noise"));
      }
    }
  }

  if(_sdf->HasElement("vertical"))
  {
    sdf::ElementPtr sdfVert = _sdf->GetElement("vertical");

    if(sdfVert->HasElement("noise"))
    {
      this->noiseVertical =
        sensors::NoiseFactory::NewNoiseModel(
            sdfVert->GetElement("noise"));
    }
  }

  double period = this->world->GetPhysicsEngine()->GetMaxStepSize();

  this->KMag = period / this->characteristicTimeForWindRise;
  this->KDir = period / this->characteristicTimeForWindOrientationChange;

  wind->SetLinearVelFunc(std::bind(&WindPlugin::LinearVel, this,
        std::placeholders::_1, std::placeholders::_2));

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&WindPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
ignition::math::Vector3d WindPlugin::LinearVel(
    std::shared_ptr<const physics::Wind> &_wind,
    const physics::Entity * /*_entity*/)
{
  // Compute magnitude
  this->magnitudeMean = (1. - this->KMag) * this->magnitudeMean +
      this->KMag * _wind->LinearVel().Length();
  double magnitude = this->magnitudeMean;
  magnitude += this->magnitudeSinAmplitudePercent * this->magnitudeMean *
    sin(2 * M_PI * this->world->GetSimTime().Double() /
        this->magnitudeSinPeriod);
  if (this->noiseMagnitude)
    magnitude = this->noiseMagnitude->Apply(magnitude);

  // Compute horizontal direction
  double direction = atan2(_wind->LinearVel().Y(), _wind->LinearVel().X());
  this->directionMean = (1. - this->KDir) * this->directionMean +
    this->KDir * direction;
  direction = this->directionMean;
  direction += this->orientationSinAmplitude *
    sin(2 * M_PI * this->world->GetSimTime().Double() /
        this->orientationSinPeriod);
  if (this->noiseDirection)
    direction = this->noiseDirection->Apply(direction);

  // Apply wind velocity
  ignition::math::Vector3d windVel;
  windVel.X(magnitude * cos(direction * M_PI / 180.));
  windVel.Y(magnitude * sin(direction * M_PI / 180.));
  if (this->noiseVertical)
    windVel.Z(noiseVertical->Apply(this->magnitudeMean));
  else
    windVel.Z(this->magnitudeMean);

  return windVel;
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
      if (!link->WindMode())
        continue;

      // Add wind velocity as a force to the body
      link->AddRelativeForce(link->GetInertial()->GetMass() *
          (link->RelativeWindLinearVel() - link->GetRelativeLinearVel().Ign()));
    }
  }
}
