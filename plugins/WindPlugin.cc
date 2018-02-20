/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <functional>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/sensors/Noise.hh"

#include "plugins/WindPlugin.hh"

/// \brief Private class for WindPlugin
class gazebo::WindPluginPrivate
{
  /// \brief World pointer.
  public: physics::WorldPtr world;

  /// \brief Connection to World Update events.
  public: event::ConnectionPtr updateConnection;

  /// \brief Time for wind to rise
  public: double characteristicTimeForWindRise = 1;

  /// \brief Wind amplitude
  public: double magnitudeSinAmplitudePercent = 0;

  /// \brief Wind period
  public: double magnitudeSinPeriod = 1;

  /// \brief Time for wind to change direction.
  public: double characteristicTimeForWindOrientationChange = 1;

  /// \brief Orientation amplitude
  public: double orientationSinAmplitude = 0;

  /// \brief Orientation period
  public: double orientationSinPeriod = 1;

  /// \brief period over characteristicTimeForWindRise
  public: double kMag = 0;

  /// \brief period over characteristicTimeForWindOrientationChange
  public: double kDir = 0;

  /// \brief Mean of the magnitude
  public: double magnitudeMean = 0;

  /// \brief Mean of the direction
  public: double directionMean = 0;

  /// \brief Noise added to magnitude
  public: sensors::NoisePtr noiseMagnitude;

  /// \brief Noise added to direction
  public: sensors::NoisePtr noiseDirection;

  /// \brief Noise added to Z axis
  public: sensors::NoisePtr noiseVertical;

  /// \brief Time for wind to rise
  public: double characteristicTimeForWindRiseVertical = 1;

  /// \brief period over characteristicTimeForWindRiseVertical
  public: double kMagVertical = 0;

  /// \brief Mean of the magnitude
  public: double magnitudeMeanVertical = 0;

  /// \brief The scaling factor to approximate wind as force on a mass.
  public: double forceApproximationScalingFactor = 0;
};


using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(WindPlugin)

/////////////////////////////////////////////////
WindPlugin::WindPlugin()
    : dataPtr(new WindPluginPrivate)
{
}

/////////////////////////////////////////////////
void WindPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world, "WindPlugin world pointer is NULL");
  this->dataPtr->world = _world;

  physics::Wind &wind = this->dataPtr->world->Wind();

  if (_sdf->HasElement("horizontal"))
  {
    sdf::ElementPtr sdfHoriz = _sdf->GetElement("horizontal");

    if (sdfHoriz->HasElement("magnitude"))
    {
      sdf::ElementPtr sdfMag = sdfHoriz->GetElement("magnitude");

      if (sdfMag->HasElement("time_for_rise"))
      {
        this->dataPtr->characteristicTimeForWindRise =
          sdfMag->Get<double>("time_for_rise");
      }

      if (sdfMag->HasElement("sin"))
      {
        sdf::ElementPtr sdfMagSin = sdfMag->GetElement("sin");

        if (sdfMagSin->HasElement("amplitude_percent"))
        {
          this->dataPtr->magnitudeSinAmplitudePercent =
            sdfMagSin->Get<double>("amplitude_percent");
        }

        if (sdfMagSin->HasElement("period"))
        {
          this->dataPtr->magnitudeSinPeriod = sdfMagSin->Get<double>("period");
        }
      }

      if (sdfMag->HasElement("noise"))
      {
        this->dataPtr->noiseMagnitude = sensors::NoiseFactory::NewNoiseModel(
              sdfMag->GetElement("noise"));
      }
    }

    if (sdfHoriz->HasElement("direction"))
    {
      sdf::ElementPtr sdfDir = sdfHoriz->GetElement("direction");

      if (sdfDir->HasElement("time_for_rise"))
      {
        this->dataPtr->characteristicTimeForWindOrientationChange =
          sdfDir->Get<double>("time_for_rise");
      }

      if (sdfDir->HasElement("sin"))
      {
        sdf::ElementPtr sdfDirSin = sdfDir->GetElement("sin");

        if (sdfDirSin->HasElement("amplitude"))
        {
          this->dataPtr->orientationSinAmplitude =
            sdfDirSin->Get<double>("amplitude");
        }

        if (sdfDirSin->HasElement("period"))
        {
          this->dataPtr->orientationSinPeriod =
            sdfDirSin->Get<double>("period");
        }
      }

      if (sdfDir->HasElement("noise"))
      {
        this->dataPtr->noiseDirection = sensors::NoiseFactory::NewNoiseModel(
            sdfDir->GetElement("noise"));
      }
    }
  }

  if (_sdf->HasElement("vertical"))
  {
    sdf::ElementPtr sdfVert = _sdf->GetElement("vertical");

    if (sdfVert->HasElement("time_for_rise"))
    {
      this->dataPtr->characteristicTimeForWindRiseVertical =
        sdfVert->Get<double>("time_for_rise");
    }

    if (sdfVert->HasElement("noise"))
    {
      this->dataPtr->noiseVertical = sensors::NoiseFactory::NewNoiseModel(
            sdfVert->GetElement("noise"));
    }
  }

  if (_sdf->HasElement("force_approximation_scaling_factor"))
  {
    sdf::ElementPtr sdfForceApprox =
      _sdf->GetElement("force_approximation_scaling_factor");

    this->dataPtr->forceApproximationScalingFactor =
        sdfForceApprox->Get<double>();
  }

  // If the forceApproximationScalingFactor is very small don't update.
  // It doesn't make sense to be negative, that would be negative wind drag.
  if (std::fabs(this->dataPtr->forceApproximationScalingFactor) < 1e-6)
  {
    gzerr << "Please set <force_approximation_scaling_factor> to a value "
          << "greater than 0" << std::endl;
    return;
  }

  double period = this->dataPtr->world->Physics()->GetMaxStepSize();

  this->dataPtr->kMag = period / this->dataPtr->characteristicTimeForWindRise;
  this->dataPtr->kMagVertical =
      period / this->dataPtr->characteristicTimeForWindRiseVertical;
  this->dataPtr->kDir =
      period / this->dataPtr->characteristicTimeForWindOrientationChange;

  wind.SetLinearVelFunc(std::bind(&WindPlugin::LinearVel, this,
        std::placeholders::_1, std::placeholders::_2));

  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&WindPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
ignition::math::Vector3d WindPlugin::LinearVel(const physics::Wind *_wind,
    const physics::Entity * /*_entity*/)
{
  // Compute magnitude
  this->dataPtr->magnitudeMean = (1. - this->dataPtr->kMag) *
      this->dataPtr->magnitudeMean + this->dataPtr->kMag *
      std::sqrt(_wind->LinearVel().X() * _wind->LinearVel().X() +
                        _wind->LinearVel().Y() * _wind->LinearVel().Y());
  double magnitude = this->dataPtr->magnitudeMean;

  // Compute magnitude
  this->dataPtr->magnitudeMeanVertical = (1. - this->dataPtr->kMagVertical) *
      this->dataPtr->magnitudeMeanVertical +
      this->dataPtr->kMagVertical * _wind->LinearVel().Z();

  magnitude += this->dataPtr->magnitudeSinAmplitudePercent *
    this->dataPtr->magnitudeMean *
    std::sin(2 * M_PI * this->dataPtr->world->SimTime().Double() /
        this->dataPtr->magnitudeSinPeriod);

  if (this->dataPtr->noiseMagnitude)
  {
    magnitude = this->dataPtr->noiseMagnitude->Apply(magnitude);
  }

  // Compute horizontal direction
  //
  double direction = IGN_RTOD(atan2(_wind->LinearVel().Y(),
                                   _wind->LinearVel().X()));

  this->dataPtr->directionMean = (1.0 - this->dataPtr->kDir) *
      this->dataPtr->directionMean + this->dataPtr->kDir * direction;

  direction = this->dataPtr->directionMean;

  direction += this->dataPtr->orientationSinAmplitude *
      std::sin(2 * M_PI * this->dataPtr->world->SimTime().Double() /
        this->dataPtr->orientationSinPeriod);

  if (this->dataPtr->noiseDirection)
    direction = this->dataPtr->noiseDirection->Apply(direction);

  // Apply wind velocity
  ignition::math::Vector3d windVel;
  windVel.X(magnitude * std::cos(IGN_DTOR(direction)));
  windVel.Y(magnitude * std::sin(IGN_DTOR(direction)));

  if (this->dataPtr->noiseVertical)
  {
    windVel.Z(this->dataPtr->noiseVertical->Apply(
        this->dataPtr->magnitudeMeanVertical));
  }
  else
  {
    windVel.Z(this->dataPtr->magnitudeMeanVertical);
  }

  return windVel;
}

/////////////////////////////////////////////////
void WindPlugin::OnUpdate()
{
  // Update loop for using the force on mass approximation
  // This is not recommended. Please use the LiftDragPlugin instead.

  // Get all the models
  physics::Model_V models = this->dataPtr->world->Models();

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
      link->AddRelativeForce(link->GetInertial()->Mass() *
          this->dataPtr->forceApproximationScalingFactor *
          (link->RelativeWindLinearVel() - link->RelativeLinearVel()));
    }
  }
}
