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
#ifndef _GAZEBO_RANDOMVELOCITY_PLUGIN_PRIVATE_HH_
#define _GAZEBO_RANDOMVELOCITY_PLUGIN_PRIVATE_HH_

#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector2.hh>

#include <gazebo/physics/Link.hh>
#include <gazebo/common/Time.hh>

namespace gazebo
{
  /// \internal
  /// \brief Private data for the RandomVelocityPlugin.
  class RandomVelocityPluginPrivate
  {
    public: RandomVelocityPluginPrivate()
            : velocityFactor(1.0),
              updatePeriod(10, 0),
              xRange(-IGN_DBL_MAX, IGN_DBL_MAX),
              yRange(-IGN_DBL_MAX, IGN_DBL_MAX),
              zRange(-IGN_DBL_MAX, IGN_DBL_MAX)
            {
            }

    /// \brief Velocity scaling factor.
    public: double velocityFactor;

    /// \brief Time between recomputing a new velocity vector
    public: common::Time updatePeriod;

    /// \brief Time the of the last update.
    public: common::Time prevUpdate;

    /// \brief Velocity to apply.
    public: ignition::math::Vector3d velocity;

    /// \brief Connects to world update event.
    public: event::ConnectionPtr updateConnection;

    /// \brief X velocity clamping values
    public: ignition::math::Vector2d xRange;

    /// \brief Y velocity clamping values
    public: ignition::math::Vector2d yRange;

    /// \brief Z velocity clamping values
    public: ignition::math::Vector2d zRange;

    /// \brief Pointer to the link that will receive the velocity.
    public: physics::LinkPtr link;
  };
}
#endif
