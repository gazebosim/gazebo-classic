/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_VECTOR3_STATS_PRIVATE_HH_
#define _GAZEBO_VECTOR3_STATS_PRIVATE_HH_

#include <ignition/math/Vector3Stats.hh>
#include <ignition/math/SignalStats.hh>
#include "gazebo/math/SignalStats.hh"

namespace gazebo
{
  namespace math
  {
    /// \brief Private data class for the Vector3Stats class.
    class Vector3StatsPrivate
    {
      public: Vector3StatsPrivate() = default;

      public: Vector3StatsPrivate(
                  const ignition::math::Vector3Stats &_v)
              : x(_v.X()), y(_v.Y()), z(_v.Z()), mag(_v.Mag()) {}

      /// \brief Statistics for x component of signal.
      public: SignalStats x;

      /// \brief Statistics for y component of signal.
      public: SignalStats y;

      /// \brief Statistics for z component of signal.
      public: SignalStats z;

      /// \brief Statistics for magnitude of signal.
      public: SignalStats mag;
    };
  }
}
#endif

