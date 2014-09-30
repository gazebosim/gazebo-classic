/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
/* Desc: A ray
 * Author: Nate Koenig, Andrew Howard
 * Date: 14 Oct 2009
 */

#ifndef _PluginRAYSHAPE_HH_
#define _PluginRAYSHAPE_HH_

#include <string>

#include "gazebo/physics/RayShape.hh"
#include "gazebo/physics/Shape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \brief Ray collision
    class GAZEBO_VISIBLE PluginRayShape : public RayShape
    {
      /// \brief Constructor for a global ray.
      /// \param[in] _physicsEngine Pointer to the physics engine.
      public: explicit PluginRayShape(PhysicsEnginePtr _physicsEngine);

      /// \brief Constructor.
      /// \param[in] _collision Collision object this ray is attached to.
      public: explicit PluginRayShape(CollisionPtr _collision);

      /// \brief Destructor.
      public: virtual ~PluginRayShape();

      /// \brief Update the ray collision
      public: virtual void Update();

      /// \brief Get the nearest intersection
      /// \param[out] _dist Distance to the intersection.
      /// \param[out] _entity Name of the entity that was hit.
      public: virtual void GetIntersection(double &_dist, std::string &_entity);
  }
}
#endif
