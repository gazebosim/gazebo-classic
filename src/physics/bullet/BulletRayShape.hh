/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
 * Author: Nate Keonig
 * Date: 24 May 2009
 */

#ifndef BULLETRAYGEOM_HH
#define BULLETRAYGEOM_HH

#include "BulletCollision.hh"
#include "RayCollision.hh"

namespace gazebo
{
  namespace physics
{
  class OgreDynamicLines;
  class Visual;

  /// \addtogroup gazebo_physics_collision
  /// \{
  /** \defgroup gazebo_ray_collision Ray collision
      \brief Ray collision

      This collision is used soley by ray sensors. It should not be directly
      included in a world file.
  */
  /// \}
  /// \addtogroup gazebo_ray_collision
  /// \{
  /// \brief Ray collision
  class BulletRayCollision : public RayCollision<BulletCollision>
  {
    /// \brief Constructor
    /// \param body Link the ray is attached to
    /// \param displayRays Indicates if the rays should be displayed when
    ///        rendering images
    public: BulletRayCollision(Link *body, bool displayRays);

    /// \brief Destructor
    public: virtual ~BulletRayCollision();

    /// \brief Set the ray based on starting and ending points relative to
    ///        the body
    /// \param posStart Start position, relative the body
    /// \param posEnd End position, relative to the body
    public: void SetPoints(const math::Vector3 &posStart,
                           const math::Vector3 &posEnd);

    /// \brief Update the tay collision
    public: void Update();
  };

  /// \}
}
}
#endif
