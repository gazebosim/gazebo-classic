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
/* Desc: Trimesh collisionetry
 * Author: Nate Keonig
 * Date: 21 May 2009
 * SVN: $Id:$
 */

#ifndef BULLETTRIMESHSHAPE_HH
#define BULLETTRIMESHSHAPE_HH

#include "TrimeshShape.hh"

namespace gazebo
{
	namespace physics
{
  class Visual;

  /// \addtogroup gazebo_physics_collision
  /// \{
  /** \defgroup gazebo_trimesh_collision Triangle Mesh collision
      \brief Trimesh collision

    \par Attributes
    The following attributes are supported.

    \htmlinclude default_collision_attr_include.html

    - scale (float tuple, meters)
      - Scale of the trimesh
      - Default: 1 1 1

    \par Example
    \verbatim
      <collision:trimesh name="pallet_collision">
        <mesh>WoodPallet.mesh</mesh>
        <scale>.2 .2 .2</scale>
        <mass>0.1</mass>

        <visual>
          <scale>.2 .2 .2</scale>
          <material>Gazebo/WoodPallet</material>
          <mesh>WoodPallet.mesh</mesh>
        </visual>
      </collision:trimesh>
    \endverbatim
  */
  /// \}
  /// \addtogroup gazebo_trimesh_collision 
  /// \{


  /// \brief Triangle mesh collision
  class BulletTrimeshShape : public TrimeshShape
  {
    /// \brief Constructor
    public: BulletTrimeshShape(Collision *parent);

    /// \brief Destructor
    public: virtual ~BulletTrimeshShape();

    /// \brief Update function 
    public: void Update();

    /// \brief Load the trimesh
    public: virtual void Load(common::XMLConfigNode *node);
  };

  /// \}
}

}
}
#endif
