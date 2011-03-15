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
/* Desc: Trimesh geometry
 * Author: Nate Koenig
 * Date: 16 Oct 2009
 * SVN: $Id$
 */

#ifndef ODETRIMESHSHAPE_HH
#define ODETRIMESHSHAPE_HH

#include "physics/TrimeshShape.hh"

namespace gazebo
{
	namespace physics
  {
    /// \addtogroup gazebo_physics_geom
    /// \{
    /** \defgroup gazebo_trimesh_geom Triangle Mesh geom
        \brief Trimesh geom
  
      \par Attributes
      The following attributes are supported.
  
      \htmlinclude default_geom_attr_include.html
  
      - scale (float tuple, meters)
        - Scale of the trimesh
        - Default: 1 1 1
  
      \par Example
      \verbatim
        <geom:trimesh name="pallet_geom">
          <mesh>WoodPallet.mesh</mesh>
          <scale>.2 .2 .2</scale>
          <mass>0.1</mass>
  
          <visual>
            <scale>.2 .2 .2</scale>
            <material>Gazebo/WoodPallet</material>
            <mesh>WoodPallet.mesh</mesh>
          </visual>
        </geom:trimesh>
      \endverbatim
    */
    /// \}
    /// \addtogroup gazebo_trimesh_geom 
    /// \{
  
  
    /// \brief Triangle mesh geom
    class ODETrimeshShape : public TrimeshShape
    {
      /// \brief Constructor
      public: ODETrimeshShape(Geom *parent);
  
      /// \brief Destructor
      public: virtual ~ODETrimeshShape();
  
      /// \brief Update function 
      public: void Update();
  
      /// \brief Load the trimesh
      protected: virtual void Load(common::XMLConfigNode *node);
  
      private: dReal matrix_dblbuff[16*2];
      private: int last_matrix_index;
    };
  
    /// \}
  }
}
#endif
