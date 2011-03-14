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
 * Author: Nate Keonig, Andrew Howard
 * Date: 8 May 2003
 * CVS: $Id$
 */

#ifndef TRIMESHSHAPE_HH
#define TRIMESHSHAPE_HH

#include "Shape.hh"

namespace gazebo
{
	namespace physics
{
  class Mesh;

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
    - centerMesh (re-center trimesh about AABB center or bottom most point)
      - aabb_center
      - aabb_bottom
      - none (default)

    - genTexCoord (generate spherical projected texture coordinates)
      - true or false, default to false

    \par Example
    \verbatim
      <geom:trimesh name="pallet_geom">
        <mesh>WoodPallet.mesh</mesh>
        <scale>.2 .2 .2</scale>
        <centerMesh>AABBCenter</centerMesh>
        <genTexCoord>True</genTexCoord>
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
  class TrimeshShape : public Shape
  {
    /// \brief Constructor
    public: TrimeshShape(Geom *parent);

    /// \brief Destructor
    public: virtual ~TrimeshShape();

    /// \brief Update function 
    public: void Update();

    /// \brief Load the trimesh
    protected: virtual void Load(XMLConfigNode *node);

    /// \brief Save child parameters
    protected: virtual void Save(std::string &prefix, std::ostream &stream);
 
    ///  name of the mesh
    protected: ParamT<std::string> *meshNameP;

    protected: ParamT<Vector3> *scaleP;
    protected: ParamT<std::string> *centerMeshP;
    protected: ParamT<bool> *genTexCoordP;

/*
    protected: unsigned int numVertices;
    protected: unsigned int numIndices;
    protected: float *vertices;
    protected: unsigned int *indices;
    */

    protected: const Mesh *mesh;
  };

  /// \}
}

}
#endif
