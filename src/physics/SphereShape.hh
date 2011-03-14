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
/* Desc: Sphere shape
 * Author: Nate Keonig
 * Date: 14 Oct 2009
 * SVN: $Id:$
 */

#ifndef SPHERESHAPE_HH
#define SPHERESHAPE_HH

#include "common/Param.hh"
#include "Shape.hh"

namespace gazebo
{
	namespace physics
{

  /// \addtogroup gazebo_physics_geom
  /// \{
  /** \defgroup gazebo_sphere_geom Sphere geom
      \brief Sphere geom

      \par Attributes
      The following attributes are supported.

      \htmlinclude default_geom_attr_include.html

      - size (float, meters)
        - Radius of the sphere
        - Default: 0

      \par Example
      \verbatim
      <geom:sphere name="geom_name">
        <xyz>1 2 3</xyz>
        <rpy>0 0 30</rpy>
        <size>0.1</size>
        <mass>0.5</mass>
        <laserFiducialId>1</laserFiducialId>
        <laserRetro>0.5</laserRetro>

        <visual>
          <mesh>default</mesh>
          <size>0.1 0.1 0.1</size>
          <material>Gazebo/Red</material>
        </visual>
      </geom:sphere>
      \endverbatim
    */
  /// \}
  /// \addtogroup gazebo_sphere_geom 
  /// \{

  /// \brief Sphere geom
  class SphereShape : public Shape
  {
    /// \brief Constructor
    public: SphereShape(Geom *parent);

    /// \brief Destructor
    public: virtual ~SphereShape();

    /// \brief Load the sphere
    public: virtual void Load(XMLConfigNode *node);

    /// \brief Save shape parameters
    public: virtual void Save(std::string &prefix, std::ostream &stream);

    /// \brief Set the size
    public: virtual void SetSize(const double &radius);

    private: ParamT<double> *radiusP;
  };

  /// \}
}

}
#endif
