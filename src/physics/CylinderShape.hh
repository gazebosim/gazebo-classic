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
/* Desc: Cylinder geometry
 * Author: Nate Keonig, Andrew Howard
 * Date: 8 May 2003
 * CVS: $Id: CylinderGeom.hh 7039 2008-09-24 18:06:29Z natepak $
 */

#ifndef CYLINDERSHAPE_HH
#define CYLINDERSHAPE_HH

#include "Shape.hh"
#include "Param.hh"

namespace gazebo
{
	namespace physics
{
  /// \addtogroup gazebo_physics_geom
  /// \{
  /** \defgroup gazebo_cylinder_geom Cylinder geom
      \brief Cylinder geom

      \par Attributes
      The following attributes are supported.

      \htmlinclude default_geom_attr_include.html

      - size (float tuple, meters)
        - Radius and height of the cylinder
        - Default: 0 0 

    \par Example
    \verbatim
    <geom:cylinder name="geom_name">
      <xyz>1 2 3</xyz>
      <rpy>0 0 30</rpy>
      <size>0.1 0.5</size>
      <mass>0.5</mass>
      <laserFiducialId>1</laserFiducialId>
      <laserRetro>0.5</laserRetro>

      <visual>
        <size>0.1 0.1 0.5</size>
        <mesh>default</mesh>
        <material>Gazebo/Red</material>
      </visual>
    </geom:box>
    \endverbatim
    */
  /// \}
  /// \addtogroup gazebo_cylinder_geom
  /// \{

  /// \brief Cylinder geom
  class CylinderShape : public Shape
  {
    /// \brief Constructor
    public: CylinderShape(Geom *parent);

    /// \brief Destructor
    public: virtual ~CylinderShape();

    /// \brief Load the cylinder
    public: virtual void Load(XMLConfigNode *node);

    /// \brief Save child parameters
    public: virtual void Save(std::string &prefix, std::ostream &stream);

    /// \brief Set radius
    public: void SetRadius(const double &radius);

    /// \brief Set length
    public: void SetLength(const double &length);


    /// \brief Set the size of the cylinder
    public: virtual void SetSize( const double &radius, const double &length  );

    private: ParamT<double> *radiusP;
    private: ParamT<double> *lengthP;
  };

  /// \}
}
}
#endif
