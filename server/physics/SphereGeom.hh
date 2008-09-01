/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Sphere geometry
 * Author: Nate Keonig, Andrew Howard
 * Date: 8 May 2003
 * CVS: $Id$
 */

#ifndef SPHEREGEOM_HH
#define SPHEREGEOM_HH

#include "Param.hh"
#include "Geom.hh"

namespace gazebo
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
  class SphereGeom : public Geom
  {
    /// \brief Constructor
    public: SphereGeom(Body *body);

    /// \brief Destructor
    public: virtual ~SphereGeom();

    /// \brief Load the sphere
    protected: void LoadChild(XMLConfigNode *node);

    /// \brief Save child parameters
    protected: void SaveChild(std::string &prefix, std::ostream &stream);

    private: ParamT<double> *radiusP;
  };

  /// \}
}

#endif
