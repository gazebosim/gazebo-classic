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

#include "Geom.hh"


namespace gazebo
{
  /// \addtogroup gazebo_physics_geom
  /// \brief Sphere geom
  /// \{
  /// \defgroup gazebo_sphere_geom Sphere geom
  /// \brief Sphere geom
  /// \{

  /// \brief Sphere geom
  class SphereGeom : public Geom
  {
    /// \brief Constructor
//    public: SphereGeom(Body *body,const std::string &name,  double radius, double mass, const std::string &meshName = "default" );
    public: SphereGeom(Body *body);

    /// \brief Destructor
    public: virtual ~SphereGeom();

    /// \brief Load the sphere
    protected: void LoadChild(XMLConfigNode *node);

  };

  /// \}
  /// \}
}

#endif
