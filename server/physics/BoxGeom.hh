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
/* Desc: Box geometry
 * Author: Nate Keonig, Andrew Howard
 * Date: 8 May 2003
 * CVS: $Id $
 */

#ifndef BOXGEOM_HH
#define BOXGEOM_HH

#include "Geom.hh"

namespace gazebo
{

  /// \addtogroup gazebo_physics_geom
  /// \brief Box geom
  /// \{
  /// \defgroup gazebo_box_geom Box Geom
  /// \brief Box geom
  /// \{

  /// \brief Box geom
  class BoxGeom : public Geom
  {
    /// \brief Constructor
    public: BoxGeom(Body *body, Vector3 size, double density, const std::string &meshName="default");

    /// \brief Destructor
    public: virtual ~BoxGeom();
  };

  /// \}
  /// \}

}
#endif
