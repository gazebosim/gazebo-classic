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
/* Desc: A ray
 * Author: Nate Keonig
 * Date: 24 May 2009
 * SVN: $Id:$
 */

#ifndef BULLETRAYGEOM_HH
#define BULLETRAYGEOM_HH

#include "BulletGeom.hh"
#include "RayGeom.hh"

namespace gazebo
{
  class OgreDynamicLines;
  class Visual;

  /// \addtogroup gazebo_physics_geom
  /// \{
  /** \defgroup gazebo_ray_geom Ray geom
      \brief Ray geom

      This geom is used soley by ray sensors. It should not be directly 
      included in a world file.
  */
  /// \}
  /// \addtogroup gazebo_ray_geom 
  /// \{

  /// \brief Ray geom 
  class BulletRayGeom : public RayGeom<BulletGeom>
  {
    /// \brief Constructor
    /// \param body Body the ray is attached to
    /// \param displayRays Indicates if the rays should be displayed when 
    ///        rendering images
    public: BulletRayGeom( Body *body, bool displayRays );
  
    /// \brief Destructor
    public: virtual ~BulletRayGeom();
  
    /// \brief Set the ray based on starting and ending points relative to 
    ///        the body
    /// \param posStart Start position, relative the body
    /// \param posEnd End position, relative to the body
    public: void SetPoints(const Vector3 &posStart, const Vector3 &posEnd);
  
    /// \brief Update the tay geom
    public: void Update();
  };
  
  /// \}
}

#endif
