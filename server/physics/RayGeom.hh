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
 * Author: Nate Keonig, Andrew Howard
 * Date: 21 May 2003
 * CVS: $Id: RayGeom.hh,v 1.17 2004/12/07 00:15:06 natepak Exp $
 */

#ifndef RAYGEOM_HH
#define RAYGEOM_HH

#include "Geom.hh"

namespace gazebo
{
  class OgreDynamicLines;

class RayGeom : public Geom
{
  /// Constructor
  /// \param body Body the ray is attached to
  public: RayGeom( Body *body );

  /// Destructor
  public: virtual ~RayGeom();

  /// Set the ray based on a starting and ending point
  /// \param posStart Start position
  /// \param posEnd End position
  public: void SetPoints(const Vector3 &posStart, const Vector3 &posEnd);

  /// Set the starting point and direction (relative coordinates)
  /// \param pos Starting position
  /// \param dir Unit length direction of the ray
  /// \param length Length of the ray
  public: void Set(const Vector3 &pos, const Vector3 &dir, double length);

  /// Get the starting and ending points
  /// \param posA Returns the starting point
  /// \param posB Returns the ending point
  public: void GetPoints(Vector3 &posA, Vector3 &posB);

  /// Get the starting point and direction (relative coordinates)
  /// \param pos Returns the starting point
  /// \param dir Returns the direction
  public: void Get(Vector3 &pos, Vector3 &dir);

  /// Set the length of the ray
  /// \param len Length of the array
  public: void SetLength( double len );

  /// Get the length of the ray
  public: double GetLength() const;

  /// Contact information; this is filled out during collision
  /// detection.  
  public: double contactRetro;
  public: int contactFiducial;

  private: double contactLen;

  private: OgreDynamicLines *line;
  private: Vector3 pos;
  private: Vector3 dir;
};

}

#endif
