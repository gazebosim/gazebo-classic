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
 * CVS: $Id$
 */

#ifndef RAYGEOM_HH
#define RAYGEOM_HH

#include "Geom.hh"

namespace gazebo
{
  class OgreDynamicLines;
  class OgreVisual;

  /// \addtogroup gazebo_physics_geom
  /// \{
  /** \defgroup gazebo_ray_geom Ray geom
      \brief Ray geom

      This geom is used soley by ray sensors. It should not be directly included in a world file.
  */
  /// \}
  /// \addtogroup gazebo_ray_geom 
  /// \{

  /// \brief Ray geom 
  class RayGeom : public Geom
  {
    /// \brief Constructor
    /// \param body Body the ray is attached to
    /// \param displayRays Indicates if the rays should be displayed when rendering images
    public: RayGeom( Body *body, bool displayRays );
  
    /// \brief Destructor
    public: virtual ~RayGeom();
  
    /// \brief Set the ray based on starting and ending points relative to the body
    /// \param posStart Start position, relative the body
    /// \param posEnd End position, relative to the body
    public: void SetPoints(const Vector3 &posStart, const Vector3 &posEnd);
  
    /// \brief Get the relative starting and ending points
    /// \param posA Returns the starting point
    /// \param posB Returns the ending point
    public: void GetRelativePoints(Vector3 &posA, Vector3 &posB);
  
    /// \brief Get the global starting and ending points
    /// \param posA Returns the starting point
    /// \param posB Returns the ending point
    public: void GetGlobalPoints(Vector3 &posA, Vector3 &posB);
  
    /// \brief Set the length of the ray
    /// \param len Length of the array
    public: void SetLength( double len );
  
    /// \brief Get the length of the ray
    public: double GetLength() const;
  
    /// \brief Update the tay geom
    public: void Update();
  
    /// \brief Set the retro-reflectivness detected by this ray
    public: void SetRetro( float retro );
  
    /// \brief Get the retro-reflectivness detected by this ray
    public: float GetRetro() const;
  
    /// \brief Set the fiducial id detected by this ray
    public: void SetFiducial( int fid );
  
    /// \brief Get the fiducial id detected by this ray
    public: int GetFiducial() const;

    /// \brief Load thte ray
    protected: virtual void LoadChild(XMLConfigNode *node);

    /// Contact information; this is filled out during collision
    /// detection.  
    private: double contactLen;
    private: double contactRetro;
    private: int contactFiducial;
  
    private: OgreDynamicLines *line;
  
    /// Start and end positions of the ray, relative to the body
    private: Vector3 relativeStartPos;
    private: Vector3 relativeEndPos;
  
    /// Start and end positions of the ray in global cs
    private: Vector3 globalStartPos;
    private: Vector3 globalEndPos;

    private: OgreVisual *visual;
  
  };
  
  /// \}
}

#endif
