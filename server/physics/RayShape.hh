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
 * Date: 14 Oct 2009
 * SVN: $Id:$
 */

#ifndef RAYSHAPE_HH
#define RAYSHAPE_HH

#include <float.h>

#include "Global.hh"
#include "Simulator.hh"
#include "OgreCreator.hh"
#include "OgreDynamicLines.hh"
#include "Visual.hh"
#include "Geom.hh"

namespace gazebo
{
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
  class RayShape : public Shape
  {
    /// \brief Constructor
    /// \param body Body the ray is attached to
    /// \param displayRays Indicates if the rays should be displayed when 
    ///        rendering images
    public: RayShape( Geom *parent, bool displayRays );
  
    /// \brief Destructor
    public: virtual ~RayShape();

    /// \brief Set to true in order to view individual rays
    public: void SetDisplayType( bool displayRays );
  
    /// \brief Set the ray based on starting and ending points relative to 
    ///        the body
    /// \param posStart Start position, relative the body
    /// \param posEnd End position, relative to the body
    public: virtual void SetPoints(const Vector3 &posStart, 
                                   const Vector3 &posEnd);
            
  
    /// \brief Get the relative starting and ending points
    /// \param posA Returns the starting point
    /// \param posB Returns the ending point
    public: virtual void GetRelativePoints(Vector3 &posA, Vector3 &posB);

    /// \brief Get the global starting and ending points
    /// \param posA Returns the starting point
    /// \param posB Returns the ending point
    public: virtual void GetGlobalPoints(Vector3 &posA, Vector3 &posB);

    /// \brief Set the length of the ray
    /// \param len Length of the array
    public: virtual void SetLength( double len );

    /// \brief Get the length of the ray
    public: double GetLength() const;

    /// \brief Update the tay geom
    public: virtual void Update() = 0;
  
    /// \brief Set the retro-reflectivness detected by this ray
    public: void SetRetro( float retro );
  
    /// \brief Get the retro-reflectivness detected by this ray
    public: float GetRetro() const;

    /// \brief Set the fiducial id detected by this ray
    public: void SetFiducial( int fid );

    /// \brief Get the fiducial id detected by this ray
    public: int GetFiducial() const;

    /// \brief Load thte ray
    protected: virtual void Load(XMLConfigNode *node);

    /// \brief Save child parameters
    protected: virtual void Save(std::string &, std::ostream &);
 
    /// Contact information; this is filled out during collision
    /// detection.  
    protected: double contactLen;
    protected: double contactRetro;
    protected: int contactFiducial;
  
    protected: OgreDynamicLines *line;
  
    /// Start and end positions of the ray, relative to the body
    protected: Vector3 relativeStartPos;
    protected: Vector3 relativeEndPos;
  
    /// Start and end positions of the ray in global cs
    protected: Vector3 globalStartPos;
    protected: Vector3 globalEndPos;

  };
  
  /// \}
}

#endif
