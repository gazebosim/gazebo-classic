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
/* Desc: A ray
 * Author: Nate Keonig
 * Date: 14 Oct 2009
 * SVN: $Id:$
 */

#ifndef RAYSHAPE_HH
#define RAYSHAPE_HH

#include <float.h>

#include "common/Global.hh"
#include "Simulator.hh"
#include "Geom.hh"
#include "transport/Publisher.hh"

namespace gazebo
{
	namespace physics
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
  
    /// Start and end positions of the ray, relative to the body
    protected: Vector3 relativeStartPos;
    protected: Vector3 relativeEndPos;
  
    /// Start and end positions of the ray in global cs
    protected: Vector3 globalStartPos;
    protected: Vector3 globalEndPos;
    protected: transport::PublisherPtr vis_pub;
  };
  
  /// \}
}

}
#endif
