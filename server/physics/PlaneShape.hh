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
/* Desc: Plane shape
 * Author: Nate Keonig
 * Date: 14 Oct 2009
 * SVN: $Id:$
 */

#ifndef PLANESHAPE_HH
#define PLANESHAPE_HH

#include "Param.hh"
#include "Shape.hh"

namespace gazebo
{
  class XMLConfig;
  class VisualMsg;

  /// \addtogroup gazebo_physics_geom
  /// \{
  /** \defgroup gazebo_plane_geom Plane Geom
      \brief Geom for an infinite plane.

       \par Attributes
      The following attributes are supported.
  
      \htmlinclude default_geom_attr_include.html
 
      - normal (float tuple, unit vector)
        - Set the plane's normal vector
        - Default: 0 0 1

      - segments (int tuple)
        - Set the number of segments of the plane. Larger numbers increase the vertex density.
        - Default: 1 1

      - uvTile (float tuple)
        - Set the UV tiling, used for texture mapping
        - Default: 1 1

      - size (float tuple, meters)
        - Size of the plane
        - Default: 0 0
  
      \par Example
      \verbatim
      <geom:plane name="plane1_geom">
        <normal>0 0 1</normal>
        <size>2000 2000</size>
        <segments>10 10</segments>
        <uvTile>100 100</uvTile>
        <material>Gazebo/GrassFloor</material>
      </geom:plane>
      \endverbatim
  */
  /// \}
  /// \addtogroup gazebo_plane_geom 
  /// \{


  /// \brief Geom for an infinite plane.
  /// 
  /// This geom is used primarily for ground planes.  Note that while
  /// the plane in infinite, only the part near the camera is drawn.  
  class PlaneShape : public Shape
  {
    /// \brief Constructor
    /// \param body Body to which we are attached.
    public: PlaneShape(Geom *parent);

    /// \brief Destructor
    public: virtual ~PlaneShape();
 
    /// \brief Load the plane
    public: virtual void Load(XMLConfigNode *node);

    /// \brief Save child parameters
    public: virtual void Save(std::string &prefix, std::ostream &stream);

    /// \brief Create the plane
    public: virtual void CreatePlane();

    /// \brief Set the altitude of the plane
    public: virtual void SetAltitude(const Vector3 &pos);

    /// \brief Set the normal
    public: void SetNormal( const Vector3 &norm );

    /// \brief Set the size
    public: void SetSize( const Vector2<double> &size );

    /// \brief Set the number of segments
    public: void SetSegments(const Vector2<double> &seg);

    /// \brief Set the uvtile
    public: void SetUVTile(const Vector2<double> &uv);

    /// \brief Set the material
    public: void SetMaterial(const std::string &mat);

    /// \brief Set cast shadows
    public: void SetCastShadows(const bool &cast);

    protected: ParamT<Vector3> *normalP;
    protected: ParamT<Vector2<double> > *sizeP;
    protected: ParamT<Vector2<double> > *segmentsP;
    protected: ParamT<Vector2<double> > *uvTileP;
    protected: ParamT<std::string> *materialP;
    protected: ParamT<bool> *castShadowsP;
    protected: VisualMsg *visualMsg;
  };
  
  /// \}
}
#endif
