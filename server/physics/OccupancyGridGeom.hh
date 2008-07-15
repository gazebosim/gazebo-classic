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
/* Desc: Occupancy grid geom
 * Author: Nate Koenig
 * Date: 14 Jly 2008
 * CVS: $Id:$
 */

#ifndef OCCUPANCYGRIDGEOM_HH
#define OCCUPANCYGRIDGEOM_HH

#include <Ogre.h>

#include "Vector2.hh"
#include "Geom.hh"

namespace gazebo
{
  /// \addtogroup gazebo_physics_geom
  /// \{
  /** \defgroup gazebo_occupancy_geom Occupancy grid geom
      \brief Occupancy grid geom

    \par Attributes
    The following attributes are supported.

    - image (string)
      - Binary image that defines an occupancy grid
      - Default: (empty)

    - size (float tuple)
      - Size of the height map
      - Default: 0 0 0

    \par Example
    \verbatim
      <geom:occupancygrid name="occ_geom">
        <image>map.png</image>
        <size>100 100 1.0</size>
      </geom:occupancygrid>
    \endverbatim
    */
  /// \}
  /// \addtogroup gazebo_occupancy_geom 
  /// \{


  class Line;
  class MapPoint;

  /// \brief Occupancy grid geom
  class OccupancyGridGeom : public Geom
  {
    /// \brief Constructor
    public: OccupancyGridGeom(Body *body);

    /// \brief Destructor
    public: virtual ~OccupancyGridGeom();

    /// \brief Update function 
    public: void UpdateChild();

    /// \brief Load the heightmap
    protected: virtual void LoadChild(XMLConfigNode *node);

    /// \brief Get the 2d lines from an image file
    private: void GenerateLines();

    private: Vector3 mapSize;

    //private: Line **walls;
    private: int wallCount;
    private: int wallMaxCount;

    // The scale factor to apply to the geoms
    private: float scale;

    // Alignment
    private: std::string halign, valign;

    // The position of the map
    private: Vector3 pos;

    // Negative image?
    private: int negative;

    // Image color threshold used for extrusion
    private: float threshold;

    // The width( thickness ) of the walls in meters
    private: float wallWidth;

    // The height of the walls in meters
    private: float wallHeight;

    // The color of the walls
    //private: Vector3 color;

    // The amount of acceptable error in the model
    private: float errBound;

    // The map dimensions
    private: unsigned int mapWidth;
    private: unsigned int mapHeight;

    private: Ogre::Image mapImage;
  };
/*
  class MapPoint
  {
    public: MapPoint() : x(0), y(0), arcCount(0) 
            {for (int i=0;i<8;i++) this->arcs[i] = NULL;}

    public: MapPoint( float x,float y ) : x(x), y(y), arcCount(0) 
            {for (int i=0; i<8; i++) this->arcs[i] = NULL;}

    public: MapPoint( const MapPoint &o) : x(o.x),y(o.y), arcCount(o.arcCount)  
            { for (int i=0; i<8; i++) this->arcs[i] = o.arcs[i]; }

    public: bool operator==( const MapPoint &p ) const
            { return p.x == this->x && p.y == this->y; }

    public: MapPoint &operator= ( const MapPoint &p )
            {this->x = p.x; this->y = p.y; this->arcCount = p.arcCount;
              for (int i=0; i<8; i++) this->arcs[i] = p.arcs[i]; return *this;}

    public: float x;
    public: float y;

    public: MapPoint *arcs[8];
    public: unsigned short arcCount;
  };

  class Line
  {
    public: Line();
    public: virtual ~Line();

    public: bool operator==(const Line& l);

    public: void Set( const MapPoint &start, const MapPoint &end );
    public: void Start( const MapPoint &start );
    public: void End( const MapPoint &end );

    public: const MapPoint& Start() const;
    public: const MapPoint& End() const;
    public: const MapPoint& Mid() const;
    public: float Length() const;
    public: float Angle() const;

    private: void Calc();

    private: MapPoint start;
    private: MapPoint end;
    private: MapPoint mid;
    private: float length;
    private: float angle;
  };
  */


  /// \}
}

#endif
