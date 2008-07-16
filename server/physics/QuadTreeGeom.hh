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

#ifndef QUADTREEGEOM_HH
#define QUADTREEGEOM_HH

#include <Ogre.h>
#include <deque>

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


  class SpaceTree;
  class QuadNode;

  /// \brief Occupancy grid geom
  class QuadTreeGeom : public Geom
  {
    /// \brief Constructor
    public: QuadTreeGeom(Body *body);

    /// \brief Destructor
    public: virtual ~QuadTreeGeom();

    /// \brief Update function 
    public: void UpdateChild();

    /// \brief Load the heightmap
    protected: virtual void LoadChild(XMLConfigNode *node);

    private: void BuildTree(QuadNode *node);

    private: void GetPixelCount(unsigned int xStart, unsigned int yStart, 
                                unsigned int width, unsigned int height, 
                                unsigned int &freePixels, 
                                unsigned int &occPixels  );

    private: void ReduceTree(QuadNode *node);

    private: void Merge(QuadNode *nodeA, QuadNode *nodeB);

    private: void CreateBoxes(QuadNode *node);

    private: Vector3 mapSize;

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

    // The color of the walls
    private: Vector3 color;

    // The amount of acceptable error in the model
    private: float errBound;

    private: float wallHeight;

    // The map dimensions
    private: unsigned int mapWidth;
    private: unsigned int mapHeight;

    private: Ogre::Image mapImage;

    private: QuadNode *root;
    private: bool merged;
  };


  class QuadNode
  {
    public: QuadNode( QuadNode *_parent ) 
            {
              parent = _parent;
              occupied = false;
              leaf = true;
              leaves = 0;
              valid = true;
            }

    public: ~QuadNode() 
            { 
              /*std::deque<QuadNode*>::iterator iter;
              for (iter = children.begin(); iter != children.end(); iter++) 
                  delete (*iter); 
                  */
            }

    public: void Print(std::string space)
            {
              std::deque<QuadNode*>::iterator iter;

              printf("%sXY[%d %d] WH[%d %d] O[%d] L[%d] V[%d]\n",space.c_str(),x,y,width, height, occupied, leaf, valid);
              space += "  ";
              for (iter = children.begin(); iter != children.end(); iter++) 
                if ((*iter)->occupied)
                  (*iter)->Print(space);
            }

    public: unsigned int x, y;
    public: unsigned int width, height;

    public: QuadNode *parent;
    public: std::deque<QuadNode*> children;
    public: bool occupied;
    public: bool leaf;
    public: int leaves;

    public: bool valid;
  };

  /// \}
}

#endif
