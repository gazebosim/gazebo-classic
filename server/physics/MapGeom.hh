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
 * CVS: $Id$
 */

#ifndef MAPGEOM_HH
#define MAPGEOM_HH

#include <Ogre.h>
#include <deque>

#include "Vector2.hh"
#include "Param.hh"
#include "Geom.hh"

namespace gazebo
{
  /// \addtogroup gazebo_physics_geom
  /// \{
  /** \defgroup gazebo_map_geom Map geom
      \brief Map geom

    \par Attributes
    The following attributes are supported.

    - image (string)
      - Binary image that defines an occupancy grid
      - Default: (empty)

    - scale (float)
      - Scaling factor
      - Default: 1

    - granularity (int)
      - Degree of coarseness when determing if an image area is occupied. Units are pixels
      - Default: 5

    - threshold (unsigned char)
      - Grayscale threshold. A pixel value greater than this amount is considered free space
      - Default: 200

    - negative (bool)
      - True if the image pixel values should be inverted.
      - Default: false

    - material (string)
      - Material to apply to the map
      - Default: (empty)



    \par Example
    \verbatim
      <geom:map name="map_geom">
        <image>map.png</image>
        <scale>0.1</scale>
      </geom:map>
    \endverbatim
    */
  /// \}
  /// \addtogroup gazebo_map_geom 
  /// \{


  class SpaceTree;
  class QuadNode;

  /// \brief Map geom
  class MapGeom : public Geom
  {
    /// \brief Constructor
    public: MapGeom(Body *body);

    /// \brief Destructor
    public: virtual ~MapGeom();

    /// \brief Update function 
    public: void UpdateChild();

    /// \brief Load the heightmap
    protected: virtual void LoadChild(XMLConfigNode *node);

    /// \brief Save child parameters
    protected: void SaveChild(std::string &prefix, std::ostream &stream);
 
    /// \brief Build the quadtree
    private: void BuildTree(QuadNode *node);

    /// \brief Get the number of free and occupied pixels in a given area
    private: void GetPixelCount(unsigned int xStart, unsigned int yStart, 
                                unsigned int width, unsigned int height, 
                                unsigned int &freePixels, 
                                unsigned int &occPixels  );

    /// \brief Reduce the number of nodes in the tree. 
    private: void ReduceTree(QuadNode *node);

    /// \brief Try to merge to nodes
    private: void Merge(QuadNode *nodeA, QuadNode *nodeB);

    private: void CreateBox();

    /// \brief Create the boxes for the map
    private: void CreateBoxes(QuadNode *node);

    // The scale factor to apply to the geoms
    private: ParamT<double> *scaleP;

    // Negative image?
    private: ParamT<int> *negativeP;

    // Image color threshold used for extrusion
    private: ParamT<double> *thresholdP;

    // The color of the walls
    private: ParamT<std::string> *materialP;

    // The amount of acceptable error in the model
    private: ParamT<int> *granularityP;

    private: ParamT<double> *wallHeightP;

    private: Ogre::Image mapImage;

    private: QuadNode *root;

    private: bool merged;
    private: static unsigned int geomCounter;
  };


  class QuadNode
  {
    public: QuadNode( QuadNode *_parent ) 
            {
              parent = _parent;
              occupied = false;
              leaf = true;
              valid = true;
            }

    public: ~QuadNode() 
            { 
              std::deque<QuadNode*>::iterator iter;
              for (iter = children.begin(); iter != children.end(); iter++) 
                  delete (*iter); 
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

    public: bool valid;

  };

  /// \}
}

#endif
