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
/* Desc: Occupancy grid geom
 * Author: Nate Koenig
 * Date: 14 Jly 2008
 * CVS: $Id$
 */

#ifndef MAPSHAPE_HH
#define MAPSHAPE_HH

#include <deque>

#include "Vector2.hh"
#include "Param.hh"
#include "Geom.hh"

namespace gazebo
{
	namespace physics
{
  class Image;

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
  class MapShape : public Shape
  {
    /// \brief Constructor
    public: MapShape(Geom *parent);

    /// \brief Destructor
    public: virtual ~MapShape();

    /// \brief Update function 
    public: void Update();

    /// \brief Load the map
    protected: virtual void Load(XMLConfigNode *node);

    /// \brief Save parameters
    protected: void Save(std::string &prefix, std::ostream &stream);
 
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

    private: Image *mapImage;

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

}
#endif
