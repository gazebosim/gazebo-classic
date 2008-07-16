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
/* Desc: A QuadTree to logically segment the geometries in a map
 * Author: Nate Koenig
 * Date: 28 July 2003
 * CVS: $Id: SpaceTree.hh,v 1.11 2004/10/10 17:57:29 inspectorg Exp $
 */

#ifndef SPACETREE_HH
#define SPACETREE_HH

#include "OccupancyGridGeom.hh"
#include "Global.hh"

namespace gazebo
{
  
  // Forward declarations
  class Body;
  
  class SpaceNode
  {
    // Constructor
    public: SpaceNode( SpaceNode *parent, float x, float y, float hw, float hh );
  
    // Destructor
    public: virtual ~SpaceNode();
  
    // Add a wall to the space
    public: void AddWall( Line *line );
  
    // Add a geom to the space
    public: void AddGeom( Geom *geom );
  
    // Create the ODE geometries
    public: void GenerateGeoms(dSpaceID spaceId, Body *body, float wallWidth, 
                float wallHeight, Vector3 &pos, float scale, 
                Vector3 color);

     // The parent of this node
    public: SpaceNode *parent;
 
    // Is this node a leaf??
    public: bool isLeaf;
  
    // Each node has 4 children since this is a quad tree
    public: SpaceNode *children[4];
  
    // The center location of this node
    public: float cx, cy;
  
    // The half length of the rectangle (in x direction)
    public: float halfWidth;
  
    // The half height of the rectangle (in y direction)
    public: float halfHeight;
  
    // The lines representing walls contained in this space
    public: Line** walls;
    public: int wallCount;
    public: int wallMaxCount;
  
    public: Geom** geoms;
    public: int geomCount;
    public: int geomMaxCount;
  
    // The spaceId for this node
    private: dSpaceID spaceId;
  };
  
  
  class SpaceTree
  {
    // Constructor
    public: SpaceTree();
  
    // Destructor
    public: virtual ~SpaceTree();
  
     // Return the root node of the tree
    public: SpaceNode *GetRoot() const;
  
    // Build the tree from a list of line segments
    public: void BuildTree(  Line** lines, unsigned int numLines, 
                                  float width, float height );
  
    // Called from BuildTree to recursively build the tree
    private: SpaceNode *Construct( SpaceNode *parent, float cx, float cy, 
                                   float hw, float hh, 
                                  Line** lines, unsigned int lineCount );
  
    // The root node
    private: SpaceNode *root;
  
    // Then maxmimum number of lines per node
    private: unsigned int linesPerNode;
  
  };
  
}

#endif
