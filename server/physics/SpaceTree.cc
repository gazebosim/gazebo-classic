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
 * CVS: $Id: SpaceTree.cc,v 1.14 2004/08/16 18:34:24 natepak Exp $
 */

#include <GL/gl.h>
#include <assert.h>

#include "Body.hh"
#include "SpaceTree.hh"
#include "BoxGeom.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
SpaceNode::SpaceNode( SpaceNode *parent, float x, float y, float hl, float hh )
  : parent( parent ), isLeaf( false ), cx( x ), cy( y ), 
    halfWidth( hl ), halfHeight( hh ), 
    walls(NULL), wallCount(0), wallMaxCount(0),
    geoms(NULL), geomCount(0), geomMaxCount(0)
{
  for (int i=0; i<4; i++)
    this->children[i] = NULL;
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
SpaceNode::~SpaceNode()
{
  int i=0;

  this->parent = NULL;

  for (int i=0; i<this->geomCount; i++)
  {
    delete this->geoms[i];
  }

  free(geoms);

  // Delete all children
  if (this->isLeaf)
  {
    dSpaceDestroy( this->spaceId );
  }
  else
  {
    // Delete all children
    for (i=0; i<4; i++)
    {
      delete this->children[i];
      this->children[i] = NULL;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
// Add a wall to this space
void SpaceNode::AddWall( Line *line )
{
  if (this->wallCount >= this->wallMaxCount)
  {
    this->wallMaxCount += 10;
    this->walls = (Line**) realloc(this->walls, this->wallMaxCount *
        sizeof(this->walls[0]));

    assert(this->walls);
  }

  this->walls[this->wallCount++] = line;
}

//////////////////////////////////////////////////////////////////////////////
// Add a geom to this space
void SpaceNode::AddGeom( Geom *geom )
{
  if (this->geomCount >= this->geomMaxCount)
  {
    this->geomMaxCount += 10;
    this->geoms = (Geom**) realloc(this->geoms, this->geomMaxCount *
        sizeof(this->geoms[0]));

    assert(this->geoms);
  }

  this->geoms[this->geomCount++] = geom;
}

//////////////////////////////////////////////////////////////////////////////
// Create the ODE geometries
void SpaceNode::GenerateGeoms( dSpaceID parentSpace, Body *body, 
                               float wallWidth, float wallHeight, 
                               Vector3 &pos, float scale, 
                               Vector3 color)
{

  // Generate recursive spaces (maximizes collision detection performance)
  this->spaceId = dSimpleSpaceCreate( parentSpace );

  // We are a space of fixed objects, and shouldnt collide with ourself
  dGeomSetCategoryBits( (dGeomID) this->spaceId, GZ_FIXED_COLLIDE );
  dGeomSetCollideBits( (dGeomID) this->spaceId, ~GZ_FIXED_COLLIDE );

  if (this->isLeaf)
  {

    for (int i=0; i<this->wallCount; i++)
    {
      std::ostringstream stream;

      // Align the walls with the ground properly
      float z = wallHeight/2.0;

      // Create the box geometry
      BoxGeom* newBox = new BoxGeom( body );

      XMLConfig *boxConfig = new XMLConfig();
      Quatern rot;
      Vector3 rpy;
      rot.SetFromAxis( 0, 0, 1, this->walls[i]->Angle() );
      rpy = rot.GetAsEuler();

      stream << "<gazebo:world xmlns:gazebo=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#gz\" xmlns:geom=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#geom\">"; 

      stream << "<geom:box name='occ_geom'>";
      stream << "  <mass>0.0</mass>";
      stream << "  <xyz>" << this->walls[i]->Mid().x * scale + pos.x << " " <<
                   this->walls[i]->Mid().y * scale + pos.y << " " << z + pos.z << "</xyz>";
      stream << "  <rpy>" << RTOD(rpy.x) << " " << RTOD(rpy.y) << " " << RTOD(rpy.z) << "</rpy>";
      stream << "  <size>" << this->walls[i]->Length() * scale + wallWidth * scale << " " << wallWidth * scale << " " << wallHeight << "</size>";
      stream << "  <visual>";
      stream << "    <mesh>unit_box</mesh>";
      stream << "    <material>Gazebo/Rocky</material>";
      stream << "    <size>" << this->walls[i]->Length() * scale + wallWidth * scale << " " << wallWidth * scale << " " << wallHeight << "</size>";
      stream << "  </visual>";
      stream << "</geom:box>";
      stream << "</gazebo:world>";

      boxConfig->LoadString( stream.str() );

      newBox->Load( boxConfig->GetRootNode()->GetChild() );
      delete boxConfig;
    }
  }
  else if (children[0])
  {
    for (int i=0; i<4; i++)
      this->children[i]->GenerateGeoms( this->spaceId, body, wallWidth, 
          wallHeight, pos, scale,color);
  }

  /* TESTING
  // Display the actual bounding box of the space
  if (dSpaceGetNumGeoms(this->spaceId) > 0)
  {
    dReal box[6];
    dGeomGetAABB((dGeomID) this->spaceId, box);

    BoxGeom *bBox = new BoxGeom( 0,
                                 box[1] - box[0],
                                 box[3] - box[2],
                                 box[5] - box[4]);
    bBox->SetPosition((box[1] + box[0]) / 2,
                      (box[3] + box[2]) / 2,
                      (box[5] + box[4]) / 2);

    bBox->SetColor( 1, 0, 0);

    bBox->SetCategoryBits( 0 );
    bBox->SetCollideBits( 0 );

    this->geoms.PushBack( bBox );
  }
  */

  // Destroy the newly created space if it has nothing in it.  This is
  // particularly important for getting the recursive spaces right.
  if (dSpaceGetNumGeoms(this->spaceId) == 0)
  {
    dSpaceDestroy(this->spaceId);
    this->spaceId = 0;
  }

  return;
}



//////////////////////////////////////////////////////////////////////////////
// Constructor
SpaceTree::SpaceTree()
  : root( NULL ), linesPerNode( 10 )
{
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
SpaceTree::~SpaceTree()
{
  // Delete the entire tree
  if (this->root)
    delete this->root;
}


//////////////////////////////////////////////////////////////////////////////
// Return the root node
SpaceNode *SpaceTree::GetRoot() const
{
  return this->root;
}


//////////////////////////////////////////////////////////////////////////////
// Build the tree from a list of line segements. 
void SpaceTree::BuildTree( Line **lines, unsigned int lineCount,
                                float width, float height )
{

  if (lineCount <= this->linesPerNode)
  {
    // Create the root node, and make it a leaf
    this->root = new SpaceNode( NULL, width/2.0, height/2.0, 
                                width/2.0, height/2.0);
    this->root->isLeaf = true;

    // Add all the lines to the root node
    for (unsigned int i=0; i<lineCount; i++)
      this->root->AddWall( lines[i] );
  }
  else
  {

    // Recursibely create the tree
    this->root = this->Construct( NULL, width/2.0, height/2.0, width/2.0, 
                                  height/2.0, lines, lineCount );
  }

}


//////////////////////////////////////////////////////////////////////////////
// Called from BuildTree to recursively build the tree
SpaceNode *SpaceTree::Construct( SpaceNode *parent, float cx, float cy, 
                                 float hw, float hh,  
                                 Line** lines, unsigned int lineCount )
{

  SpaceNode *node = new SpaceNode( parent, cx, cy, hw, hh );

  // If the count is within range, then create a leaf node
  if (lineCount < this->linesPerNode)
  {
    node->isLeaf = true;

    for (unsigned int i=0; i<lineCount; i++)
      node->AddWall( lines[i] );

  } 

  // Otherwise create a new node and its children 
  else if (lineCount > 0) 
  {
    delete node->walls;
    unsigned int i=0;

    // Grab the lines contained within this node
    while (i != lineCount)
    {
      if (lines[i]->Mid().x < cx+hw && lines[i]->Mid().x >= cx-hw &&
          lines[i]->Mid().y < cy+hh && lines[i]->Mid().y >= cy-hh )
      {
        node->AddWall( lines[i] );
        //iter = lines.Erase( iter );
      }
      i++;
    }

    // Create the node's children
    node->children[0] = this->Construct( node, cx+hw/2.0, cy+hh/2.0, hw/2.0, 
                                         hh/2.0, node->walls, node->wallCount );
    node->children[1] = this->Construct( node, cx-hw/2.0, cy+hh/2.0, hw/2.0, 
                                         hh/2.0, node->walls, node->wallCount );
    node->children[2] = this->Construct( node, cx-hw/2.0, cy-hh/2.0, hw/2.0, 
                                         hh/2.0, node->walls, node->wallCount);
    node->children[3] = this->Construct( node, cx+hw/2.0, cy-hh/2.0, hw/2.0, 
                                         hh/2.0, node->walls, node->wallCount );
  }

  return node;
}

