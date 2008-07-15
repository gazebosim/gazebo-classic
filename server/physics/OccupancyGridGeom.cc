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
/* Desc: OccupancyGrid geometry
 * Author: Nate Koenig
 * Date: 14 July 2008
 * CVS: $Id:$
 */

#include <ode/ode.h>
#include <Ogre.h>
#include <iostream>
#include <string.h>
#include <math.h>

#include "GazeboError.hh"
#include "OgreAdaptor.hh"
#include "Simulator.hh"
#include "OgreAdaptor.hh"
#include "Global.hh"
#include "Body.hh"
#include "OccupancyGridGeom.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
OccupancyGridGeom::OccupancyGridGeom(Body *body)
    : Geom(body)
{
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
OccupancyGridGeom::~OccupancyGridGeom()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Update function.
void OccupancyGridGeom::UpdateChild()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Load the heightmap
void OccupancyGridGeom::LoadChild(XMLConfigNode *node)
{
  OgreAdaptor *ogreAdaptor;

  ogreAdaptor = Simulator::Instance()->GetRenderEngine();

  std::string imageFilename = node->GetString("image","",1);
  this->mapSize = node->GetVector3("size",Vector3(10,10,10));

  this->negative = node->GetBool("negative", 0);
  this->threshold = node->GetDouble( "threshold", 200.0);

  this->wallWidth = node->GetDouble( "width", 0.1, 0 );
  this->wallHeight = node->GetDouble( "height", 1.0, 0 );

  //this->color = node->GetColor( "color", GzColor(1.0, 1.0, 1.0) );

  // Make sure they are ok
  if (this->scale <= 0) this->scale = 0.1;
  if (this->threshold <=0) this->threshold = 200;
  if (this->wallWidth <=0) this->wallWidth = 0.1;
  if (this->wallHeight <= 0) this->wallHeight = 1.0;
  if (this->errBound <= 0) this->errBound = 0.0;


  // Load the image 
  this->mapImage.load(imageFilename,
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);


  // Limit the format to 8-bit grayscale
  if (this->mapImage.getFormat() != Ogre::PF_L8)
  {
    gzerr(0) << "Invalid image format[" << this->mapImage.getFormat() << "]\n";
  }

  // Create the 2d lines of the map
  this->GenerateLines();
  std::cout << "OccupancyGrid: found [%d] walls" << this->wallCount;
      
  // Create the quad tree
  /*this->tree = new SpaceTree();
  tree->BuildTree( this->walls, this->wallCount, 
                   this->mapWidth, this->mapHeight );

  // Create the extruded geometry
  this->GenerateGeometry();
  */
}

//////////////////////////////////////////////////////////////////////////////
// Get the 2d lines from an image file
void OccupancyGridGeom::GenerateLines()
{
  unsigned int x, y;
  unsigned char v;
  MapPoint **map;
  MapPoint *pt;
  const unsigned char *imageData;
  Ogre::ColourValue pixColor;

  this->mapWidth = this->mapImage.getWidth();
  this->mapHeight = this->mapImage.getHeight();

  //imageData = this->mapImage->getData();

  // We may need to change the position of the map
  if (this->halign == "center")
    this->pos.x -= this->mapWidth / 2.0 * this->scale;
  if (this->valign == "center")
    this->pos.y -= this->mapHeight / 2.0 * this->scale;

  // Allocate on heap (some platforms have stack-size limit)
  map = new MapPoint*[this->mapWidth * this->mapHeight];

  for (y=0; y< this->mapHeight; y++)
  {
    for (x=0; x<this->mapWidth; x++)
    {
      // Compute the pixel value
      pixColor = this->mapImage.getColourAt(x, y, 0);
      v = (unsigned char)(255 * ((pixColor[0] + pixColor[1] + pixColor[2]) / 3.0));

      if (this->negative)
        v = 255 - v;
 
      std::cout << "XY[" << x << " " << y << "] Color[" << pixColor[0] << " " << pixColor[1] << " " << pixColor[2] << "]";
      printf("V[%d]\n",v);

      // If the image data is beyond the threshold, then create a new map
      // point
      if (v >= this->threshold)
      {
        // Create the new point
        map[r*this->mapWidth+c] = new MapPoint( c, r );

        // Point to the North
        if (r>0 && map[(r-1)*this->mapWidth+c] != NULL)
        {
          map[r*this->mapWidth+c]->arcs[0] = map[(r-1)*this->mapWidth+c];
          map[r*this->mapWidth+c]->arcCount++;

          map[(r-1)*this->mapWidth+c]->arcs[4] = map[r*this->mapWidth+c];
          map[(r-1)*this->mapWidth+c]->arcCount++;
        }

        // Point the NorthWest
        if (c>0 && r>0 && map[(r-1)*this->mapWidth+c-1] != NULL)
        {
          map[r*this->mapWidth+c]->arcs[7] = map[(r-1)*this->mapWidth+c-1];
          map[r*this->mapWidth+c]->arcCount++;

          map[(r-1)*this->mapWidth+c-1]->arcs[3] = map[r*this->mapWidth+c];
          map[(r-1)*this->mapWidth+c-1]->arcCount++;
        }
 
        // Point to the West
        if (c>0 && map[r*this->mapWidth+c-1] != NULL)
        {
          map[r*this->mapWidth+c]->arcs[6] = map[r*this->mapWidth+c-1]; 
          map[r*this->mapWidth+c]->arcCount++;

          map[r*this->mapWidth+c-1]->arcs[2] = map[r*this->mapWidth+c];
          map[r*this->mapWidth+c-1]->arcCount++;
        }

        // Point to the NorthEast
        if (c+1<this->mapWidth && r>0 && map[(r-1)*this->mapWidth+c+1] != NULL)
        {
          map[r*this->mapWidth+c]->arcs[1] = map[(r-1)*this->mapWidth+c+1]; 
          map[r*this->mapWidth+c]->arcCount++;

          map[(r-1)*this->mapWidth+c+1]->arcs[5] = map[r*this->mapWidth+c];
          map[(r-1)*this->mapWidth+c+1]->arcCount++;
        }
       
        // Point to the East
        //     if (c+1<this->mapWidth && map[r*this->mapWidth+c+1] != NULL)
        //       {
        //       map[r*this->mapWidth+c]->arcs[2] = map[r*this->mapWidth+c+1]; 
        //       map[r*this->mapWidth+c]->arcCount++;

        //       map[r*this->mapWidth+c+1]->arcs[6] = map[r*this->mapWidth+c];
        //       map[r*this->mapWidth+c+1]->arcCount++;
        //       }
         
      }
      else
      {
        map[r*this->mapWidth+c] = NULL;
      }

    }
  }
  int i=0;

  for (r=0; r<this->mapHeight; r++)
  {
    for (c=0; c<this->mapWidth; c++)
    {
      pt = map[r*this->mapWidth+c];

      // If the map point is valid
      if (pt != NULL && pt->arcCount < 8)
      {
        // Generate a line to the east
        this->GenerateLine(pt,2);

        // Generate a line to the south
        this->GenerateLine(pt,4);

        delete pt;
        pt = NULL;
      }
    }
  }

  // Free the map data
  delete [] map;

  // Free the image
  delete dataSet;

  // If the user has specified an error bound, then reduce the graph. 
  // This will only reduce curved surefaces
  if (this->errBound > 0)
  {
    ReduceLines();
  }

}
/*
//////////////////////////////////////////////////////////////////////////////
// Create a single line starting from point pt in direction dir
void OccupancyGridGeom::GenerateLine( MapPoint* pt, int dir )
{
  assert( dir>=0 && dir<=7 );

  if (pt->arcs[dir] != NULL)
  {
    MapPoint* next = pt;
    MapPoint* tmp;
    //float err=0;

    while (next->arcs[dir] != NULL && next->arcs[dir]->arcCount < 8)
    {
      tmp = next->arcs[dir];
      next->arcCount=8;
      next->arcs[dir] = NULL;
      tmp->arcs[(dir+4)%8] = NULL;
      next = tmp;
    }

    if (next != pt)
    {
      Line* newLine = new Line();
      newLine->Set( *pt, *next );
      this->AddWall( newLine );
    }
  }

}


//////////////////////////////////////////////////////////////////////////////
// Reduce the number of lines in the map
void OccupancyGridGeom::ReduceLines()
{
  int i,j;
  Line *line1 = NULL; 
  Line *line2 = NULL;


  // Loop through all the walls
  for( i=0; i<this->wallCount; i++)
  {
    line1 = this->walls[i];

    // Loop through all the walls
    for (j=i + 1; j<this->wallCount; j++)
    {
      line2 = this->walls[j];

      // If the end point of one line connects with the start point of
      // another, then try to reduce
      if ((line1->End() == line2->Start() && 
            (fabs( line1->Start().x - line2->End().x ) + 
             fabs( line1->Start().y - line2->End().y ) <= this->errBound)))

      {
        line1->End( line2->End() );
        this->EraseWall(j);
        j--;
      } 

      // If the start point of one line connects with the start point of
      // another, then try to reduce
      else if (i != j && (line1->Start() == line2->Start()) &&
          (fabs( line1->End().x - line2->End().x ) + 
           fabs( line1->End().y - line2->End().y ) <= this->errBound))
      {

        line1->Set( line1->End(), line2->End() );
        this->EraseWall(j);
        j--;
      } 
     
      // If the end point of one line connects with the end point of
      // another, then try to reduce
      else if (i != j && (line1->End() == line2->End()) &&
          (fabs( line1->Start().x - line2->Start().x ) + 
           fabs( line1->Start().y - line2->Start().y ) <= this->errBound))
      {
        line1->End( line2->Start() );
        this->EraseWall(j);
        j--;
      }
    }

  }

}

//////////////////////////////////////////////////////////////////////////////
// Create the geometry from a list of 2d lines
void OccupancyGridGeom::GenerateGeometry()
{
  // We are a space of fixed objects, and shouldnt collide with ourself
  dGeomSetCategoryBits( (dGeomID) this->modelSpaceId, GZ_FIXED_COLLIDE );
  dGeomSetCollideBits( (dGeomID) this->modelSpaceId, ~GZ_FIXED_COLLIDE );
  
  this->tree->GetRoot()->GenerateGeoms( this->modelSpaceId, this->body,
      this->wallWidth, this->wallHeight, this->pos,
      this->scale, this->color, this->shadeModel, this->polygonMode);

  this->AddSpaceGeoms(this->tree->GetRoot());
}

void OccupancyGridGeom::AddSpaceGeoms( SpaceNode *node)
{
  //int i = 0;
  if (node==NULL)
    return;

  // FIX
  //for (i=0; i<node->geomCount; i++)
  //{
  //  this->AddGeom(node->geoms[i]);
  //}

  //for (i=0; i<4; i++)
  //{
  //  this->AddSpaceGeoms(node->children[i]);
  //}
}


//////////////////////////////////////////////////////////////////////////////
// Calculates the shortest distance from a point to a line
float OccupancyGridGeom::PointLineDist( const MapPoint &p1, const MapPoint &p2, 
                                  const MapPoint &p )
{
  return fabs( (p2.x-p1.x)*(p1.y-p.y) - (p1.x-p.x)*(p2.y-p1.y) ) / 
         sqrt( (p2.x-p1.x)*(p2.x-p1.x) + (p2.y-p1.y)*(p2.y-p1.y) );
}

//////////////////////////////////////////////////////////////////////////////
// Adds a wall
void OccupancyGridGeom::AddWall( Line *line )
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
// Deletes a wall segment
void OccupancyGridGeom::EraseWall( int index )
{
  assert( index >=0 && index < this->wallCount );

  delete this->walls[index];
  this->walls[index] = 0;

  this->wallCount--;

  for (int i=index; i<this->wallCount; i++)
  {
    this->walls[i] = this->walls[i+1];
  }

}


Line::Line()
{
}

Line::~Line()
{
}

bool Line::operator==( const Line& l )
{
  return this->start == l.start && this->end == l.end; 
}

void Line::Set( const MapPoint &start, const MapPoint &end )
{
  this->start = start;
  this->end = end;

  this->Calc();
}

void Line::Start( const MapPoint &start )
{
  this->start = start;
  this->Calc();
}

void Line::End( const MapPoint &end )
{
  this->end = end;
  this->Calc();
}

const MapPoint& Line::Start() const
{
  return this->start;
}

const MapPoint& Line::End() const
{
  return this->end;
}

const MapPoint& Line::Mid() const
{
  return this->mid;
}

float Line::Length() const
{
  return this->length;
}

float Line::Angle() const
{
  return this->angle;
}

void Line::Calc()
{

  // The length of the line segment
  this->length = sqrt( pow( this->start.x - this->end.x,2 ) + 
      pow(this->start.y - this->end.y,2 ) );

  // The angle of the line segement
  if (this->start.x == this->end.x)
    this->angle = M_PI/2.0;
  else
    this->angle = atan2( (this->start.y - this->end.y),
                         (this->start.x - this->end.x) );

  // Calc. the center x location
  if (this->start.x - this->end.x <= 0)
  {
    this->mid.x = this->start.x + fabs( this->start.x - this->end.x ) / 2.0;
  } else {
    this->mid.x = this->start.x - fabs(this->start.x - this->end.x ) / 2.0;
  }

  // Calc. the center y location
  if (this->start.y - this->end.y <= 0)
  {
    this->mid.y = this->start.y + fabs( this->start.y - this->end.y ) / 2.0; 
  } else {
    this->mid.y = this->start.y - fabs( this->start.y - this->end.y ) / 2.0; 
  }
}

*/
