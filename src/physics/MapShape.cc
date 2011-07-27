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
/* Desc: Map shape
 * Author: Nate Koenig
*/

#include <string.h>
#include <math.h>

#include "common/Image.hh"
#include "common/Exception.hh"

#include "physics/World.hh"
#include "physics/PhysicsEngine.hh"
#include "physics/BoxShape.hh"
#include "physics/Geom.hh"
#include "physics/MapShape.hh"

using namespace gazebo;
using namespace physics;


unsigned int MapShape::geomCounter = 0;

//////////////////////////////////////////////////////////////////////////////
// Constructor
MapShape::MapShape(GeomPtr parent)
    : Shape(parent)
{
  this->AddType(Base::MAP_SHAPE);

  this->root = new QuadNode(NULL);
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
MapShape::~MapShape()
{
  if (this->root)
    delete this->root;

  if (this->mapImage)
    delete this->mapImage;
  this->mapImage = NULL;
}

//////////////////////////////////////////////////////////////////////////////
/// Update function.
void MapShape::Update()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Load the heightmap
void MapShape::Load( sdf::ElementPtr _sdf )
{
  Shape::Load(_sdf);

  std::string imageFilename = _sdf->GetValueString("filename");

  // Make sure they are ok
  if (_sdf->GetValueDouble("scale") <= 0) 
    _sdf->GetAttribute("scale")->Set( 0.1 );
  if (this->sdf->GetValueInt("threshold") <=0) 
    _sdf->GetAttribute("threshold")->Set(200);
  if (this->sdf->GetValueDouble("height") <= 0) 
    _sdf->GetAttribute("height")->Set( 1.0 );

  // Load the image 
  this->mapImage = new common::Image();
  this->mapImage->Load(imageFilename);

  if (!this->mapImage->Valid())
    gzthrow(std::string("Unable to open image file[") + imageFilename + "]" );
}

//////////////////////////////////////////////////////////////////////////////
// Init the map
void MapShape::Init()
{
  this->root->x = 0;
  this->root->y = 0;

  this->root->width = this->mapImage->GetWidth();
  this->root->height = this->mapImage->GetHeight();

  this->BuildTree(this->root);

  this->merged = true;
  while (this->merged)
  {
    this->merged =false;
    this->ReduceTree(this->root);
  }

  this->CreateBoxes(this->root);
}


//////////////////////////////////////////////////////////////////////////////
// Create the ODE boxes
void MapShape::CreateBoxes(QuadNode * /*_node*/)
{
  /*TODO: fix this to use SDF 
  if (node->leaf)
  {
    if (!node->valid || !node->occupied)
      return;

    std::ostringstream stream;

    // Create the box geometry
    GeomPtr geom = this->GetWorld()->GetPhysicsEngine()->CreateGeom("box", this->geomParent->GetLink());
    geom->SetSaveable(false);

    stream << "<gazebo:world xmlns:gazebo=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#gz\" xmlns:geom=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#geom\">"; 

    float x = (node->x + node->width / 2.0) * this->sdf->GetValueDouble("scale");
    float y = (node->y + node->height / 2.0) * this->sdf->GetValueDouble("scale");
    float z = this->sdf->GetValueDouble("height") / 2.0;
    float xSize = (node->width) * this->sdf->GetValueDouble("scale");
    float ySize = (node->height) * this->sdf->GetValueDouble("scale");
    float zSize = this->sdf->GetValueDouble("height");

    char geomName[256];
    sprintf(geomName,"map_geom_%d",geomCounter++);

    stream << "<geom:box name='" << geomName << "'>";
    stream << "  <xyz>" << x << " " << y << " " << z << "</xyz>";
    stream << "  <rpy>0 0 0</rpy>";
    stream << "  <size>" << xSize << " " << ySize << " " << zSize << "</size>";
    stream << "  <static>true</static>";
    stream << "  <visual>";
    stream << "    <mesh>unit_box</mesh>";
    stream << "    <material>" << this->materialP->GetValue() << "</material>";
    stream << "    <size>" << xSize << " "<< ySize << " " << zSize << "</size>";
    stream << "  </visual>";
    stream << "</geom:box>";
    stream << "</gazebo:world>";

    boxConfig->LoadString( stream.str() );

    geom->SetStatic(true);
    geom->Load( boxConfig->GetRootNode()->GetChild() );

    delete boxConfig;
  }
  else
  {
    std::deque<QuadNode*>::iterator iter;
    for (iter = node->children.begin(); iter != node->children.end(); iter++)
    {
      this->CreateBoxes(*iter);
    }
  }
  */
}

//////////////////////////////////////////////////////////////////////////////
// Reduce the size of the quad tree
void MapShape::ReduceTree(QuadNode *node)
{
  std::deque<QuadNode*>::iterator iter;

  if (!node->valid)
    return;

  if (!node->leaf)
  {
    unsigned int count = 0;
    int size = node->children.size();

    for (int i = 0; i < size; i++)
    {
      if (node->children[i]->valid)
      {
        this->ReduceTree(node->children[i]);
      }
      if (node->children[i]->leaf)
        count++;
    }

    if (node->parent && count == node->children.size())
    {
      for (iter = node->children.begin(); iter != node->children.end(); iter++)
      {
        node->parent->children.push_back( *iter );
        (*iter)->parent = node->parent;
      }
      node->valid = false;
    }
    else
    {
      bool done = false;
      while (!done)
      {
        done = true;
        for (iter = node->children.begin(); 
             iter != node->children.end();iter++ )
        {
          if (!(*iter)->valid)
          {
            node->children.erase(iter, iter+1);
            done = false;
            break;
          }
        }
      }
    }

  }
  else
  {
    this->Merge(node, node->parent);
  }
}

//////////////////////////////////////////////////////////////////////////////
// Merege quad tree cells
void MapShape::Merge(QuadNode *nodeA, QuadNode *nodeB)
{
  std::deque<QuadNode*>::iterator iter;

  if (!nodeB)
    return;

  if (nodeB->leaf)
  {
    if (nodeB->occupied != nodeA->occupied)
      return;

    if ( nodeB->x == nodeA->x + nodeA->width && 
         nodeB->y == nodeA->y && 
         nodeB->height == nodeA->height )
    {
      nodeA->width += nodeB->width;
      nodeB->valid = false;
      nodeA->valid = true;

      this->merged = true;
    }

    if (nodeB->x == nodeA->x && 
        nodeB->width == nodeA->width &&
        nodeB->y == nodeA->y + nodeA->height )
    {
      nodeA->height += nodeB->height;
      nodeB->valid = false;
      nodeA->valid = true;

      this->merged = true;
    }
  }
  else
  {

    for (iter = nodeB->children.begin(); iter != nodeB->children.end(); iter++)
    {
      if ((*iter)->valid)
      {
        this->Merge(nodeA, (*iter));
      }
    }
  }
}


//////////////////////////////////////////////////////////////////////////////
// Construct the quad tree
void MapShape::BuildTree(QuadNode *node)
{
  QuadNode *newNode = NULL;
  unsigned int freePixels, occPixels;

  this->GetPixelCount(node->x, node->y, node->width, node->height, 
                      freePixels, occPixels);

  //int diff = labs(freePixels - occPixels);

  if ((int)(node->width*node->height) > this->sdf->GetValueInt("granularity"))
  {
    float newX, newY;
    float newW, newH;
   
    newY = node->y;
    newW = node->width / 2.0;
    newH = node->height / 2.0;

    // Create the children for the node
    for (int i=0; i<2; i++)
    {
      newX = node->x;

      for (int j=0; j<2; j++)
      {
        newNode = new QuadNode(node);
        newNode->x = (unsigned int)newX;
        newNode->y = (unsigned int)newY;

        if (j == 0)
          newNode->width = (unsigned int)floor(newW);
        else
          newNode->width = (unsigned int)ceil(newW);

        if (i==0)
          newNode->height = (unsigned int)floor(newH);
        else
          newNode->height = (unsigned int)ceil(newH);

        node->children.push_back(newNode);

        this->BuildTree(newNode);

        newX += newNode->width;

        if (newNode->width == 0 || newNode->height ==0)
          newNode->valid = false;
      }

      if (i==0)
        newY += floor(newH);
      else
        newY += ceil(newH);
    }

    //node->occupied = true;
    node->occupied = false;
    node->leaf = false;
  }
  else if (occPixels == 0)
  {
    node->occupied = false;
    node->leaf = true;
  }
  else
  {
    node->occupied = true;
    node->leaf = true;
  }

}

//////////////////////////////////////////////////////////////////////////////
// Get a count of pixels within a given area
void MapShape::GetPixelCount(unsigned int xStart, unsigned int yStart, 
                                 unsigned int width, unsigned int height, 
                                 unsigned int &freePixels, 
                                 unsigned int &occPixels )
{
  common::Color pixColor;
  unsigned char v;
  unsigned int x,y;

  freePixels = occPixels = 0;

  for (y = yStart; y < yStart + height; y++)
  {
    for (x = xStart; x < xStart + width; x++)
    {
      pixColor = this->mapImage->GetPixel(x, y);

      v = (unsigned char)(255 * ((pixColor.R() + pixColor.G() + pixColor.B()) / 3.0));
      //if (this->sdf->GetValueBool("negative"))
        //v = 255 - v;

      if (v > this->sdf->GetValueInt("threshold"))
        freePixels++;
      else
        occPixels++;
    }
  }
}
