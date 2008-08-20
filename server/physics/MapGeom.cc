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
/* Desc: Map geometry
 * Author: Nate Koenig
 * Date: 14 July 2008
 * CVS: $Id:$
 */

#include <ode/ode.h>
#include <Ogre.h>
#include <iostream>
#include <string.h>
#include <math.h>

#include "BoxGeom.hh"
#include "GazeboError.hh"
#include "OgreAdaptor.hh"
#include "Simulator.hh"
#include "OgreAdaptor.hh"
#include "OgreVisual.hh"
#include "Global.hh"
#include "Body.hh"
#include "MapGeom.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
MapGeom::MapGeom(Body *body)
    : Geom(body)
{
  this->root = new QuadNode(NULL);

  this->negativeP = new Param<int>("negative", 0, 0);
  this->thresholdP = new Param<double>( "threshold", 200.0, 0);
  this->wallHeightP = new Param<double>( "height", 1.0, 0 );
  this->scaleP = new Param<double>("scale",1.0,0);
  this->materialP = new Param<std::string>("material", "", 0);
  this->granularityP = new Param<int>("granularity", 5, 0);
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
MapGeom::~MapGeom()
{
  if (this->root)
    delete this->root;

  delete this->negativeP;
  delete this->thresholdP;
  delete this->wallHeightP;
  delete this->scaleP;
  delete this->materialP;
  delete this->granularityP;
}

//////////////////////////////////////////////////////////////////////////////
/// Update function.
void MapGeom::UpdateChild()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Load the heightmap
void MapGeom::LoadChild(XMLConfigNode *node)
{
  OgreAdaptor *ogreAdaptor;

  ogreAdaptor = Simulator::Instance()->GetRenderEngine();

  std::string imageFilename = node->GetString("image","",1);

  this->negativeP->Load(node);
  this->thresholdP->Load(node);
  this->wallHeightP->Load(node);
  this->scaleP->Load(node);
  this->materialP->Load(node);
  this->granularityP->Load(node);

  // Make sure they are ok
  if (this->scaleP->GetValue() <= 0) this->scaleP->SetValue( 0.1 );
  if (this->thresholdP->GetValue() <=0) this->thresholdP->SetValue(200);
  if (this->wallHeightP->GetValue() <= 0) this->wallHeightP->SetValue( 1.0 );

  // Load the image 
  this->mapImage.load(imageFilename,
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  this->root->x = 0;
  this->root->y = 0;
  this->root->width = this->mapImage.getWidth();
  this->root->height = this->mapImage.getHeight();

  this->BuildTree(this->root);

  this->merged = true;
  while (this->merged)
  {
    this->merged =false;
    this->ReduceTree(this->root);
  }

  this->CreateBoxes(this->root);

  this->visualNode->MakeStatic();
}


//////////////////////////////////////////////////////////////////////////////
// Create the ODE boxes
void MapGeom::CreateBoxes(QuadNode *node)
{
  if (node->leaf)
  {
    if (!node->valid || !node->occupied)
      return;

    std::ostringstream stream;

    // Create the box geometry
    BoxGeom* newBox = new BoxGeom( this->body );

    XMLConfig *boxConfig = new XMLConfig();

    stream << "<gazebo:world xmlns:gazebo=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#gz\" xmlns:geom=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#geom\">"; 

    float x = (node->x + node->width / 2.0) * this->scaleP->GetValue();
    float y = (node->y + node->height / 2.0) * this->scaleP->GetValue();
    float z = this->wallHeightP->GetValue() / 2.0;
    float xSize = (node->width) * this->scaleP->GetValue();
    float ySize = (node->height) * this->scaleP->GetValue();
    float zSize = this->wallHeightP->GetValue();

    stream << "<geom:box name='map_geom'>";
    stream << "  <mass>0.0</mass>";
    stream << "  <xyz>" << x << " " << y << " " << z << "</xyz>";
    stream << "  <rpy>0 0 0</rpy>";
    stream << "  <size>" << xSize << " " << ySize << " " << zSize << "</size>";
    stream << "  <visual>";
    stream << "    <mesh>unit_box</mesh>";
    stream << "    <material>" << this->materialP->GetValue() << "</material>";
    stream << "  <size>" << xSize << " " << ySize << " " << zSize << "</size>";
    stream << "  </visual>";
    stream << "</geom:box>";
    stream << "</gazebo:world>";

    boxConfig->LoadString( stream.str() );

    newBox->Load( boxConfig->GetRootNode()->GetChild() );
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
}

//////////////////////////////////////////////////////////////////////////////
// Reduce the size of the quad tree
void MapGeom::ReduceTree(QuadNode *node)
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
void MapGeom::Merge(QuadNode *nodeA, QuadNode *nodeB)
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
void MapGeom::BuildTree(QuadNode *node)
{
  QuadNode *newNode = NULL;
  unsigned int freePixels, occPixels;

  this->GetPixelCount(node->x, node->y, node->width, node->height, 
                      freePixels, occPixels);

  int diff = labs(freePixels - occPixels);

  if (diff > this->granularityP->GetValue())
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

    node->occupied = true;
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
void MapGeom::GetPixelCount(unsigned int xStart, unsigned int yStart, 
                                 unsigned int width, unsigned int height, 
                                 unsigned int &freePixels, 
                                 unsigned int &occPixels )
{
  Ogre::ColourValue pixColor;
  unsigned char v;
  unsigned int x,y;

  freePixels = occPixels = 0;

  for (y = yStart; y < yStart + height; y++)
  {
    for (x = xStart; x < xStart + width; x++)
    {
      pixColor = this->mapImage.getColourAt(x, y, 0);
      v = (unsigned char)(255 * ((pixColor[0] + pixColor[1] + pixColor[2]) / 3.0));
      if (this->negativeP->GetValue())
        v = 255 - v;

      if (v > this->thresholdP->GetValue())
        freePixels++;
      else
        occPixels++;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
/// Save child parameters
void MapGeom::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(this->negativeP) << "\n";
  stream << prefix << *(this->thresholdP) << "\n";
  stream << prefix << *(this->wallHeightP) << "\n";
  stream << prefix << *(this->scaleP) << "\n";
  stream << prefix << *(this->materialP) << "\n";
  stream << prefix << *(this->granularityP) << "\n";
}
