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
/* Desc: QuadTree geometry
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
#include "SpaceTree.hh"
#include "QuadTreeGeom.hh"

using namespace gazebo;

//////////////////////////////////////////////////////////////////////////////
// Constructor
QuadTreeGeom::QuadTreeGeom(Body *body)
    : Geom(body)
{
  this->root = new QuadNode(NULL);
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
QuadTreeGeom::~QuadTreeGeom()
{
  if (this->root)
    delete this->root;
}

//////////////////////////////////////////////////////////////////////////////
/// Update function.
void QuadTreeGeom::UpdateChild()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Load the heightmap
void QuadTreeGeom::LoadChild(XMLConfigNode *node)
{
  OgreAdaptor *ogreAdaptor;

  ogreAdaptor = Simulator::Instance()->GetRenderEngine();

  std::string imageFilename = node->GetString("image","",1);
  this->mapSize = node->GetVector3("size",Vector3(10,10,10));

  this->negative = node->GetBool("negative", 0);
  this->threshold = node->GetDouble( "threshold", 200.0);

  this->wallHeight = node->GetDouble( "height", 1.0, 0 );

  //this->color = node->GetColor( "color", GzColor(1.0, 1.0, 1.0) );

  // Make sure they are ok
  if (this->scale <= 0) this->scale = 0.1;
  if (this->threshold <=0) this->threshold = 200;
  if (this->wallHeight <= 0) this->wallHeight = 1.0;
  if (this->errBound <= 0) this->errBound = 0.0;

  // Load the image 
  this->mapImage.load(imageFilename,
                Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  this->root->x = 0;
  this->root->y = 0;
  this->root->width = this->mapImage.getWidth();
  this->root->height = this->mapImage.getHeight();

  this->BuildTree(this->root);
  this->root->Print(" ");

  printf("\n\n");

  this->merged = true;
  while (this->merged)
  {
    printf("---------------------------\n");
    this->merged =false;
    this->ReduceTree(this->root);
  }

  this->root->Print(" ");

}

void QuadTreeGeom::ReduceTree(QuadNode *node)
{
  std::vector<QuadNode*>::iterator iter;

  if (!node->valid)
    return;

  if (!node->leaf)
  {
    for (iter = node->children.begin(); iter != node->children.end(); iter++)
    {
      if ((*iter)->valid)
        this->ReduceTree((*iter));
    }
  }
  else
  {
    this->Merge(node, this->root);

/*    for (unsigned int i=0; i<4; i++)
    {
      QuadNode *sibling = node->parent->children[i];
      this->Merge(node, sibling);

      if ( sibling->x == node->x+node->width && 
           sibling->y == node->y && 
           sibling->occupied == node->occupied)
      {
        node->width += sibling->width;
        sibling->valid = false;
        node->valid = true;

        printf("Combine XY[%d %d][%d] XY[%d %d][%d]\n",node->x, node->y, node->occupied, sibling->x, sibling->y, sibling->occupied);
      }
    }*/
  }
}

void QuadTreeGeom::Merge(QuadNode *nodeA, QuadNode *nodeB)
{
  std::vector<QuadNode*>::iterator iter;

  if (!nodeB)
    return;

  if (nodeB->leaf)
  {
    if ( nodeB->x == nodeA->x + nodeA->width && 
         nodeB->y == nodeA->y && 
         nodeB->occupied == nodeA->occupied )
    {
      nodeA->width += nodeB->width;
      nodeB->valid = false;
      nodeA->valid = true;

      this->merged = true;

      printf("Combine XY[%d %d][%d] XY[%d %d][%d]\n",nodeA->x, nodeA->y, nodeA->occupied, nodeB->x, nodeB->y, nodeB->occupied);
    }

    /*if (nodeB->x == nodeA->x && nodeB->width == nodeA->width &&
        nodeB->y == nodeA->y + nodeA->height &&
        nodeB->occupied == nodeA->occupied)
    {
      nodeA->height += nodeA->height;
      nodeB->valid = false;
      nodeA->valid = true;

      this->merged = true;

      printf("Combine XY[%d %d][%d] XY[%d %d][%d]\n",nodeA->x, nodeA->y, nodeA->occupied, nodeB->x, nodeB->y, nodeB->occupied);

    }*/
  }
  else
  {

    for (iter = nodeB->children.begin(); iter != nodeB->children.end(); iter++)
    {
      if ((*iter)->valid)
        this->Merge(nodeA, (*iter));
    }
  }
}


void QuadTreeGeom::BuildTree(QuadNode *node)
{
  QuadNode *newNode = NULL;
  unsigned int freePixels, occPixels;

  this->GetPixelCount(node->x, node->y, node->width, node->height, 
                      freePixels, occPixels);

  if (freePixels > 0 && occPixels > 0)
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
    node->parent->leaves++;
  }
  else
  {
    node->occupied = true;
    node->leaf = true;
    node->parent->leaves++;
  }

}

void QuadTreeGeom::GetPixelCount(unsigned int xStart, unsigned int yStart, 
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
      if (this->negative)
        v = 255 - v;

      if (v < this->threshold)
        freePixels++;
      else
        occPixels++;
    }
  }
}

