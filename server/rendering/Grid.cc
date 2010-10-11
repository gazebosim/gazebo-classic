/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>

#include <sstream>

#include "Scene.hh"
#include "OgreAdaptor.hh"
#include "Grid.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Grid::Grid( Scene *scene,  unsigned int cell_count, float cell_length, 
            float line_width, const Color& color )
: scene( scene )
{
  this->height = 0;

  Param::Begin(&this->parameters);
  this->cellCountP = new ParamT<unsigned int>("cellCount",0,0);
  this->cellLengthP = new ParamT<float>("cellLength",1,0);
  this->lineWidthP = new ParamT<float>("lineWidth",0.03,0);
  this->colorP = new ParamT<Color>("color",Color(1,1,1,1),0);
  this->h_offsetP = new ParamT<float>("height_offset",0,0);
  Param::End();

  this->cellCountP->SetValue(cell_count);
  this->cellLengthP->SetValue(cell_length);
  this->lineWidthP->SetValue(line_width);
  this->colorP->SetValue(color);
  this->h_offsetP->SetValue(0.005);

  static uint32_t gridCount = 0;
  std::stringstream ss;
  ss << "Grid" << gridCount++;

  this->name = ss.str();
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Grid::~Grid()
{
  this->scene->GetManager()->destroySceneNode( this->sceneNode->getName() );
  this->scene->GetManager()->destroyManualObject( this->manualObject );

  this->material->unload();
}

////////////////////////////////////////////////////////////////////////////////
void Grid::SetCellCount(uint32_t count)
{
  this->cellCountP->SetValue(count);

  this->Create();
}

////////////////////////////////////////////////////////////////////////////////
void Grid::SetCellLength(float len)
{
  this->cellLengthP->SetValue(len);

  this->Create();
}

////////////////////////////////////////////////////////////////////////////////
void Grid::SetLineWidth(float width)
{
  this->lineWidthP->SetValue( width );

  this->Create();
}

////////////////////////////////////////////////////////////////////////////////
void Grid::SetColor(const Color& color)
{
  this->colorP->SetValue( color );

  this->material->setDiffuse( color.R(), color.G(), color.B(), color.A() );
  this->material->setAmbient( color.R(), color.G(), color.B() );

  if ( (**this->colorP).A() < 0.9998 )
  {
    this->material->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
    this->material->setDepthWriteEnabled( false );
  }
  else
  {
    this->material->setSceneBlending( Ogre::SBT_REPLACE );
    this->material->setDepthWriteEnabled( true );
  }

  this->Create();
}


////////////////////////////////////////////////////////////////////////////////
void Grid::SetHeight(uint32_t height)
{
  this->height = height;

  this->Create();
}

////////////////////////////////////////////////////////////////////////////////
// Init
void Grid::Init()
{
  this->manualObject = this->scene->GetManager()->createManualObject( this->name );

  Ogre::SceneNode *parent_node = this->scene->GetManager()->getRootSceneNode();

  this->sceneNode = parent_node->createChildSceneNode();
  this->sceneNode->attachObject( this->manualObject );

  std::stringstream ss;
  ss << this->name << "Material";
  this->material = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
  this->material->setReceiveShadows(false);
  this->material->getTechnique(0)->setLightingEnabled(false);

  this->SetColor(**this->colorP);

  this->Create();
}

////////////////////////////////////////////////////////////////////////////////
void Grid::Create()
{
  this->manualObject->clear();

  float extent = (**this->cellLengthP*((double)**this->cellCountP))/2;

  this->manualObject->setCastShadows(false);
  this->manualObject->estimateVertexCount( **this->cellCountP * 4 * this->height + ((**this->cellCountP + 1) * (**this->cellCountP + 1)));
  this->manualObject->begin( this->material->getName(), Ogre::RenderOperation::OT_LINE_LIST );

  for (uint32_t h = 0; h <= this->height; ++h)
  {
    float h_real = **this->h_offsetP + (this->height / 2.0f - (float)h) * **this->cellLengthP;
    for( uint32_t i = 0; i <= **this->cellCountP; i++ )
    {
      float inc = extent - ( i * **this->cellLengthP );

      Ogre::Vector3 p1(inc, -extent,h_real );
      Ogre::Vector3 p2(inc, extent ,h_real );
      Ogre::Vector3 p3(-extent,inc, h_real );
      Ogre::Vector3 p4(extent, inc, h_real );

      this->manualObject->position(p1);
      this->manualObject->colour( (**this->colorP).GetOgreColor() );
      this->manualObject->position(p2);
      this->manualObject->colour( (**this->colorP).GetOgreColor() );

      this->manualObject->position(p3);
      this->manualObject->colour( (**this->colorP).GetOgreColor() );
      this->manualObject->position(p4);
      this->manualObject->colour( (**this->colorP).GetOgreColor() );
    }
  }

  if (this->height > 0)
  {
    for (uint32_t x = 0; x <= **this->cellCountP; ++x)
    {
      for (uint32_t y = 0; y <= **this->cellCountP; ++y)
      {
        float x_real = extent - x * **this->cellLengthP;
        float y_real = extent - y * **this->cellLengthP;

        float z_top = (this->height / 2.0f) * **this->cellLengthP;
        float z_bottom = -z_top;

        this->manualObject->position( x_real, y_real, z_bottom );
        this->manualObject->colour( (**this->colorP).GetOgreColor() );
        this->manualObject->position(x_real, y_real, z_bottom);
        this->manualObject->colour( (**this->colorP).GetOgreColor() );
      }
    }
  }

  this->manualObject->end();
}

////////////////////////////////////////////////////////////////////////////////
void Grid::SetUserData( const Ogre::Any& data )
{
  this->manualObject->setUserAny( data );
}
