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

#include <sstream>

#include "rendering/ogre.h"

#include "rendering/Conversions.hh"
#include "rendering/Scene.hh"
#include "rendering/Grid.hh"

using namespace gazebo;
using namespace rendering;


////////////////////////////////////////////////////////////////////////////////
/// Constructor
Grid::Grid( Scene *scene_,  unsigned int cellCount_, float cellLength_, 
            float lineWidth_, const common::Color& color_ )
: scene( scene_ )
{
  this->height = 0;

  this->cellCountP = cellCount_;
  this->cellLengthP = cellLength_;
  this->lineWidthP = lineWidth_;
  this->colorP = color_;
  this->h_offsetP = 0.001;

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
void Grid::SetCellCount(uint32_t count_)
{
  this->cellCountP = count_;

  this->Create();
}

////////////////////////////////////////////////////////////////////////////////
void Grid::SetCellLength(float len_)
{
  this->cellLengthP = len_;

  this->Create();
}

////////////////////////////////////////////////////////////////////////////////
void Grid::SetLineWidth(float width_)
{
  this->lineWidthP = width_;

  this->Create();
}

////////////////////////////////////////////////////////////////////////////////
void Grid::SetColor(const common::Color& color_)
{
  this->colorP = color_;

  this->material->setDiffuse( color_.R(), color_.G(), color_.B(), color_.A() );
  this->material->setAmbient( color_.R(), color_.G(), color_.B() );

  if ( (this->colorP).A() < 0.9998 )
    this->material->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );
  else
    this->material->setSceneBlending( Ogre::SBT_REPLACE );

  this->material->setDepthWriteEnabled( false );
  this->material->setDepthCheckEnabled( false );

  this->Create();
}


////////////////////////////////////////////////////////////////////////////////
void Grid::SetHeight(uint32_t height_)
{
  this->height = height_;

  this->Create();
}

////////////////////////////////////////////////////////////////////////////////
// Init
void Grid::Init()
{
  this->manualObject = this->scene->GetManager()->createManualObject( this->name );
  this->manualObject->setVisibilityFlags( GZ_VISIBILITY_GUI );
  this->manualObject->setDynamic(true);
  this->manualObject->setRenderQueueGroup(Ogre::RENDER_QUEUE_WORLD_GEOMETRY_1 - 1);

  Ogre::SceneNode *parent_node = this->scene->GetManager()->getRootSceneNode();

  this->sceneNode = parent_node->createChildSceneNode(this->name);
  this->sceneNode->attachObject( this->manualObject );

  std::stringstream ss;
  ss << this->name << "Material";
  this->material = Ogre::MaterialManager::getSingleton().create( ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
  this->material->setReceiveShadows(false);
  this->material->getTechnique(0)->setLightingEnabled(false);

  this->SetColor(this->colorP);

  this->Create();
}

////////////////////////////////////////////////////////////////////////////////
void Grid::Create()
{
  this->manualObject->clear();

  float extent = (this->cellLengthP*((double)this->cellCountP))/2;

  this->manualObject->setCastShadows(false);
  this->manualObject->estimateVertexCount( this->cellCountP * 4 * this->height + ((this->cellCountP + 1) * (this->cellCountP + 1)));
  this->manualObject->begin( this->material->getName(), Ogre::RenderOperation::OT_LINE_LIST );

  for (uint32_t h = 0; h <= this->height; ++h)
  {
    float h_real = this->h_offsetP + (this->height / 2.0f - (float)h) * this->cellLengthP;
    for( uint32_t i = 0; i <= this->cellCountP; i++ )
    {
      float inc = extent - ( i * this->cellLengthP );

      Ogre::Vector3 p1(inc, -extent,h_real );
      Ogre::Vector3 p2(inc, extent ,h_real );
      Ogre::Vector3 p3(-extent,inc, h_real );
      Ogre::Vector3 p4(extent, inc, h_real );

      this->manualObject->position(p1);
      this->manualObject->colour( Conversions::Convert(this->colorP) );
      this->manualObject->position(p2);
      this->manualObject->colour( Conversions::Convert(this->colorP) );

      this->manualObject->position(p3);
      this->manualObject->colour( Conversions::Convert(this->colorP) );
      this->manualObject->position(p4);
      this->manualObject->colour( Conversions::Convert(this->colorP) );
    }
  }

  if (this->height > 0)
  {
    for (uint32_t x = 0; x <= this->cellCountP; ++x)
    {
      for (uint32_t y = 0; y <= this->cellCountP; ++y)
      {
        float x_real = extent - x * this->cellLengthP;
        float y_real = extent - y * this->cellLengthP;

        float z_top = (this->height / 2.0f) * this->cellLengthP;
        float z_bottom = -z_top;

        this->manualObject->position( x_real, y_real, z_bottom );
        this->manualObject->colour( Conversions::Convert(this->colorP) );
        this->manualObject->position(x_real, y_real, z_bottom);
        this->manualObject->colour( Conversions::Convert(this->colorP) );
      }
    }
  }

  this->manualObject->end();
}

////////////////////////////////////////////////////////////////////////////////
void Grid::SetUserData( const Ogre::Any& data_ )
{
  this->manualObject->setUserAny( data_ );
}
