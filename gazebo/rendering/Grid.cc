/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Grid.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \brief Private data for the Grid class.
    class GridPrivate
    {
      /// \brief Pointer to the manual object.
      public: Ogre::ManualObject *manualObject = nullptr;

      /// \brief Pointer to the material.
      public: Ogre::MaterialPtr material;

      /// \brief Number of cells in each direction.
      public: uint32_t cellCount;

      /// \brief Length of each cell in each direction.
      public: float cellLength;

      /// \brief Width of the lines.
      public: float lineWidth;

      /// \brief Line color.
      public: common::Color color;

      /// \brief Height offset.
      public: float heightOffset;

      /// \brief Grid name.
      public: std::string name;

      /// \brief Number of cells in the normal direction.
      public: uint32_t height;

      /// \brief Pointer to the scene.
      public: Scene *scene = nullptr;

      /// \brief Grid visual that contains the grid lines
      public: VisualPtr gridVis;
    };
  }
}

using namespace gazebo;
using namespace rendering;

//////////////////////////////////////////////////
Grid::Grid(Scene *_scene, const unsigned int _cellCount,
    const float _cellLength, const float _lineWidth,
    const common::Color &_color)
: dataPtr(new GridPrivate)
{
  this->dataPtr->scene = _scene;
  this->dataPtr->height = 0;

  this->dataPtr->cellCount = _cellCount;
  this->dataPtr->cellLength = _cellLength;
  this->dataPtr->lineWidth = _lineWidth;
  this->dataPtr->color = _color;
  this->dataPtr->heightOffset = 0.015;

  static uint32_t gridCount = 0;
  std::stringstream ss;
  ss << "Grid" << gridCount++;

  this->dataPtr->name = ss.str();
}

//////////////////////////////////////////////////
Grid::~Grid()
{
  this->dataPtr->gridVis->Fini();
  this->dataPtr->material->unload();
}

//////////////////////////////////////////////////
void Grid::SetCellCount(const uint32_t _count)
{
  this->dataPtr->cellCount = _count;

  this->Create();
}

//////////////////////////////////////////////////
void Grid::SetCellLength(const float _len)
{
  this->dataPtr->cellLength = _len;

  this->Create();
}

//////////////////////////////////////////////////
void Grid::SetLineWidth(const float _width)
{
  this->dataPtr->lineWidth = _width;

  gzwarn << "Line width is currently not supported. Issue #1978" << std::endl;
  // Uncomment once line width is implemented
  // this->Create();
}

//////////////////////////////////////////////////
void Grid::SetColor(const common::Color &_color)
{
  this->dataPtr->color = _color;

  this->dataPtr->material->setDiffuse(_color.r, _color.g, _color.b, _color.a);
  this->dataPtr->material->setAmbient(_color.r, _color.g, _color.b);

  this->dataPtr->material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);

  this->dataPtr->material->setDepthWriteEnabled(false);
  this->dataPtr->material->setDepthCheckEnabled(true);

  this->Create();
}

//////////////////////////////////////////////////
void Grid::SetHeight(const uint32_t _height)
{
  this->dataPtr->height = _height;

  this->Create();
}

//////////////////////////////////////////////////
void Grid::SetHeightOffset(const double _offset)
{
  this->dataPtr->heightOffset = _offset;

  this->Create();
}

//////////////////////////////////////////////////
void Grid::Init()
{
  this->dataPtr->manualObject =
      this->dataPtr->scene->OgreSceneManager()->createManualObject(
      this->dataPtr->name);
  this->dataPtr->manualObject->setVisibilityFlags(GZ_VISIBILITY_GUI);

  this->dataPtr->manualObject->setDynamic(false);
  // this->dataPtr->manualObject->setRenderQueueGroup(
  //    Ogre::RENDER_QUEUE_SKIES_EARLY+3);
  //    Ogre::RENDER_QUEUE_WORLD_GEOMETRY_1 - 1);

  this->dataPtr->gridVis.reset(
      new Visual(this->dataPtr->name, this->dataPtr->scene->WorldVisual(),
      false));
  this->dataPtr->gridVis->Load();
  this->dataPtr->gridVis->GetSceneNode()->attachObject(
      this->dataPtr->manualObject);

  std::stringstream ss;
  ss << this->dataPtr->name << "Material";
  this->dataPtr->material =
    Ogre::MaterialManager::getSingleton().create(ss.str(),
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  this->dataPtr->material->setReceiveShadows(false);
  this->dataPtr->material->getTechnique(0)->setLightingEnabled(false);

  this->SetColor(this->dataPtr->color);
}

//////////////////////////////////////////////////
void Grid::Create()
{
  this->dataPtr->manualObject->clear();

  float extent = (this->dataPtr->cellLength *
      static_cast<double>(this->dataPtr->cellCount))/2;

  this->dataPtr->manualObject->setCastShadows(false);
  this->dataPtr->manualObject->estimateVertexCount(
      this->dataPtr->cellCount * 4 * this->dataPtr->height +
      ((this->dataPtr->cellCount + 1) * (this->dataPtr->cellCount + 1)));

  this->dataPtr->manualObject->begin(this->dataPtr->material->getName(),
      Ogre::RenderOperation::OT_LINE_LIST);

  for (uint32_t h = 0; h <= this->dataPtr->height; ++h)
  {
    float h_real = this->dataPtr->heightOffset +
      (this->dataPtr->height / 2.0f - static_cast<float>(h)) *
      this->dataPtr->cellLength;

    for (uint32_t i = 0; i <= this->dataPtr->cellCount; i++)
    {
      float inc = extent - (i * this->dataPtr->cellLength);

      Ogre::Vector3 p1(inc, -extent, h_real);
      Ogre::Vector3 p2(inc, extent , h_real);
      Ogre::Vector3 p3(-extent, inc, h_real);
      Ogre::Vector3 p4(extent, inc, h_real);

      this->dataPtr->manualObject->position(p1);
      this->dataPtr->manualObject->colour(Conversions::Convert(
          this->dataPtr->color));
      this->dataPtr->manualObject->position(p2);
      this->dataPtr->manualObject->colour(Conversions::Convert(
          this->dataPtr->color));

      this->dataPtr->manualObject->position(p3);
      this->dataPtr->manualObject->colour(Conversions::Convert(
          this->dataPtr->color));
      this->dataPtr->manualObject->position(p4);
      this->dataPtr->manualObject->colour(Conversions::Convert(
          this->dataPtr->color));
    }
  }

  if (this->dataPtr->height > 0)
  {
    for (uint32_t x = 0; x <= this->dataPtr->cellCount; ++x)
    {
      for (uint32_t y = 0; y <= this->dataPtr->cellCount; ++y)
      {
        float x_real = extent - x * this->dataPtr->cellLength;
        float y_real = extent - y * this->dataPtr->cellLength;

        float z_top =
            (this->dataPtr->height / 2.0f) * this->dataPtr->cellLength;
        float z_bottom = -z_top;

        this->dataPtr->manualObject->position(x_real, y_real, z_bottom);
        this->dataPtr->manualObject->colour(Conversions::Convert(
            this->dataPtr->color));
        this->dataPtr->manualObject->position(x_real, y_real, z_bottom);
        this->dataPtr->manualObject->colour(Conversions::Convert(
            this->dataPtr->color));
      }
    }
  }

  this->dataPtr->manualObject->end();
}

//////////////////////////////////////////////////
void Grid::SetUserData(const Ogre::Any &_data)
{
  this->dataPtr->manualObject->getUserObjectBindings().setUserAny(_data);
}

//////////////////////////////////////////////////
void Grid::Enable(const bool _enable)
{
  this->dataPtr->gridVis->SetVisible(_enable);
}

//////////////////////////////////////////////////
Ogre::SceneNode *Grid::GetSceneNode()
{
  return this->dataPtr->gridVis->GetSceneNode();
}

//////////////////////////////////////////////////
VisualPtr Grid::GridVisual() const
{
  return this->dataPtr->gridVis;
}

//////////////////////////////////////////////////
common::Color Grid::GetColor() const
{
  return this->Color();
}

//////////////////////////////////////////////////
common::Color Grid::Color() const
{
  return this->dataPtr->color;
}

//////////////////////////////////////////////////
uint32_t Grid::GetCellCount() const
{
  return this->CellCount();
}

//////////////////////////////////////////////////
uint32_t Grid::CellCount() const
{
  return this->dataPtr->cellCount;
}

//////////////////////////////////////////////////
float Grid::GetCellLength() const
{
  return this->CellLength();
}

//////////////////////////////////////////////////
float Grid::CellLength() const
{
  return this->dataPtr->cellLength;
}

//////////////////////////////////////////////////
float Grid::GetLineWidth() const
{
  return this->LineWidth();
}

//////////////////////////////////////////////////
float Grid::LineWidth() const
{
  gzwarn << "Line width is currently not supported. Issue #1978" << std::endl;
  return this->dataPtr->lineWidth;
}

//////////////////////////////////////////////////
uint32_t Grid::GetHeight() const
{
  return this->Height();
}

//////////////////////////////////////////////////
uint32_t Grid::Height() const
{
  return this->dataPtr->height;
}

//////////////////////////////////////////////////
double Grid::HeightOffset() const
{
  return this->dataPtr->heightOffset;
}
