/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <math.h>

#include <cmath>
#include <sstream>
#include <ignition/common/Profiler.hh>
#include <ignition/math/Color.hh>
#include "gazebo/rendering/ogre_gazebo.h"

#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/rendering/DynamicLines.hh"

using namespace gazebo;
using namespace rendering;

enum {POSITION_BINDING, TEXCOORD_BINDING};


/// \brief Private implementation
class gazebo::rendering::DynamicLinesPrivate
{
  /// \brief list of colors at each point
  public: std::vector<ignition::math::Color> colors;
};

/////////////////////////////////////////////////
DynamicLines::DynamicLines(RenderOpType opType)
  : dataPtr(new DynamicLinesPrivate)
{
  this->Init(opType, false);
  this->setCastShadows(false);
  this->dirty = true;
}

/////////////////////////////////////////////////
DynamicLines::~DynamicLines()
{
}

/////////////////////////////////////////////////
std::string DynamicLines::GetMovableType()
{
  return "gazebo::dynamiclines";
}

/////////////////////////////////////////////////
const Ogre::String &DynamicLines::getMovableType() const
{
  static Ogre::String moveType = DynamicLines::GetMovableType();
  return moveType;
}

/////////////////////////////////////////////////
void DynamicLines::AddPoint(const ignition::math::Vector3d &_pt,
                            const ignition::math::Color &_color)
{
  this->points.push_back(_pt);
  this->dataPtr->colors.push_back(_color);
  this->dirty = true;
}

/////////////////////////////////////////////////
void DynamicLines::AddPoint(const double _x, const double _y, const double _z,
                            const ignition::math::Color &_color)
{
  this->AddPoint(ignition::math::Vector3d(_x, _y, _z), _color);
}

/////////////////////////////////////////////////
void DynamicLines::SetPoint(const unsigned int _index,
                            const ignition::math::Vector3d &_value)
{
  if (_index >= this->points.size())
  {
    gzerr << "Point index[" << _index << "] is out of bounds[0-"
           << this->points.size()-1 << "]\n";
    return;
  }

  this->points[_index] = _value;

  this->dirty = true;
}

/////////////////////////////////////////////////
void DynamicLines::SetColor(const unsigned int _index,
                            const ignition::math::Color &_color)
{
  this->dataPtr->colors[_index] = _color;
  this->dirty = true;
}

/////////////////////////////////////////////////
ignition::math::Vector3d DynamicLines::Point(
    const unsigned int _index) const
{
  if (_index >= this->points.size())
  {
    gzerr << "Point index[" << _index << "] is out of bounds[0-"
           << this->points.size()-1 << "]\n";

    return ignition::math::Vector3d(ignition::math::INF_D,
                                    ignition::math::INF_D,
                                    ignition::math::INF_D);
  }

  return this->points[_index];
}

/////////////////////////////////////////////////
unsigned int DynamicLines::GetPointCount() const
{
  return this->points.size();
}

/////////////////////////////////////////////////
void DynamicLines::Clear()
{
  this->points.clear();
  this->dirty = true;
}

/////////////////////////////////////////////////
void DynamicLines::Update()
{
  IGN_PROFILE("rendering::DynamicLines::Update");
  if (this->dirty && this->points.size() > 1)
    this->FillHardwareBuffers();
}

/////////////////////////////////////////////////
void DynamicLines::CreateVertexDeclaration()
{
  Ogre::VertexDeclaration *decl =
    this->mRenderOp.vertexData->vertexDeclaration;

  decl->addElement(POSITION_BINDING, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  decl->addElement(1, 0, Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);
}

/////////////////////////////////////////////////
void DynamicLines::FillHardwareBuffers()
{
  int size = this->points.size();
  this->PrepareHardwareBuffers(size, 0);

  if (!size)
  {
    this->mBox.setExtents(Ogre::Vector3::ZERO, Ogre::Vector3::ZERO);
    this->dirty = false;
  }

  Ogre::HardwareVertexBufferSharedPtr vbuf =
    this->mRenderOp.vertexData->vertexBufferBinding->getBuffer(0);

  Ogre::Real *prPos =
    static_cast<Ogre::Real*>(vbuf->lock(Ogre::HardwareBuffer::HBL_NORMAL));
  {
    for (int i = 0; i < size; i++)
    {
      *prPos++ = this->points[i].X();
      *prPos++ = this->points[i].Y();
      *prPos++ = this->points[i].Z();

      this->mBox.merge(Conversions::Convert(this->points[i]));
    }
  }
  vbuf->unlock();

  // Update the colors
  Ogre::HardwareVertexBufferSharedPtr cbuf =
    this->mRenderOp.vertexData->vertexBufferBinding->getBuffer(1);

  Ogre::RGBA *colorArrayBuffer =
        static_cast<Ogre::RGBA*>(cbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
  Ogre::RenderSystem *renderSystemForVertex =
        Ogre::Root::getSingleton().getRenderSystem();
  for (int i = 0; i < size; ++i)
  {
    Ogre::ColourValue color = Conversions::Convert(this->dataPtr->colors[i]);
    renderSystemForVertex->convertColourValue(color, &colorArrayBuffer[i]);
  }
  cbuf->unlock();

  // need to update after mBox change, otherwise the lines goes in and out
  // of scope based on old mBox
  this->getParentSceneNode()->needUpdate();

  this->dirty = false;
}
