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
/* Desc: Dynamic line generator
 * Author: Nate Koenig
 * Date: 28 June 2007
 * CVS: $Id$
 */

#include <Ogre.h>
#include <cassert>
#include <cmath>
#include <math.h>
#include <sstream>

#include "GazeboError.hh"
#include "OgreDynamicLines.hh"

using namespace gazebo;

enum { POSITION_BINDING, TEXCOORD_BINDING };

OgreDynamicLines::OgreDynamicLines(RenderOpType opType)
{
  this->Init(opType, false);
  this->setCastShadows(false);
  this->dirty = true;
}

OgreDynamicLines::~OgreDynamicLines()
{
}

/// Returns "gazebo::ogredynamicslines"
const Ogre::String &OgreDynamicLines::getMovableType() const
{
  static Ogre::String moveType = "gazebo::ogredynamiclines";
  return moveType;
}

void OgreDynamicLines::AddPoint(const Vector3 &pt)
{
  this->points.push_back(pt);
  this->dirty = true;
}

void OgreDynamicLines::SetPoint(unsigned int index, const Vector3 &value)
{
  if (index >= this->points.size())
  {
    std::ostringstream stream;
    stream << "Point index[" << index << "] is out of bounds[0-" << this->points.size()-1 << "]";
    gzthrow(stream.str());
  }

  this->points[index] = value;

  this->dirty = true;
}

const Vector3& OgreDynamicLines::GetPoint(unsigned int index) const
{
  if (index >= this->points.size())
  {
    gzthrow("Point index is out of bounds");
  }

  return this->points[index];
}

unsigned int OgreDynamicLines::GetNumPoints() const
{
  return this->points.size();
}

void OgreDynamicLines::Clear()
{
  this->points.clear();
  this->dirty = true;
}

void OgreDynamicLines::Update()
{
  if (this->dirty && this->points.size() > 1)
    this->FillHardwareBuffers();
}


void OgreDynamicLines::CreateVertexDeclaration()
{
  Ogre::VertexDeclaration *decl = this->mRenderOp.vertexData->vertexDeclaration;

  decl->addElement(POSITION_BINDING, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);

}

void OgreDynamicLines::FillHardwareBuffers()
{
  int size = this->points.size();
  this->PrepareHardwareBuffers(size,0);

  //std::cout << "debug size " << size << std::endl;

  if (!size)
  {
    this->mBox.setExtents(Ogre::Vector3::ZERO, Ogre::Vector3::ZERO);
    this->dirty=false;
  }

  Ogre::HardwareVertexBufferSharedPtr vbuf =
    this->mRenderOp.vertexData->vertexBufferBinding->getBuffer(0);

  //Ogre::Real *prPos = static_cast<Ogre::Real*>(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
  Ogre::Real *prPos = static_cast<Ogre::Real*>(vbuf->lock(Ogre::HardwareBuffer::HBL_NORMAL));
  {
    for (int i = 0; i < size; i++)
    {
      *prPos++ = this->points[i].x;
      *prPos++ = this->points[i].y;
      *prPos++ = this->points[i].z;

      this->mBox.merge(Ogre::Vector3(this->points[i].x,this->points[i].y,this->points[i].z));
    }
  }
  vbuf->unlock();

  // std::cout << "debug mBox merging "
  //    << this->getBoundingBox().getMinimum().x << " , "
  //    << this->getBoundingBox().getMinimum().y << " , "
  //    << this->getBoundingBox().getMinimum().z << " | "
  //    << this->getBoundingBox().getMaximum().x << " , "
  //    << this->getBoundingBox().getMaximum().y << " , "
  //    << this->getBoundingBox().getMaximum().z << std::endl;
  //this->mBox.setExtents(Ogre::Vector3(-1e9,-1e9,-1e9),Ogre::Vector3(1e9,1e9,1e9) );

  // need to update after mBox change, otherwise the lines goes in and out of scope based on old mBox
  this->getParentSceneNode()->needUpdate();

  //this->getParentSceneNode()->showBoundingBox(true); // debug

  this->dirty = false;
}

