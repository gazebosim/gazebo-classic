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
/* Desc: Dynamic line generator
 * Author: Nate Koenig
 * Date: 28 June 2007
 * CVS: $Id$
 */

#include <Ogre.h>
#include <cassert>
#include <cmath>
#include <sstream>

#include "GazeboError.hh"
#include "OgreDynamicLines.hh"

using namespace gazebo;

enum { POSITION_BINDING, TEXCOORD_BINDING };

OgreDynamicLines::OgreDynamicLines(OperationType opType)
{
  this->Init(opType, false);
  this->setCastShadows(false);
  this->dirty = true;
}

OgreDynamicLines::~OgreDynamicLines()
{
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
  if (this->dirty) 
    this->FillHardwareBuffers();
}


void OgreDynamicLines::CreateVertexDeclaration()
{
  Ogre::VertexDeclaration *decl = this->mRenderOp.vertexData->vertexDeclaration;

  decl->addElement(POSITION_BINDING, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);

}

void OgreDynamicLines::FillHardwareBuffers()
{
  Vector3 vaabMin, vaabMax;
  int size = this->points.size();
  this->PrepareHardwareBuffers(size,0);

  if (!size) 
  { 
    this->mBox.setExtents(Ogre::Vector3::ZERO, Ogre::Vector3::ZERO);
    this->dirty=false;
  }

  vaabMin = this->points[0];
  vaabMax = this->points[0];

  Ogre::HardwareVertexBufferSharedPtr vbuf =
    this->mRenderOp.vertexData->vertexBufferBinding->getBuffer(0);

  Ogre::Real *prPos = static_cast<Ogre::Real*>(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));
  {
    for(int i = 0; i < size; i++)
    {
      *prPos++ = this->points[i].x;
      *prPos++ = this->points[i].y;
      *prPos++ = this->points[i].z;

      if(this->points[i].x < vaabMin.x)
        vaabMin.x = this->points[i].x;
      if(this->points[i].y < vaabMin.y)
        vaabMin.y = this->points[i].y;
      if(this->points[i].z < vaabMin.z)
        vaabMin.z = this->points[i].z;

      if(this->points[i].x > vaabMax.x)
        vaabMax.x = this->points[i].x;
      if(this->points[i].y > vaabMax.y)
        vaabMax.y = this->points[i].y;
      if(this->points[i].z > vaabMax.z)
        vaabMax.z = this->points[i].z;
    }
  }
  vbuf->unlock();

  if ((float)vaabMin.x >= (float)vaabMax.x)
    vaabMin.x = vaabMax.x - 10;

  if ((float)vaabMin.y >= (float)vaabMax.y)
    vaabMin.y = vaabMax.y - 10;

  if ((float)vaabMin.z >= (float)vaabMax.z)
    vaabMin.z = vaabMax.z - 10;

  this->mBox.setExtents(Ogre::Vector3(vaabMin.x, vaabMin.y, vaabMin.z), 
                        Ogre::Vector3(vaabMax.x, vaabMax.y, vaabMax.z) );
 
  this->dirty = false;
}

