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

#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Road2d.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
Road2d::Road2d()
{
  this->width = 2.0;
}

/////////////////////////////////////////////////
Road2d::~Road2d()
{
  this->parent.reset();
}

/////////////////////////////////////////////////
void Road2d::Load(VisualPtr _parent)
{
  this->parent = _parent;

  this->points.push_back(math::Vector3(0, 0, 0.01));
  this->points.push_back(math::Vector3(4, 4, 0.01));
  this->points.push_back(math::Vector3(4, 8, 0.01));
  this->points.push_back(math::Vector3(8, 8, 0.01));
  this->points.push_back(math::Vector3(20, 0, 0.01));
  this->points.push_back(math::Vector3(10, -20, 0.01));

  this->mRenderOp.vertexData = new Ogre::VertexData;
  this->mRenderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_STRIP;
  this->mRenderOp.indexData = 0;
  this->mRenderOp.useIndexes = false;

  Ogre::VertexDeclaration *vertexDecl =
    this->mRenderOp.vertexData->vertexDeclaration;

  size_t currOffset = 0;

  // Positions
  vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  // Normals
  vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
  currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  // two dimensional ->roadObjtexture coordinates
  vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT2,
                         Ogre::VES_TEXTURE_COORDINATES, 0);

  // allocate the vertex buffer
  this->mRenderOp.vertexData->vertexCount = this->points.size() * 2;
  Ogre::HardwareVertexBufferSharedPtr vBuf =
    Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
        vertexDecl->getVertexSize(0), this->mRenderOp.vertexData->vertexCount,
        Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);

  Ogre::VertexBufferBinding *binding =
    this->mRenderOp.vertexData->vertexBufferBinding;
  binding->setBinding(0, vBuf);
  float *vertices = static_cast<float*>(
      vBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  math::Vector3 pA, pB, tangent;
  double factor = 1.0;
  double theta = 0.0;

  math::Box bounds;
  bounds.min.Set(GZ_DBL_MAX, GZ_DBL_MAX, GZ_DBL_MAX);
  bounds.max.Set(GZ_DBL_MIN, GZ_DBL_MIN, GZ_DBL_MIN);

  // Generate the triangles for the road
  for (unsigned int i = 0; i < this->points.size(); ++i)
  {
    factor = 1.0;

    if (i == 0)
    {
      tangent = (this->points[i+1] - this->points[i]).Normalize();
    }
    else if (i == this->points.size() - 1)
    {
      tangent = (this->points[i] - this->points[i-1]).Normalize();
    }
    else
    {
      math::Vector3 v1 = (this->points[i+1] - this->points[i]).Normalize();
      math::Vector3 v0 = (this->points[i] - this->points[i-1]).Normalize();
      double dot = v0.Dot(v1 * -1);
      tangent = (this->points[i+1] - this->points[i-1]).Normalize();

      // Check to see if the points are not colinear
      if (dot > -.97 && dot < 0.97)
      {
        factor = 1.0 / sin(acos(dot) * 0.5);
      }
    }

    theta = atan2(tangent.x, -tangent.y);

    pA = pB = this->points[i];
    double w = (this->width * factor) * 0.5;

    pA.x += cos(theta) * w;
    pA.y += sin(theta) * w;

    pB.x -= cos(theta) * w;
    pB.y -= sin(theta) * w;

    bounds.min.SetToMin(pA);
    bounds.min.SetToMin(pB);

    bounds.max.SetToMax(pA);
    bounds.max.SetToMax(pB);

    // Position
    *vertices++ = pA.x;
    *vertices++ = pA.y;
    *vertices++ = pA.z;

    // Normal
    *vertices++ = 0;
    *vertices++ = 0;
    *vertices++ = 1;

    // UV Coord
    *vertices++ = 0;
    *vertices++ = 1;

    // Position
    *vertices++ = pB.x;
    *vertices++ = pB.y;
    *vertices++ = pB.z;

    // Normal
    *vertices++ = 0;
    *vertices++ = 0;
    *vertices++ = 1;

    // UV Coord
    *vertices++ = 1;
    *vertices++ = 1;
  }

  this->mBox.setExtents(Conversions::Convert(bounds.min),
                        Conversions::Convert(bounds.max));

  vBuf->unlock();
  this->parent->AttachObject(this);

  this->setMaterial("Gazebo/Road");
}

//////////////////////////////////////////////////
Ogre::Real Road2d::getBoundingRadius() const
{
  return Ogre::Math::Sqrt(std::max(this->mBox.getMaximum().squaredLength(),
                                   this->mBox.getMinimum().squaredLength()));
}

//////////////////////////////////////////////////
Ogre::Real Road2d::getSquaredViewDepth(
    const Ogre::Camera *_cam) const
{
  Ogre::Vector3 vMin, vMax, vMid, vDist;
  vMin = this->mBox.getMinimum();
  vMax = this->mBox.getMaximum();
  vMid = ((vMax - vMin) * 0.5) + vMin;
  vDist = _cam->getDerivedPosition() - vMid;
  return vDist.squaredLength();
}
