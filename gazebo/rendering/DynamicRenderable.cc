/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/common/Console.hh"
#include "gazebo/rendering/DynamicRenderable.hh"

using namespace gazebo;
using namespace rendering;


//////////////////////////////////////////////////
DynamicRenderable::DynamicRenderable()
{
}

//////////////////////////////////////////////////
DynamicRenderable::~DynamicRenderable()
{
  delete this->mRenderOp.vertexData;
  delete this->mRenderOp.indexData;
}

//////////////////////////////////////////////////
void DynamicRenderable::Init(RenderOpType operationType, bool useIndices)
{
  this->SetOperationType(operationType);

  // Initialize render operation
  this->mRenderOp.useIndexes = useIndices;
  this->mRenderOp.vertexData = new Ogre::VertexData;

  if (this->mRenderOp.useIndexes)
    this->mRenderOp.indexData = new Ogre::IndexData;

  // Reset buffer capacities
  this->vertexBufferCapacity = 0;
  this->indexBufferCapacity = 0;

  // Create vertex declaration
  this->CreateVertexDeclaration();
}

//////////////////////////////////////////////////
void DynamicRenderable::SetOperationType(RenderOpType opType)
{
  switch (opType)
  {
    case RENDERING_POINT_LIST:
      this->mRenderOp.operationType = Ogre::RenderOperation::OT_POINT_LIST;
      break;

    case RENDERING_LINE_LIST:
      this->mRenderOp.operationType = Ogre::RenderOperation::OT_LINE_LIST;
      break;

    case RENDERING_LINE_STRIP:
      this->mRenderOp.operationType = Ogre::RenderOperation::OT_LINE_STRIP;
      break;

    case RENDERING_TRIANGLE_LIST:
      this->mRenderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
      break;

    case RENDERING_TRIANGLE_STRIP:
      this->mRenderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_STRIP;
      break;

    case RENDERING_TRIANGLE_FAN:
      this->mRenderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_FAN;
      break;
    case RENDERING_MESH_RESOURCE:
      gzwarn << "RENDERING_MESH_RESOURCE not handled\n";
      break;
    default:
      gzerr << "Unknown render operation type[" << opType << "]\n";
      break;
  }
}

//////////////////////////////////////////////////
RenderOpType DynamicRenderable::GetOperationType() const
{
  RenderOpType type;
  switch (this->mRenderOp.operationType)
  {
    case Ogre::RenderOperation::OT_LINE_LIST:
      type = RENDERING_LINE_LIST;
      break;

    case Ogre::RenderOperation::OT_LINE_STRIP:
      type = RENDERING_LINE_STRIP;
      break;

    case Ogre::RenderOperation::OT_TRIANGLE_LIST:
      type = RENDERING_TRIANGLE_LIST;
      break;

    case Ogre::RenderOperation::OT_TRIANGLE_STRIP:
      type = RENDERING_TRIANGLE_STRIP;
      break;

    case Ogre::RenderOperation::OT_TRIANGLE_FAN:
      type = RENDERING_TRIANGLE_FAN;
      break;

    default:
    case Ogre::RenderOperation::OT_POINT_LIST:
      type = RENDERING_POINT_LIST;
      break;
  }

  return type;
}

//////////////////////////////////////////////////
void DynamicRenderable::PrepareHardwareBuffers(size_t vertexCount,
                                               size_t indexCount)
{
  // Prepare vertex buffer
  size_t newVertCapacity = this->vertexBufferCapacity;

  if ((vertexCount > this->vertexBufferCapacity) ||
      (!this->vertexBufferCapacity))
  {
    // vertexCount exceeds current capacity!
    // It is necessary to reallocate the buffer.

    // Check if this is the first call
    if (!newVertCapacity)
      newVertCapacity = 1;

    // Make capacity the next power of two
    while (newVertCapacity < vertexCount)
      newVertCapacity <<= 1;
  }
  else if (vertexCount < this->vertexBufferCapacity>>1)
  {
    // Make capacity the previous power of two
    while (vertexCount < newVertCapacity>>1)
      newVertCapacity >>= 1;
  }

  if (newVertCapacity != this->vertexBufferCapacity)
  {
    this->vertexBufferCapacity = newVertCapacity;

    // Create new vertex buffer
    Ogre::HardwareVertexBufferSharedPtr vbuf =
      Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
        this->mRenderOp.vertexData->vertexDeclaration->getVertexSize(0),
        this->vertexBufferCapacity,
        Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);

    Ogre::HardwareVertexBufferSharedPtr cbuf =
      Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
        Ogre::VertexElement::getTypeSize(Ogre::VET_COLOUR),
        this->vertexBufferCapacity,
        Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);

    // TODO: Custom HBU_?

    // Bind buffer
    this->mRenderOp.vertexData->vertexBufferBinding->setBinding(0, vbuf);

    this->mRenderOp.vertexData->vertexBufferBinding->setBinding(1, cbuf);
  }

  // Update vertex count in the render operation
  this->mRenderOp.vertexData->vertexCount = vertexCount;

  if (this->mRenderOp.useIndexes)
  {
    OgreAssert(indexCount <= std::numeric_limits<uint16_t>::max(),
        "indexCount exceeds 16 bit");

    size_t newIndexCapacity = this->indexBufferCapacity;

    // Prepare index buffer
    if ((indexCount > newIndexCapacity) || (!newIndexCapacity))
    {
      // indexCount exceeds current capacity!
      // It is necessary to reallocate the buffer.

      // Check if this is the first call
      if (!newIndexCapacity)
        newIndexCapacity = 1;

      // Make capacity the next power of two
      while (newIndexCapacity < indexCount)
        newIndexCapacity <<= 1;
    }
    else if (indexCount < newIndexCapacity>>1)
    {
      // Make capacity the previous power of two
      while (indexCount < newIndexCapacity>>1)
        newIndexCapacity >>= 1;
    }

    if (newIndexCapacity != this->indexBufferCapacity)
    {
      this->indexBufferCapacity = newIndexCapacity;

      // Create new index buffer
      this->mRenderOp.indexData->indexBuffer =
        Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
          Ogre::HardwareIndexBuffer::IT_16BIT,
          this->indexBufferCapacity,
          Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);
      // TODO: Custom HBU_?
    }

    // Update index count in the render operation
    this->mRenderOp.indexData->indexCount = indexCount;
  }
}

//////////////////////////////////////////////////
Ogre::Real DynamicRenderable::getBoundingRadius() const
{
  return Ogre::Math::Sqrt(std::max(mBox.getMaximum().squaredLength(),
                                   mBox.getMinimum().squaredLength()));
}

//////////////////////////////////////////////////
Ogre::Real DynamicRenderable::getSquaredViewDepth(const Ogre::Camera* cam) const
{
  Ogre::Vector3 vMin, vMax, vMid, vDist;
  vMin = mBox.getMinimum();
  vMax = mBox.getMaximum();
  vMid = ((vMax - vMin) * 0.5) + vMin;
  vDist = cam->getDerivedPosition() - vMid;
  return vDist.squaredLength();
}

/////////////////////////////////////////////////
std::string DynamicRenderable::GetMovableType() const
{
  return "gazebo::DynamicRenderable";
}
