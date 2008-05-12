#include <OgreCamera.h>
#include <OgreHardwareBufferManager.h>

#include "OgreDynamicRenderable.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
OgreDynamicRenderable::OgreDynamicRenderable()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
OgreDynamicRenderable::~OgreDynamicRenderable()
{
  delete this->mRenderOp.vertexData;
  delete this->mRenderOp.indexData;
}

////////////////////////////////////////////////////////////////////////////////
/// Initializes the dynamic renderable.
void OgreDynamicRenderable::Init(OperationType operationType, bool useIndices)
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

////////////////////////////////////////////////////////////////////////////////
// Set the render operation type
void OgreDynamicRenderable::SetOperationType(OperationType opType)
{
  switch (opType)
  {
    case OT_POINT_LIST:
      this->mRenderOp.operationType = Ogre::RenderOperation::OT_POINT_LIST;
      break;

    case OT_LINE_LIST:
      this->mRenderOp.operationType = Ogre::RenderOperation::OT_LINE_LIST;
      break;

    case OT_LINE_STRIP:
      this->mRenderOp.operationType = Ogre::RenderOperation::OT_LINE_STRIP;
      break;

    case OT_TRIANGLE_LIST:
      this->mRenderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
      break;

    case OT_TRIANGLE_STRIP:
      this->mRenderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_STRIP;
      break;

    case OT_TRIANGLE_FAN:
      this->mRenderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_FAN;
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Get the render operation type
OgreDynamicRenderable::OperationType OgreDynamicRenderable::GetOperationType() const
{
  OperationType type = OT_POINT_LIST;
  switch (this->mRenderOp.operationType)
  {
    case Ogre::RenderOperation::OT_POINT_LIST:
      type = OT_POINT_LIST;

    case Ogre::RenderOperation::OT_LINE_LIST:
      type = OT_LINE_LIST;

    case Ogre::RenderOperation::OT_LINE_STRIP:
      type = OT_LINE_STRIP;

    case Ogre::RenderOperation::OT_TRIANGLE_LIST:
      type = OT_TRIANGLE_LIST;

    case Ogre::RenderOperation::OT_TRIANGLE_STRIP:
      type = OT_TRIANGLE_STRIP;

    case Ogre::RenderOperation::OT_TRIANGLE_FAN:
      type = OT_TRIANGLE_FAN;
  }
  return type;
}

////////////////////////////////////////////////////////////////////////////////
/// Prepares the hardware buffers for the requested vertex and index counts.
void OgreDynamicRenderable::PrepareHardwareBuffers(size_t vertexCount, size_t indexCount)
{
  // Prepare vertex buffer
  size_t newVertCapacity = this->vertexBufferCapacity;

  if ((vertexCount > this->vertexBufferCapacity) || (!this->vertexBufferCapacity))
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

    // TODO: Custom HBU_?

    // Bind buffer
    this->mRenderOp.vertexData->vertexBufferBinding->setBinding(0, vbuf);
  }

  // Update vertex count in the render operation
  this->mRenderOp.vertexData->vertexCount = vertexCount;

  if (this->mRenderOp.useIndexes)
  {
    OgreAssert(indexCount <= std::numeric_limits<unsigned short>::max(), "indexCount exceeds 16 bit");

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

////////////////////////////////////////////////////////////////////////////////
/// Implementation of Ogre::SimpleRenderable
Ogre::Real OgreDynamicRenderable::getBoundingRadius() const
{
  return Ogre::Math::Sqrt(std::max(mBox.getMaximum().squaredLength(),
                                   mBox.getMinimum().squaredLength()));
}

////////////////////////////////////////////////////////////////////////////////
/// Implementation of Ogre::SimpleRenderable
Ogre::Real OgreDynamicRenderable::getSquaredViewDepth(const Ogre::Camera* cam) const
{
  Ogre::Vector3 vMin, vMax, vMid, vDist;
  vMin = mBox.getMinimum();
  vMax = mBox.getMaximum();
  vMid = ((vMin - vMax) * 0.5) + vMin;
  vDist = cam->getDerivedPosition() - vMid;

  return vDist.squaredLength();
}
