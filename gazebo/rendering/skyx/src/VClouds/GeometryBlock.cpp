/*
--------------------------------------------------------------------------------
This source file is part of SkyX.
Visit http://www.paradise-studios.net/products/skyx/

Copyright (C) 2009-2012 Xavier Verguín González <xavyiy@gmail.com>

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU Lesser General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place - Suite 330, Boston, MA 02111-1307, USA, or go to
http://www.gnu.org/copyleft/lesser.txt.
--------------------------------------------------------------------------------
*/

#include <algorithm>
#include "VClouds/GeometryBlock.h"

#include "VClouds/VClouds.h"

namespace SkyX { namespace VClouds
{
  GeometryBlock::GeometryBlock(VClouds* vc,
      const float& Height, const Ogre::Radian& Alpha, const Ogre::Radian& Beta,
      const float& Radius, const Ogre::Radian& Phi, const int& Na,
      const int& Nb, const int& Nc, const int& A,
      const int& B, const int& C, const int& Position)
    : mVClouds(vc)
      , mCreated(false)
      , mSubMesh(0)
      , mEntity(0)
      , mVertices(0)
      , mNumberOfTriangles(0)
      , mVertexCount(0)
      , mHeight(Height)
      , mAlpha(Alpha)
      , mBeta(Beta)
      , mRadius(Radius)
      , mPhi(Phi)
      , mNa(Na), mNb(Nb), mNc(Nc)
      , mA(A), mB(B), mC(C)
      , mPosition(Position)
      , mDisplacement(Ogre::Vector3(0, 0, 0))
      , mWorldOffset(Ogre::Vector2(0, 0))
      , mCamera(0)
      , mLastFallingDistance(0)
  {
    _calculateDataSize();
  }

  GeometryBlock::~GeometryBlock()
  {
    remove();
  }

  void GeometryBlock::create()
  {
    remove();

    // Create mesh and submesh
    mMesh = Ogre::MeshManager::getSingleton().createManual(
        "_SkyX_VClouds_Block" + Ogre::StringConverter::toString(mPosition),
        SKYX_RESOURCE_GROUP);
    mSubMesh = mMesh->createSubMesh();
    mSubMesh->useSharedVertices = false;

    // Create mesh geometry
    _createGeometry();

    // Build edge list
    mMesh->buildEdgeList();

    // End mesh creation
    mMesh->load();
    mMesh->touch();

    // Create entity
    std::string entityName =
        "_SkyX_VClouds_BlockEnt" + Ogre::StringConverter::toString(mPosition);
    while (mVClouds->getSceneManager()->hasEntity(entityName))
      entityName += "_";
    mEntity = mVClouds->getSceneManager()->createEntity(entityName,
        "_SkyX_VClouds_Block" + Ogre::StringConverter::toString(mPosition));
    mEntity->setMaterialName("SkyX_VolClouds");
    mEntity->setCastShadows(false);
    mEntity->setRenderQueueGroup(mVClouds->getRenderQueueGroups().vclouds);

    // Set bounds
    mMesh->_setBounds(_buildAABox(mLastFallingDistance));

    mCreated = true;
  }

  void GeometryBlock::remove()
  {
    if (!mCreated)
    {
      return;
    }

    Ogre::MeshManager::getSingleton().remove(mMesh->getName());
    mVClouds->getSceneManager()->destroyEntity(mEntity);

    mMesh.setNull();
    mSubMesh = 0;
    mEntity = 0;
    mVertexBuffer.setNull();
    mIndexBuffer.setNull();

    delete [] mVertices;

    mCreated = false;
  }

  const Ogre::AxisAlignedBox GeometryBlock::_buildAABox(const float& fd) const
  {
    Ogre::Vector2 Center = Ogre::Vector2(0, 0);
    Ogre::Vector2 V1     = mRadius *
      Ogre::Vector2(Ogre::Math::Cos(mPhi*mPosition),
          Ogre::Math::Sin(mPhi*mPosition));
    Ogre::Vector2 V2     = mRadius *
      Ogre::Vector2(Ogre::Math::Cos(mPhi*(mPosition+1)),
          Ogre::Math::Sin(mPhi*(mPosition+1)));

    Ogre::Vector2 Max    = Ogre::Vector2(std::max<float>(
          std::max<float>(V1.x, V2.x), Center.x),
        std::max<float>(std::max<float>(V1.y, V2.y), Center.y) );
    Ogre::Vector2 Min    = Ogre::Vector2(std::min<float>(
          std::min<float>(V1.x, V2.x), Center.x),
        std::min<float>(std::min<float>(V1.y, V2.y), Center.y) );

    return Ogre::AxisAlignedBox(
        // Min x,y,z
        Min.x, Min.y, -std::max<float>(fd, 0),
        // Max x,y,z
        Max.x, Max.y, mHeight - std::min<float>(fd, 0));
  }

  void GeometryBlock::_calculateDataSize()
  {
    mVertexCount = 7*mNa + 6*mNb + 4*mNc;
    mNumberOfTriangles = 5*mNa + 4*mNb + 2*mNc;

    mV2Cos = Ogre::Vector2(Ogre::Math::Cos(mPosition*mPhi),
        Ogre::Math::Cos((mPosition+1)*mPhi));
    mV2Sin = Ogre::Vector2(Ogre::Math::Sin(mPosition*mPhi),
        Ogre::Math::Sin((mPosition+1)*mPhi));

    mBetaSin  = Ogre::Math::Sin(Ogre::Math::PI-mBeta.valueRadians());
    mAlphaSin = Ogre::Math::Sin(Ogre::Math::PI-mAlpha.valueRadians());
  }

  void GeometryBlock::_createGeometry()
  {
    // Vertex buffers
    mSubMesh->vertexData = new Ogre::VertexData();
    mSubMesh->vertexData->vertexStart = 0;
    mSubMesh->vertexData->vertexCount = mVertexCount;

    Ogre::VertexDeclaration* vdecl = mSubMesh->vertexData->vertexDeclaration;
    Ogre::VertexBufferBinding* vbind =
      mSubMesh->vertexData->vertexBufferBinding;

    size_t offset = 0;
    // Position
    vdecl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    // 3D coords
    vdecl->addElement(0, offset, Ogre::VET_FLOAT3,
        Ogre::VES_TEXTURE_COORDINATES, 0);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    // Noise coords
    vdecl->addElement(0, offset, Ogre::VET_FLOAT2,
        Ogre::VES_TEXTURE_COORDINATES, 1);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);
    // Opacity
    vdecl->addElement(0, offset, Ogre::VET_FLOAT1,
        Ogre::VES_TEXTURE_COORDINATES, 2);

    mVertexBuffer = Ogre::HardwareBufferManager::getSingleton().
      createVertexBuffer(sizeof(VERTEX),
          mVertexCount,
          Ogre::HardwareBuffer::HBU_DYNAMIC_WRITE_ONLY);

    vbind->setBinding(0, mVertexBuffer);

    uint16_t *indexbuffer = new uint16_t[mNumberOfTriangles*3];

    int IndexOffset = 0;
    int VertexOffset = 0;

    // C
    for (int k = 0; k < mNc; k++)
    {
      // First triangle
      indexbuffer[IndexOffset]   = VertexOffset;
      indexbuffer[IndexOffset+1] = VertexOffset+1;
      indexbuffer[IndexOffset+2] = VertexOffset+3;

      // Second triangle
      indexbuffer[IndexOffset+3] = VertexOffset;
      indexbuffer[IndexOffset+4] = VertexOffset+3;
      indexbuffer[IndexOffset+5] = VertexOffset+2;

      IndexOffset  += 6;
      VertexOffset += 4;
    }

    // B
    for (int k = 0; k < mNb; k++)
    {
      // First triangle
      indexbuffer[IndexOffset]   = VertexOffset;
      indexbuffer[IndexOffset+1] = VertexOffset+1;
      indexbuffer[IndexOffset+2] = VertexOffset+3;

      // Second triangle
      indexbuffer[IndexOffset+3] = VertexOffset;
      indexbuffer[IndexOffset+4] = VertexOffset+3;
      indexbuffer[IndexOffset+5] = VertexOffset+2;

      // Third triangle
      indexbuffer[IndexOffset+6] = VertexOffset+2;
      indexbuffer[IndexOffset+7] = VertexOffset+3;
      indexbuffer[IndexOffset+8] = VertexOffset+5;

      // Fourth triangle
      indexbuffer[IndexOffset+9] = VertexOffset+2;
      indexbuffer[IndexOffset+10] = VertexOffset+5;
      indexbuffer[IndexOffset+11] = VertexOffset+4;

      IndexOffset  += 12;
      VertexOffset += 6;
    }

    // A
    for (int k = 0; k < mNa; k++)
    {
      // First triangle
      indexbuffer[IndexOffset]   = VertexOffset;
      indexbuffer[IndexOffset+1] = VertexOffset+1;
      indexbuffer[IndexOffset+2] = VertexOffset+3;

      // Second triangle
      indexbuffer[IndexOffset+3] = VertexOffset;
      indexbuffer[IndexOffset+4] = VertexOffset+3;
      indexbuffer[IndexOffset+5] = VertexOffset+2;

      // Third triangle
      indexbuffer[IndexOffset+6]   = VertexOffset+2;
      indexbuffer[IndexOffset+7] = VertexOffset+3;
      indexbuffer[IndexOffset+8] = VertexOffset+5;

      // Fourth triangle
      indexbuffer[IndexOffset+9] = VertexOffset+2;
      indexbuffer[IndexOffset+10] = VertexOffset+5;
      indexbuffer[IndexOffset+11] = VertexOffset+4;

      // Fifth triangle
      indexbuffer[IndexOffset+12] = VertexOffset+4;
      indexbuffer[IndexOffset+13] = VertexOffset+5;
      indexbuffer[IndexOffset+14] = VertexOffset+6;

      IndexOffset  += 15;
      VertexOffset += 7;
    }

    // Prepare buffer for indices
    mIndexBuffer =
      Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
          Ogre::HardwareIndexBuffer::IT_16BIT,
          mNumberOfTriangles*3,
          Ogre::HardwareBuffer::HBU_STATIC, true);

    mIndexBuffer->
      writeData(0,
          mIndexBuffer->getSizeInBytes(),
          indexbuffer,
          true);

    delete []indexbuffer;

    // Set index buffer for this submesh
    mSubMesh->indexData->indexBuffer = mIndexBuffer;
    mSubMesh->indexData->indexStart = 0;
    mSubMesh->indexData->indexCount = mNumberOfTriangles*3;

    // Create our internal buffer for manipulations
    mVertices = new VERTEX[mVertexCount];
  }

  void GeometryBlock::updateGeometry(Ogre::Camera* c,
                                     const Ogre::Vector3& displacement)
  {
    if (!mCreated)
    {
      return;
    }

    mDisplacement = displacement;

    float fallingDistance = mVClouds->getDistanceFallingParams().x*
      (mEntity->getParentSceneNode()->_getDerivedPosition().z-
       c->getDerivedPosition().z);

    if (mVClouds->getDistanceFallingParams().y > 0)  // -1 means no max falling
    {
      if (fallingDistance > 0)
      {
        if (fallingDistance > mVClouds->getDistanceFallingParams().y)
        {
          fallingDistance = mVClouds->getDistanceFallingParams().y;
        }
      }
      else
      {
        if (-fallingDistance > mVClouds->getDistanceFallingParams().y)
        {
          fallingDistance = -mVClouds->getDistanceFallingParams().y;
        }
      }
    }

    if (!equal(fallingDistance, mLastFallingDistance))
    {
      mLastFallingDistance = fallingDistance;
      mMesh->_setBounds(_buildAABox(mLastFallingDistance));
    }

    if (isInFrustum(c))
    {
      mCamera = c;
      _updateGeometry();
    }
  }

  void GeometryBlock::_updateGeometry()
  {
    // Update zone C
    for (int k = 0; k < mNc; k++)
    {
      _updateZoneCSlice(k);
    }

    // Update zone B
    for (int k = 0; k < mNb; k++)
    {
      _updateZoneBSlice(k);
    }

    // Update zone A
    for (int k = 0; k < mNa; k++)
    {
      _updateZoneASlice(k);
    }

    // Upload changes
    mVertexBuffer->
      writeData(0,
          mVertexBuffer->getSizeInBytes(),
          mVertices,
          true);
  }

  void GeometryBlock::_updateZoneCSlice(const int& n)
  {
    int VertexOffset = n*4;

    // TODO calculate constants by zone, not by slice
    float Radius = mB+((mC-mB)/mNc)*(mNc-n);

    Radius += mDisplacement.z;

    float opacity = 1;

    if (n == 0)
    {
      opacity = 1 - mDisplacement.z/((mC-mB)/mNc);
    }
    else if (n == mNc-1)
    {
      opacity = mDisplacement.z/((mC-mB)/mNc);
    }

    Ogre::Vector2 x1 = Radius*mV2Cos,
      x2 = Radius*mBetaSin*mV2Cos,
      z1 = Radius*mV2Sin,
      z2 = Radius*mBetaSin*mV2Sin;

    Ogre::Vector3 or0 = Ogre::Vector3(x1.x, 0, z1.x),
      or1 = Ogre::Vector3(x1.y, 0, z1.y);

    float y0 = Radius*Ogre::Math::Sin(mAlpha),
          d = Ogre::Vector2(x1.x - x2.x, z1.x - z2.x).length(),
          ang = Ogre::Math::ATan(y0/d).valueRadians(),
          hip = mHeight / Ogre::Math::Sin(ang);

    // Vertex 0
    _setVertexData(VertexOffset, or0, opacity);
    // Vertex 1
    _setVertexData(VertexOffset+1, or1, opacity);
    // Vertex 2
    _setVertexData(VertexOffset+2, or0+
        (Ogre::Vector3(x2.x, y0, z2.x)-or0).normalisedCopy()*hip, opacity);
    // Vertex 3
    _setVertexData(VertexOffset+3,
        or1+(Ogre::Vector3(x2.y, y0, z2.y)-or1).normalisedCopy()*hip, opacity);
  }

  void GeometryBlock::_updateZoneBSlice(const int& n)
  {
    int VertexOffset = mNc*4 + n*6;

    // TODO
    float Radius = mA+((mB-mA)/mNb)*(mNb-n);

    Radius += mDisplacement.y;

    float opacity = 1;

    if (n == 0)
    {
      opacity = 1-mDisplacement.y/((mB-mA)/mNb);
    }
    else if (n == mNb-1)
    {
      opacity = mDisplacement.y/((mB-mA)/mNb);
    }

    Ogre::Vector2 x1 = Radius*mV2Cos,
      x2 = Radius*mBetaSin*mV2Cos,
      z1 = Radius*mV2Sin,
      z2 = Radius*mBetaSin*mV2Sin;

    float y0 = Radius*Ogre::Math::Sin(mAlpha);

    // Vertex 0
    _setVertexData(VertexOffset, Ogre::Vector3(x1.x, 0, z1.x), opacity);
    // Vertex 1
    _setVertexData(VertexOffset+1, Ogre::Vector3(x1.y, 0, z1.y), opacity);
    // Vertex 2
    _setVertexData(VertexOffset+2, Ogre::Vector3(x2.x, y0, z2.x), opacity);
    // Vertex 3
    _setVertexData(VertexOffset+3, Ogre::Vector3(x2.y, y0, z2.y), opacity);

    Ogre::Vector2 x3 = Radius*mAlphaSin*mV2Cos,
      z3 = Radius*mAlphaSin*mV2Sin;

    Ogre::Vector3 or0 = Ogre::Vector3(x2.x, y0, z2.x),
      or1 = Ogre::Vector3(x2.y, y0, z2.y);

    float y1 = Radius*Ogre::Math::Sin(mBeta),
          y3 = y1-y0,
          d = Ogre::Vector2(x3.x - x2.x, z3.x - z2.x).length(),
          ang = Ogre::Math::ATan(y3/d).valueRadians(),
          hip = (mHeight-y0) / Ogre::Math::Sin(ang);

    // Vertex 4
    _setVertexData(VertexOffset+4, or0 +
        (Ogre::Vector3(x3.x, y1, z3.x)-or0).normalisedCopy()*hip, opacity);
    // Vertex 5
    _setVertexData(VertexOffset+5, or1 +
        (Ogre::Vector3(x3.y, y1, z3.y)-or1).normalisedCopy()*hip, opacity);
  }

  void GeometryBlock::_updateZoneASlice(const int& n)
  {
    int VertexOffset = mNc*4 + mNb*6 +n*7;

    // TODO
    float Radius = (mA/mNa)*(mNa-n);

    Radius += mDisplacement.x;

    float opacity = (n == 0) ? (1-mDisplacement.x/(mA/mNa)) : 1.0f;

    Ogre::Vector2 x1 = Radius*mV2Cos,
      x2 = Radius*mBetaSin*mV2Cos,
      z1 = Radius*mV2Sin,
      z2 = Radius*mBetaSin*mV2Sin;

    float y0 = Radius*Ogre::Math::Sin(mAlpha);

    // Vertex 0
    _setVertexData(VertexOffset, Ogre::Vector3(x1.x, 0, z1.x), opacity);
    // Vertex 1
    _setVertexData(VertexOffset+1, Ogre::Vector3(x1.y, 0, z1.y), opacity);
    // Vertex 2
    _setVertexData(VertexOffset+2, Ogre::Vector3(x2.x, y0, z2.x), opacity);
    // Vertex 3
    _setVertexData(VertexOffset+3, Ogre::Vector3(x2.y, y0, z2.y), opacity);

    Ogre::Vector2 x3 = Radius*mAlphaSin*mV2Cos,
      z3 = Radius*mAlphaSin*mV2Sin;

    float y1 = Radius*Ogre::Math::Sin(mBeta);

    // Vertex 4
    _setVertexData(VertexOffset+4, Ogre::Vector3(x3.x, y1, z3.x), opacity);
    // Vertex 5
    _setVertexData(VertexOffset+5, Ogre::Vector3(x3.y, y1, z3.y), opacity);

    // Vertex 6
    _setVertexData(VertexOffset+6, Ogre::Vector3(0, Radius, 0), opacity);
  }

  void GeometryBlock::_setVertexData(const int& index, const Ogre::Vector3& p,
      const float& o)
  {
    float fallingDistance = mVClouds->getDistanceFallingParams().x*
      (mEntity->getParentSceneNode()->_getDerivedPosition().z-
       mCamera->getDerivedPosition().z)*
      (Ogre::Vector2(p.x, p.z).length()/mRadius);

    if (mVClouds->getDistanceFallingParams().y > 0)  // -1 means no max falling
    {
      if (fallingDistance > 0)
      {
        if (fallingDistance > mVClouds->getDistanceFallingParams().y)
        {
          fallingDistance = mVClouds->getDistanceFallingParams().y;
        }
      }
      else
      {
        if (-fallingDistance > mVClouds->getDistanceFallingParams().y)
        {
          fallingDistance = -mVClouds->getDistanceFallingParams().y;
        }
      }
    }

    // Position
    mVertices[index].x = p.x;
    // Z-Up
    mVertices[index].y = p.z;
    mVertices[index].z = p.y - fallingDistance;

    // 3D coords (Z-UP)
    float scale = mVClouds->getCloudFieldScale()/mRadius;
    mVertices[index].xc = (p.x+mWorldOffset.x)*scale;
    mVertices[index].yc = (p.z+mWorldOffset.y)*scale;
    mVertices[index].zc = Ogre::Math::Clamp<Ogre::Real>(p.y/mHeight, 0, 1);

    // Noise coords
    float noise_scale = mVClouds->getNoiseScale()/mRadius;
    float xz_length_radius = Ogre::Vector2(p.x, p.z).length() / mRadius;
    Ogre::Vector3 origin = Ogre::Vector3(0,
        -(mEntity->getParentSceneNode()->_getDerivedPosition().y-
          mCamera->getDerivedPosition().y) -mRadius*(0.5f+0.5f*
          Ogre::Vector2(p.x, p.z).length()/mRadius), 0);

    Ogre::Vector3 dir = (p-origin).normalisedCopy();
    float hip = Ogre::Math::Sqrt(
        Ogre::Math::Pow(xz_length_radius * mRadius, 2) +
        Ogre::Math::Pow(origin.y, 2));

    Ogre::Vector3 uv = dir*hip;  // Only x/z, += origin doesn't need
    mVertices[index].u = (uv.x+mWorldOffset.x)*noise_scale;
    mVertices[index].v = (uv.z+mWorldOffset.y)*noise_scale;

    // Opacity
    mVertices[index].o = o * mVClouds->getGlobalOpacity();
  }

  bool GeometryBlock::isInFrustum(Ogre::Camera *c) const
  {
    if (!mCreated)
    {
      return false;
    }

    // TODO: Use a world bounding box for each geometry zone, this way the
    // culling is going to be more acurrated and the geometry falling is going
    // to be culled
    // when the falling factor is bigger than 1.
    return c->isVisible(mEntity->getParentSceneNode()->_getWorldAABB());
  }
}}
