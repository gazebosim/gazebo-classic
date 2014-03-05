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

#include <OgreMeshManager.h>
#include <OgreSubMesh.h>
#include <OgreHardwareBufferManager.h>

#include "gazebo/common/Console.hh"
#include "gazebo/rendering/deferred_shading/GeomUtils.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
void GeomUtils::CreateSphere(const Ogre::String &_strName, float _radius,
    int _nRings, int _nSegments, bool _bNormals, bool _bTexCoords)
{
  Ogre::MeshPtr pSphere = Ogre::MeshManager::getSingleton().createManual(
      _strName, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  Ogre::SubMesh *pSphereVertex = pSphere->createSubMesh();
  pSphere->sharedVertexData = new Ogre::VertexData();

  CreateSphere(pSphere->sharedVertexData, pSphereVertex->indexData,
               _radius, _nRings, _nSegments, _bNormals, _bTexCoords);

  // Generate face list
  pSphereVertex->useSharedVertices = true;

  std::cout << "CreateSphere. Radius[" << _radius << "]\n";
  // the original code was missing this line:
  pSphere->_setBounds(Ogre::AxisAlignedBox(
        Ogre::Vector3(-_radius, -_radius, -_radius),
        Ogre::Vector3(_radius, _radius, _radius)), false);
  pSphere->_setBoundingSphereRadius(_radius);

  // this line makes clear the mesh is loaded (avoids memory leaks)
  pSphere->load();
}

/////////////////////////////////////////////////
void GeomUtils::CreateSphere(Ogre::VertexData *&_vertexData,
  Ogre::IndexData *&_indexData, float _radius, int _nRings, int _nSegments,
  bool _bNormals, bool _bTexCoords)
{
  if (!_vertexData || !_indexData)
  {
    gzerr << "Invalid vertex or index data\n";
    return;
  }

  // define the vertex format
  Ogre::VertexDeclaration *vertexDecl = _vertexData->vertexDeclaration;
  size_t currOffset = 0;

  // positions
  vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  if (_bNormals)
  {
    // normals
    vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
    currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
  }

  // two dimensional texture coordinates
  if (_bTexCoords)
  {
    vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT2,
                           Ogre::VES_TEXTURE_COORDINATES, 0);
  }

  // allocate the vertex buffer
  _vertexData->vertexCount = (_nRings + 1) * (_nSegments+1);
  Ogre::HardwareVertexBufferSharedPtr vBuf =
    Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
        vertexDecl->getVertexSize(0), _vertexData->vertexCount,
        Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);

  Ogre::VertexBufferBinding *binding = _vertexData->vertexBufferBinding;
  binding->setBinding(0, vBuf);
  float *pVertex = static_cast<float*>(
      vBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  // allocate index buffer
  _indexData->indexCount = 6 * _nRings * (_nSegments + 1);
  _indexData->indexBuffer =
    Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
        Ogre::HardwareIndexBuffer::IT_16BIT, _indexData->indexCount,
        Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);

  Ogre::HardwareIndexBufferSharedPtr iBuf = _indexData->indexBuffer;
  uint16_t *pIndices = static_cast<uint16_t*>(
      iBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  float fDeltaRingAngle = (M_PI / _nRings);
  float fDeltaSegAngle = (2 * M_PI / _nSegments);
  uint16_t wVerticeIndex = 0;

  // Generate the group of rings for the sphere
  for (int ring = 0; ring <= _nRings; ++ring)
  {
    float r0 = _radius * sinf(ring * fDeltaRingAngle);
    float y0 = _radius * cosf(ring * fDeltaRingAngle);

    // Generate the group of segments for the current ring
    for (int seg = 0; seg <= _nSegments; ++seg)
    {
      float x0 = r0 * sinf(seg * fDeltaSegAngle);
      float z0 = r0 * cosf(seg * fDeltaSegAngle);

      // Add one vertex to the strip which makes up the sphere
      *pVertex++ = x0;
      *pVertex++ = y0;
      *pVertex++ = z0;

      if (_bNormals)
      {
        Ogre::Vector3 vNormal = Ogre::Vector3(x0, y0, z0).normalisedCopy();
        *pVertex++ = vNormal.x;
        *pVertex++ = vNormal.y;
        *pVertex++ = vNormal.z;
      }

      if (_bTexCoords)
      {
        *pVertex++ = seg / static_cast<float>(_nSegments);
        *pVertex++ = ring / static_cast<float>(_nRings);
      }

      if (ring != _nRings)
      {
        // each vertex (except the last) has six indices pointing to it
        *pIndices++ = wVerticeIndex + _nSegments + 1;
        *pIndices++ = wVerticeIndex;
        *pIndices++ = wVerticeIndex + _nSegments;
        *pIndices++ = wVerticeIndex + _nSegments + 1;
        *pIndices++ = wVerticeIndex + 1;
        *pIndices++ = wVerticeIndex;
        wVerticeIndex++;
      }
    }
  }

  // Unlock
  vBuf->unlock();
  iBuf->unlock();
}

/////////////////////////////////////////////////
void GeomUtils::CreateQuad(Ogre::VertexData *&_vertexData)
{
  if (!_vertexData)
  {
    gzerr << "Invalid vertex data\n";
    return;
  }

  _vertexData->vertexCount = 4;
  _vertexData->vertexStart = 0;

  Ogre::VertexDeclaration* vertexDecl = _vertexData->vertexDeclaration;
  Ogre::VertexBufferBinding* bind = _vertexData->vertexBufferBinding;

  vertexDecl->addElement(0, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);

  Ogre::HardwareVertexBufferSharedPtr vbuf =
    Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
        vertexDecl->getVertexSize(0),
        _vertexData->vertexCount,
        Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);

  // Bind buffer
  bind->setBinding(0, vbuf);

  // Upload data
  float data[] = {
  -1, 1, -1,    // corner 1
  -1, -1, -1,   // corner 2
  1, 1, -1,    // corner 3
  1, -1, -1};  // corner 4

  vbuf->writeData(0, sizeof(data), data, true);
}

/////////////////////////////////////////////////
void GeomUtils::CreateCone(const Ogre::String &_strName, float _radius,
                           float _height, int _nVerticesInBase)
{
  Ogre::MeshPtr pCone =
    Ogre::MeshManager::getSingleton().createManual(_strName,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  Ogre::SubMesh *pConeVertex = pCone->createSubMesh();
  pCone->sharedVertexData = new Ogre::VertexData();

  CreateCone(pCone->sharedVertexData, pConeVertex->indexData,
             _radius, _height, _nVerticesInBase);

  // Generate face list
  pConeVertex->useSharedVertices = true;

  // the original code was missing this line:
  pCone->_setBounds(Ogre::AxisAlignedBox(
        Ogre::Vector3(-_radius, 0, -_radius),
        Ogre::Vector3(_radius, _height, _radius)), false);

  pCone->_setBoundingSphereRadius(
      Ogre::Math::Sqrt(_height * _height + _radius * _radius));

  // this line makes clear the mesh is loaded (avoids memory leaks)
  pCone->load();
}

/////////////////////////////////////////////////
void GeomUtils::CreateCone(Ogre::VertexData *&_vertexData,
    Ogre::IndexData *&_indexData, float _radius, float _height,
    int _nVerticesInBase)
{
  if (!_vertexData || !_indexData)
  {
    gzerr << "Inavlid vertex or index data\n";
    return;
  }

  // define the vertex format
  Ogre::VertexDeclaration *vertexDecl = _vertexData->vertexDeclaration;

  // positions
  vertexDecl->addElement(0, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);

  // allocate the vertex buffer
  _vertexData->vertexCount = _nVerticesInBase + 1;
  Ogre::HardwareVertexBufferSharedPtr vBuf =
    Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
        vertexDecl->getVertexSize(0), _vertexData->vertexCount,
        Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);

  Ogre::VertexBufferBinding *binding = _vertexData->vertexBufferBinding;
  binding->setBinding(0, vBuf);
  float* pVertex = static_cast<float*>(
      vBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  // allocate index buffer - cone and base
  _indexData->indexCount = (3 * _nVerticesInBase) +
                           (3 * (_nVerticesInBase - 2));
  _indexData->indexBuffer =
    Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
        Ogre::HardwareIndexBuffer::IT_16BIT, _indexData->indexCount,
        Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);

  Ogre::HardwareIndexBufferSharedPtr iBuf = _indexData->indexBuffer;
  uint16_t *pIndices = static_cast<uint16_t*>(
      iBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  // Positions : cone head and base
  for (int i = 0; i < 3; ++i)
    *pVertex++ = 0.0f;

  // Base
  float fDeltaBaseAngle = (2 * M_PI) / _nVerticesInBase;

  for (int i = 0; i < _nVerticesInBase; ++i)
  {
    float angle = i * fDeltaBaseAngle;
    *pVertex++ = _radius * cosf(angle);
    *pVertex++ = _height;
    *pVertex++ = _radius * sinf(angle);
  }

  // Indices
  // Cone head to vertices
  for (int i = 0; i < _nVerticesInBase; ++i)
  {
    *pIndices++ = 0;
    *pIndices++ = (i % _nVerticesInBase) + 1;
    *pIndices++ = ((i + 1) % _nVerticesInBase) + 1;
  }

  // Cone base
  for (int i = 0; i < _nVerticesInBase - 2; ++i)
  {
    *pIndices++ = 1;
    *pIndices++ = i + 3;
    *pIndices++ = i + 2;
  }

  // Unlock
  vBuf->unlock();
  iBuf->unlock();
}
