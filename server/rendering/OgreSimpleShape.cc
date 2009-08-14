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
/* Desc: Create various 3d shapes
 * Author: Nathan Koenig
 * Date: 3 Jan 2008
 */

#include <cmath>
#include <iostream>

#include "GazeboMessage.hh"
#include "OgreSimpleShape.hh"
#include "Vector3.hh"

#define SIGN(r) Ogre::Math::Sign(r)

using namespace gazebo;


////////////////////////////////////////////////////////////////////////////////
// Constructor
OgreSimpleShape::OgreSimpleShape()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
OgreSimpleShape::~OgreSimpleShape()
{
}

////////////////////////////////////////////////////////////////////////////////
// Create a sphere
void OgreSimpleShape::CreateSphere(const std::string &name, float radius, int rings, int segments)
{
  Ogre::MeshPtr mesh;
  Ogre::SubMesh *subMesh;
  Ogre::VertexData *vertexData;
  Ogre::VertexDeclaration* vertexDecl;
  Ogre::HardwareVertexBufferSharedPtr vBuf;
  Ogre::HardwareIndexBufferSharedPtr iBuf;

  float *vertices;
  unsigned short *indices;

  int ring, seg;
  float r0;
  float deltaSegAngle = (2.0 * M_PI / segments);
  float deltaRingAngle = (M_PI / rings);
  Vector3 vert, norm;
  unsigned short verticeIndex = 0;
  size_t currOffset = 0;

  // Create a new mesh specifically for manual definition.
  mesh = Ogre::MeshManager::getSingleton().createManual(name,
         Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  subMesh = mesh->createSubMesh();

  mesh->sharedVertexData = new Ogre::VertexData();
  vertexData = mesh->sharedVertexData;

  // define the vertex format
  vertexDecl = vertexData->vertexDeclaration;

  // The vertexDecl should contain positions, blending weights, normals,
  // diffiuse colors, specular colors, tex coords. In that order.

  // positions
  vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  // TODO: blending weights

  // normals
  vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
  currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  // TODO: diffuse colors

  // TODO: specular colors

  // two dimensional texture coordinates
  vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT2,
                         Ogre::VES_TEXTURE_COORDINATES, 0);
  currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);


  // allocate the vertex buffer
  vertexData->vertexCount = (rings + 1) * (segments+1);
  vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
           vertexDecl->getVertexSize(0),
           vertexData->vertexCount,
           Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
           false);

  vertexData->vertexBufferBinding->setBinding(0, vBuf);
  vertices = static_cast<float*>(vBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  // allocate index buffer
  subMesh->indexData->indexCount = 6 * rings * (segments + 1);
  subMesh->indexData->indexBuffer =
    Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
      Ogre::HardwareIndexBuffer::IT_16BIT,
      subMesh->indexData->indexCount,
      Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
      false);

  iBuf = subMesh->indexData->indexBuffer;
  indices = static_cast<unsigned short*>(iBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  // Generate the group of rings for the sphere
  for (ring = 0; ring <= rings; ring++)
  {
    r0 = radius * sinf (ring * deltaRingAngle);
    vert.y = radius * cosf (ring * deltaRingAngle);

    // Generate the group of segments for the current ring
    for (seg = 0; seg <= segments; seg++)
    {
      vert.x = r0 * sinf(seg * deltaSegAngle);
      vert.z = r0 * cosf(seg * deltaSegAngle);

      // TODO: Don't think these normals are correct.
      norm = vert;
      norm.Normalize();

      // Add one vertex to the strip which makes up the sphere
      *vertices++ = vert.x;
      *vertices++ = vert.y;
      *vertices++ = vert.z;

      *vertices++ = norm.x;
      *vertices++ = norm.y;
      *vertices++ = norm.z;

      // Texture coords
      *vertices++ = (float) seg / (float) segments;
      *vertices++ = (float) ring / (float) rings;

      if (ring != rings)
      {
        // each vertex (except the last) has six indices pointing to it
        *indices++ = verticeIndex + segments + 1;
        *indices++ = verticeIndex;
        *indices++ = verticeIndex + segments;
        *indices++ = verticeIndex + segments + 1;
        *indices++ = verticeIndex + 1;
        *indices++ = verticeIndex;
        verticeIndex++;
      }
    }
  }

  // Unlock
  vBuf->unlock();
  iBuf->unlock();

  // Generate face list
  subMesh->useSharedVertices = true;

  mesh->_setBounds( Ogre::AxisAlignedBox(
                      Ogre::Vector3(-radius, -radius, -radius),
                      Ogre::Vector3(radius, radius, radius)), false);

  mesh->_setBoundingSphereRadius(radius);

  // this line makes clear the mesh is loaded (avoids memory leaks)
  mesh->load();
}


////////////////////////////////////////////////////////////////////////////////
/// Create a Box mesh
void OgreSimpleShape::CreateBox(const std::string &name, const Vector3 &sides,
    const Vector2<double> &uvCoords)
{
  Ogre::MeshPtr mesh;
  Ogre::SubMesh *subMesh;
  Ogre::VertexData *vertexData;
  Ogre::VertexDeclaration* vertexDecl;
  Ogre::HardwareVertexBufferSharedPtr vBuf;
  Ogre::HardwareIndexBufferSharedPtr iBuf;
  float *vertices;
  unsigned short *indices;
  Vector3 vert, norm;
  size_t currOffset = 0;
  int i,j,k;

  if (Ogre::MeshManager::getSingleton().resourceExists(name))
  {
    return;
  }

  // Create a new mesh specifically for manual definition.
  mesh = Ogre::MeshManager::getSingleton().createManual(name,
         Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  subMesh = mesh->createSubMesh();

  mesh->sharedVertexData = new Ogre::VertexData();
  vertexData = mesh->sharedVertexData;

  // define the vertex format
  vertexDecl = vertexData->vertexDeclaration;

  // The vertexDecl should contain positions, blending weights, normals,
  // diffiuse colors, specular colors, tex coords. In that order.

  // positions
  vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  // TODO: blending weights

  // normals
  vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
  currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

  // TODO: diffuse colors

  // TODO: specular colors

  // two dimensional texture coordinates
  vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT2,
                         Ogre::VES_TEXTURE_COORDINATES, 0);
  currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);


  // allocate the vertex buffer
  vertexData->vertexCount = 24;
  vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
           vertexDecl->getVertexSize(0),
           vertexData->vertexCount,
           Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
           false);

  vertexData->vertexBufferBinding->setBinding(0, vBuf);
  vertices = static_cast<float*>(vBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  // allocate index buffer
  subMesh->indexData->indexCount = 36;
  subMesh->indexData->indexBuffer =
    Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
      Ogre::HardwareIndexBuffer::IT_16BIT,
      subMesh->indexData->indexCount,
      Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
      false);

  iBuf = subMesh->indexData->indexBuffer;
  indices = static_cast<unsigned short*>(iBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

  // Vertex values
  float v[8][3] =
  {
    {-1, -1, -1}, {-1, -1, +1}, {+1, -1, +1}, {+1, -1, -1},
    {-1, +1, -1}, {-1, +1, +1}, {+1, +1, +1}, {+1, +1, -1}
  };

  // Normals for each vertex
  float n[8][3]=
  {
    {-0.577350, -0.577350, -0.577350},
    {-0.577350, -0.577350, 0.577350},
    {0.577350, -0.577350, 0.577350},
    {0.577350, -0.577350, -0.577350},
    {-0.577350, 0.577350, -0.577350},
    {-0.577350, 0.577350, 0.577350},
    {0.577350, 0.577350, 0.577350},
    {0.577350, 0.577350, -0.577350}
  };

  // Texture coords
  float t[4][2] =
  {
    {uvCoords.x, 0}, {0, 0}, {0,uvCoords.y}, {uvCoords.x, uvCoords.y}
  };

  // Vertices
  int faces[6][4] =
  {
    {2, 1, 0, 3}, {5, 6, 7, 4},
    {2, 6, 5, 1}, {1, 5, 4, 0},
    {0, 4, 7, 3}, {6, 2, 3, 7}
  };

  // Indices
  int ind[36] =
  {
    0, 1, 2,
    2, 3, 0,
    4, 5, 7,
    7, 5, 6,
    11,8,9,
    9,10,11,
    12, 13, 15,
    15, 13, 14,
    16, 17, 18,
    18, 19, 16,
    21,22,23,
    23,20,21,
  };

  // Compute the vertices
  for (i = 0; i < 8; i++)
  {
    v[i][0] *= sides.x * 0.5;
    v[i][1] *= sides.y * 0.5;
    v[i][2] *= sides.z * 0.5;
  }

  // For each face
  for (i = 0; i < 6; i++)
  {
    // For each vertex in the face
    for (k=0; k<4; k++)
    {
      // Set the vertex
      for (j=0; j<3; j++)
      {
        *vertices++ = v[faces[i][k]][j];
      }

      // Set the normal
      for (j=0; j<3; j++)
        *vertices++ = n[faces[i][k]][j];

      // Set the texture coord
      *vertices++ = t[k][0];
      *vertices++ = t[k][1];
    }
  }

  // Set the indices
  for (i=0;i<36; i++)
  {
    *indices++ = ind[i];
  }

  // Unlock
  vBuf->unlock();
  iBuf->unlock();

  // Generate face list
  subMesh->useSharedVertices = true;

  mesh->_setBounds( Ogre::AxisAlignedBox(
                      Ogre::Vector3(-sides.x*0.5, -sides.y*0.5, -sides.z*0.5),
                      Ogre::Vector3(sides.x*0.5, sides.y*0.5, sides.z*0.5)), 
                    false);

  // this line makes clear the mesh is loaded (avoids memory leaks)
  mesh->load();
}


////////////////////////////////////////////////////////////////////////////////
/// Create a cylinder mesh
void OgreSimpleShape::CreateCylinder(const std::string &name, float radius, float height, int rings, int segments)
{
  Ogre::MeshPtr mesh;
  Ogre::SubMesh *subMesh;
  Ogre::VertexData *vertexData;
  Ogre::VertexDeclaration* vertexDecl;
  Ogre::HardwareVertexBufferSharedPtr vBuf;
  Ogre::HardwareIndexBufferSharedPtr iBuf;
  float *vertices, *vertStart;
  unsigned short *indices, *indStart;
  Vector3 vert, norm;
  unsigned short verticeIndex = 0;
  size_t currOffset = 0;
  unsigned int i,j;
  int ring, seg;
  float deltaSegAngle = (2.0 * M_PI / segments);

  try
  {
    // Create a new mesh specifically for manual definition.
    mesh = Ogre::MeshManager::getSingleton().createManual(name,
           Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    subMesh = mesh->createSubMesh();

    mesh->sharedVertexData = new Ogre::VertexData();
    vertexData = mesh->sharedVertexData;

    // define the vertex format
    vertexDecl = vertexData->vertexDeclaration;

    // The vertexDecl should contain positions, blending weights, normals,
    // diffiuse colors, specular colors, tex coords. In that order.

    // positions
    vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

    // TODO: blending weights

    // normals
    vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
    currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

    // TODO: diffuse colors

    // TODO: specular colors

    // two dimensional texture coordinates
    vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT2,
                           Ogre::VES_TEXTURE_COORDINATES, 0);
    currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);


    // allocate the vertex buffer
    vertexData->vertexCount = (rings + 1) * (segments+1) + 2;
    vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
             vertexDecl->getVertexSize(0),
             vertexData->vertexCount,
             Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
             false);

    vertexData->vertexBufferBinding->setBinding(0, vBuf);
    vertices = static_cast<float*>(vBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

    // allocate index buffer
    subMesh->indexData->indexCount = 6 * rings * (segments + 1) + (segments*6)+2;
    subMesh->indexData->indexBuffer =
      Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
        Ogre::HardwareIndexBuffer::IT_16BIT,
        subMesh->indexData->indexCount,
        Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
        false);

    iBuf = subMesh->indexData->indexBuffer;
    indices = static_cast<unsigned short*>(iBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

    vertStart = vertices;
    indStart = indices;

    // Generate the group of rings for the sphere
    for (ring = 0; ring <= rings; ring++)
    {
      vert.z = ring * height/rings - height/2.0;

      // Generate the group of segments for the current ring
      for (seg = 0; seg <= segments; seg++)
      {
        vert.y = radius * cosf(seg * deltaSegAngle);
        vert.x = radius * sinf(seg * deltaSegAngle);


        // TODO: Don't think these normals are correct.
        norm = vert;
        norm.Normalize();

        // Add one vertex to the strip which makes up the sphere
        *vertices++ = vert.x;
        *vertices++ = vert.y;
        *vertices++ = vert.z;

        *vertices++ = norm.x;
        *vertices++ = norm.y;
        *vertices++ = norm.z;

        // Texture coords
        *vertices++ = (float) seg / (float) segments;
        *vertices++ = (float) ring / (float) rings;

        if (ring != rings)
        {
          // each vertex (except the last) has six indices pointing to it
          *indices++ = verticeIndex + segments + 1;
          *indices++ = verticeIndex;
          *indices++ = verticeIndex + segments;
          *indices++ = verticeIndex + segments + 1;
          *indices++ = verticeIndex + 1;
          *indices++ = verticeIndex;

          verticeIndex++;
        }
      }
    }

    /// The top cap vertex
    *vertices++ = 0;
    *vertices++ = 0;
    *vertices++ = height/2.0;

    *vertices++ = 0;
    *vertices++ = 0;
    *vertices++ = 1;

    *vertices++ = 0;
    *vertices++ = 0;


    // The bottom cap vertex
    *vertices++ = 0;
    *vertices++ = 0;
    *vertices++ = -height/2.0;

    *vertices++ = 0;
    *vertices++ = 0;
    *vertices++ = -1;

    *vertices++ = 0;
    *vertices++ = 0;

    // Create the top fan
    verticeIndex += segments + 1;
    for (seg=0; seg < segments; seg++)
    {
      *indices++ = verticeIndex;
      *indices++ = verticeIndex - segments + seg;
      *indices++ = verticeIndex - segments + seg - 1;
    }

    // Create the bottom fan
    verticeIndex++;
    for (seg=0; seg < segments; seg++)
    {
      *indices++ = verticeIndex;
      *indices++ = seg;
      *indices++ = seg+1;
    }

    // Fix all the normals
    for (i=0; i+3<subMesh->indexData->indexCount; i+=3)
    {
      norm.Set();

      for (j=0; j<3; j++)
      {
        int index = indStart[i+j];
        norm += Vector3(vertStart[index*8+3],
                        vertStart[index*8+4],
                        vertStart[index*8+5]);
      }

      norm /= 3;
      norm.Normalize();

      for (j=0; j<3; j++)
      {
        for (int k=0; k<3; k++)
        {
          int index = indStart[i+j];
          vertStart[index*8+3+k] = norm[k];
        }
      }
    }

    // Unlock
    vBuf->unlock();
    iBuf->unlock();

    // Generate face list
    subMesh->useSharedVertices = true;

    mesh->_setBounds( Ogre::AxisAlignedBox(
                        Ogre::Vector3(-radius, -height/2, -radius),
                        Ogre::Vector3(radius, height/2, radius)), false);

    // this line makes clear the mesh is loaded (avoids memory leaks)
    mesh->load();
  }
  catch (Ogre::Exception e)
  {
    std::cerr << "Unable to create a basic Unit cylinder object" << std::endl;
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Create a cone mesh
void OgreSimpleShape::CreateCone(const std::string &name, float radius, float height, int rings, int segments)
{
  Ogre::MeshPtr mesh;
  Ogre::SubMesh *subMesh;
  Ogre::VertexData *vertexData;
  Ogre::VertexDeclaration* vertexDecl;
  Ogre::HardwareVertexBufferSharedPtr vBuf;
  Ogre::HardwareIndexBufferSharedPtr iBuf;
  float *vertices, *vertStart;
  unsigned short *indices, *indStart;
  Vector3 vert, norm;
  unsigned short verticeIndex = 0;
  size_t currOffset = 0;
  unsigned int i,j;
  int ring, seg;

  if (segments <3)
    segments = 3;

  float deltaSegAngle = (2.0 * M_PI / segments);

  try
  {
    // Create a new mesh specifically for manual definition.
    mesh = Ogre::MeshManager::getSingleton().createManual(name,
           Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    subMesh = mesh->createSubMesh();

    mesh->sharedVertexData = new Ogre::VertexData();
    vertexData = mesh->sharedVertexData;

    // define the vertex format
    vertexDecl = vertexData->vertexDeclaration;

    // The vertexDecl should contain positions, blending weights, normals,
    // diffiuse colors, specular colors, tex coords. In that order.

    // positions
    vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

    // TODO: blending weights

    // normals
    vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
    currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

    // TODO: diffuse colors

    // TODO: specular colors

    // two dimensional texture coordinates
    vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT2,
                           Ogre::VES_TEXTURE_COORDINATES, 0);
    currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);


    // allocate the vertex buffer
    vertexData->vertexCount = rings * (segments+1) + 2;
    vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
             vertexDecl->getVertexSize(0),
             vertexData->vertexCount,
             Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
             false);


    vertexData->vertexBufferBinding->setBinding(0, vBuf);
    vertices = static_cast<float*>(vBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

    // allocate index buffer
    subMesh->indexData->indexCount = 6 * (rings-1) * (segments+1) + (segments*6);
    subMesh->indexData->indexBuffer =
      Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
        Ogre::HardwareIndexBuffer::IT_16BIT,
        subMesh->indexData->indexCount,
        Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
        false);

    iBuf = subMesh->indexData->indexBuffer;
    indices = static_cast<unsigned short*>(iBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

    vertStart = vertices;
    indStart = indices;

    double ringRadius;

    // Generate the group of rings for the cone
    for (ring = 0; ring < rings; ring++)
    {
      vert.z = ring * height/rings - height/2.0;

      ringRadius = ((height - (vert.z+height/2.0)) / height) * radius;

      // Generate the group of segments for the current ring
      for (seg = 0; seg <= segments; seg++)
      {
        vert.y = ringRadius * cosf(seg * deltaSegAngle);
        vert.x = ringRadius * sinf(seg * deltaSegAngle);

        // TODO: Don't think these normals are correct.
        norm = vert;
        norm.Normalize();

        // Add one vertex to the strip which makes up the sphere
        *vertices++ = vert.x;
        *vertices++ = vert.y;
        *vertices++ = vert.z;

        *vertices++ = norm.x;
        *vertices++ = norm.y;
        *vertices++ = norm.z;

        // Texture coords
        *vertices++ = (float) seg / (float) segments;
        *vertices++ = (float) ring / (float) rings;

        if (ring != (rings-1))
        {
          // each vertex (except the last) has six indices pointing to it
          *indices++ = verticeIndex + segments + 1;
          *indices++ = verticeIndex;
          *indices++ = verticeIndex + segments;
          *indices++ = verticeIndex + segments + 1;
          *indices++ = verticeIndex + 1;
          *indices++ = verticeIndex;

          verticeIndex++;
        }
      }
    }

    /// The top point vertex
    *vertices++ = 0;
    *vertices++ = 0;
    *vertices++ = height/2.0;

    *vertices++ = 0;
    *vertices++ = 0;
    *vertices++ = 1;

    *vertices++ = 0;
    *vertices++ = 0;


    // The bottom cap vertex
    *vertices++ = 0;
    *vertices++ = 0;
    *vertices++ = -height/2.0;

    *vertices++ = 0;
    *vertices++ = 0;
    *vertices++ = -1;

    *vertices++ = 0;
    *vertices++ = 0;

    // Create the top fan
    verticeIndex += segments+1;
    for (seg=0; seg < segments; seg++)
    {
      *indices++ = verticeIndex;
      *indices++ = verticeIndex - segments + seg;
      *indices++ = verticeIndex - segments + seg - 1;
    }

    // Create the bottom fan
    verticeIndex++;
    //verticeIndex += segments + 1;
    for (seg=0; seg < segments; seg++)
    {
      *indices++ = verticeIndex;
      *indices++ = seg;
      *indices++ = seg+1;
    }

    // Fix all the normals
    for (i=0; i+3<subMesh->indexData->indexCount; i+=3)
    {
      norm.Set();

      for (j=0; j<3; j++)
      {
        int index = indStart[i+j];
        norm += Vector3(vertStart[index*8+3],
                        vertStart[index*8+4],
                        vertStart[index*8+5]);
      }

      norm /= 3;
      norm.Normalize();

      for (j=0; j<3; j++)
      {
        for (int k=0; k<3; k++)
        {
          int index = indStart[i+j];
          vertStart[index*8+3+k] = norm[k];
        }
      }
    }

    // Unlock
    vBuf->unlock();
    iBuf->unlock();

    // Generate face list
    subMesh->useSharedVertices = true;

    mesh->_setBounds( Ogre::AxisAlignedBox(
                        Ogre::Vector3(-radius, -height/2, -radius),
                        Ogre::Vector3(radius, height/2, radius)), false);

    // this line makes clear the mesh is loaded (avoids memory leaks)
    mesh->load();
  }
  catch (Ogre::Exception e)
  {
    std::cerr << "Unable to create a basic Unit cylinder object" << std::endl;
  }

}

////////////////////////////////////////////////////////////////////////////////
/// Create a tube mesh
void OgreSimpleShape::CreateTube(const std::string &name, float innerRadius, float outterRadius, float height, int rings, int segments)
{
  Ogre::MeshPtr mesh;
  Ogre::SubMesh *subMesh;
  Ogre::VertexData *vertexData;
  Ogre::VertexDeclaration* vertexDecl;
  Ogre::HardwareVertexBufferSharedPtr vBuf;
  Ogre::HardwareIndexBufferSharedPtr iBuf;
  float *vertices, *vertStart;
  unsigned short *indices, *indStart;
  Vector3 vert, norm;
  unsigned short verticeIndex = 0;
  size_t currOffset = 0;
  int ring, seg;
  float deltaSegAngle = (2.0 * M_PI / segments);

  try
  {
    // Create a new mesh specifically for manual definition.
    mesh = Ogre::MeshManager::getSingleton().createManual(name,
           Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    subMesh = mesh->createSubMesh();

    mesh->sharedVertexData = new Ogre::VertexData();
    vertexData = mesh->sharedVertexData;

    // define the vertex format
    vertexDecl = vertexData->vertexDeclaration;

    // The vertexDecl should contain positions, blending weights, normals,
    // diffiuse colors, specular colors, tex coords. In that order.

    // positions
    vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

    // TODO: blending weights

    // normals
    vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
    currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

    // TODO: diffuse colors

    // TODO: specular colors

    // two dimensional texture coordinates
    vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT2,
                           Ogre::VES_TEXTURE_COORDINATES, 0);
    currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);


    // Needs at lest 2 rings, and 3 segments
    rings = std::max(rings,1);
    segments = std::max(segments,3);

    // allocate the vertex buffer
    vertexData->vertexCount = (rings+1) * (segments+1) * 2;

    vBuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
             vertexDecl->getVertexSize(0),
             vertexData->vertexCount,
             Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
             false);

    vertexData->vertexBufferBinding->setBinding(0, vBuf);
    vertices = static_cast<float*>(vBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

    // allocate index buffer
    subMesh->indexData->indexCount = (6 * rings * (segments+1)) * 2 + ((segments+1) * 6) * 2;

    subMesh->indexData->indexBuffer =
      Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
        Ogre::HardwareIndexBuffer::IT_16BIT,
        subMesh->indexData->indexCount,
        Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
        false);

    iBuf = subMesh->indexData->indexBuffer;
    indices = static_cast<unsigned short*>(iBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

    vertStart = vertices;
    indStart = indices;

    float radius= 0;

    radius = outterRadius;

    // Generate the group of rings for the outsides of the cylinder
    for (ring = 0; ring <= rings; ring++)
    {
      printf("RING[%d]\n", ring);
      vert.z = ring * height/rings - height/2.0;

      // Generate the group of segments for the current ring
      for (seg = 0; seg <= segments; seg++)
      {
        printf("Seg[%d]\n", seg);
        vert.y = radius * cosf(seg * deltaSegAngle);
        vert.x = radius * sinf(seg * deltaSegAngle);
        printf("Vertexp[%f %f %f]\n", vert.x, vert.y, vert.z);

        // TODO: Don't think these normals are correct.
        norm = vert;
        norm.Normalize();

        // Add one vertex to the strip which makes up the tube
        *vertices++ = vert.x;
        *vertices++ = vert.y;
        *vertices++ = vert.z;

        *vertices++ = norm.x;
        *vertices++ = norm.y;
        *vertices++ = norm.z;

        // Texture coords
        *vertices++ = (float) seg / (float) segments;
        *vertices++ = (float) ring / (float) rings;

        if (ring != rings)
        {
          // each vertex (except the last) has six indices
          *indices++ = verticeIndex + segments + 1;
          *indices++ = verticeIndex;
          *indices++ = verticeIndex + segments;
          *indices++ = verticeIndex + segments + 1;
          *indices++ = verticeIndex + 1;
          *indices++ = verticeIndex;
        }
        else
        {
          // This indices form the top cap
          *indices++ = verticeIndex;
          *indices++ = verticeIndex + segments + 1;
          *indices++ = verticeIndex+1;
          *indices++ = verticeIndex+1;
          *indices++ = verticeIndex + segments + 1;
          *indices++ = verticeIndex + segments + 2;
        }

        // There indices form the bottom cap
        if (ring == 0 && seg < segments)
        {
          *indices++ = verticeIndex+1;
          *indices++ = verticeIndex + (segments+1) * (((rings+1)*2)-1);
          *indices++ = verticeIndex;
          *indices++ = verticeIndex + (segments+1) * (((rings+1)*2)-1) + 1;
          *indices++ = verticeIndex + (segments+1) * (((rings+1)*2)-1);
          *indices++ = verticeIndex+1;
        }

        verticeIndex++;
      }
    }

    // Generate the group of rings for the inside of the cylinder
    radius = innerRadius;
    for (ring = 0; ring <= rings; ring++)
    {
      vert.z = (height/2.0) - (ring * height/rings);

      // Generate the group of segments for the current ring
      for (seg = 0; seg <= segments; seg++)
      {
        vert.y = radius * cosf(seg * deltaSegAngle);
        vert.x = radius * sinf(seg * deltaSegAngle);

        // TODO: Don't think these normals are correct.
        norm = vert;
        norm.Normalize();

        // Add one vertex to the strip which makes up the tube
        *vertices++ = vert.x;
        *vertices++ = vert.y;
        *vertices++ = vert.z;

        *vertices++ = norm.x;
        *vertices++ = norm.y;
        *vertices++ = norm.z;

        // Texture coords
        *vertices++ = (float) seg / (float) segments;
        *vertices++ = (float) ring / (float) rings;

        if (ring != rings)
        {
          // each vertex (except the last) has six indices
          *indices++ = verticeIndex + segments + 1;
          *indices++ = verticeIndex;
          *indices++ = verticeIndex + segments;

          *indices++ = verticeIndex + segments + 1;
          *indices++ = verticeIndex + 1;
          *indices++ = verticeIndex;
        }

        verticeIndex++;
      }
    }

    // Unlock
    vBuf->unlock();
    iBuf->unlock();

    // Generate face list
    subMesh->useSharedVertices = true;

    mesh->_setBounds( Ogre::AxisAlignedBox(
                        Ogre::Vector3(-outterRadius, -height/2, -outterRadius),
                        Ogre::Vector3(outterRadius, height/2, outterRadius)), 
                        false );

    // this line makes clear the mesh is loaded (avoids memory leaks)
    mesh->load();
  }
  catch (Ogre::Exception e)
  {
    std::cerr << "Unable to create a basic Unit cylinder object" << std::endl;
  }
}
