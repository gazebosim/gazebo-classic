#include <cmath>
#include <iostream>

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
void OgreSimpleShape::CreateBox(const std::string &name, const Vector3 &sides)
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
    {-1, -1, -1}, {+1, -1, -1}, {+1, +1, -1}, {-1, +1, -1},
    {-1, -1, +1}, {+1, -1, +1}, {+1, +1, +1}, {-1, +1, +1}
  };

  // Normals for each vertex
  float n[8][3]=
  {
    {-0.577350, -0.577350, -0.577350},
    {0.577350, -0.577350, -0.577350},
    {0.577350, 0.577350, -0.577350},
    {-0.577350, 0.577350, -0.577350},
    {-0.577350, -0.577350, 0.577350},
    {0.577350, -0.577350, 0.577350},
    {0.577350, 0.577350, 0.577350},
    {-0.577350, 0.577350, 0.577350}
  };

  // Texture coords
  int t[4][2] =
  {
    {0, 1}, {1,1}, {1, 0}, {0,0}
  };

  // Vertices
  int faces[6][4] =
  {
    {2, 1, 0, 3}, {6, 7, 4, 5},
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
    8, 9, 11,
    11, 9, 10,
    12, 13, 15,
    15, 13, 14,
    16, 17, 18,
    18, 19, 16,
    20, 21, 23,
    23, 21, 22
  };

  // Compute the vertices
  for (i = 0; i < 8; i++)
  {
    v[i][0] *= sides.x / 2;
    v[i][1] *= sides.y / 2;
    v[i][2] *= sides.z / 2;
  }

  // For each face
  for (i = 0; i < 6; i++)
  {
    // For each vertex in the face
    for (k=0; k<4; k++)
    {
      // Set the vertex
      for (j=0; j<3; j++)
        *vertices++ = v[faces[i][k]][j];

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
                      Ogre::Vector3(-sides.x/2.0, -sides.y/2.0, -sides.z/2.0),
                      Ogre::Vector3(sides.x/2.0, sides.y/2.0, sides.z/2.0)), false);

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
