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

#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/RayQueryPrivate.hh"
#include "gazebo/rendering/RayQuery.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
RayQuery::RayQuery(CameraPtr _camera)
  : dataPtr(new RayQueryPrivate)
{
  this->dataPtr->camera = _camera;
}

/////////////////////////////////////////////////
RayQuery::~RayQuery()
{
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
bool RayQuery::RaycastFromPoint(int _x, int _y, math::Vector3 &_result)
{
  Ogre::SceneManager *sceneMgr =
      this->dataPtr->camera->GetScene()->GetManager();
  Ogre::RaySceneQuery *raySceneQuery =
      sceneMgr->createRayQuery(Ogre::Ray(),
      Ogre::SceneManager::WORLD_GEOMETRY_TYPE_MASK);

  if(!raySceneQuery)
  {
    gzerr << "Failed to create a ray scene query " << std::endl;
    return false;
  }

  raySceneQuery->setSortByDistance(true);

  // create the ray to test
  // Ogre::Ray ray(Conversions::Convert(_point), Conversions::Convert(_normal));
  Ogre::Ray ray =
      this->dataPtr->camera->GetOgreCamera()->getCameraToViewportRay(
      static_cast<float>(_x) / this->dataPtr->camera->GetViewportWidth(),
      static_cast<float>(_y) / this->dataPtr->camera->GetViewportHeight());

  // create a query object
  raySceneQuery->setRay(ray);

  // execute the query, returns a vector of hits
  if (raySceneQuery->execute().size() <= 0)
    // raycast did not hit an objects bounding box
    return false;

  // at this point we have raycast to a series of different objects
  // bounding boxes. We need to test these different objects to see which is
  // the first polygon hit. There are some minor optimizations (distance based)
  // that mean we wont have to check all of the objects most of the time,
  // but the worst case scenario is that we need to test every triangle of
  // every object.

  Ogre::Real closestDistance = -1.0f;
  Ogre::Vector3 closestResult;
  Ogre::RaySceneQueryResult& queryResult = raySceneQuery->getLastResults();

  for (unsigned int qrIdx = 0, size = queryResult.size(); qrIdx < size; ++qrIdx)
  {
    // stop checking if we have found a raycast hit that is closer
    // than all remaining entities
    if( closestDistance >= 0.0f &&
        closestDistance < queryResult[qrIdx].distance)
       break;

    // only check this result if its a hit against an _entity
    if (queryResult[qrIdx].movable)
    {
      const std::string &movableType =
          queryResult[qrIdx].movable->getMovableType();
      // _mesh data to retrieve
      // unsigned int vertexCount;
      // unsigned int indexCount;
      std::vector<Ogre::Vector3> vertices;
      std::vector<unsigned long> indices;

      if (movableType == "ManualObject")
      {
        // get the manual object to check
        Ogre::ManualObject *manual =
            static_cast<Ogre::ManualObject *>(queryResult[qrIdx].movable);
        // get the _mesh information
        GetManualObjectInformation(manual,
           manual->getParentNode()->_getDerivedPosition(),
           manual->getParentNode()->_getDerivedOrientation(),
           manual->getParentNode()->_getDerivedScale(),
           vertices, indices
        );
      } else if (movableType == "Entity")
      {
        // get the entity to check
        Ogre::Entity *entity =
            static_cast<Ogre::Entity *>(queryResult[qrIdx].movable);
        // get the _mesh information
        GetEntityInformation(entity,
           entity->getParentNode()->_getDerivedPosition(),
           entity->getParentNode()->_getDerivedOrientation(),
           entity->getParentNode()->_getDerivedScale(),
           vertices, indices
       );
      } else
      {
        continue;
      }

      // test for hitting individual triangles on the _mesh
      bool newClosestFound = false;
      for (unsigned int i = 0; i < indices.size(); i += 3)
      {
        // check for a hit against this triangle
        std::pair<bool, Ogre::Real> hit =
            Ogre::Math::intersects(ray, vertices[indices[i]],
            vertices[indices[i+1]], vertices[indices[i+2]], true, false);

        // if it was a hit check if its the closest
        if (hit.first &&
            (closestDistance < 0.0f || hit.second < closestDistance))
        {
          // this is the closest so far, save it off
          closestDistance = hit.second;
          newClosestFound = true;
        }
      }

      // free the verticies and indicies memory
      // delete [] vertices;
      // delete [] indices;

      // if we found a new closest raycast for this object, update the
      // closestResult before moving on to the next object.
      if (newClosestFound)
        closestResult = ray.getPoint(closestDistance);
    }
  }

  sceneMgr->destroyQuery(raySceneQuery);

  // return the result
  if (closestDistance >= 0.0f)
  {
    // raycast success
    _result = Conversions::Convert(closestResult);
    return true;
  }
  // raycast failed
  return false;
}

/////////////////////////////////////////////////
void RayQuery::GetEntityInformation(const Ogre::Entity *_entity,
          const Ogre::Vector3 &_position, const Ogre::Quaternion &_orient,
          const Ogre::Vector3 &_scale, std::vector<Ogre::Vector3> &_vertices,
          std::vector<unsigned long> &_indices)
{
  bool addedShared = false;
  unsigned int currentOffset = 0;
  unsigned int sharedOffset = 0;
  unsigned int nextOffset = 0;
  unsigned int indexOffset = 0;
  unsigned int vertexCount = 0;
  unsigned int indexCount = 0;
  // _vertexCount = _indexCount = 0;

  Ogre::MeshPtr mesh = _entity->getMesh();

  bool useSoftwareBlendingVertices = _entity->hasSkeleton();

  if (useSoftwareBlendingVertices)
    const_cast<Ogre::Entity *>(_entity)->_updateAnimation();

  // Calculate how many vertices and indices we're going to need
  for (unsigned short i = 0, size = mesh->getNumSubMeshes(); i < size; ++i)
  {
    Ogre::SubMesh *submesh = mesh->getSubMesh(i);

    // We only need to add the shared vertices once
    if (submesh->useSharedVertices)
    {
      if (!addedShared)
      {
        vertexCount += mesh->sharedVertexData->vertexCount;
        addedShared = true;
      }
    }
    else
    {
      vertexCount += submesh->vertexData->vertexCount;
    }

    // Add the indices
    indexCount += submesh->indexData->indexCount;
  }

  // Allocate space for the vertices and indices
  // _vertices = new Ogre::Vector3[_vertexCount];
  // _indices = new unsigned long[_indexCount];
  _vertices.resize(vertexCount);
  _indices.resize(indexCount);

  addedShared = false;

  // Run through the submeshes again, adding the data into the arrays
  for (unsigned short i = 0, size = mesh->getNumSubMeshes(); i < size; ++i)
  {
    Ogre::SubMesh *submesh = mesh->getSubMesh(i);

    // GET VERTEXDATA

    // Ogre::VertexData* vertexData = submesh->useSharedVertices ?
    //    _mesh->sharedVertexData : submesh->vertexData;
    Ogre::VertexData* vertexData;

    // When there is animation:
    if (useSoftwareBlendingVertices)
    {
      vertexData = submesh->useSharedVertices ?
          _entity->_getSkelAnimVertexData() :
          _entity->getSubEntity(i)->_getSkelAnimVertexData();
    }
    else
    {
      vertexData = submesh->useSharedVertices ?
          mesh->sharedVertexData : submesh->vertexData;
    }

    if ((!submesh->useSharedVertices) ||
        (submesh->useSharedVertices && !addedShared))
    {
      if (submesh->useSharedVertices)
      {
        addedShared = true;
        sharedOffset = currentOffset;
      }

      const Ogre::VertexElement* posElem =
          vertexData->vertexDeclaration->findElementBySemantic(
          Ogre::VES_POSITION);

      Ogre::HardwareVertexBufferSharedPtr vbuf =
          vertexData->vertexBufferBinding->getBuffer(posElem->getSource());

      unsigned char* vertex =
          static_cast<unsigned char*>(
          vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

      // There is _no_ baseVertexPointerToElement() which takes an Ogre::Real
      // or a double as second argument. So make it float, to avoid trouble
      // when Ogre::Real will be compiled/typedefed as double:
      // Ogre::Real* pReal;
      float* pReal = 0;

      for( unsigned int j = 0; j < vertexData->vertexCount;
          ++j, vertex += vbuf->getVertexSize())
      {
        posElem->baseVertexPointerToElement(vertex, &pReal);
        Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);
        _vertices[currentOffset + j] = (_orient * (pt * _scale)) + _position;
      }

      vbuf->unlock();
      nextOffset += vertexData->vertexCount;
    }

    Ogre::IndexData *indexData = submesh->indexData;
    unsigned int numTris = indexData->indexCount / 3;
    Ogre::HardwareIndexBufferSharedPtr ibuf = indexData->indexBuffer;

    bool use32bitindexes =
        (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);

    unsigned long *pLong =
        static_cast<unsigned long *>(
        ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
    unsigned short *pShort =
        reinterpret_cast<unsigned short *>(pLong);

    unsigned int offset =
        (submesh->useSharedVertices)? sharedOffset : currentOffset;
    unsigned int indexStart = indexData->indexStart;
    unsigned int lastIndex = numTris*3 + indexStart;

    if (use32bitindexes)
    {
      for (unsigned int k = indexStart; k < lastIndex; ++k)
        _indices[indexOffset++] = pLong[k] + static_cast<unsigned long>(offset);
    }
    else
    {
      for (unsigned int k = indexStart; k < lastIndex; ++k)
      {
        _indices[indexOffset++] =
            static_cast<unsigned long>(pShort[k]) +
            static_cast<unsigned long>(offset);
      }
    }

    ibuf->unlock();
    currentOffset = nextOffset;
  }
}
/*
/////////////////////////////////////////////////
void RayQuery::GetMeshInformation(const Ogre::MeshPtr _mesh,
          const Ogre::Vector3 &_position, const Ogre::Quaternion &_orient,
          const Ogre::Vector3 &_scale, unsigned int &_vertexCount,
          Ogre::Vector3 *&_vertices, unsigned int &_indexCount,
          unsigned long *&_indices)
{
  // Get the _mesh information for the given _mesh.
  // Code found in Wiki: www.ogre3d.org/wiki/index.php/RetrieveVertexData

  bool addedShared = false;
  unsigned int currentOffset = 0;
  unsigned int sharedOffset = 0;
  unsigned int nextOffset = 0;
  unsigned int indexOffset = 0;

  _vertexCount = _indexCount = 0;

  // Calculate how many vertices and indices we're going to need
  for( unsigned short i=0, size=_mesh->getNumSubMeshes(); i<size; ++i )
  {
    Ogre::SubMesh* submesh = _mesh->getSubMesh(i);

    // We only need to add the shared vertices once
    if(submesh->useSharedVertices){
      if( !addedShared ){
        _vertexCount += _mesh->sharedVertexData->vertexCount;
        addedShared = true;
      }
    }else{
      _vertexCount += submesh->vertexData->vertexCount;
    }

    // Add the indices
    _indexCount += submesh->indexData->indexCount;
  }


  // Allocate space for the vertices and indices
  _vertices = new Ogre::Vector3[_vertexCount];
  _indices = new unsigned long[_indexCount];

  addedShared = false;

  // Run through the submeshes again, adding the data into the arrays
  for( unsigned short i=0, size=_mesh->getNumSubMeshes(); i<size; ++i )
  {
    Ogre::SubMesh* submesh = _mesh->getSubMesh(i);
    Ogre::VertexData* vertexData = submesh->useSharedVertices ? _mesh->sharedVertexData : submesh->vertexData;

    if( (!submesh->useSharedVertices) || (submesh->useSharedVertices && !addedShared) )
    {
      if( submesh->useSharedVertices ){
        addedShared = true;
        sharedOffset = currentOffset;
      }

      const Ogre::VertexElement* posElem = vertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
      Ogre::HardwareVertexBufferSharedPtr vbuf = vertexData->vertexBufferBinding->getBuffer(posElem->getSource());

      unsigned char* vertex = static_cast<unsigned char*>(vbuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));

      // There is _no_ baseVertexPointerToElement() which takes an Ogre::Real or a double
      //  as second argument. So make it float, to avoid trouble when Ogre::Real will
      //  be comiled/typedefed as double:
      //      Ogre::Real* pReal;
      float* pReal;

      for( unsigned int j=0; j < vertexData->vertexCount; ++j, vertex += vbuf->getVertexSize() )
      {
        posElem->baseVertexPointerToElement(vertex, &pReal);
        Ogre::Vector3 pt(pReal[0], pReal[1], pReal[2]);
        _vertices[currentOffset + j] = (_orient * (pt * _scale)) + _position;
      }

      vbuf->unlock();
      nextOffset += vertexData->vertexCount;
    }


    Ogre::IndexData* indexData = submesh->indexData;
    unsigned int numTris = indexData->indexCount / 3;
    Ogre::HardwareIndexBufferSharedPtr ibuf = indexData->indexBuffer;
    if( ibuf.isNull() ) continue; // need to check if index buffer is valid (which will be not if the _mesh doesn't have triangles like a pointcloud)

    bool use32bitindexes = (ibuf->getType() == Ogre::HardwareIndexBuffer::IT_32BIT);

    unsigned long*  pLong = static_cast<unsigned long*>(ibuf->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
    unsigned short* pShort = reinterpret_cast<unsigned short*>(pLong);


    unsigned int offset = (submesh->useSharedVertices)? sharedOffset : currentOffset;
    unsigned int indexStart = indexData->indexStart;
    unsigned int lastIndex = numTris*3 + indexStart;

    if( use32bitindexes ){
      for( unsigned int k=indexStart; k<lastIndex; ++k )
      {
        _indices[indexOffset++] = pLong[k] + static_cast<unsigned long>(offset);
      }

    }else{
      for( unsigned int k=indexStart; k<lastIndex; ++k )
      {
        _indices[indexOffset++] =
          static_cast<unsigned long>(pShort[k]) +
          static_cast<unsigned long>(offset);
      }
    }

    ibuf->unlock();
    currentOffset = nextOffset;
  }
}*/

/////////////////////////////////////////////////
void RayQuery::GetManualObjectInformation(const Ogre::ManualObject *_manual,
          const Ogre::Vector3 &_position, const Ogre::Quaternion &_orient,
          const Ogre::Vector3 &_scale, std::vector<Ogre::Vector3> &_vertices,
          std::vector<unsigned long> &_indices)
{
  std::vector<Ogre::Vector3> returnVertices;
  std::vector<unsigned long> returnIndices;
  unsigned long thisSectionStart = 0;
  for (unsigned int i = 0, size = _manual->getNumSections(); i < size; ++i)
  {
    Ogre::ManualObject::ManualObjectSection *section = _manual->getSection(i);
    Ogre::RenderOperation *renderOp = section->getRenderOperation();

    std::vector<Ogre::Vector3> pushVertices;
    // Collect the vertices
    {
      const Ogre::VertexElement *vertexElement =
          renderOp->vertexData->vertexDeclaration->findElementBySemantic(
          Ogre::VES_POSITION);
      Ogre::HardwareVertexBufferSharedPtr vertexBuffer =
          renderOp->vertexData->vertexBufferBinding->getBuffer(
          vertexElement->getSource());

      char *verticesBuffer = static_cast<char *>(
          vertexBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
      float *positionArrayHolder;

      thisSectionStart = returnVertices.size() + pushVertices.size();

      pushVertices.reserve(renderOp->vertexData->vertexCount);

      for (unsigned int j = 0; j < renderOp->vertexData->vertexCount; ++j)
      {
        vertexElement->baseVertexPointerToElement(
            verticesBuffer + j * vertexBuffer->getVertexSize(),
            &positionArrayHolder);
        Ogre::Vector3 vertexPos = Ogre::Vector3(positionArrayHolder[0],
            positionArrayHolder[1],positionArrayHolder[2]);
        vertexPos = (_orient * (vertexPos * _scale)) + _position;
        pushVertices.push_back(vertexPos);
      }

      vertexBuffer->unlock();
    }
    // Collect the indices
    {
      if (renderOp->useIndexes){
        Ogre::HardwareIndexBufferSharedPtr indexBuffer =
            renderOp->indexData->indexBuffer;

        if (indexBuffer.isNull() ||
            renderOp->operationType != Ogre::RenderOperation::OT_TRIANGLE_LIST)
        {
          // No triangles here, so we just drop the collected vertices and
          // move along to the next section.
          continue;
        } else
        {
          returnVertices.reserve(returnVertices.size() + pushVertices.size());
          returnVertices.insert(returnVertices.end(), pushVertices.begin(),
              pushVertices.end());
        }

        unsigned int *pLong = static_cast<unsigned int *>(
            indexBuffer->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
        unsigned short *pShort = reinterpret_cast<unsigned short *>(pLong);

        returnIndices.reserve(
            returnIndices.size() + renderOp->indexData->indexCount);

        for (unsigned int j = 0; j < renderOp->indexData->indexCount; ++j)
        {
          unsigned long index;
          // We also have got to remember that for a multi section object,
          // each section has different vertices, so the indices will not be
          // correct. To correct this, we have to add the position of the first
          // vertex in this section to the index
          // (At least I think so...)
          if (indexBuffer->getType() == Ogre::HardwareIndexBuffer::IT_32BIT)
            index = static_cast<unsigned long>(pLong[j]) + thisSectionStart;
          else
            index = static_cast<unsigned long>(pShort[j]) + thisSectionStart;

          returnIndices.push_back(index);
        }

        indexBuffer->unlock();
      }
    }
  }

  // Now we simply return the data.
  // _indexCount = returnIndices.size();
  // _vertexCount = returnVertices.size();

  _vertices = returnVertices;
  _indices = returnIndices;
  // _vertices = new Ogre::Vector3[_vertexCount];
  // for (unsigned long i = 0; i <_vertexCount; ++i)
  //   _vertices[i] = returnVertices[i];
  // _indices = new unsigned long[_indexCount];
  // for (unsigned long i = 0; i < _indexCount; ++i)
  //   _indices[i] = returnIndices[i];

  // All done.
  return;
}
