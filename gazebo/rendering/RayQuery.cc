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

#include "gazebo/common/MeshManager.hh"
#include "gazebo/rendering/Visual.hh"

#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Camera.hh"
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
bool RayQuery::SelectMeshTriangle(int _x, int _y, VisualPtr _visual,
    math::Vector3 &_intersect, std::vector<math::Vector3> &_vertices)
{
  // create the ray to test
  Ogre::Ray ray =
      this->dataPtr->camera->GetOgreCamera()->getCameraToViewportRay(
      static_cast<float>(_x) / this->dataPtr->camera->GetViewportWidth(),
      static_cast<float>(_y) / this->dataPtr->camera->GetViewportHeight());

  const common::Mesh *mesh =
      common::MeshManager::Instance()->GetMesh(_visual->GetMeshName());

  if (!mesh)
  {
    gzwarn << "No mesh found " << std::endl;
    return false;
  }

  std::vector<Ogre::Vector3> vertices;
  Ogre::Real closestDistance = -1.0f;
  Ogre::Vector3 closestResult;

  Ogre::Matrix4 transform = _visual->GetSceneNode()->_getFullTransform();
  // test for hitting individual triangles on the mesh
  bool newClosestFound = false;
  for (unsigned int i = 0; i < mesh->GetSubMeshCount(); ++i)
  {
    const common::SubMesh *submesh = mesh->GetSubMesh(i);
    for (unsigned int j = 0; j < submesh->GetIndexCount(); j += 3)
    {
      math::Vector3 vertexA = submesh->GetVertex(submesh->GetIndex(j));
      math::Vector3 vertexB = submesh->GetVertex(submesh->GetIndex(j+1));
      math::Vector3 vertexC = submesh->GetVertex(submesh->GetIndex(j+2));

      Ogre::Vector3 worldVertexA = transform * Conversions::Convert(vertexA);
      Ogre::Vector3 worldVertexB = transform * Conversions::Convert(vertexB);
      Ogre::Vector3 worldVertexC = transform * Conversions::Convert(vertexC);

      // check for a hit against this triangle
      std::pair<bool, Ogre::Real> hit = Ogre::Math::intersects(ray,
          worldVertexA, worldVertexB, worldVertexC, true, false);

      // if it was a hit check if its the closest
      if (hit.first &&
          (closestDistance < 0.0f || hit.second < closestDistance))
      {
        // this is the closest so far, save it off
        closestDistance = hit.second;
        vertices.clear();
        vertices.push_back(worldVertexA);
        vertices.push_back(worldVertexB);
        vertices.push_back(worldVertexC);
        newClosestFound = true;
      }
    }
  }

  // if we found a new closest raycast for this object, update the
  // closestResult before moving on to the next object.
  if (newClosestFound)
    closestResult = ray.getPoint(closestDistance);

  // return the result
  if (closestDistance >= 0.0f && vertices.size() == 3u)
  {
    // raycast success
    _intersect = Conversions::Convert(closestResult);
    _vertices.clear();
    _vertices.push_back(Conversions::Convert(vertices[0]));
    _vertices.push_back(Conversions::Convert(vertices[1]));
    _vertices.push_back(Conversions::Convert(vertices[2]));
    // std::cerr << _intersect  << std::endl;

    return true;
  }
  // raycast failed
  return false;
}
