/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#include <ignition/math/Triangle.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/common/MeshManager.hh"

#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/RayQueryPrivate.hh"
#include "gazebo/rendering/RayQuery.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/ogre_gazebo.h"

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
}

/////////////////////////////////////////////////
bool RayQuery::SelectMeshTriangle(int _x, int _y, VisualPtr _visual,
    math::Vector3 &_intersect, std::vector<math::Vector3> &_vertices)
{
  ignition::math::Vector3d intersect;
  ignition::math::Triangle3d triangle;

  auto result = this->SelectMeshTriangle(_x, _y, _visual, intersect, triangle);

  _intersect = intersect;
  _vertices.clear();
  _vertices.push_back(triangle[0]);
  _vertices.push_back(triangle[1]);
  _vertices.push_back(triangle[2]);

  return result;
}

/////////////////////////////////////////////////
bool RayQuery::SelectMeshTriangle(const int _x, const int _y,
    const VisualPtr &_visual, ignition::math::Vector3d &_intersect,
    ignition::math::Triangle3d &_triangle) const
{
  // create the ray to test
  Ogre::Ray ray =
      this->dataPtr->camera->OgreCamera()->getCameraToViewportRay(
      static_cast<float>(_x) / this->dataPtr->camera->ViewportWidth(),
      static_cast<float>(_y) / this->dataPtr->camera->ViewportHeight());

  std::vector<rendering::VisualPtr> visuals;
  this->MeshVisuals(_visual, visuals);

  Ogre::Real closestDistance = -1.0f;
  Ogre::Vector3 closestResult;
  bool newClosestFound = false;
  std::vector<Ogre::Vector3> vertices;

  for (unsigned int i = 0; i < visuals.size(); ++i)
  {
    const common::Mesh *mesh =
        common::MeshManager::Instance()->GetMesh(visuals[i]->GetMeshName());

    if (!mesh)
      continue;

    Ogre::Matrix4 transform = visuals[i]->GetSceneNode()->_getFullTransform();
    // test for hitting individual triangles on the mesh
    for (unsigned int j = 0; j < mesh->GetSubMeshCount(); ++j)
    {
      const common::SubMesh *submesh = mesh->GetSubMesh(j);
      for (unsigned int k = 0; k < submesh->GetIndexCount(); k += 3)
      {
        ignition::math::Vector3d vertexA =
          submesh->Vertex(submesh->GetIndex(k));
        ignition::math::Vector3d vertexB =
          submesh->Vertex(submesh->GetIndex(k+1));
        ignition::math::Vector3d vertexC =
          submesh->Vertex(submesh->GetIndex(k+2));

        Ogre::Vector3 worldVertexA = transform * Conversions::Convert(vertexA);
        Ogre::Vector3 worldVertexB = transform * Conversions::Convert(vertexB);
        Ogre::Vector3 worldVertexC = transform * Conversions::Convert(vertexC);

        // check for a hit against this triangle
        std::pair<bool, Ogre::Real> hit = Ogre::Math::intersects(ray,
            worldVertexA, worldVertexB, worldVertexC,
           (worldVertexB - worldVertexA).crossProduct(
           worldVertexC - worldVertexA));

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
  }

  // if we found a new closest raycast for this object, update the
  // closestResult before moving on to the next object.
  if (newClosestFound)
    closestResult = ray.getPoint(closestDistance);

  // return the result
  if (closestDistance >= 0.0f && vertices.size() == 3u)
  {
    // raycast success
    _intersect = Conversions::ConvertIgn(closestResult);
    _triangle.Set(
        Conversions::ConvertIgn(vertices[0]),
        Conversions::ConvertIgn(vertices[1]),
        Conversions::ConvertIgn(vertices[2]));
    return true;
  }
  // raycast failed
  return false;
}

/////////////////////////////////////////////////
void RayQuery::MeshVisuals(const rendering::VisualPtr _visual,
    std::vector<rendering::VisualPtr> &_visuals) const
{
  if (!_visual->GetMeshName().empty() &&
      (_visual->GetVisibilityFlags() & GZ_VISIBILITY_SELECTABLE))
    _visuals.push_back(_visual);

  for (unsigned int i = 0; i < _visual->GetChildCount(); ++i)
    this->MeshVisuals(_visual->GetChild(i), _visuals);
}
