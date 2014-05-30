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

#ifndef _RAYQUERY_HH_
#define _RAYQUERY_HH_

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/util/system.hh"

namespace Ogre
{

}

namespace gazebo
{
  /// \ingroup gazebo_rendering
  /// \brief Rendering namespace
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class RayQuery RayQuery.hh rendering/rendering.hh
    /// \brief RayQuery adapted from Ogre3D wiki
    ///
    /// Provides the interface to load, initialize the rendering engine.
    class GAZEBO_VISIBLE RayQuery
    {
      /// \brief Raycast from a point in to the scene.
      /// \param[in] _point Point to analyse
      /// \param[in] _normal Direction
      /// \param[out] _result Intersection point on the object
      /// \return True if there is intersection
      public: static bool RaycastFromPoint(ScenePtr scene,
          const Ogre::Vector3 &_point, const Ogre::Vector3 &_normal,
          Ogre::Vector3 &_result);

      /// \brief Get geometry information of an entity.
      /// \param[in] _entity Ogre entity.
      /// \param[in] _position Position of the mesh in world coordinates.
      /// \param[in] _orient Position of the mesh in world coordinates.
      /// \param[in] _scale Scale of the mesh in world coordinates.
      /// \param[out] _vertexCount Number of vertices in the mesh.
      /// \param[out] _vertices Mesh vertices.
      /// \param[out] _indexCount Number of indices in the mesh.
      /// \param[out] _indices Mesh indices.
      private: static void GetEntityInformation(const Ogre::Entity *_entity,
          const Ogre::Vector3 &_position, const Ogre::Quaternion &_orient,
          const Ogre::Vector3 &_scale, std::vector<Ogre::Vector3> &_vertices,
          std::vector<unsigned long> &_indices);

      /// \brief Get geometry information of a manual object
      /// \param[in] _mesh Ogre manual object.
      /// \param[in] _position Position of the mesh in world coordinates.
      /// \param[in] _orient Position of the mesh in world coordinates.
      /// \param[in] _scale Scale of the mesh in world coordinates.
      /// \param[out] _vertexCount Number of vertices in the mesh.
      /// \param[out] _vertices Mesh vertices.
      /// \param[out] _indexCount Number of indices in the mesh.
      /// \param[out] _indices Mesh indices.
      private: static void GetManualObjectInformation(
          const Ogre::ManualObject *_manual,
          const Ogre::Vector3 &_position, const Ogre::Quaternion &_orient,
          const Ogre::Vector3 &_scale, std::vector<Ogre::Vector3> &_vertices,
          std::vector<unsigned long> &_indices);

      /// \brief Get geometry information of a mesh
      /// \param[in] _mesh Ogre mesh
      /// \param[in] _position Position of the mesh in world coordinates.
      /// \param[in] _orient Position of the mesh in world coordinates.
      /// \param[in] _scale Scale of the mesh in world coordinates.
      /// \param[out] _vertexCount Number of vertices in the mesh.
      /// \param[out] _vertices Mesh vertices.
      /// \param[out] _indexCount Number of indices in the mesh.
      /// \param[out] _indices Mesh indices.
      private: static void GetMeshInformation(const Ogre::MeshPtr _mesh,
          const Ogre::Vector3 &_position, const Ogre::Quaternion &_orient,
          const Ogre::Vector3 &_scale, unsigned int &_vertexCount,
          Ogre::Vector3 *&_vertices, unsigned int &_indexCount,
          unsigned long *&_indices);
    };
    /// \}
  }
}
#endif
