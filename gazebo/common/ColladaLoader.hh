/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_COLLADALOADER_HH_
#define _GAZEBO_COLLADALOADER_HH_

#include <string>
#include <vector>
#include <map>

#include <ignition/math/Matrix4.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Vector2.hh>

#include "gazebo/common/MeshLoader.hh"
#include "gazebo/util/system.hh"

class TiXmlElement;

namespace gazebo
{
  namespace common
  {
    class Material;
    class ColladaLoaderPrivate;

    /// \addtogroup gazebo_common Common
    /// \{

    /// \class ColladaLoader ColladaLoader.hh common/common.hh
    /// \brief Class used to load Collada mesh files
    class GZ_COMMON_VISIBLE ColladaLoader : public MeshLoader
    {
      /// \brief Constructor
      public: ColladaLoader();

      /// \brief Destructor
      public: virtual ~ColladaLoader();

      /// \brief Load a mesh
      /// \param[in] _filename Collada file to load
      /// \return Pointer to a new Mesh
      public: virtual Mesh *Load(const std::string &_filename);

      /// \brief Load a controller instance
      /// \param[in] _contrXml Pointer to the control XML instance
      /// \param[in] _skelXml Pointer the skeleton xml instance
      /// \param[in] _transform A tranform to apply
      /// \param[in,out] _mesh The mesh being loaded
      private: void LoadController(TiXmlElement *_contrXml,
          TiXmlElement *_skelXml,
          const ignition::math::Matrix4d &_transform, Mesh *_mesh);

      /// \brief Load animations for a skeleton
      /// \param[in] _xml Animation XML instance
      /// \param[in,out] _skel Pointer to the skeleton
      private: void LoadAnimations(TiXmlElement *_xml, Skeleton *_skel);

      /// \brief Load a set of animations for a skeleton
      /// \param[in] _xml Pointer to the animation set XML instance
      /// \param[in,out] _skel Pointer to the skeleton
      private: void LoadAnimationSet(TiXmlElement *_xml, Skeleton *_skel);

      /// \brief Load skeleton nodes
      /// \param[in] _xml Pointer to the XML instance
      /// \param[in,out] _xml Pointer to the Skeleton node parent
      private: SkeletonNode* LoadSkeletonNodes(TiXmlElement *_xml,
                                               SkeletonNode *_parent);

      /// \brief Set the tranform for a skeleton node
      /// \param[in] _xml Pointer to the XML instance
      /// \param[in,out] _node The skeleton node
      private: void SetSkeletonNodeTransform(TiXmlElement *_elem,
                                             SkeletonNode *_node);

      /// \brief Load geometry elements
      /// \param[in] _xml Pointer to the XML instance
      /// \param[in] _tranform Transform to apply to the loaded geometry
      /// \param[in,out] _mesh Pointer to the mesh currently being loaded
      private: void LoadGeometry(TiXmlElement *_xml,
                   const ignition::math::Matrix4d &_transform, Mesh *_mesh);

      /// \brief Get an XML element by ID
      /// \param[in] _parent The parent element
      /// \param[in] _name String name of the element
      /// \param[out] _id String ID of the element
      private: TiXmlElement *GetElementId(TiXmlElement *_parent,
                                          const std::string &_name,
                                          const std::string &_id);

      /// \brief Get an XML element by ID
      /// \param[in] _name String name of the element
      /// \param[out] _id String ID of the element
      private: TiXmlElement *GetElementId(const std::string &_name,
                                           const std::string &_id);

      /// \brief Load a node
      /// \param[in] _elem Pointer to the node XML instance
      /// \param[in,out] _mesh Pointer to the current mesh
      /// \param[out] _transform Transform to apply to the node
      private: void LoadNode(TiXmlElement *_elem, Mesh *_mesh,
                   const ignition::math::Matrix4d &_transform);

      /// \brief Load a transform
      /// \param[in] _elem Pointer to the transform XML instance
      /// \return A Matrix4 transform
      private: ignition::math::Matrix4d LoadNodeTransform(TiXmlElement *_elem);

      /// \brief Load vertices
      /// \param[in] _id String id of the vertices XML node
      /// \param[in] _transform Transform to apply to all vertices
      /// \param[out] _verts Holds the resulting vertices
      /// \param[out] _norms Holds the resulting normals
      private: void LoadVertices(const std::string &_id,
         const ignition::math::Matrix4d &_transform,
         std::vector<ignition::math::Vector3d> &_verts,
         std::vector<ignition::math::Vector3d> &_norms);

      /// \brief Load vertices
      /// \param[in] _id String id of the vertices XML node
      /// \param[in] _transform Transform to apply to all vertices
      /// \param[out] _verts Holds the resulting vertices
      /// \param[out] _norms Holds the resulting normals
      /// \param[out] _vertDup Holds a map of duplicate position indices
      /// \param[out] _normDup Holds a map of duplicate normal indices
      private: void LoadVertices(const std::string &_id,
         const ignition::math::Matrix4d &_transform,
         std::vector<ignition::math::Vector3d> &_verts,
         std::vector<ignition::math::Vector3d> &_norms,
         std::map<unsigned int, unsigned int> &_vertDup,
         std::map<unsigned int, unsigned int> &_normDup);

      /// \brief Load positions
      /// \param[in] _id String id of the XML node
      /// \param[in] _transform Transform to apply to all positions
      /// \param[out] _values Holds the resulting position values
      /// \param[out] _values Holds a map of duplicate position indices
      private: void LoadPositions(const std::string &_id,
          const ignition::math::Matrix4d &_transform,
          std::vector<ignition::math::Vector3d> &_values,
          std::map<unsigned int, unsigned int> &_duplicates);

      /// \brief Load normals
      /// \param[in] _id String id of the XML node
      /// \param[in] _transform Transform to apply to all normals
      /// \param[out] _values Holds the resulting normal values
      /// \param[out] _values Holds a map of duplicate normal indices
      private: void LoadNormals(const std::string &_id,
          const ignition::math::Matrix4d &_transform,
          std::vector<ignition::math::Vector3d> &_values,
          std::map<unsigned int, unsigned int> &_duplicates);

      /// \brief Load texture coordinates
      /// \param[in] _id String id of the XML node
      /// \param[out] _values Holds the resulting uv values
      /// \param[out] _values Holds a map of duplicate uv indices
      private: void LoadTexCoords(const std::string &_id,
          std::vector<ignition::math::Vector2d> &_values,
          std::map<unsigned int, unsigned int> &_duplicates);

      /// \brief Load a material
      /// \param _name Name of the material XML element
      /// \return A pointer to the new material
      private: Material *LoadMaterial(const std::string &_name);

      /// \brief Load a color or texture
      /// \param[in] _eleme Pointer to the XML element
      /// \param[in] _type One of {diffuse, ambient, emission}
      /// \param[out] _mat Material to load the texture or color into
      private: void LoadColorOrTexture(TiXmlElement *_elem,
                                       const std::string &_type,
                                       Material *_mat);

      /// \brief Load triangles
      /// \param[in] _trianglesXml Pointer the triangles XML instance
      /// \param[in] _transform Transform to apply to all triangles
      /// \param[out] _mesh Mesh that is currently being loaded
      private: void LoadTriangles(TiXmlElement *_trianglesXml,
                                   const ignition::math::Matrix4d &_transform,
                                   Mesh *_mesh);

      /// \brief Load a polygon list
      /// \param[in] _polylistXml Pointer to the XML element
      /// \param[in] _transform Transform to apply to each polygon
      /// \param[out] _mesh Mesh that is currently being loaded
      private: void LoadPolylist(TiXmlElement *_polylistXml,
                                   const ignition::math::Matrix4d &_transform,
                                   Mesh *_mesh);

      /// \brief Load lines
      /// \param[in] _xml Pointer to the XML element
      /// \param[in] _transform Transform to apply
      /// \param[out] _mesh Mesh that is currently being loaded
      private: void LoadLines(TiXmlElement *_xml,
                               const ignition::math::Matrix4d &_transform,
                               Mesh *_mesh);

      /// \brief Load an entire scene
      /// \param[out] _mesh Mesh that is currently being loaded
      private: void LoadScene(Mesh *_mesh);

      /// \brief Load a float value
      /// \param[out] _elem Pointer to the XML element
      /// \return The float value
      private: float LoadFloat(TiXmlElement *_elem);

      /// \brief Load a transparent material. NOT IMPLEMENTED
      /// \param[in] _elem Pointer to the XML element
      /// \param[out] _mat Material to hold the transparent properties
      private: void LoadTransparent(TiXmlElement *_elem, Material *_mat);

      /// \internal
      /// \brief Pointer to private data.
      private: ColladaLoaderPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
