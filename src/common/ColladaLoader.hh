/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#ifndef COLLADALOADER_HH
#define COLLADALOADER_HH

#include "common/MeshLoader.hh"
#include "math/MathTypes.hh"

class TiXmlElement;

namespace gazebo
{
  namespace common
  {
    class Material;

    /// \addtogroup gazebo_common Common
    /// \{
    /// \brief Class used to load Collada mesh files
    class ColladaLoader : public MeshLoader
    {
      /// \brief Constructor
      public: ColladaLoader();

      /// \brief Destructor
      public: virtual ~ColladaLoader();

      /// \brief Load a mesh
      public: virtual Mesh *Load(const std::string &filename);

      private: void LoadGeometry(TiXmlElement *_xml,
                                 const math::Matrix4 &_transform, Mesh *_mesh);
      private: TiXmlElement *GetElementId(TiXmlElement *_parent,
                                          const std::string &_name,
                                          const std::string &_id);
      private: TiXmlElement *GetElementId(const std::string &_name,
                                           const std::string &_id);

      private: void LoadNode(TiXmlElement *_elem, Mesh *_mesh,
                             const math::Matrix4 &_transform);

      private: math::Matrix4 LoadNodeTransform(TiXmlElement *_elem);

      private: void LoadVertices(const std::string &_id,
                                 const math::Matrix4 &_transform,
                                 std::vector<math::Vector3> &_verts,
                                 std::vector<math::Vector3> &_norms);
      private: void LoadPositions(const std::string &_id,
                                  const math::Matrix4 &_transform,
                                  std::vector<math::Vector3> &_values);

      private: void LoadNormals(const std::string &_id,
                                const math::Matrix4 &_transform,
                                std::vector<math::Vector3> &_values);
      private: void LoadTexCoords(const std::string &_id,
                                 std::vector<math::Vector2d> &_values);

      private: Material *LoadMaterial(const std::string &_name);

      private: void LoadColorOrTexture(TiXmlElement *_elem,
                                       const std::string &_type,
                                       Material *_mat);

      private: void LoadTriangles(TiXmlElement *_trianglesXml,
                                   const math::Matrix4 &_transform,
                                   Mesh *_mesh);

      private: void LoadLines(TiXmlElement *_xml,
                               const math::Matrix4 &_transform,
                               Mesh *_mesh);

      private: void LoadScene(Mesh *_mesh);

      private: float LoadFloat(TiXmlElement *_elem);
      private: void LoadTransparent(TiXmlElement *_elem, Material *_mat);

      private: double meter;
      private: std::map<std::string, std::string> materialMap;

      private: TiXmlElement *colladaXml;
      private: std::string path;
    };
  }
}

#endif

