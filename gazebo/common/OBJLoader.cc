/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <memory>

#include "gazebo/common/Console.hh"
#include "gazebo/common/Material.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/OBJLoader.hh"

#define GAZEBO_TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

namespace gazebo
{
  namespace common
  {
    /// \internal
    /// \brief OBJLoader private data
    class OBJLoaderPrivate
    {
    };
  }
}

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
OBJLoader::OBJLoader()
: MeshLoader(), dataPtr(new OBJLoaderPrivate)
{
}

//////////////////////////////////////////////////
OBJLoader::~OBJLoader()
{
}

//////////////////////////////////////////////////
Mesh *OBJLoader::Load(const std::string &_filename)
{
  std::map<std::string, Material *> materialIds;

  std::string path;
  size_t idx = _filename.rfind('/');
  if (idx != std::string::npos)
    path = _filename.substr(0, idx+1);

  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;

  // convert polygons to triangles
  bool triangulate = true;

  std::string err;
  bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err,
      _filename.c_str(), path.c_str(), triangulate);

  if (!err.empty())
  {
    gzerr << err << std::endl;
  }

  if (!ret)
  {
    gzerr << "Failed to load/parse " << _filename << std::endl;
    return nullptr;
  }

  Mesh *mesh = new Mesh();
  mesh->SetPath(path);

  for (auto const s: shapes)
  {
    // obj mesh assigns a material id to each 'face' but gazebo assigns a single
    // material to each 'submesh'. The strategy here is to identify
    // the number of unique material ids in each obj shape and create a new
    // submesh per unique material id
    std::map<int, SubMesh *> subMeshMatId;
    for (auto const id :  s.mesh.material_ids)
    {
      if (subMeshMatId.find(id) == subMeshMatId.end())
      {
        std::unique_ptr<SubMesh> subMesh(new SubMesh());
        subMesh->SetName(s.name);
        subMesh->SetPrimitiveType(SubMesh::TRIANGLES);
        subMeshMatId[id] = subMesh.get();

        Material *mat = nullptr;
        auto m = materials[id];
        if (materialIds.find(m.name) != materialIds.end())
        {
          mat = materialIds[m.name];
        }
        else
        {
          // Create new material and pass it to mesh who will take ownership of
          // the object
          mat = new Material();
          mat->SetAmbient(Color(m.ambient[0], m.ambient[1], m.ambient[2]));
          mat->SetDiffuse(Color(m.diffuse[0], m.diffuse[1], m.diffuse[2]));
          mat->SetSpecular(Color(m.specular[0], m.specular[1], m.specular[2]));
          mat->SetEmissive(Color(m.emission[0], m.emission[1], m.emission[2]));
          mat->SetShininess(m.shininess);
          mat->SetTransparency(1.0 - m.dissolve);
          mat->SetTextureImage(m.diffuse_texname, path.c_str());
          materialIds[m.name] = mat;
        }
        int matIndex = mesh->GetMaterialIndex(mat);
        if (matIndex < 0)
          matIndex = mesh->AddMaterial(mat);
        subMesh->SetMaterialIndex(matIndex);
        mesh->AddSubMesh(subMesh.release());
      }
    }

    unsigned int indexOffset = 0;
    // For each face
    for (unsigned int f = 0; f < s.mesh.num_face_vertices.size(); ++f)
    {
      // find the submesh that corresponds to the current face material
      unsigned int matId = s.mesh.material_ids[f];
      SubMesh *subMesh = subMeshMatId[matId];

      unsigned int fnum = s.mesh.num_face_vertices[f];
      // For each vertex in the face
      for (unsigned int v = 0; v < fnum; ++v)
      {
        auto i = s.mesh.indices[indexOffset + v];

        // vertices
        int vIdx = i.vertex_index;
        ignition::math::Vector3d vertex(attrib.vertices[3 * vIdx],
                                        attrib.vertices[3 * vIdx + 1],
                                        attrib.vertices[3 * vIdx + 2]);
        subMesh->AddVertex(vertex);

        // normals
        if (attrib.normals.size() > 0)
        {
          int nIdx = i.normal_index;
          ignition::math::Vector3d normal(attrib.normals[3 * nIdx],
                                          attrib.normals[3 * nIdx + 1],
                                          attrib.normals[3 * nIdx + 2]);
          subMesh->AddNormal(normal);
        }
        // texcoords
        if (attrib.texcoords.size() > 0)
        {
          int tIdx = i.texcoord_index;
          ignition::math::Vector2d uv(attrib.texcoords[2 * tIdx],
                                      attrib.texcoords[2 * tIdx + 1]);
          subMesh->AddTexCoord(uv.X(), 1.0-uv.Y());
        }
        subMesh->AddIndex(subMesh->GetIndexCount());
      }
      indexOffset += fnum;
    }
  }

  return mesh;
}
