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

#include <boost/filesystem/path.hpp>

#include "gazebo/common/Console.hh"
#include "gazebo/common/Material.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/OBJLoader.hh"

#define GAZEBO_TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

using namespace gazebo;
using namespace common;

namespace gazebo
{
  namespace common
  {
    /// \internal
    /// \brief OBJLoader private data
    class OBJLoaderPrivate
    {
      // empty for now
    };
  }
}

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
  boost::filesystem::path p(_filename);
  std::string path = p.parent_path().generic_string() + "/";

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
    gzerr << "Failed to load/parse .obj." << std::endl;
    return nullptr;
  }

  Mesh *mesh = new Mesh();
  mesh->SetPath(path);

  for (unsigned int i = 0; i < shapes.size(); ++i)
  {
    auto s = shapes[i];
    SubMesh *subMesh = new SubMesh();
    subMesh->SetName(s.name);
    subMesh->SetPrimitiveType(SubMesh::TRIANGLES);

    // load faces
    int index = 0;
    for (unsigned int j = 0; j < s.mesh.indices.size() / 3; ++j)
    {
      auto idx0 = s.mesh.indices[3*j];
      auto idx1 = s.mesh.indices[3*j+1];
      auto idx2 = s.mesh.indices[3*j+2];

      // vertices
      int vi0 = idx0.vertex_index;
      int vi1 = idx1.vertex_index;
      int vi2 = idx2.vertex_index;
      if (vi0 < 0 || vi1 < 0 || vi2 < 0)
      {
        gzerr << "Error loading submesh vertex index" <<std::endl;
        continue;
      }

      ignition::math::Vector3d v0(attrib.vertices[3 * vi0],
                                  attrib.vertices[3 * vi0 + 1],
                                  attrib.vertices[3 * vi0 + 2]);
      ignition::math::Vector3d v1(attrib.vertices[3 * vi1],
                                  attrib.vertices[3 * vi1 + 1],
                                  attrib.vertices[3 * vi1 + 2]);
      ignition::math::Vector3d v2(attrib.vertices[3 * vi2],
                                  attrib.vertices[3 * vi2 + 1],
                                  attrib.vertices[3 * vi2 + 2]);
      subMesh->AddVertex(v0);
      subMesh->AddVertex(v1);
      subMesh->AddVertex(v2);

      // normals
      if (attrib.normals.size() > 0)
      {
        int ni0 = idx0.vertex_index;
        int ni1 = idx1.vertex_index;
        int ni2 = idx2.vertex_index;
        if (ni0 < 0 || ni1 < 0 || ni2 < 0)
        {
          gzerr << "Error loading submesh normal index" <<std::endl;
        }
        else
        {
          ignition::math::Vector3d n0(attrib.normals[3 * ni0],
                                      attrib.normals[3 * ni0 + 1],
                                      attrib.normals[3 * ni0 + 2]);
          ignition::math::Vector3d n1(attrib.normals[3 * ni1],
                                      attrib.normals[3 * ni1 + 1],
                                      attrib.normals[3 * ni1 + 2]);
          ignition::math::Vector3d n2(attrib.normals[3 * ni2],
                                      attrib.normals[3 * ni2 + 1],
                                      attrib.normals[3 * ni2 + 2]);
          subMesh->AddNormal(n0);
          subMesh->AddNormal(n1);
          subMesh->AddNormal(n2);
        }
      }

      // texcoords
      if (attrib.texcoords.size() > 0)
      {
        int ti0 = idx0.texcoord_index;
        int ti1 = idx1.texcoord_index;
        int ti2 = idx2.texcoord_index;

        if (ti0 < 0 || ti1 < 0 || ti2 < 0)
        {
          gzerr << "Error loading submesh texcoord index" <<std::endl;
        }
        else
        {
          ignition::math::Vector2d t0(attrib.texcoords[2 * ti0],
                                      attrib.texcoords[2 * ti0 + 1]);
          ignition::math::Vector2d t1(attrib.texcoords[2 * ti1],
                                      attrib.texcoords[2 * ti1 + 1]);
          ignition::math::Vector2d t2(attrib.texcoords[2 * ti2],
                                      attrib.texcoords[2 * ti2 + 1]);

          // the v texcoords seem to be inverted
          subMesh->AddTexCoord(t0.X(), 1.0-t0.Y());
          subMesh->AddTexCoord(t1.X(), 1.0-t1.Y());
          subMesh->AddTexCoord(t2.X(), 1.0-t2.Y());
        }
      }

      // indices
      subMesh->AddIndex(index++);
      subMesh->AddIndex(index++);
      subMesh->AddIndex(index++);
    }

    // materials
    if (!materials.empty() && !s.mesh.material_ids.empty())
    {
      auto m = materials[s.mesh.material_ids[0]];
      Material *mat = new Material();
      mat->SetAmbient(Color(m.ambient[0], m.ambient[1], m.ambient[2]));
      mat->SetDiffuse(Color(m.diffuse[0], m.diffuse[1], m.diffuse[2]));
      mat->SetSpecular(Color(m.specular[0], m.specular[1], m.specular[2]));
      mat->SetEmissive(Color(m.emission[0], m.emission[1], m.emission[2]));
      mat->SetShininess(m.shininess);
      mat->SetTransparency(1.0 - m.dissolve);
      mat->SetTextureImage(m.diffuse_texname, path.c_str());

      int matIndex = mesh->GetMaterialIndex(mat);
      if (matIndex < 0)
        matIndex = mesh->AddMaterial(mat);
      subMesh->SetMaterialIndex(matIndex);
    }
    mesh->AddSubMesh(subMesh);
  }

  return mesh;
}
