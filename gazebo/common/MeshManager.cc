/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include <sys/stat.h>
#include <string>

#include "gazebo/math/Plane.hh"
#include "gazebo/math/Matrix3.hh"
#include "gazebo/math/Matrix4.hh"

#include "gazebo/common/Common.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/ColladaLoader.hh"
#include "gazebo/common/STLLoader.hh"

#include "gazebo/common/MeshManager.hh"

#ifdef HAVE_GTS
  #include "gts.h"
#endif

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
MeshManager::MeshManager()
{
  this->colladaLoader = new ColladaLoader();
  this->stlLoader = new STLLoader();

  // Create some basic shapes
  this->CreatePlane("unit_plane",
      math::Plane(math::Vector3(0, 0, 1), math::Vector2d(1, 1), 0),
      math::Vector2d(1, 1),
      math::Vector2d(1, 1));

  this->CreateSphere("unit_sphere", 0.5, 32, 32);
  this->CreateSphere("joint_anchor", 0.01, 32, 32);
  this->CreateBox("body_cg", math::Vector3(0.014, 0.014, 0.014),
      math::Vector2d(0.014, 0.014));
  this->CreateBox("unit_box", math::Vector3(1, 1, 1),
      math::Vector2d(1, 1));
  this->CreateCylinder("unit_cylinder", 0.5, 1.0, 1, 32);
  this->CreateCone("unit_cone", 0.5, 1.0, 5, 32);
  this->CreateCamera("unit_camera", 0.5);

  this->CreateCylinder("axis_shaft", 0.01, 0.2, 1, 16);
  this->CreateCone("axis_head", 0.02, 0.08, 1, 16);

  this->CreateTube("selection_tube", 1.0, 1.2, 0.01, 1, 64);

  this->CreateBox("box2", math::Vector3(0.5, 0.5, 1.5),
      math::Vector2d(1, 1));
  this->CreateBoolean("sphere_box", this->GetMesh("unit_box"),
    this->GetMesh("box2"), DIFFERENCE);

  this->fileExtensions.push_back("stl");
  this->fileExtensions.push_back("dae");
}

//////////////////////////////////////////////////
MeshManager::~MeshManager()
{
  delete this->colladaLoader;
  delete this->stlLoader;
  std::map<std::string, Mesh*>::iterator iter;
  for (iter = this->meshes.begin(); iter != this->meshes.end(); ++iter)
    delete iter->second;
  this->meshes.clear();
}

//////////////////////////////////////////////////
const Mesh *MeshManager::Load(const std::string &_filename)
{
  if (!this->IsValidFilename(_filename))
  {
    gzerr << "Invalid mesh filename extension[" << _filename << "]\n";
    return NULL;
  }

  Mesh *mesh = NULL;

  std::string extension;

  if (this->HasMesh(_filename))
  {
    return this->meshes[_filename];

    // This breaks trimesh geom. Each new trimesh should have a unique name.
    /*
    // erase mesh from this->meshes. This allows a mesh to be modified and
    // inserted into gazebo again without closing gazebo.
    std::map<std::string, Mesh*>::iterator iter;
    iter = this->meshes.find(_filename);
    delete iter->second;
    iter->second = NULL;
    this->meshes.erase(iter);
    */
  }

  std::string fullname = common::find_file(_filename);

  if (!fullname.empty())
  {
    extension = fullname.substr(fullname.rfind(".")+1, fullname.size());
    std::transform(extension.begin(), extension.end(),
        extension.begin(), ::tolower);
    MeshLoader *loader = NULL;

    if (extension == "stl" || extension == "stlb" || extension == "stla")
      loader = this->stlLoader;
    else if (extension == "dae")
      loader = this->colladaLoader;
    else
      gzerr << "Unsupported mesh format for file[" << _filename << "]\n";

    try
    {
      // This mutex prevents two threads from loading the same mesh at the
      // same time.
      boost::mutex::scoped_lock lock(this->mutex);
      if (!this->HasMesh(_filename))
      {
        if ((mesh = loader->Load(fullname)) != NULL)
        {
          mesh->SetName(_filename);
          this->meshes.insert(std::make_pair(_filename, mesh));
        }
        else
          gzerr << "Unable to load mesh[" << fullname << "]\n";
      }
      else
      {
        mesh = this->meshes[_filename];
      }
    }
    catch(gazebo::common::Exception &e)
    {
      gzerr << "Error loading mesh[" << fullname << "]\n";
      gzerr << e << "\n";
      gzthrow(e);
    }
  }
  else
    gzerr << "Unable to find file[" << _filename << "]\n";

  return mesh;
}

//////////////////////////////////////////////////
bool MeshManager::IsValidFilename(const std::string &_filename)
{
  std::string extension;

  extension = _filename.substr(_filename.rfind(".")+1, _filename.size());
  if (extension.empty())
    return false;
  std::transform(extension.begin(), extension.end(),
                 extension.begin(), ::tolower);

  return std::find(this->fileExtensions.begin(), this->fileExtensions.end(),
      extension) != this->fileExtensions.end();
}


//////////////////////////////////////////////////
void MeshManager::GetMeshAABB(const Mesh *_mesh, math::Vector3 &_center,
    math::Vector3 &_min_xyz, math::Vector3 &_max_xyz)
{
  if (this->HasMesh(_mesh->GetName()))
    this->meshes[_mesh->GetName()]->GetAABB(_center, _min_xyz, _max_xyz);
}

//////////////////////////////////////////////////
void MeshManager::GenSphericalTexCoord(const Mesh *_mesh, math::Vector3 _center)
{
  if (this->HasMesh(_mesh->GetName()))
    this->meshes[_mesh->GetName()]->GenSphericalTexCoord(_center);
}

//////////////////////////////////////////////////
void MeshManager::AddMesh(Mesh *_mesh)
{
  if (!this->HasMesh(_mesh->GetName()))
    this->meshes[_mesh->GetName()] = _mesh;
}

//////////////////////////////////////////////////
const Mesh *MeshManager::GetMesh(const std::string &_name) const
{
  std::map<std::string, Mesh*>::const_iterator iter;

  iter = this->meshes.find(_name);

  if (iter != this->meshes.end())
    return iter->second;

  gzerr << "Unable to find mesh with name[" << _name << "]\n";
  return NULL;
}

//////////////////////////////////////////////////
bool MeshManager::HasMesh(const std::string &_name) const
{
  if (_name.empty())
    return false;

  std::map<std::string, Mesh*>::const_iterator iter;
  iter = this->meshes.find(_name);

  return iter != this->meshes.end();
}

//////////////////////////////////////////////////
void MeshManager::CreateSphere(const std::string &name, float radius,
    int rings, int segments)
{
  if (this->HasMesh(name))
  {
    return;
  }

  int ring, seg;
  float r0;
  float deltaSegAngle = (2.0 * M_PI / segments);
  float deltaRingAngle = (M_PI / rings);
  math::Vector3 vert, norm;
  unsigned int verticeIndex = 0;

  Mesh *mesh = new Mesh();
  mesh->SetName(name);
  this->meshes.insert(std::make_pair(name, mesh));

  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

  // Generate the group of rings for the sphere
  for (ring = 0; ring <= rings; ring++)
  {
    r0 = radius * sinf(ring * deltaRingAngle);
    vert.y = radius * cosf(ring * deltaRingAngle);

    // Generate the group of segments for the current ring
    for (seg = 0; seg <= segments; seg++)
    {
      vert.x = r0 * sinf(seg * deltaSegAngle);
      vert.z = r0 * cosf(seg * deltaSegAngle);

      // TODO: Don't think these normals are correct.
      norm = vert;
      norm.Normalize();

      // Add one vertex to the strip which makes up the sphere
      subMesh->AddVertex(vert);
      subMesh->AddNormal(norm);
      subMesh->AddTexCoord(
          static_cast<float>(seg) / static_cast<float>(segments),
          static_cast<float>(ring) /static_cast<float>(rings));

      if (ring != rings)
      {
        // each vertex (except the last) has six indices pointing to it
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex);
        subMesh->AddIndex(verticeIndex + segments);
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex + 1);
        subMesh->AddIndex(verticeIndex);

        verticeIndex++;
      }
    }
  }

  mesh->RecalculateNormals();
}

//////////////////////////////////////////////////
void MeshManager::CreatePlane(const std::string &name, const math::Plane &plane,
                              const math::Vector2d &segments,
                              const math::Vector2d &uvTile)
{
  this->CreatePlane(name, plane.normal, plane.d, plane.size, segments, uvTile);
}

//////////////////////////////////////////////////
void MeshManager::CreatePlane(const std::string &name,
    const math::Vector3 &normal, double d, const math::Vector2d &size,
    const math::Vector2d &segments, const math::Vector2d &uvTile)
{
  if (this->HasMesh(name))
  {
    return;
  }

  Mesh *mesh = new Mesh();
  mesh->SetName(name);
  this->meshes.insert(std::make_pair(name, mesh));

  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

  math::Vector3 zAxis, yAxis, xAxis;
  zAxis = normal;
  zAxis.Normalize();
  yAxis = zAxis.GetPerpendicular();
  xAxis = yAxis.Cross(zAxis);

  math::Matrix4 xlate, xform, rot;
  xlate = rot = math::Matrix4::IDENTITY;

  math::Matrix3 rot3;
  rot3.SetFromAxes(xAxis, yAxis, zAxis);

  rot = rot3;

  xlate.SetTranslate(normal * -d);
  xform = xlate * rot;

  math::Vector3 vec;
  math::Vector3 norm(0, 0, 1);
  double xSpace = size.x / segments.x;
  double ySpace = size.y / segments.y;
  double halfWidth = size.x / 2.0;
  double halfHeight = size.y / 2.0;
  double xTex = uvTile.x / segments.x;
  double yTex = uvTile.y / segments.y;

  for (int y = 0; y <= segments.y; y++)
  {
    for (int x = 0; x <= segments.x; x++)
    {
      // Compute the position of the vertex
      vec.x = (x * xSpace) - halfWidth;
      vec.y = (y * ySpace) - halfHeight;
      vec.z = 0.0;
      vec = xform.TransformAffine(vec);
      subMesh->AddVertex(vec);

      // Compute the normal
      vec = xform.TransformAffine(norm);
      subMesh->AddNormal(vec);

      // Compute the texture coordinate
      subMesh->AddTexCoord(x * xTex, 1 - (y * yTex));
    }
  }

  this->Tesselate2DMesh(subMesh, segments.x + 1, segments.y + 1, false);
}

//////////////////////////////////////////////////
void MeshManager::CreateBox(const std::string &name, const math::Vector3 &sides,
    const math::Vector2d &uvCoords)
{
  int i, k;

  if (this->HasMesh(name))
  {
    return;
  }

  Mesh *mesh = new Mesh();
  mesh->SetName(name);
  this->meshes.insert(std::make_pair(name, mesh));

  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

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
    {uvCoords.x, 0}, {0, 0}, {0, uvCoords.y}, {uvCoords.x, uvCoords.y}
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
    11, 8, 9,
    9, 10, 11,
    12, 13, 15,
    15, 13, 14,
    16, 17, 18,
    18, 19, 16,
    21, 22, 23,
    23, 20, 21,
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
    for (k = 0; k < 4; k++)
    {
      subMesh->AddVertex(v[faces[i][k]][0], v[faces[i][k]][1],
          v[faces[i][k]][2]);
      subMesh->AddNormal(n[faces[i][k]][0], n[faces[i][k]][1],
          n[faces[i][k]][2]);
      subMesh->AddTexCoord(t[k][0], t[k][1]);
    }
  }

  // Set the indices
  for (i = 0; i < 36; i++)
    subMesh->AddIndex(ind[i]);

  subMesh->RecalculateNormals();
}

//////////////////////////////////////////////////
void MeshManager::CreateCamera(const std::string &_name, float _scale)
{
  int i, k;

  if (this->HasMesh(_name))
  {
    return;
  }

  Mesh *mesh = new Mesh();
  mesh->SetName(_name);
  this->meshes.insert(std::make_pair(_name, mesh));

  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

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
  /*float t[4][2] =
    {
    {uvCoords.x, 0}, {0, 0}, {0, uvCoords.y}, {uvCoords.x, uvCoords.y}
    };*/

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
    11, 8, 9,
    9, 10, 11,
    12, 13, 15,
    15, 13, 14,
    16, 17, 18,
    18, 19, 16,
    21, 22, 23,
    23, 20, 21,
  };

  // Compute the vertices
  for (i = 0; i < 8; i++)
  {
    v[i][0] *= _scale * 0.5;
    v[i][1] *= _scale * 0.5;
    v[i][2] *= _scale * 0.5;
  }

  // For each face
  for (i = 0; i < 6; i++)
  {
    // For each vertex in the face
    for (k = 0; k < 4; k++)
    {
      subMesh->AddVertex(v[faces[i][k]][0], v[faces[i][k]][1],
          v[faces[i][k]][2]);
      subMesh->AddNormal(n[faces[i][k]][0], n[faces[i][k]][1],
          n[faces[i][k]][2]);
      // subMesh->AddTexCoord(t[k][0], t[k][1]);
    }
  }

  // Set the indices
  for (i = 0; i < 36; i++)
    subMesh->AddIndex(ind[i]);

  mesh->RecalculateNormals();
}

//////////////////////////////////////////////////
void MeshManager::CreateCylinder(const std::string &name, float radius,
                                 float height, int rings, int segments)
{
  math::Vector3 vert, norm;
  unsigned int verticeIndex = 0;
  unsigned int i, j;
  int ring, seg;
  float deltaSegAngle = (2.0 * M_PI / segments);

  if (this->HasMesh(name))
  {
    return;
  }

  Mesh *mesh = new Mesh();
  mesh->SetName(name);
  this->meshes.insert(std::make_pair(name, mesh));

  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);


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
      subMesh->AddVertex(vert);
      subMesh->AddNormal(norm);
      subMesh->AddTexCoord(
          static_cast<float>(seg) / static_cast<float>(segments),
          static_cast<float>(ring) / static_cast<float>(rings));

      if (ring != rings)
      {
        // each vertex (except the last) has six indices pointing to it
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex);
        subMesh->AddIndex(verticeIndex + segments);
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex + 1);
        subMesh->AddIndex(verticeIndex);
        verticeIndex++;
      }
    }
  }

  /// The top cap vertex
  subMesh->AddVertex(0, 0, height/2.0);
  subMesh->AddNormal(0, 0, 1);
  subMesh->AddTexCoord(0, 0);

  // The bottom cap vertex
  subMesh->AddVertex(0, 0, -height/2.0);
  subMesh->AddNormal(0, 0, -1);
  subMesh->AddTexCoord(0, 0);

  // Create the top fan
  verticeIndex += segments + 1;
  for (seg = 0; seg < segments; seg++)
  {
    subMesh->AddIndex(verticeIndex);
    subMesh->AddIndex(verticeIndex - segments + seg);
    subMesh->AddIndex(verticeIndex - segments + seg - 1);
  }

  // Create the bottom fan
  verticeIndex++;
  for (seg = 0; seg < segments; seg++)
  {
    subMesh->AddIndex(verticeIndex);
    subMesh->AddIndex(seg);
    subMesh->AddIndex(seg+1);
  }

  // Fix all the normals
  for (i = 0; i+3 < subMesh->GetIndexCount(); i+= 3)
  {
    norm.Set();

    for (j = 0; j < 3; j++)
      norm += subMesh->GetNormal(subMesh->GetIndex(i+j));

    norm /= 3;
    norm.Normalize();

    for (j = 0; j < 3; j++)
      subMesh->SetNormal(subMesh->GetIndex(i+j), norm);
  }

  mesh->RecalculateNormals();
}

//////////////////////////////////////////////////
void MeshManager::CreateCone(const std::string &name, float radius,
    float height, int rings, int segments)
{
  math::Vector3 vert, norm;
  unsigned int verticeIndex = 0;
  unsigned int i, j;
  int ring, seg;

  if (this->HasMesh(name))
  {
    return;
  }

  Mesh *mesh = new Mesh();
  mesh->SetName(name);
  this->meshes.insert(std::make_pair(name, mesh));

  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

  if (segments <3)
    segments = 3;

  float deltaSegAngle = (2.0 * M_PI / segments);

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
      subMesh->AddVertex(vert);
      subMesh->AddNormal(norm);
      subMesh->AddTexCoord(
          static_cast<float>(seg) / static_cast<float>(segments),
          static_cast<float>(ring) / static_cast<float>(rings));

      if (ring != (rings-1))
      {
        // each vertex (except the last) has six indices pointing to it
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex);
        subMesh->AddIndex(verticeIndex + segments);
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex + 1);
        subMesh->AddIndex(verticeIndex);
        verticeIndex++;
      }
    }
  }

  /// The top point vertex
  subMesh->AddVertex(0, 0, height/2.0);
  subMesh->AddNormal(0, 0, 1);
  subMesh->AddTexCoord(0, 0);

  // The bottom cap vertex
  subMesh->AddVertex(0, 0, -height/2.0);
  subMesh->AddNormal(0, 0, -1);
  subMesh->AddTexCoord(0, 0);

  // Create the top fan
  verticeIndex += segments+1;
  for (seg = 0; seg < segments; seg++)
  {
    subMesh->AddIndex(verticeIndex);
    subMesh->AddIndex(verticeIndex - segments + seg);
    subMesh->AddIndex(verticeIndex - segments + seg - 1);
  }

  // Create the bottom fan
  verticeIndex++;
  for (seg = 0; seg < segments; seg++)
  {
    subMesh->AddIndex(verticeIndex);
    subMesh->AddIndex(seg);
    subMesh->AddIndex(seg+1);
  }

  // Fix all the normals
  for (i = 0; i + 3 < subMesh->GetIndexCount(); i += 3)
  {
    norm.Set();

    for (j = 0; j < 3; j++)
      norm += subMesh->GetNormal(subMesh->GetIndex(i+j));

    norm /= 3;
    norm.Normalize();

    for (j = 0; j < 3; j++)
      subMesh->SetNormal(subMesh->GetIndex(i+j), norm);
  }

  mesh->RecalculateNormals();
}

//////////////////////////////////////////////////
void MeshManager::CreateTube(const std::string &name, float innerRadius,
    float outterRadius, float height, int rings,
    int segments)
{
  math::Vector3 vert, norm;
  unsigned int verticeIndex = 0;
  int ring, seg;
  float deltaSegAngle = (2.0 * M_PI / segments);

  // Needs at lest 2 rings, and 3 segments
  rings = std::max(rings, 1);
  segments = std::max(segments, 3);

  float radius = 0;

  radius = outterRadius;

  if (this->HasMesh(name))
    return;

  Mesh *mesh = new Mesh();
  mesh->SetName(name);
  this->meshes.insert(std::make_pair(name, mesh));
  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

  // Generate the group of rings for the outsides of the cylinder
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

      // Add one vertex to the strip which makes up the tube
      subMesh->AddVertex(vert);
      subMesh->AddNormal(norm);
      subMesh->AddTexCoord(
          static_cast<float>(seg) / static_cast<float>(segments),
          static_cast<float>(ring) / static_cast<float>(rings));

      if (ring != rings)
      {
        // each vertex (except the last) has six indices
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex);
        subMesh->AddIndex(verticeIndex + segments);
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex + 1);
        subMesh->AddIndex(verticeIndex);
      }
      else
      {
        // This indices form the top cap
        subMesh->AddIndex(verticeIndex);
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex+1);
        subMesh->AddIndex(verticeIndex+1);
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex + segments + 2);
      }

      // There indices form the bottom cap
      if (ring == 0 && seg < segments)
      {
        subMesh->AddIndex(verticeIndex+1);
        subMesh->AddIndex(verticeIndex + (segments+1) * (((rings+1)*2)-1));
        subMesh->AddIndex(verticeIndex);
        subMesh->AddIndex(verticeIndex + (segments+1) * (((rings+1)*2)-1) + 1);
        subMesh->AddIndex(verticeIndex + (segments+1) * (((rings+1)*2)-1));
        subMesh->AddIndex(verticeIndex+1);
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
      subMesh->AddVertex(vert);
      subMesh->AddNormal(norm);
      subMesh->AddTexCoord(
          static_cast<float>(seg) / static_cast<float>(segments),
          static_cast<float>(ring) / static_cast<float>(rings));

      if (ring != rings)
      {
        // each vertex (except the last) has six indices
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex);
        subMesh->AddIndex(verticeIndex + segments);

        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex + 1);
        subMesh->AddIndex(verticeIndex);
      }
      verticeIndex++;
    }
  }

  mesh->RecalculateNormals();
}

//////////////////////////////////////////////////
void MeshManager::Tesselate2DMesh(SubMesh *sm, int meshWidth, int meshHeight,
    bool doubleSided)
{
  int vInc, uInc, v, u, iterations;
  int vCount, uCount;

  if (doubleSided)
  {
    iterations = 2;
    vInc = 1;
    v = 0;
  }
  else
  {
    iterations = 1;
    vInc = 1;
    v = 0;
  }

  int v1, v2, v3;

  while (iterations--)
  {
    // Make tris in a zigzag pattern (compatible with strips)
    u = 0;
    uInc = 1;

    vCount = meshHeight - 1;
    while (vCount--)
    {
      uCount = meshWidth - 1;
      while (uCount--)
      {
        // First tri in cell
        v1 = ((v + vInc) * meshWidth) + u;
        v2 = (v * meshWidth) + u;
        v3 = ((v + vInc) * meshWidth) + (u + uInc);
        // Output indexes
        sm->AddIndex(v1);
        sm->AddIndex(v2);
        sm->AddIndex(v3);
        // Second Tri in cell
        v1 = ((v + vInc) * meshWidth) + (u + uInc);
        v2 = (v * meshWidth) + u;
        v3 = (v * meshWidth) + (u + uInc);
        // Output indexes
        sm->AddIndex(v1);
        sm->AddIndex(v2);
        sm->AddIndex(v3);

        // Next column
        u += uInc;
      }

      // Next row
      v += vInc;
      u = 0;
    }

    // Reverse vInc for double sided
    v = meshHeight - 1;
    vInc = -vInc;
  }
}

#ifdef HAVE_GTS
//////////////////////////////////////////////////
static void MergeVertices(GPtrArray * _vertices, double _epsilon)
{
  GPtrArray *array;
  GNode *kdtree;
  GtsVertex **verticesData = reinterpret_cast<GtsVertex **>(_vertices->pdata);
  array = g_ptr_array_new();
  for (unsigned int i = 0; i < _vertices->len; ++i)
    g_ptr_array_add (array, verticesData[i]);
  kdtree = gts_kdtree_new(array, NULL);
  g_ptr_array_free (array, true);

  for (unsigned int i = 0; i < _vertices->len; i++)
  {
    GtsVertex *v = reinterpret_cast<GtsVertex *>(verticesData[i]);

    if (!GTS_OBJECT (v)->reserved)
    {
      // Do something only if v is active
      GtsBBox *bbox;
      GSList *selected, *j;

      // build bounding box
      bbox = gts_bbox_new (gts_bbox_class(),
			   v,
			   GTS_POINT(v)->x - _epsilon,
			   GTS_POINT(v)->y - _epsilon,
			   GTS_POINT(v)->z - _epsilon,
			   GTS_POINT(v)->x + _epsilon,
			   GTS_POINT(v)->y + _epsilon,
			   GTS_POINT(v)->z + _epsilon);

      /* select vertices which are inside bbox using kdtree */
      j = selected = gts_kdtree_range (kdtree, bbox, NULL);
      while (j)
      {
	      GtsVertex *sv = reinterpret_cast<GtsVertex *>(j->data);
        // mark sv as inactive
	      if (sv != v && !GTS_OBJECT(sv)->reserved)
	        GTS_OBJECT(sv)->reserved = v;
	      j = j->next;
      }
      g_slist_free(selected);
      gts_object_destroy(GTS_OBJECT(bbox));
    }
  }

  gts_kdtree_destroy(kdtree);

  // destroy inactive vertices

  // we want to control vertex destruction
  gts_allow_floating_vertices = true;

  for (unsigned int i = 0; i < _vertices->len; ++i)
  {
    GtsVertex * v = reinterpret_cast<GtsVertex *>(verticesData[i]);
    // v is inactive
    if (GTS_OBJECT(v)->reserved)
    {
      verticesData[i] =
          reinterpret_cast<GtsVertex *>(GTS_OBJECT(v)->reserved);
      gts_object_destroy(GTS_OBJECT(v));
    }
  }
  gts_allow_floating_vertices = false;
}

//////////////////////////////////////////////////
static void FillVertex(GtsPoint *_p, gpointer *_data)
{
  /*(*GTS_OBJECT (p)->klass->write) (GTS_OBJECT (p), (FILE *) data[0]);
  if (!GTS_POINT_CLASS (GTS_OBJECT (p)->klass)->binary)
    fputc ('\n', (FILE *) data[0]);*/

  SubMesh *subMesh = reinterpret_cast<SubMesh *>(_data[0]);
  GHashTable* vIndex = reinterpret_cast<GHashTable *>(_data[2]);
  subMesh->AddVertex(GTS_POINT(_p)->x, GTS_POINT(_p)->y, GTS_POINT (_p)->z);
  g_hash_table_insert(vIndex, _p, GUINT_TO_POINTER (++(*((guint *)_data[1]))));
}

//////////////////////////////////////////////////
static void FillEdge(GtsSegment *_s, gpointer *_data)
{
/*  fprintf ((FILE *) data[0], "%u %u",
	   GPOINTER_TO_UINT (g_hash_table_lookup (data[2], s->v1)),
	   GPOINTER_TO_UINT (g_hash_table_lookup (data[2], s->v2)));
  if (GTS_OBJECT (s)->klass->write)
    (*GTS_OBJECT (s)->klass->write) (GTS_OBJECT (s), (FILE *) data[0]);
  fputc ('\n', (FILE *) data[0]);*/
  GHashTable* eIndex = reinterpret_cast<GHashTable *>(_data[3]);
  g_hash_table_insert (eIndex, _s, GUINT_TO_POINTER (++(*((guint *)_data[1]))));
}

//////////////////////////////////////////////////
static void FillFace(GtsTriangle *_t, gpointer *_data)
{
/*  fprintf (data[0], "%u %u %u",
	   GPOINTER_TO_UINT (g_hash_table_lookup (data[3], t->e1)),
	   GPOINTER_TO_UINT (g_hash_table_lookup (data[3], t->e2)),
	   GPOINTER_TO_UINT (g_hash_table_lookup (data[3], t->e3)));
  if (GTS_OBJECT (t)->klass->write)
    (*GTS_OBJECT (t)->klass->write) (GTS_OBJECT (t), data[0]);
  fputc ('\n', data[0]);*/
  SubMesh *subMesh = reinterpret_cast<SubMesh *>(_data[0]);
  GHashTable* eIndex = reinterpret_cast<GHashTable *>(_data[3]);
  subMesh->AddIndex(GPOINTER_TO_UINT(g_hash_table_lookup (eIndex, _t->e1)));
  subMesh->AddIndex(GPOINTER_TO_UINT(g_hash_table_lookup (eIndex, _t->e2)));
  subMesh->AddIndex(GPOINTER_TO_UINT(g_hash_table_lookup (eIndex, _t->e3)));
}

//////////////////////////////////////////////////
void MeshManager::CreateBoolean(const std::string &_name, const Mesh *_m1,
    const Mesh *_m2, int _operation)
{
  if (this->HasMesh(_name))
    return;

  GtsSurface *s1, *s2, *s3;
  GtsSurfaceInter *si;
  GNode *tree1, *tree2;

  bool closed = true;
  bool is_open1 = false;
  bool is_open2 = false;

  s1 = gts_surface_new(gts_surface_class(), gts_face_class(), gts_edge_class(),
      gts_vertex_class());
  s2 = gts_surface_new(gts_surface_class(), gts_face_class(), gts_edge_class(),
      gts_vertex_class());
  s3 = gts_surface_new(gts_surface_class(), gts_face_class(), gts_edge_class(),
      gts_vertex_class());

  this->ConvertMeshToGTS(_m1, s1);
  this->ConvertMeshToGTS(_m2, s2);

  // build bounding box tree for first surface
  tree1 = gts_bb_tree_surface (s1);
  is_open1 = gts_surface_volume (s1) < 0. ? true : false;

  // build bounding box tree for second surface
  tree2 = gts_bb_tree_surface (s2);
  is_open2 = gts_surface_volume (s2) < 0. ? true : false;

  si = gts_surface_inter_new (gts_surface_inter_class (), s1, s2, tree1, tree2,
      is_open1, is_open2);
  assert(gts_surface_inter_check (si, &closed));
  if (!closed)
  {
    gzerr << "the intersection of " << _m1->GetName() << " and "
        << _m2->GetName() << " is not a closed curve\n";
    return;
  }

  /*FILE *output1 = fopen("output3.gts", "w");
  gts_surface_write(s1, output1);
  fclose(output1);

  FILE *output2 = fopen("output4.gts", "w");
  gts_surface_write(s2, output2);
  fclose(output2);*/

  if (_operation == MeshManager::UNION)
  {
    gts_surface_inter_boolean (si, s3, GTS_1_OUT_2);
    gts_surface_inter_boolean (si, s3, GTS_2_OUT_1);
  }
  else if (_operation == MeshManager::INTERSECTION)
  {
    gts_surface_inter_boolean (si, s3, GTS_1_IN_2);
    gts_surface_inter_boolean (si, s3, GTS_2_IN_1);
  }
  else if (_operation == MeshManager::DIFFERENCE)
  {
    gts_surface_inter_boolean (si, s3, GTS_1_OUT_2);
    gts_surface_inter_boolean (si, s3, GTS_2_IN_1);
    gts_surface_foreach_face (si->s2, (GtsFunc) gts_triangle_revert, NULL);
    gts_surface_foreach_face (s2, (GtsFunc) gts_triangle_revert, NULL);
  }

  /*FILE *output = fopen("output.gts", "w");
  gts_surface_write(s3, output);
  fclose(output);*/

  // create the boolean mesh in gazebo
  Mesh *mesh = new Mesh();
  mesh->SetName(_name);
  this->meshes.insert(std::make_pair(_name, mesh));

  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

  // fill the submesh with data generated by GTS
  unsigned int n;
  gpointer data[4];
  GHashTable *vIndex = g_hash_table_new(NULL, NULL);
  GHashTable *eIndex = g_hash_table_new(NULL, NULL);

  data[0] = subMesh;
  data[1] = &n;
  data[2] = vIndex;
  data[3] = eIndex;

  n = 0;
  gts_surface_foreach_vertex(s3, (GtsFunc)FillVertex, data);
  n = 0;
  gts_surface_foreach_edge(s3, (GtsFunc)FillEdge, data);
  gts_surface_foreach_face(s3, (GtsFunc)FillFace, data);
  g_hash_table_destroy(vIndex);
  g_hash_table_destroy(eIndex);

  mesh->RecalculateNormals();

  // destroy surfaces
  gts_object_destroy(GTS_OBJECT(s1));
  gts_object_destroy(GTS_OBJECT(s2));
  gts_object_destroy(GTS_OBJECT(s3));
  gts_object_destroy(GTS_OBJECT(si));

  // destroy bounding box trees (including bounding boxes)
  gts_bb_tree_destroy(tree1, true);
  gts_bb_tree_destroy(tree2, true);

}

//////////////////////////////////////////////////
void MeshManager::ConvertMeshToGTS(const Mesh *_mesh, GtsSurface *_surface)
{
  if (!_surface)
  {
    gzerr << "Surface is NULL ";
//    _surface = gts_surface_new(gts_surface_class(), gts_face_class(),
//        gts_edge_class(), gts_vertex_class());
    return;
  }

  GPtrArray *vertices = g_ptr_array_new();

  for (unsigned int i = 0; i < _mesh->GetSubMeshCount(); ++i)
  {
    const SubMesh *subMesh = _mesh->GetSubMesh(i);
    unsigned int indexCount = subMesh->GetIndexCount();
    if (subMesh->GetVertexCount() <= 2)
      continue;

    for (unsigned int j = 0; j < subMesh->GetVertexCount(); ++j)
    {
      math::Vector3 vertex = subMesh->GetVertex(j);
      g_ptr_array_add (vertices, gts_vertex_new(gts_vertex_class(), vertex.x,
          vertex.y, vertex.z));
    }

    // merge duplicate or close-by vertices
    // duplicate vertices will produce undesirable results
    MergeVertices(vertices, 0.01);

    GtsVertex **verticesData =
        reinterpret_cast<GtsVertex **>(vertices->pdata);
    for (unsigned int j = 0; j < indexCount/3; ++j)
    {
      GtsEdge *e1 = GTS_EDGE(gts_vertices_are_connected(
          verticesData[subMesh->GetIndex(3*j)],
          verticesData[subMesh->GetIndex(3*j+1)]));
      GtsEdge *e2 = GTS_EDGE(gts_vertices_are_connected(
          verticesData[subMesh->GetIndex(3*j+1)],
          verticesData[subMesh->GetIndex(3*j+2)]));
      GtsEdge *e3 = GTS_EDGE(gts_vertices_are_connected(
          verticesData[subMesh->GetIndex(3*j+2)],
          verticesData[subMesh->GetIndex(3*j)]));
      if (e1 == NULL && verticesData[subMesh->GetIndex(3*j)]
          != verticesData[subMesh->GetIndex(3*j+1)])
      {
        e1 = gts_edge_new(_surface->edge_class,
            verticesData[subMesh->GetIndex(3*j)],
            verticesData[subMesh->GetIndex(3*j+1)]);
      }
      if (e2 == NULL && verticesData[subMesh->GetIndex(3*j+1)]
          != verticesData[subMesh->GetIndex(3*j+2)])
      {
        e2 = gts_edge_new(_surface->edge_class,
            verticesData[subMesh->GetIndex(3*j+1)],
            verticesData[subMesh->GetIndex(3*j+2)]);
      }
      if (e3 == NULL && verticesData[subMesh->GetIndex(3*j+2)]
          != verticesData[subMesh->GetIndex(3*j)])
      {
        e3 = gts_edge_new(_surface->edge_class,
            verticesData[subMesh->GetIndex(3*j+2)],
            verticesData[subMesh->GetIndex(3*j)]);
      }
      if (e1 != NULL && e2 != NULL && e3 != NULL)
      {
        gts_surface_add_face(_surface, gts_face_new(_surface->face_class, e1,
            e2, e3));
      }
      else
      {
        gzwarn << _mesh->GetName() << ": Ignoring degenerate facet!";
      }
    }
  }
  //gts_object_destroy (GTS_OBJECT(vertices));
}
#endif
