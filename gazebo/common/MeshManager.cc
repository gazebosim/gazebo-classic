/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include <map>

#include "gazebo/math/Plane.hh"
#include "gazebo/math/Matrix3.hh"
#include "gazebo/math/Matrix4.hh"
#include "gazebo/math/Vector2i.hh"

#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/ColladaLoader.hh"
#include "gazebo/common/ColladaExporter.hh"
#include "gazebo/common/STLLoader.hh"
#include "gazebo/gazebo_config.h"

#ifdef HAVE_GTS
  #include "gazebo/common/MeshCSG.hh"
  #include "gazebo/common/GTSMeshUtils.hh"
#endif

#include "gazebo/common/MeshManager.hh"

using namespace gazebo;
using namespace common;

//////////////////////////////////////////////////
MeshManager::MeshManager()
{
  this->colladaLoader = new ColladaLoader();
  this->colladaExporter = new ColladaExporter();
  this->stlLoader = new STLLoader();

  // Create some basic shapes
  this->CreatePlane("unit_plane",
      ignition::math::Planed(
        ignition::math::Vector3d(0, 0, 1), ignition::math::Vector2d(1, 1), 0),
      ignition::math::Vector2d(1, 1),
      ignition::math::Vector2d(1, 1));

  this->CreateSphere("unit_sphere", 0.5, 32, 32);
  this->CreateSphere("joint_anchor", 0.01, 32, 32);
  this->CreateBox("body_cg", ignition::math::Vector3d(0.014, 0.014, 0.014),
      ignition::math::Vector2d(0.014, 0.014));
  this->CreateBox("unit_box", ignition::math::Vector3d(1, 1, 1),
      ignition::math::Vector2d(1, 1));
  this->CreateCylinder("unit_cylinder", 0.5, 1.0, 1, 32);
  this->CreateCone("unit_cone", 0.5, 1.0, 5, 32);
  this->CreateCamera("unit_camera", 0.5);

  this->CreateCylinder("axis_shaft", 0.01, 0.2, 1, 16);
  this->CreateCone("axis_head", 0.02, 0.08, 1, 16);

  this->CreateTube("selection_tube", 1.0, 1.2, 0.01, 1, 64);

  this->fileExtensions.push_back("stl");
  this->fileExtensions.push_back("dae");
}

//////////////////////////////////////////////////
MeshManager::~MeshManager()
{
  delete this->colladaLoader;
  delete this->colladaExporter;
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
void MeshManager::Export(const Mesh *_mesh, const std::string &_filename,
    const std::string &_extension, bool _exportTextures)
{
  if (_extension == "dae")
  {
    this->colladaExporter->Export(_mesh, _filename, _exportTextures);
  }
  else
  {
    gzerr << "Unsupported mesh format for file[" << _filename << "]\n";
  }
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
void MeshManager::GetMeshAABB(const Mesh *_mesh,
    ignition::math::Vector3d &_center,
    ignition::math::Vector3d &_minXYZ, ignition::math::Vector3d &_maxXYZ)
{
  if (this->HasMesh(_mesh->GetName()))
    this->meshes[_mesh->GetName()]->GetAABB(_center, _minXYZ, _maxXYZ);
}

//////////////////////////////////////////////////
void MeshManager::GenSphericalTexCoord(const Mesh *_mesh,
    const ignition::math::Vector3d &_center)
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
  float deltaSegAngle = (2.0 * M_PI / segments);
  float deltaRingAngle = (M_PI / rings);
  ignition::math::Vector3d vert, norm;
  unsigned int verticeIndex = 0;

  Mesh *mesh = new Mesh();
  mesh->SetName(name);
  this->meshes.insert(std::make_pair(name, mesh));

  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

  // Generate the group of rings for the sphere
  for (ring = 0; ring <= rings; ++ring)
  {
    float r0 = radius * sinf(ring * deltaRingAngle);
    vert.Y() = radius * cosf(ring * deltaRingAngle);

    // Generate the group of segments for the current ring
    for (seg = 0; seg <= segments; ++seg)
    {
      vert.X() = r0 * sinf(seg * deltaSegAngle);
      vert.Z() = r0 * cosf(seg * deltaSegAngle);

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
}

//////////////////////////////////////////////////
void MeshManager::CreatePlane(const std::string &_name,
    const ignition::math::Planed &_plane,
    const ignition::math::Vector2d &_segments,
    const ignition::math::Vector2d &_uvTile)
{
  this->CreatePlane(_name, _plane.Normal(), _plane.Offset(), _plane.Size(),
      _segments, _uvTile);
}

//////////////////////////////////////////////////
void MeshManager::CreatePlane(const std::string &_name,
    const ignition::math::Vector3d &_normal,
    const double _d,
    const ignition::math::Vector2d &_size,
    const ignition::math::Vector2d &_segments,
    const ignition::math::Vector2d &_uvTile)
{
  if (this->HasMesh(_name))
  {
    return;
  }

  Mesh *mesh = new Mesh();
  mesh->SetName(_name);
  this->meshes.insert(std::make_pair(_name, mesh));

  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

  ignition::math::Vector3d zAxis, yAxis, xAxis;
  zAxis = _normal;
  zAxis.Normalize();
  yAxis = zAxis.Perpendicular();
  xAxis = yAxis.Cross(zAxis);

  ignition::math::Matrix4d xlate, xform, rot;
  xlate = rot = ignition::math::Matrix4d::Identity;

  ignition::math::Matrix3d rot3;
  rot3.Axes(xAxis, yAxis, zAxis);

  rot = rot3;

  xlate.Translate(_normal * -_d);
  xform = xlate * rot;

  ignition::math::Vector3d vec;
  ignition::math::Vector3d norm(0, 0, 1);
  double xSpace = _size.X() / _segments.X();
  double ySpace = _size.Y() / _segments.Y();
  double halfWidth = _size.X() / 2.0;
  double halfHeight = _size.Y() / 2.0;
  double xTex = _uvTile.X() / _segments.X();
  double yTex = _uvTile.Y() / _segments.Y();

  // Give it some thickness to reduce shadow artifacts.
  double thickness = 0.01;

  for (int i = 0; i <= 1; ++i)
  {
    double z = i*thickness;
    for (int y = 0; y <= _segments.Y(); ++y)
    {
      for (int x = 0; x <= _segments.X(); ++x)
      {
        // Compute the position of the vertex
        vec.X() = (x * xSpace) - halfWidth;
        vec.Y() = (y * ySpace) - halfHeight;
        vec.Z() = -z;
        vec = xform.TransformAffine(vec);
        subMesh->AddVertex(vec);

        // Compute the normal
        vec = xform.TransformAffine(norm);
        subMesh->AddNormal(vec);

        // Compute the texture coordinate
        subMesh->AddTexCoord(x * xTex, 1 - (y * yTex));
      }
    }
  }

  this->Tesselate2DMesh(subMesh, _segments.X() + 1, _segments.Y() + 1, false);
}

//////////////////////////////////////////////////
void MeshManager::CreateBox(const std::string &_name,
    const ignition::math::Vector3d &_sides,
    const ignition::math::Vector2d &_uvCoords)
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
    {-1, -1, -1},
    {-1, -1, +1},
    {+1, -1, +1},
    {+1, -1, -1},
    {-1, +1, -1},
    {-1, +1, +1},
    {+1, +1, +1},
    {+1, +1, -1}
  };

  // Normals for each face
  float n[6][3]=
  {
    {+0, -1, +0},
    {+0, +1, +0},
    {+0, +0, +1},
    {-1, +0, +0},
    {+0, +0, -1},
    {+1, +0, +0},
  };

  // Texture coords
  double t[4][2] =
  {
    {_uvCoords.X(), 0}, {0, 0}, {0, _uvCoords.Y()},
    {_uvCoords.X(), _uvCoords.Y()}
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
  for (i = 0; i < 8; ++i)
  {
    v[i][0] *= _sides.X() * 0.5;
    v[i][1] *= _sides.Y() * 0.5;
    v[i][2] *= _sides.Z() * 0.5;
  }

  // For each face
  for (i = 0; i < 6; ++i)
  {
    // For each vertex in the face
    for (k = 0; k < 4; k++)
    {
      subMesh->AddVertex(v[faces[i][k]][0],
                         v[faces[i][k]][1],
                         v[faces[i][k]][2]);
      subMesh->AddNormal(n[i][0], n[i][1], n[i][2]);
      subMesh->AddTexCoord(t[k][0], t[k][1]);
    }
  }

  // Set the indices
  for (i = 0; i < 36; ++i)
    subMesh->AddIndex(ind[i]);
}

//////////////////////////////////////////////////
void MeshManager::CreateExtrudedPolyline(const std::string &_name,
    const std::vector<std::vector<ignition::math::Vector2d> > &_polys,
    double _height)
{
  // distance tolerence between 2 points. This is used when creating a list
  // of distinct points in the polylines.
  double tol = 1e-4;
  #if !HAVE_GTS
    gzerr << "GTS library not found. Can not extrude polyline" << std::endl;
    return;
  #endif
  auto polys = _polys;
  // close all the loops
  for (auto &poly : polys)
  {
    // does the poly ends with the first point?
    auto first = poly[0];
    auto last = poly[poly.size()-1];
    double d = (first.X() - last.X()) * (first.X() - last.X());
    d += (first.Y() - last.Y()) * (first.Y() - last.Y());

    // within range
    if (d >  tol * tol)
    {
      // add the first point at the end
      poly.push_back(first);
    }
  }

  if (this->HasMesh(_name))
  {
    return;
  }

  Mesh *mesh = new Mesh();
  mesh->SetName(_name);
  this->meshes.insert(std::make_pair(_name, mesh));

  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

  std::vector<ignition::math::Vector2d> vertices;
  std::vector<ignition::math::Vector2i> edges;
  MeshManager::ConvertPolylinesToVerticesAndEdges(polys,
                                                  tol,
                                                  vertices,
                                                  edges);
  #if HAVE_GTS
  if (!GTSMeshUtils::DelaunayTriangulation(vertices, edges, subMesh))
  {
    gzerr << "Unable to triangulate polyline." << std::endl;
    delete mesh;
    return;
  }
  #endif

  std::vector<ignition::math::Vector3d> normals;
  for (unsigned int i  = 0; i < edges.size(); ++i)
  {
    // we retrieve each edge's coordinates
    int i0 = edges[i][0];
    int i1 = edges[i][1];
    ignition::math::Vector2d edgeV0 = vertices[i0];
    ignition::math::Vector2d edgeV1 = vertices[i1];

    // we look for those points in the subMesh (where indices may have changed)
    for (unsigned int j = 0; j < subMesh->GetIndexCount(); j+=3)
    {
      ignition::math::Vector3d v0 = subMesh->Vertex(subMesh->GetIndex(j));
      ignition::math::Vector3d v1 = subMesh->Vertex(subMesh->GetIndex(j+1));
      ignition::math::Vector3d v2 = subMesh->Vertex(subMesh->GetIndex(j+2));

      std::vector<ignition::math::Vector3d> triangle;
      triangle.push_back(v0);
      triangle.push_back(v1);
      triangle.push_back(v2);

      int ev0 = -1;
      for (unsigned int k = 0; k < triangle.size(); ++k)
      {
        if (ignition::math::Vector2d(triangle[k].X(), triangle[k].Y()) ==
            edgeV0)
        {
          // found a vertex in triangle that matches the vertex of the edge
          ev0 = k;
          break;
        }
      }
      if (ev0 >=0)
      {
        int ev1 = -1;
        int ev2 = -1;
        for (unsigned int k = 0; k < triangle.size()-1; ++k)
        {
          int index = (ev0 + k + 1) % triangle.size();
          ignition::math::Vector3d triV = triangle[index];
          if (math::Vector2d(triV.X(), triV.Y()) == edgeV1)
          {
            // found another vertex in triangle that matches the vertex of the
            // other edge.
            ev1 = index;
            // Store the index of the third triangle vertex.
            // It's either 0, 1, or 2. Find it using simple bitwise operation.
            ev2 =  ~(ev1 | ev0) & 0x03;
            break;
          }
        }
        if (ev1 >= 0 && ev2 >= 0 && ev0 != ev1 && ev0 != ev2)
        {
          // Found an edge in triangle that matches the exterior edge.
          // Now find its normal.

          ignition::math::Vector3d edgeVec = triangle[ev0] - triangle[ev1];
          edgeVec.Normalize();
          ignition::math::Vector3d normal(edgeVec.Y(), -edgeVec.X(), 0);

          ignition::math::Vector3d otherEdgeVec = triangle[ev0] - triangle[ev2];
          otherEdgeVec.Normalize();
          double angle0 = otherEdgeVec.Dot(normal);
          double angle1 = otherEdgeVec.Dot(-normal);

          if (angle0 > angle1)
          {
            if (angle0 >= 0)
              normals.push_back(normal);
          }
          else
          {
            if (angle1 >= 0)
              normals.push_back(-normal);
          }
        }
      }
    }
  }

  // number of exterior edge normals found should be equal to the number of
  // exterior edges
  if (normals.size() != edges.size())
  {
    gzerr << "Unable to extrude mesh. Triangulation failed" << std::endl;
    return;
  }

  unsigned int numVertices = subMesh->GetVertexCount();

  // add normal for bottom face
  for (unsigned int i = 0; i < numVertices; ++i)
    subMesh->AddNormal(-ignition::math::Vector3d::UnitZ);

  // create the top face
  for (unsigned int i = 0; i < numVertices; ++i)
  {
    ignition::math::Vector3d v = subMesh->Vertex(i);
    subMesh->AddVertex(v.X(), v.Y(), _height);
    subMesh->AddNormal(ignition::math::Vector3d::UnitZ);
  }
  unsigned int numIndices = subMesh->GetIndexCount();
  for (unsigned int i = 0; i < numIndices; i+=3)
  {
    unsigned int i0 = subMesh->GetIndex(i);
    unsigned int i1 = subMesh->GetIndex(i+1);
    unsigned int i2 = subMesh->GetIndex(i+2);
    subMesh->AddIndex(numVertices+i0);
    subMesh->AddIndex(numVertices+i2);
    subMesh->AddIndex(numVertices+i1);
  }

  // create the side faces
  for (unsigned int i = 0; i < edges.size(); ++i)
  {
    // we retrieve each edge's coordinates
    int i0 = edges[i][0];
    int i1 = edges[i][1];
    ignition::math::Vector2d v0 = vertices[i0];
    ignition::math::Vector2d v1 = vertices[i1];

    ignition::math::Vector2d edge2d = v1 - v0;
    ignition::math::Vector3d edge(edge2d.X(), edge2d.Y(), 0);
    ignition::math::Vector3d cross = edge.Cross(normals[i]);

    unsigned int vCount = subMesh->GetVertexCount();

    subMesh->AddVertex(ignition::math::Vector3d(v0.X(), v0.Y(), 0));
    if (cross.Z() >0)
    {
      subMesh->AddVertex(ignition::math::Vector3d(v0.X(), v0.Y(), _height));
      subMesh->AddVertex(ignition::math::Vector3d(v1.X(), v1.Y(), _height));
    }
    else
    {
      subMesh->AddVertex(ignition::math::Vector3d(v1.X(), v1.Y(), _height));
      subMesh->AddVertex(ignition::math::Vector3d(v0.X(), v0.Y(), _height));
    }
    subMesh->AddVertex(ignition::math::Vector3d(v0.X(), v0.Y(), 0));
    if (cross.Z() >0)
    {
      subMesh->AddVertex(ignition::math::Vector3d(v1.X(), v1.Y(), _height));
      subMesh->AddVertex(ignition::math::Vector3d(v1.X(), v1.Y(), 0));
    }
    else
    {
      subMesh->AddVertex(ignition::math::Vector3d(v1.X(), v1.Y(), 0));
      subMesh->AddVertex(ignition::math::Vector3d(v1.X(), v1.Y(), _height));
    }
    for (unsigned int j = 0; j < 6; ++j)
    {
      subMesh->AddIndex(vCount++);
      subMesh->AddNormal(normals[i]);
    }
  }
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
    {uvCoords.X(), 0}, {0, 0}, {0, uvCoords.Y()}, {uvCoords.X(), uvCoords.Y()}
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
  for (i = 0; i < 8; ++i)
  {
    v[i][0] *= _scale * 0.5;
    v[i][1] *= _scale * 0.5;
    v[i][2] *= _scale * 0.5;
  }

  // For each face
  for (i = 0; i < 6; ++i)
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
  for (i = 0; i < 36; ++i)
    subMesh->AddIndex(ind[i]);

  mesh->RecalculateNormals();
}

//////////////////////////////////////////////////
void MeshManager::CreateCylinder(const std::string &name, float radius,
                                 float height, int rings, int segments)
{
  ignition::math::Vector3d vert, norm;
  unsigned int verticeIndex = 0;
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

  // Generate the group of rings for the cylinder
  for (ring = 0; ring <= rings; ++ring)
  {
    vert.Z() = ring * height/rings - height/2.0;

    // Generate the group of segments for the current ring
    for (seg = 0; seg <= segments; ++seg)
    {
      vert.Y() = radius * cosf(seg * deltaSegAngle);
      vert.X() = radius * sinf(seg * deltaSegAngle);

      // TODO: Don't think these normals are correct.
      norm = vert;
      norm.Z() = 0;
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

  // This block generates the top cap
  {
    vert.Z() = height/2.0;
    // Generate the group of segments for the top ring
    for (seg = 0; seg <= segments; ++seg)
    {
      vert.Y() = radius * cosf(seg * deltaSegAngle);
      vert.X() = radius * sinf(seg * deltaSegAngle);
      subMesh->AddVertex(vert);
      subMesh->AddNormal(0, 0, 1);
      subMesh->AddTexCoord(
            static_cast<float>(seg) / static_cast<float>(segments), 1.0);
    }

    // The top-middle cap vertex
    subMesh->AddVertex(0, 0, height/2.0);
    subMesh->AddNormal(0, 0, 1);
    subMesh->AddTexCoord(0, 0);

    // Create the top fan
    verticeIndex = subMesh->GetVertexCount()-1;
    for (seg = 0; seg < segments; seg++)
    {
      subMesh->AddIndex(verticeIndex);
      subMesh->AddIndex(verticeIndex - segments + seg);
      subMesh->AddIndex(verticeIndex - segments + seg - 1);
    }
  }

  // This block generates the bottom cap
  {
    vert.Z() = -height/2.0;
    // Generate the group of segments for the bottom ring
    for (seg = 0; seg <= segments; ++seg)
    {
      vert.Y() = radius * cosf(seg * deltaSegAngle);
      vert.X() = radius * sinf(seg * deltaSegAngle);
      subMesh->AddVertex(vert);
      subMesh->AddNormal(0, 0, -1);
      subMesh->AddTexCoord(
            static_cast<float>(seg) / static_cast<float>(segments), 0.0);
    }

    // The bottom-middle cap vertex
    subMesh->AddVertex(0, 0, -height/2.0);
    subMesh->AddNormal(0, 0, -1);
    subMesh->AddTexCoord(0, 0);

    // Create the bottom fan
    verticeIndex = subMesh->GetVertexCount()-1;
    for (seg = 0; seg < segments; seg++)
    {
      subMesh->AddIndex(verticeIndex);
      subMesh->AddIndex(verticeIndex - segments + seg - 1);
      subMesh->AddIndex(verticeIndex - segments + seg);
    }
  }
}

//////////////////////////////////////////////////
void MeshManager::CreateCone(const std::string &name, float radius,
    float height, int rings, int segments)
{
  ignition::math::Vector3d vert, norm;
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

  // Generate the group of rings for the cone
  for (ring = 0; ring < rings; ring++)
  {
    vert.Z() = ring * height/rings - height/2.0;

    double ringRadius = ((height - (vert.Z()+height/2.0)) / height) * radius;

    // Generate the group of segments for the current ring
    for (seg = 0; seg <= segments; seg++)
    {
      vert.Y() = ringRadius * cosf(seg * deltaSegAngle);
      vert.X() = ringRadius * sinf(seg * deltaSegAngle);

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

    for (j = 0; j < 3; ++j)
      norm += subMesh->Normal(subMesh->GetIndex(i+j));

    norm /= 3;
    norm.Normalize();

    for (j = 0; j < 3; ++j)
      subMesh->SetNormal(subMesh->GetIndex(i+j), norm);
  }

  mesh->RecalculateNormals();
}

//////////////////////////////////////////////////
void MeshManager::CreateTube(const std::string &_name, float _innerRadius,
    float _outerRadius, float _height, int _rings, int _segments, double _arc)
{
  ignition::math::Vector3d vert, norm;
  unsigned int verticeIndex = 0;
  int ring, seg;

  // Needs at lest 1 ring, and 3 segments
  int rings = std::max(_rings, 1);
  int segments = std::max(_segments, 3);

  float deltaSegAngle = (_arc / segments);

  float radius = 0;

  radius = _outerRadius;

  if (this->HasMesh(_name))
    return;

  Mesh *mesh = new Mesh();
  mesh->SetName(_name);
  this->meshes.insert(std::make_pair(_name, mesh));
  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

  // Generate the group of rings for the outsides of the cylinder
  for (ring = 0; ring <= rings; ++ring)
  {
    vert.Z() = ring * _height/rings - _height/2.0;

    // Generate the group of segments for the current ring
    for (seg = 0; seg <= segments; ++seg)
    {
      vert.Y() = radius * cosf(seg * deltaSegAngle);
      vert.X() = radius * sinf(seg * deltaSegAngle);

      // TODO: Don't think these normals are correct.
      norm = vert;
      norm.Normalize();

      // Add one vertex to the strip which makes up the tube
      subMesh->AddVertex(vert);
      subMesh->AddNormal(norm);
      subMesh->AddTexCoord(
          static_cast<float>(seg) / static_cast<float>(segments),
          static_cast<float>(ring) / static_cast<float>(rings));

      // outer triangles connecting ring [ring] to ring [ring + 1]
      if (ring != rings)
      {
        if (seg != 0)
        {
          subMesh->AddIndex(verticeIndex + segments + 1);
          subMesh->AddIndex(verticeIndex);
          subMesh->AddIndex(verticeIndex + segments);
        }
        if (seg != segments)
        {
          subMesh->AddIndex(verticeIndex + segments + 1);
          subMesh->AddIndex(verticeIndex + 1);
          subMesh->AddIndex(verticeIndex);
        }
      }
      // ring [rings] is the edge of the top cap
      else if (seg != segments)
      {
        // These indices form the top cap
        subMesh->AddIndex(verticeIndex);
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex+1);

        subMesh->AddIndex(verticeIndex+1);
        subMesh->AddIndex(verticeIndex + segments + 1);
        subMesh->AddIndex(verticeIndex + segments + 2);
      }

      // ring [0] is the edge of the bottom cap
      if (ring == 0 && seg < segments)
      {
        // These indices form the bottom cap
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
  radius = _innerRadius;
  for (ring = 0; ring <= rings; ++ring)
  {
    vert.Z() = (_height/2.0) - (ring * _height/rings);

    // Generate the group of segments for the current ring
    for (seg = 0; seg <= segments; ++seg)
    {
      vert.Y() = radius * cosf(seg * deltaSegAngle);
      vert.X() = radius * sinf(seg * deltaSegAngle);

      // TODO: Don't think these normals are correct.
      norm = vert;
      norm.Normalize();

      // Add one vertex to the strip which makes up the tube
      subMesh->AddVertex(vert);
      subMesh->AddNormal(norm);
      subMesh->AddTexCoord(
          static_cast<float>(seg) / static_cast<float>(segments),
          static_cast<float>(ring) / static_cast<float>(rings));

      // inner triangles connecting ring [ring] to ring [ring + 1]
      if (ring != rings)
      {
        // each vertex has six indices (2 triangles)
        if (seg != 0)
        {
          subMesh->AddIndex(verticeIndex + segments + 1);
          subMesh->AddIndex(verticeIndex);
          subMesh->AddIndex(verticeIndex + segments);
        }
        if (seg != segments)
        {
          subMesh->AddIndex(verticeIndex + segments + 1);
          subMesh->AddIndex(verticeIndex + 1);
          subMesh->AddIndex(verticeIndex);
        }
      }
      verticeIndex++;
    }
  }

  // Close ends in case it's not a full circle
  if (!ignition::math::equal(_arc, 2.0 * M_PI))
  {
    for (ring = 0; ring < rings; ++ring)
    {
      // Close beginning
      subMesh->AddIndex((segments+1)*(ring+1));
      subMesh->AddIndex((segments+1)*ring);
      subMesh->AddIndex((segments+1)*((rings+1)*2-2-ring));

      subMesh->AddIndex((segments+1)*((rings+1)*2-2-ring));
      subMesh->AddIndex((segments+1)*ring);
      subMesh->AddIndex((segments+1)*((rings+1)*2-1-ring));

      // Close end
      subMesh->AddIndex((segments+1)*((rings+1)*2-2-ring)+segments);
      subMesh->AddIndex((segments+1)*((rings+1)*2-1-ring)+segments);
      subMesh->AddIndex((segments+1)*(ring+1)+segments);

      subMesh->AddIndex((segments+1)*(ring+1)+segments);
      subMesh->AddIndex((segments+1)*((rings+1)*2-1-ring)+segments);
      subMesh->AddIndex((segments+1)*ring+segments);
    }
  }

  mesh->RecalculateNormals();
}

//////////////////////////////////////////////////
void MeshManager::Tesselate2DMesh(SubMesh *sm, int meshWidth, int meshHeight,
    bool doubleSided)
{
  int vInc, v, iterations;
  int uCount;

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
    int u = 0;
    int uInc = 1;

    int vCount = meshHeight - 1;
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
void MeshManager::CreateBoolean(const std::string &_name, const Mesh *_m1,
    const Mesh *_m2, int _operation, const ignition::math::Pose3d &_offset)
{
  if (this->HasMesh(_name))
    return;

  MeshCSG csg;
  Mesh *mesh = csg.CreateBoolean(_m1, _m2, _operation, _offset);
  mesh->SetName(_name);
  this->meshes.insert(std::make_pair(_name, mesh));
}
#endif

//////////////////////////////////////////////////
size_t MeshManager::AddUniquePointToVerticesTable(
                     std::vector<ignition::math::Vector2d> &_vertices,
                     const ignition::math::Vector2d &_p,
                     double _tol)
{
  double sqrTol = _tol * _tol;
  for (auto i = 0u; i != _vertices.size(); ++i)
  {
    auto v = _vertices[i] - _p;
    double d = (v.X() * v.X() + v.Y() * v.Y());
    if ( d < sqrTol)
    {
      return i;
    }
  }
  _vertices.push_back(_p);
  size_t r =  _vertices.size() -1;
  return r;
}

//////////////////////////////////////////////////
void MeshManager::ConvertPolylinesToVerticesAndEdges(
    const std::vector<std::vector<ignition::math::Vector2d> > &_polys,
    double _tol,
    std::vector<ignition::math::Vector2d> &_vertices,
    std::vector<ignition::math::Vector2i> &edges)
{
  for (auto poly : _polys)
  {
    ignition::math::Vector2d previous = poly[0];
    for (auto i = 1u; i != poly.size(); ++i)
    {
      auto p = poly[i];
      auto startPointIndex = AddUniquePointToVerticesTable(_vertices,
          previous, _tol);
      auto endPointIndex = AddUniquePointToVerticesTable(_vertices,
          p, _tol);
      // current end point is now the starting point for the next edge
      previous = p;
      if (startPointIndex == endPointIndex)
      {
        gzwarn << "Ignoring edge without 2 distinct vertices" << std::endl;
        continue;
      }
      // add the new edge
      ignition::math::Vector2i e(startPointIndex, endPointIndex);
      edges.push_back(e);
    }
  }
}
