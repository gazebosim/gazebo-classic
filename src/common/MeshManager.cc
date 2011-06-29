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
#include <sys/stat.h>
#include <string>

#include "math/Plane.hh"
#include "math/Matrix3.hh"
#include "math/Matrix4.hh"

#include "common/SystemPaths.hh"
#include "common/Exception.hh"
#include "common/Console.hh"
#include "common/Mesh.hh"
#include "common/OgreLoader.hh"
#include "common/AssimpLoader.hh"
#include "common/STLLoader.hh"

#include "common/MeshManager.hh"

using namespace gazebo;
using namespace common;


////////////////////////////////////////////////////////////////////////////////
/// Constructor
MeshManager::MeshManager()
{
  this->assimpLoader = new AssimpLoader();
  this->ogreLoader = new OgreLoader();
  this->stlLoader = new STLLoader();

  // Create some basic shapes
  this->CreatePlane("unit_plane", math::Plane(math::Vector3(0,0,1),
                                              math::Vector2d(1,1), 0), 
                                  math::Vector2d(1,1), 
                                  math::Vector2d(1,1) );

  this->CreateSphere("unit_sphere",0.5, 32, 32);
  this->CreateSphere("joint_anchor",0.01, 32, 32);
  this->CreateBox("body_cg", math::Vector3(0.014,0.014,0.014), 
                             math::Vector2d(0.014,0.014));
  this->CreateBox("unit_box", math::Vector3(1,1,1), 
                             math::Vector2d(1,1));
  this->CreateCylinder("unit_cylinder", 0.5, 1.0, 1, 32);
  this->CreateCone("unit_cone", 0.5, 1.0, 5, 32);
  this->CreateCamera("unit_camera", 0.5);
  this->CreateCylinder("axis_cylinder",0.005,0.5,1,32);

  this->CreateTube("selection_tube", 1.0, 1.2, 0.01, 1, 64);

  this->fileExtensions.push_back("mesh");
  this->fileExtensions.push_back("stl");
  this->fileExtensions.push_back("dae");
  this->fileExtensions.push_back("3ds");
  this->fileExtensions.push_back("xml");
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
MeshManager::~MeshManager()
{
  delete this->assimpLoader;
  delete this->ogreLoader;
  delete this->stlLoader;
}

////////////////////////////////////////////////////////////////////////////////
// Load a mesh from a file
const Mesh *MeshManager::Load(const std::string &filename)
{
  if (!this->IsValidFilename(filename))
  {
    gzerr << "Invalid mesh filename extension[" << filename << "]\n";
    return NULL;
  }

  struct stat st;
  std::string fullname;
  bool found = false;
  Mesh *mesh = NULL;

  std::string extension;

  if (this->HasMesh(filename))
  {
    return this->meshes[filename];
    
    // This breaks trimesh geom. Each new trimesh should have a unique name.
    /*
      // erase mesh from this->meshes. This allows a mesh to be modified and
      // inserted into gazebo again without closing gazebo.
      std::map<std::string, Mesh*>::iterator iter;
      iter = this->meshes.find(filename);
      delete iter->second;
      iter->second = NULL;
      this->meshes.erase(iter);
    */
  }


  fullname =  std::string("./")+filename;
  if (stat(fullname.c_str(), &st) == 0)
  {
    found = true;
  }
  else if ( stat(filename.c_str(), &st) == 0)
  {
    fullname =  filename;
    found = true;
  }
  else
  {
    std::list<std::string> gazeboPaths;
    gazeboPaths = SystemPaths::Instance()->GetGazeboPaths();
    for (std::list<std::string>::iterator iter=gazeboPaths.begin(); 
        iter!=gazeboPaths.end(); ++iter)
    {
      fullname = (*iter)+"/Media/models/"+filename;
      if (stat(fullname.c_str(), &st) == 0)
      {
        found = true;
        break;
      }
    }
  }

  if (found)
  {
    extension = fullname.substr(fullname.rfind(".")+1, fullname.size());
    std::transform(extension.begin(),extension.end(),extension.begin(),::tolower);
    MeshLoader *loader = NULL;

    if (extension == "mesh")
      loader = this->ogreLoader;
    else if (extension == "stl" || extension == "stlb" || extension == "stla")
      loader= this->stlLoader;
    else
      loader = this->assimpLoader;

    try 
    {
      if (!this->HasMesh(filename))
      {
        mesh = loader->Load(fullname);
        mesh->SetName(filename);
        this->meshes.insert( std::make_pair(filename, mesh) );
      }
      else
      {
        mesh = this->meshes[filename];
      }
    } 
    catch (gazebo::common::Exception e)
    {
      gzerr << "Error loading mesh[" << fullname << "]\n";
      gzerr << e << "\n";
      gzthrow(e);
    }
  }
  else
    gzerr << "Unable to find file[" << filename << "]\n";

  //mesh->RecalculateNormals();
  return mesh;
}

////////////////////////////////////////////////////////////////////////////////
/// Return true if the file extension is loadable
bool MeshManager::IsValidFilename(const std::string &filename)
{
  std::string extension;

  extension = filename.substr(filename.rfind(".")+1, filename.size());
  if (extension.empty())
    return false;
  std::transform(extension.begin(),extension.end(),extension.begin(),::tolower);

  return std::find(this->fileExtensions.begin(), this->fileExtensions.end(), 
                   extension) != this->fileExtensions.end();
}


////////////////////////////////////////////////////////////////////////////////
/// modify mesh setting its center to aabb center
void MeshManager::SetMeshCenter(const Mesh *mesh,math::Vector3 center)
{
  if (this->HasMesh(mesh->GetName()))
    this->meshes[mesh->GetName()]->SetMeshCenter(center);
}

////////////////////////////////////////////////////////////////////////////////
/// get mesh aabb
void MeshManager::GetMeshAABB(const Mesh *mesh,math::Vector3 &center, math::Vector3 &min_xyz, math::Vector3 &max_xyz)
{
  if (this->HasMesh(mesh->GetName()))
    this->meshes[mesh->GetName()]->GetAABB(center,min_xyz,max_xyz);
}

////////////////////////////////////////////////////////////////////////////////
/// generate spherical texture coordinates
void MeshManager::GenSphericalTexCoord(const Mesh *mesh,math::Vector3 center)
{
  if (this->HasMesh(mesh->GetName()))
    this->meshes[mesh->GetName()]->GenSphericalTexCoord(center);
}

////////////////////////////////////////////////////////////////////////////////
/// Add a mesh to the manager
void MeshManager::AddMesh(Mesh *mesh)
{
  if (!this->HasMesh(mesh->GetName()))
    this->meshes[mesh->GetName()] = mesh;
}

////////////////////////////////////////////////////////////////////////////////
/// Get a mesh by name
const Mesh *MeshManager::GetMesh(const std::string &name) const
{
  std::map<std::string, Mesh*>::const_iterator iter;

  iter = this->meshes.find(name);

  if (iter != this->meshes.end())
    return iter->second;

  gzerr << "Unable to find mesh with name[" << name << "]\n";
  return NULL;
}
 
////////////////////////////////////////////////////////////////////////////////
/// Return true if the mesh exists
bool MeshManager::HasMesh(const std::string &name) const
{
  if (name.empty())
    return false;

  std::map<std::string, Mesh*>::const_iterator iter;
  iter = this->meshes.find(name);

  return iter != this->meshes.end();
}

////////////////////////////////////////////////////////////////////////////////
// Create a sphere
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
  unsigned short verticeIndex = 0;

  Mesh *mesh = new Mesh();
  mesh->SetName(name);
  this->meshes.insert( std::make_pair(name, mesh));

  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

  // Generate the group of rings for the sphere
  for (ring = 0; ring <= rings; ring++)
  {
    r0 = radius * sinf (ring * deltaRingAngle);
    vert.y = radius * cosf (ring * deltaRingAngle);

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
      subMesh->AddTexCoord((float) seg / (float) segments, 
                        (float) ring / (float) rings );

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

////////////////////////////////////////////////////////////////////////////////
// Create a plane
void MeshManager::CreatePlane(const std::string &name, const math::Plane &plane,
                              const math::Vector2d &segments, 
                              const math::Vector2d &uvTile)
{
  this->CreatePlane(name, plane.normal, plane.d, plane.size, segments, uvTile);
}

////////////////////////////////////////////////////////////////////////////////
// This function was taken from OGRE:
// Copyright (c) 2000-2009 Torus Knot Software Ltd
void MeshManager::CreatePlane(const std::string &name, const math::Vector3 &normal, 
    double d, const math::Vector2d &size, const math::Vector2d &segments,
    const math::Vector2d &uvTile)
{
  if (this->HasMesh(name))
  {
    return;
  }

  Mesh *mesh = new Mesh();
  mesh->SetName(name);
  this->meshes.insert( std::make_pair(name, mesh) );

  SubMesh *subMesh = new SubMesh();
  mesh->AddSubMesh(subMesh);

  math::Vector3 zAxis, yAxis, xAxis;
  zAxis = normal;
  zAxis.Normalize();
  yAxis = zAxis.GetPerpendicular();
  xAxis = yAxis.GetCrossProd(zAxis);

  math::Matrix4 xlate, xform, rot;
  xlate = rot = math::Matrix4::IDENTITY;

  math::Matrix3 rot3;
  rot3.SetFromAxes(xAxis, yAxis, zAxis);

  rot = rot3;
 
  xlate.SetTrans( normal * -d );
  xform = xlate * rot;

  math::Vector3 vec;
  math::Vector3 norm(0,0,1);
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

  this->Tesselate2DMesh( subMesh, segments.x + 1, segments.y + 1, false  );
}

////////////////////////////////////////////////////////////////////////////////
/// Create a Box mesh
void MeshManager::CreateBox(const std::string &name, const math::Vector3 &sides,
                            const math::Vector2d &uvCoords)
{
  int i,k;

  if (this->HasMesh(name))
  {
    return;
  }

  Mesh *mesh = new Mesh();
  mesh->SetName(name);
  this->meshes.insert( std::make_pair(name, mesh) );

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
    {uvCoords.x, 0}, {0, 0}, {0,uvCoords.y}, {uvCoords.x, uvCoords.y}
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
    11,8,9,
    9,10,11,
    12, 13, 15,
    15, 13, 14,
    16, 17, 18,
    18, 19, 16,
    21,22,23,
    23,20,21,
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
    for (k=0; k<4; k++)
    {
      subMesh->AddVertex(v[faces[i][k]][0], v[faces[i][k]][1], 
          v[faces[i][k]][2]);
      subMesh->AddNormal(n[faces[i][k]][0], n[faces[i][k]][1], 
          n[faces[i][k]][2]);
      subMesh->AddTexCoord(t[k][0], t[k][1]);
    }
  }

  // Set the indices
  for (i=0;i<36; i++)
    subMesh->AddIndex(ind[i]);

  subMesh->RecalculateNormals();
}

////////////////////////////////////////////////////////////////////////////////
/// Create a Camera mesh
void MeshManager::CreateCamera(const std::string &name, float scale)
{
  int i,k;

  if (this->HasMesh(name))
  {
    return;
  }

  Mesh *mesh = new Mesh();
  mesh->SetName(name);
  this->meshes.insert( std::make_pair(name, mesh) );

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
    {uvCoords.x, 0}, {0, 0}, {0,uvCoords.y}, {uvCoords.x, uvCoords.y}
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
    11,8,9,
    9,10,11,
    12, 13, 15,
    15, 13, 14,
    16, 17, 18,
    18, 19, 16,
    21,22,23,
    23,20,21,
  };

  // Compute the vertices
  for (i = 0; i < 8; i++)
  {
    v[i][0] *= scale * 0.5;
    v[i][1] *= scale * 0.5;
    v[i][2] *= scale * 0.5;
  }

  // For each face
  for (i = 0; i < 6; i++)
  {
    // For each vertex in the face
    for (k=0; k<4; k++)
    {
      subMesh->AddVertex(v[faces[i][k]][0], v[faces[i][k]][1], 
          v[faces[i][k]][2]);
      subMesh->AddNormal(n[faces[i][k]][0], n[faces[i][k]][1], 
          n[faces[i][k]][2]);
      //subMesh->AddTexCoord(t[k][0], t[k][1]);
    }
  }

  // Set the indices
  for (i=0;i<36; i++)
    subMesh->AddIndex(ind[i]);

  mesh->RecalculateNormals();
}

////////////////////////////////////////////////////////////////////////////////
/// Create a cylinder mesh
void MeshManager::CreateCylinder(const std::string &name, float radius, 
                                 float height, int rings, int segments)
{
  math::Vector3 vert, norm;
  unsigned short verticeIndex = 0;
  unsigned int i,j;
  int ring, seg;
  float deltaSegAngle = (2.0 * M_PI / segments);

  if (this->HasMesh(name))
  {
    return;
  }

  Mesh *mesh = new Mesh();
  mesh->SetName(name);
  this->meshes.insert( std::make_pair(name, mesh) );

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
      subMesh->AddTexCoord((float) seg / (float) segments,
                           (float) ring / (float) rings );

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
  subMesh->AddVertex(0,0,height/2.0);
  subMesh->AddNormal(0,0,1);
  subMesh->AddTexCoord(0,0);

  // The bottom cap vertex
  subMesh->AddVertex(0,0,-height/2.0);
  subMesh->AddNormal(0,0,-1);
  subMesh->AddTexCoord(0,0);

  // Create the top fan
  verticeIndex += segments + 1;
  for (seg=0; seg < segments; seg++)
  {
    subMesh->AddIndex(verticeIndex);
    subMesh->AddIndex(verticeIndex - segments + seg);
    subMesh->AddIndex(verticeIndex - segments + seg - 1);
  }

  // Create the bottom fan
  verticeIndex++;
  for (seg=0; seg < segments; seg++)
  {
    subMesh->AddIndex(verticeIndex);
    subMesh->AddIndex(seg);
    subMesh->AddIndex(seg+1);
  }

  // Fix all the normals
  for (i=0; i+3 < subMesh->GetIndexCount(); i+=3)
  {
    norm.Set();

    for (j=0; j<3; j++)
      norm += subMesh->GetNormal( subMesh->GetIndex(i+j) );

    norm /= 3;
    norm.Normalize();

    for (j=0; j<3; j++)
      subMesh->SetNormal(subMesh->GetIndex(i+j), norm );
  }

  mesh->RecalculateNormals();
}

////////////////////////////////////////////////////////////////////////////////
/// Create a cone mesh
void MeshManager::CreateCone(const std::string &name, float radius, 
                             float height, int rings, int segments)
{
  math::Vector3 vert, norm;
  unsigned short verticeIndex = 0;
  unsigned int i,j;
  int ring, seg;

  if (this->HasMesh(name))
  {
    return;
  }

  Mesh *mesh = new Mesh();
  mesh->SetName(name);
  this->meshes.insert( std::make_pair(name, mesh) );

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
      subMesh->AddTexCoord((float) seg / (float) segments,
                           (float) ring / (float) rings);

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
  subMesh->AddVertex(0,0,height/2.0);
  subMesh->AddNormal(0,0,1);
  subMesh->AddTexCoord(0,0);

  // The bottom cap vertex
  subMesh->AddVertex(0,0,-height/2.0);
  subMesh->AddNormal(0,0,-1);
  subMesh->AddTexCoord(0,0);

  // Create the top fan
  verticeIndex += segments+1;
  for (seg=0; seg < segments; seg++)
  {
    subMesh->AddIndex(verticeIndex);
    subMesh->AddIndex(verticeIndex - segments + seg);
    subMesh->AddIndex(verticeIndex - segments + seg - 1);
  }

  // Create the bottom fan
  verticeIndex++;
  for (seg=0; seg < segments; seg++)
  {
    subMesh->AddIndex(verticeIndex);
    subMesh->AddIndex(seg);
    subMesh->AddIndex(seg+1);
  }

  // Fix all the normals
  for (i=0; i+3<subMesh->GetIndexCount(); i+=3)
  {
    norm.Set();

    for (j=0; j<3; j++)
      norm += subMesh->GetNormal( subMesh->GetIndex(i+j) );

    norm /= 3;
    norm.Normalize();

    for (j=0; j<3; j++)
      subMesh->SetNormal(subMesh->GetIndex(i+j), norm );
  }

  mesh->RecalculateNormals();
}

////////////////////////////////////////////////////////////////////////////////
/// Create a tube mesh
void MeshManager::CreateTube(const std::string &name, float innerRadius, 
                             float outterRadius, float height, int rings, 
                             int segments)
{
  math::Vector3 vert, norm;
  unsigned short verticeIndex = 0;
  int ring, seg;
  float deltaSegAngle = (2.0 * M_PI / segments);

  // Needs at lest 2 rings, and 3 segments
  rings = std::max(rings,1);
  segments = std::max(segments,3);

  float radius= 0;

  radius = outterRadius;

  if (this->HasMesh(name))
    return;

  Mesh *mesh = new Mesh();
  mesh->SetName(name);
  this->meshes.insert( std::make_pair(name, mesh) );
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
      subMesh->AddTexCoord((float) seg / (float) segments,
                           (float) ring / (float) rings );

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
      subMesh->AddTexCoord((float) seg / (float) segments,
                           (float) ring / (float) rings);

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

////////////////////////////////////////////////////////////////////////////////
// This function was taken from OGRE:
// Copyright (c) 2000-2009 Torus Knot Software Ltd
void MeshManager::Tesselate2DMesh(SubMesh *sm, int meshWidth, int meshHeight, 
                                  bool doubleSided)
{
  int vInc, uInc, v, u, iterations;
  int vCount, uCount;

  if (doubleSided)
  {
    iterations = 2;
    vInc = 1;
    v = 0; // Start with the front
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
    uInc = 1; // Start with moving +u

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

