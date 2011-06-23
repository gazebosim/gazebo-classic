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
#include <algorithm>
#include <float.h>
#include <string.h>

#include "common/Exception.hh"
#include "common/Console.hh"
#include "common/Mesh.hh"

using namespace gazebo;
using namespace common;


////////////////////////////////////////////////////////////////////////////////
/// Constructor
Mesh::Mesh()
{
  this->name = "unknown";
}
  
////////////////////////////////////////////////////////////////////////////////
/// Destructor
Mesh::~Mesh()
{
  std::vector<SubMesh*>::iterator iter;
  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); iter++)
    delete *iter;
  this->submeshes.clear();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the name of this mesh
void Mesh::SetName(const std::string &n)
{
  this->name = n;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the name of this mesh
std::string Mesh::GetName() const
{
  return this->name;
}

////////////////////////////////////////////////////////////////////////////////
// Get the maximun X,Y,Z values
math::Vector3 Mesh::GetMax() const
{
  math::Vector3 max;
  std::vector<SubMesh*>::const_iterator iter;

  max.x = -FLT_MAX;
  max.y = -FLT_MAX;
  max.z = -FLT_MAX;

  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); iter++)
  {
    if ((*iter)->GetVertexCount() <= 2)
      continue;

    math::Vector3 smax = (*iter)->GetMax();
    max.x = std::max(max.x, smax.x);
    max.y = std::max(max.y, smax.y);
    max.z = std::max(max.z, smax.z);
  }

  return max;
}

////////////////////////////////////////////////////////////////////////////////
// Get the minimum X,Y,Z values
math::Vector3 Mesh::GetMin() const
{
  math::Vector3 min;
  std::vector<SubMesh *>::const_iterator iter;

  min.x = FLT_MAX;
  min.y = FLT_MAX;
  min.z = FLT_MAX;

  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); iter++)
  {
    if ((*iter)->GetVertexCount() <= 2)
      continue;

    math::Vector3 smin = (*iter)->GetMin();
    min.x = std::min(min.x, smin.x);
    min.y = std::min(min.y, smin.y);
    min.z = std::min(min.z, smin.z);
  }

  return min;
}

////////////////////////////////////////////////////////////////////////////////
// Return the number of vertices
unsigned int Mesh::GetVertexCount() const
{
  unsigned int sum = 0;
  std::vector<SubMesh *>::const_iterator iter;

  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); iter++)
  {
    if ((*iter)->GetVertexCount() <= 2)
      continue;
    sum += (*iter)->GetVertexCount();
  }

  return sum;
}

////////////////////////////////////////////////////////////////////////////////
/// Return the number of normals
unsigned int Mesh::GetNormalCount() const
{
  unsigned int sum = 0;
  std::vector<SubMesh *>::const_iterator iter;

  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); iter++)
  {
    if ((*iter)->GetVertexCount() <= 2)
      continue;
    sum += (*iter)->GetNormalCount();
  }

  return sum;
}

////////////////////////////////////////////////////////////////////////////////
// Return the number of indicies
unsigned int Mesh::GetIndexCount() const
{
  unsigned int sum = 0;
  std::vector<SubMesh *>::const_iterator iter;

  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); iter++)
  {
    if ((*iter)->GetVertexCount() <= 2)
      continue;
    sum += (*iter)->GetIndexCount();
  }

  return sum;
}

////////////////////////////////////////////////////////////////////////////////
/// Return the number of texture coordinates
unsigned int Mesh::GetTexCoordCount() const
{
  unsigned int sum = 0;
  std::vector<SubMesh *>::const_iterator iter;

  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); iter++)
  {
    if ((*iter)->GetVertexCount() <= 2)
      continue;
    sum += (*iter)->GetTexCoordCount();
  }

  return sum;
}


////////////////////////////////////////////////////////////////////////////////
/// Add a child mesh
void Mesh::AddSubMesh(SubMesh *sub)
{
  this->submeshes.push_back(sub);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the number of children
unsigned int Mesh::GetSubMeshCount() const
{
  return this->submeshes.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Get a child
const SubMesh *Mesh::GetSubMesh(unsigned int i) const
{
  if (i < this->submeshes.size())
    return this->submeshes[i];
  else
    gzthrow("Invalid index: " << i << " >= " << this->submeshes.size() << "\n");
}

////////////////////////////////////////////////////////////////////////////////
/// Add a material to the mesh
void Mesh::AddMaterial( Material *mat )
{
  this->materials.push_back(mat);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the number of materials
unsigned int Mesh::GetMaterialCount() const
{
  return this->materials.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Get a material
const Material *Mesh::GetMaterial(unsigned int index) const
{
  if (index < this->materials.size())
    return this->materials[index];

  return NULL;
}

////////////////////////////////////////////////////////////////////////////////
///  Put all the data into flat arrays
void Mesh::FillArrays(float **vertArr, unsigned int **indArr) const
{
  std::vector<SubMesh *>::const_iterator iter;

  unsigned int vertCount = 0;
  unsigned int indCount = 0;

  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); iter++)
  {
    if ((*iter)->GetVertexCount() <= 2)
      continue;
    vertCount += (*iter)->GetVertexCount();
    indCount += (*iter)->GetIndexCount();
  }

  if (*vertArr)
    delete [] *vertArr;

  if (*indArr)
    delete [] *indArr;

  *vertArr = new float[ vertCount * 3 ];
  *indArr = new unsigned int[ indCount ];

  float *vPtr = *vertArr;
  unsigned int index = 0;
  unsigned int offset = 0;

  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); iter++)
  {
    if ((*iter)->GetVertexCount() <= 2)
      continue;

    float *vertTmp = NULL;
    unsigned int *indTmp = NULL;
    (*iter)->FillArrays(&vertTmp, &indTmp);

    memcpy(vPtr, vertTmp, sizeof(float)*(*iter)->GetVertexCount()*3);

    for (unsigned int i=0; i < (*iter)->GetIndexCount(); i++)
    {
      (*indArr)[index++] = (*iter)->GetIndex(i) + offset;
    }

    offset = offset + (*iter)->GetMaxIndex() + 1;

    vPtr += (*iter)->GetVertexCount()*3;

    delete [] vertTmp;
    delete [] indTmp;
  }

}

////////////////////////////////////////////////////////////////////////////////
/// Recalculate all the normals.
void Mesh::RecalculateNormals()
{
  std::vector<SubMesh*>::iterator iter;
  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); iter++)
    (*iter)->RecalculateNormals();
}



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// SUBMESH
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Constructor
SubMesh::SubMesh()
{
  this->materialIndex = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
SubMesh::~SubMesh()
{
  this->vertices.clear();
  this->indices.clear();
}

////////////////////////////////////////////////////////////////////////////////
/// Add an index to the mesh
void SubMesh::AddIndex( unsigned int i)
{
  this->indices.push_back( i );
}

////////////////////////////////////////////////////////////////////////////////
/// Add a vertex to the mesh
void SubMesh::AddVertex( math::Vector3 v )
{
  this->vertices.push_back( v );
}

////////////////////////////////////////////////////////////////////////////////
/// Add a vertex to the mesh
void SubMesh::AddVertex(double x, double y, double z )
{
  this->AddVertex( math::Vector3(x,y,z) );
}

////////////////////////////////////////////////////////////////////////////////
/// Add a normal to the mesh
void SubMesh::AddNormal( math::Vector3 n )
{
  this->normals.push_back( n );
}

////////////////////////////////////////////////////////////////////////////////
/// Add a normal to the mesh
void SubMesh::AddNormal(double x, double y, double z )
{
  this->AddNormal( math::Vector3(x,y,z) );
}

////////////////////////////////////////////////////////////////////////////////
/// Add a texture coord to the mesh
void SubMesh::AddTexCoord(double u, double v )
{
  this->texCoords.push_back( math::Vector2d(u,v) );
}

////////////////////////////////////////////////////////////////////////////////
/// Get a vertex
math::Vector3 SubMesh::GetVertex(unsigned int i) const
{
  if (i >= this->vertices.size())
    gzthrow("Index too large");

  return this->vertices[i];
}

////////////////////////////////////////////////////////////////////////////////
/// Set a vertex
void SubMesh::SetVertex(unsigned int i, const math::Vector3 &v)
{
  if (i >= this->vertices.size())
    gzthrow("Index too large");

  this->vertices[i] = v;
}

////////////////////////////////////////////////////////////////////////////////
/// Get a normal
math::Vector3 SubMesh::GetNormal(unsigned int i) const
{
  if (i >= this->normals.size())
    gzthrow("Index too large");

  return this->normals[i];
}

////////////////////////////////////////////////////////////////////////////////
/// Set a normal
void SubMesh::SetNormal(unsigned int i, const math::Vector3 &n)
{
  if (i >= this->normals.size())
    gzthrow("Index too large");

  this->normals[i] = n;
}

////////////////////////////////////////////////////////////////////////////////
/// Get a tex coord
math::Vector2d SubMesh::GetTexCoord(unsigned int i) const
{
  if (i >= this->texCoords.size())
    gzthrow("Index too large");

  return this->texCoords[i];
}

////////////////////////////////////////////////////////////////////////////////
/// Set a tex coord
void SubMesh::SetTexCoord(unsigned int i, const math::Vector2d &t)
{
  if (i >= this->texCoords.size())
    gzthrow("Index too large");

  this->texCoords[i] = t;
}

////////////////////////////////////////////////////////////////////////////////
/// Get an index
unsigned int SubMesh::GetIndex(unsigned int i) const
{
  if (i > this->indices.size())
    gzthrow("Index too large");

  return this->indices[i];
}

////////////////////////////////////////////////////////////////////////////////
// Get the maximun X,Y,Z values
math::Vector3 SubMesh::GetMax() const
{
  math::Vector3 max;
  std::vector<math::Vector3>::const_iterator iter;

  max.x = -FLT_MAX;
  max.y = -FLT_MAX;
  max.z = -FLT_MAX;

  for (iter = this->vertices.begin(); iter != this->vertices.end(); iter++)
  {
    max.x = std::max(max.x, (*iter).x);
    max.y = std::max(max.y, (*iter).y);
    max.z = std::max(max.z, (*iter).z);
  }

  return max;
}

////////////////////////////////////////////////////////////////////////////////
// Get the minimum X,Y,Z values
math::Vector3 SubMesh::GetMin() const
{
  math::Vector3 min;
  std::vector<math::Vector3>::const_iterator iter;

  min.x = FLT_MAX;
  min.y = FLT_MAX;
  min.z = FLT_MAX;

  for (iter = this->vertices.begin(); iter != this->vertices.end(); iter++)
  {
    min.x = std::min(min.x, (*iter).x);
    min.y = std::min(min.y, (*iter).y);
    min.z = std::min(min.z, (*iter).z);
  }

  return min;
}

////////////////////////////////////////////////////////////////////////////////
// Return the number of vertices
unsigned int SubMesh::GetVertexCount() const
{
  return this->vertices.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Return the number of normals
unsigned int SubMesh::GetNormalCount() const
{
  return this->normals.size();
}

////////////////////////////////////////////////////////////////////////////////
// Return the number of indicies
unsigned int SubMesh::GetIndexCount() const
{
  return this->indices.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Return the number of texture coordinates
unsigned int SubMesh::GetTexCoordCount() const
{
  return this->texCoords.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the highest index value
unsigned int SubMesh::GetMaxIndex() const
{
  std::vector<unsigned int>::const_iterator maxIter;
  maxIter = std::max_element(this->indices.begin(), this->indices.end());

  if (maxIter != this->indices.end())
    return *maxIter;

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the material index. Relates to the parent mesh material list
void SubMesh::SetMaterialIndex(unsigned int index)
{
  this->materialIndex = index;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the material index
unsigned int SubMesh::GetMaterialIndex() const
{
  return this->materialIndex;
}

////////////////////////////////////////////////////////////////////////////////
/// Return true if this submesh has the vertex
bool SubMesh::HasVertex( const math::Vector3 &v ) const
{
  std::vector< math::Vector3 >::const_iterator iter;

  for (iter = this->vertices.begin(); iter != this->vertices.end(); iter++)
    if (v == (*iter))
      return true;

  return false;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the index of the vertex
unsigned int SubMesh::GetVertexIndex(const math::Vector3 &v) const
{
  std::vector< math::Vector3 >::const_iterator iter;

  for (iter = this->vertices.begin(); iter != this->vertices.end(); iter++)
    if (v == (*iter))
      return iter - this->vertices.begin();

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Put all the data into flat arrays
void SubMesh::FillArrays(float **vertArr, unsigned int **indArr) const
{
  if (this->vertices.size() ==0 || this->indices.size() == 0)
    gzerr << "No vertices or indices\n";

  std::vector< math::Vector3 >::const_iterator  viter;
  std::vector< unsigned int >::const_iterator  iiter;
  unsigned int i;

  if (*vertArr)
    delete [] *vertArr;

  if (*indArr)
    delete [] *indArr;

  *vertArr = new float[ this->vertices.size() * 3 ];
  *indArr = new unsigned int[ this->indices.size() ];

  for (viter = this->vertices.begin(), i = 0; viter != this->vertices.end(); 
       viter++)
  {
    (*vertArr)[i++] =  (float)(*viter).x;
    (*vertArr)[i++] =  (float)(*viter).y;
    (*vertArr)[i++] =  (float)(*viter).z;
  }

  for (iiter = this->indices.begin(), i=0; 
       iiter != this->indices.end(); iiter++)
    (*indArr)[i++] = (*iiter);
}

////////////////////////////////////////////////////////////////////////////////
/// Recalculate all the normals.
void SubMesh::RecalculateNormals()
{
  unsigned int i;

  // Reset all the normals
  for (i=0; i < this->normals.size(); i++)
    this->normals[i].Set(0,0,0);

  // For each face, which is defined by three indices, calculate the normals
  for (i=0; i < this->indices.size(); i+=3)
  {
    math::Vector3 v1 = this->vertices[this->indices[i]];
    math::Vector3 v2 = this->vertices[this->indices[i+1]];
    math::Vector3 v3 = this->vertices[this->indices[i+2]];
    math::Vector3 n = math::Vector3::GetNormal(v1, v2, v3);
    this->normals[this->indices[i]] += n;
    this->normals[this->indices[i+1]] += n;
    this->normals[this->indices[i+2]] += n;
  }

  // Normalize the results
  for (i=0; i < this->normals.size(); i++)
  {
    this->normals[i].Normalize();
  }
}

////////////////////////////////////////////////////////////////////////////////
// find aabb and aabb center of mesh
void Mesh::GetAABB(math::Vector3 &center,math::Vector3 &min_xyz,math::Vector3 &max_xyz)
{
  // find aabb center
  min_xyz.x =  1e15;
  max_xyz.x = -1e15;
  min_xyz.y =  1e15;
  max_xyz.y = -1e15;
  min_xyz.z =  1e15;
  max_xyz.z = -1e15;
  center.x = 0;
  center.y = 0;
  center.z = 0;

  std::vector<SubMesh*>::iterator siter;
  for (siter = this->submeshes.begin(); siter != this->submeshes.end(); siter++)
  {
    math::Vector3 max = (*siter)->GetMax();
    math::Vector3 min = (*siter)->GetMin();
    min_xyz.x = std::min(min_xyz.x,min.x);
    max_xyz.x = std::max(max_xyz.x,max.x);
    min_xyz.y = std::min(min_xyz.y,min.y);
    max_xyz.y = std::max(max_xyz.y,max.y);
    min_xyz.z = std::min(min_xyz.z,min.z);
    max_xyz.z = std::max(max_xyz.z,max.z);
  }
  center.x = 0.5*(min_xyz.x+max_xyz.x);
  center.y = 0.5*(min_xyz.y+max_xyz.y);
  center.z = 0.5*(min_xyz.z+max_xyz.z);
}

////////////////////////////////////////////////////////////////////////////////
// Set mesh center
void Mesh::SetMeshCenter(math::Vector3 center)
{
  std::vector<SubMesh*>::iterator siter;
  for (siter = this->submeshes.begin(); siter != this->submeshes.end(); siter++)
    (*siter)->SetSubMeshCenter(center);
}

////////////////////////////////////////////////////////////////////////////////
// Assign uv coordinates using spherical projection
void Mesh::GenSphericalTexCoord(math::Vector3 center)
{
  std::vector<SubMesh*>::iterator siter;
  for (siter = this->submeshes.begin(); siter != this->submeshes.end(); siter++)
    (*siter)->GenSphericalTexCoord(center);
}

////////////////////////////////////////////////////////////////////////////////
// Set submesh center
void SubMesh::SetSubMeshCenter(math::Vector3 center)
{
  std::vector<math::Vector3>::iterator viter;
  for (viter = this->vertices.begin(); viter != this->vertices.end(); viter++)
  {
    // generate projected texture coordinates, projected from center of aabb
    // get x,y,z for computing texture coordinate projections
    (*viter).x = (*viter).x-center.x;
    (*viter).y = (*viter).y-center.y;
    (*viter).z = (*viter).z-center.z;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Assign uv coordinates using spherical projection
void SubMesh::GenSphericalTexCoord(math::Vector3 center)
{
  std::vector<math::Vector3>::const_iterator viter;
  for (viter = this->vertices.begin(); viter != this->vertices.end(); viter++)
  {
    // generate projected texture coordinates, projected from center
    // get x,y,z for computing texture coordinate projections
    double x = (*viter).x-center.x;
    double y = (*viter).y-center.y;
    double z = (*viter).z-center.z;

    double r = std::max(0.000001,sqrt(x*x+y*y+z*z));
    double s = std::min(1.0,std::max(-1.0,z/r));
    double t = std::min(1.0,std::max(-1.0,y/r));
    double u = acos(s) / M_PI;
    double v = acos(t) / M_PI;
    //gzerr << "uv1 debug: " << u << "," << v << std::endl;
    this->AddTexCoord(u,v);
  }
}

