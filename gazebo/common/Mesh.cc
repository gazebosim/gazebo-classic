/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include <float.h>
#include <string.h>
#include <algorithm>

#include "gazebo/math/Helpers.hh"

#include "gazebo/common/Material.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/common/Skeleton.hh"
#include "gazebo/gazebo_config.h"

using namespace gazebo;
using namespace common;


//////////////////////////////////////////////////
Mesh::Mesh()
{
  this->name = "unknown";
  this->skeleton = NULL;
}

//////////////////////////////////////////////////
Mesh::~Mesh()
{
  for (std::vector<SubMesh*>::iterator iter = this->submeshes.begin();
       iter != this->submeshes.end(); ++iter)
  {
    delete *iter;
  }
  this->submeshes.clear();

  for (std::vector<Material*>::iterator iter = this->materials.begin();
       iter != this->materials.end(); ++iter)
  {
    delete *iter;
  }
  this->materials.clear();

  delete this->skeleton;
}

//////////////////////////////////////////////////
void Mesh::SetPath(const std::string &_path)
{
  this->path = _path;
}

//////////////////////////////////////////////////
std::string Mesh::GetPath() const
{
  return this->path;
}

//////////////////////////////////////////////////
void Mesh::SetName(const std::string &_n)
{
  this->name = _n;
}

//////////////////////////////////////////////////
std::string Mesh::GetName() const
{
  return this->name;
}

//////////////////////////////////////////////////
math::Vector3 Mesh::GetMax() const
{
  math::Vector3 max;
  std::vector<SubMesh*>::const_iterator iter;

  max.x = -FLT_MAX;
  max.y = -FLT_MAX;
  max.z = -FLT_MAX;

  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); ++iter)
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

//////////////////////////////////////////////////
math::Vector3 Mesh::GetMin() const
{
  math::Vector3 min;
  std::vector<SubMesh *>::const_iterator iter;

  min.x = FLT_MAX;
  min.y = FLT_MAX;
  min.z = FLT_MAX;

  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); ++iter)
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

//////////////////////////////////////////////////
unsigned int Mesh::GetVertexCount() const
{
  unsigned int sum = 0;
  std::vector<SubMesh *>::const_iterator iter;

  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); ++iter)
  {
    if ((*iter)->GetVertexCount() <= 2)
      continue;
    sum += (*iter)->GetVertexCount();
  }

  return sum;
}

//////////////////////////////////////////////////
unsigned int Mesh::GetNormalCount() const
{
  unsigned int sum = 0;
  std::vector<SubMesh *>::const_iterator iter;

  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); ++iter)
  {
    if ((*iter)->GetVertexCount() <= 2)
      continue;
    sum += (*iter)->GetNormalCount();
  }

  return sum;
}

//////////////////////////////////////////////////
unsigned int Mesh::GetIndexCount() const
{
  unsigned int sum = 0;
  std::vector<SubMesh *>::const_iterator iter;

  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); ++iter)
  {
    if ((*iter)->GetVertexCount() <= 2)
      continue;
    sum += (*iter)->GetIndexCount();
  }

  return sum;
}

//////////////////////////////////////////////////
unsigned int Mesh::GetTexCoordCount() const
{
  unsigned int sum = 0;
  std::vector<SubMesh *>::const_iterator iter;

  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); ++iter)
  {
    if ((*iter)->GetVertexCount() <= 2)
      continue;
    sum += (*iter)->GetTexCoordCount();
  }

  return sum;
}


//////////////////////////////////////////////////
void Mesh::AddSubMesh(SubMesh *_sub)
{
  this->submeshes.push_back(_sub);
}

//////////////////////////////////////////////////
unsigned int Mesh::GetSubMeshCount() const
{
  return this->submeshes.size();
}

//////////////////////////////////////////////////
const SubMesh *Mesh::GetSubMesh(unsigned int i) const
{
  if (i < this->submeshes.size())
    return this->submeshes[i];
  else
    gzthrow("Invalid index: " << i << " >= " << this->submeshes.size() << "\n");
}

//////////////////////////////////////////////////
const SubMesh *Mesh::GetSubMesh(const std::string &_name) const
{
  // Find the submesh with the provided name.
  for (std::vector<SubMesh *>::const_iterator iter = this->submeshes.begin();
       iter != this->submeshes.end(); ++iter)
  {
    if ((*iter)->GetName() == _name)
      return *iter;
  }

  return NULL;
}

//////////////////////////////////////////////////
int Mesh::AddMaterial(Material *_mat)
{
  int result = -1;

  if (_mat)
  {
    this->materials.push_back(_mat);
    result = this->materials.size()-1;
  }

  return result;
}

//////////////////////////////////////////////////
unsigned int Mesh::GetMaterialCount() const
{
  return this->materials.size();
}

//////////////////////////////////////////////////
const Material *Mesh::GetMaterial(int index) const
{
  if (index >= 0 && index < static_cast<int>(this->materials.size()))
    return this->materials[index];

  return NULL;
}

//////////////////////////////////////////////////
void Mesh::FillArrays(float **_vertArr, int **_indArr) const
{
  std::vector<SubMesh *>::const_iterator iter;

  unsigned int vertCount = 0;
  unsigned int indCount = 0;

  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); ++iter)
  {
    if ((*iter)->GetVertexCount() <= 2)
      continue;
    vertCount += (*iter)->GetVertexCount();
    indCount += (*iter)->GetIndexCount();
  }

  if (*_vertArr)
    delete [] *_vertArr;

  if (*_indArr)
    delete [] *_indArr;

  *_vertArr = new float[vertCount * 3];
  *_indArr = new int[indCount];

  float *vPtr = *_vertArr;
  unsigned int index = 0;
  unsigned int offset = 0;

  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); ++iter)
  {
    if ((*iter)->GetVertexCount() <= 2)
      continue;

    float *vertTmp = NULL;
    int *indTmp = NULL;
    (*iter)->FillArrays(&vertTmp, &indTmp);

    memcpy(vPtr, vertTmp, sizeof(vertTmp[0])*(*iter)->GetVertexCount()*3);

    for (unsigned int i = 0; i < (*iter)->GetIndexCount(); i++)
    {
      (*_indArr)[index++] = (*iter)->GetIndex(i) + offset;
    }

    offset = offset + (*iter)->GetMaxIndex() + 1;

    vPtr += (*iter)->GetVertexCount()*3;

    delete [] vertTmp;
    delete [] indTmp;
  }
}

//////////////////////////////////////////////////
void Mesh::RecalculateNormals()
{
  std::vector<SubMesh*>::iterator iter;
  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); ++iter)
    (*iter)->RecalculateNormals();
}

//////////////////////////////////////////////////
void Mesh::SetSkeleton(Skeleton* _skel)
{
  this->skeleton = _skel;
}

//////////////////////////////////////////////////
Skeleton* Mesh::GetSkeleton() const
{
  return this->skeleton;
}

//////////////////////////////////////////////////
bool Mesh::HasSkeleton() const
{
  if (this->skeleton)
    return true;
  else
    return false;
}

//////////////////////////////////////////////////
void Mesh::Scale(double _factor)
{
  std::vector<SubMesh*>::iterator iter;
  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); ++iter)
    (*iter)->Scale(_factor);
}

//////////////////////////////////////////////////
void Mesh::SetScale(const math::Vector3 &_factor)
{
  std::vector<SubMesh*>::iterator iter;
  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); ++iter)
    (*iter)->SetScale(_factor);
}

//////////////////////////////////////////////////
void Mesh::GenSphericalTexCoord(const math::Vector3 &_center)
{
  std::vector<SubMesh*>::iterator siter;
  for (siter = this->submeshes.begin(); siter != this->submeshes.end(); ++siter)
    (*siter)->GenSphericalTexCoord(_center);
}

//////////////////////////////////////////////////
void Mesh::Center(const math::Vector3 &_center)
{
  math::Vector3 min, max, half;
  min = this->GetMin();
  max = this->GetMax();
  half = (max - min) * 0.5;

  this->Translate(_center - (min + half));
}

//////////////////////////////////////////////////
void Mesh::Translate(const math::Vector3 &_vec)
{
  std::vector<SubMesh*>::iterator iter;

  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); ++iter)
  {
    if ((*iter)->GetVertexCount() <= 2)
      continue;

    (*iter)->Translate(_vec);
  }
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////

//////////////////////////////////////////////////
SubMesh::SubMesh()
{
  this->materialIndex = -1;
  this->primitiveType = TRIANGLES;
}

//////////////////////////////////////////////////
SubMesh::SubMesh(const SubMesh *_mesh)
{
  this->name = _mesh->name;
  this->materialIndex = _mesh->materialIndex;
  this->primitiveType = _mesh->primitiveType;

  std::copy(_mesh->nodeAssignments.begin(), _mesh->nodeAssignments.end(),
      std::back_inserter(this->nodeAssignments));

  std::copy(_mesh->indices.begin(), _mesh->indices.end(),
      std::back_inserter(this->indices));
  std::copy(_mesh->normals.begin(), _mesh->normals.end(),
      std::back_inserter(this->normals));
  std::copy(_mesh->texCoords.begin(), _mesh->texCoords.end(),
      std::back_inserter(this->texCoords));
  std::copy(_mesh->vertices.begin(), _mesh->vertices.end(),
      std::back_inserter(this->vertices));
}

//////////////////////////////////////////////////
SubMesh::~SubMesh()
{
  this->vertices.clear();
  this->indices.clear();
  this->nodeAssignments.clear();
}

//////////////////////////////////////////////////
void SubMesh::SetPrimitiveType(PrimitiveType _type)
{
  this->primitiveType = _type;
}

//////////////////////////////////////////////////
SubMesh::PrimitiveType SubMesh::GetPrimitiveType() const
{
  return this->primitiveType;
}

//////////////////////////////////////////////////
void SubMesh::CopyVertices(const std::vector<math::Vector3> &_verts)
{
  this->vertices.clear();
  this->vertices.resize(_verts.size());
  std::copy(_verts.begin(), _verts.end(), this->vertices.begin());
}

//////////////////////////////////////////////////
void SubMesh::CopyNormals(const std::vector<math::Vector3> &_norms)
{
  this->normals.clear();
  this->normals.resize(_norms.size());
  for (unsigned int i = 0; i < _norms.size(); i++)
  {
    this->normals[i] = _norms[i];
    this->normals[i].Normalize();
    if (math::equal(this->normals[i].GetLength(), 0.0))
    {
      this->normals[i].Set(0, 0, 1);
    }
  }
}

//////////////////////////////////////////////////
void SubMesh::SetVertexCount(unsigned int _count)
{
  this->vertices.resize(_count);
}

//////////////////////////////////////////////////
void SubMesh::SetIndexCount(unsigned int _count)
{
  this->indices.resize(_count);
}

//////////////////////////////////////////////////
void SubMesh::SetNormalCount(unsigned int _count)
{
  this->normals.resize(_count);
}

//////////////////////////////////////////////////
void SubMesh::SetTexCoordCount(unsigned int _count)
{
  this->texCoords.resize(_count);
}

//////////////////////////////////////////////////
void SubMesh::AddIndex(unsigned int _i)
{
  this->indices.push_back(_i);
}

//////////////////////////////////////////////////
void SubMesh::AddVertex(const math::Vector3 &_v)
{
  this->vertices.push_back(_v);
}

//////////////////////////////////////////////////
void SubMesh::AddVertex(double _x, double _y, double _z)
{
  this->AddVertex(math::Vector3(_x, _y, _z));
}

//////////////////////////////////////////////////
void SubMesh::AddNormal(const math::Vector3 &_n)
{
  this->normals.push_back(_n);
}

//////////////////////////////////////////////////
void SubMesh::AddNormal(double _x, double _y, double _z)
{
  this->AddNormal(math::Vector3(_x, _y, _z));
}

//////////////////////////////////////////////////
void SubMesh::AddTexCoord(double _u, double _v)
{
  this->texCoords.push_back(math::Vector2d(_u, _v));
}

//////////////////////////////////////////////////
void SubMesh::AddNodeAssignment(unsigned int _vertex, unsigned int _node,
                                     float _weight)
{
  NodeAssignment na;
  na.vertexIndex = _vertex;
  na.nodeIndex = _node;
  na.weight = _weight;

  this->nodeAssignments.push_back(na);
}

//////////////////////////////////////////////////
math::Vector3 SubMesh::GetVertex(unsigned int _i) const
{
  if (_i >= this->vertices.size())
    gzthrow("Index too large");

  return this->vertices[_i];
}

//////////////////////////////////////////////////
void SubMesh::SetVertex(unsigned int _i, const math::Vector3 &_v)
{
  if (_i >= this->vertices.size())
    gzthrow("Index too large");

  this->vertices[_i] = _v;
}

//////////////////////////////////////////////////
math::Vector3 SubMesh::GetNormal(unsigned int _i) const
{
  if (_i >= this->normals.size())
    gzthrow("Index too large");

  return this->normals[_i];
}

//////////////////////////////////////////////////
void SubMesh::SetNormal(unsigned int _i, const math::Vector3 &_n)
{
  if (_i >= this->normals.size())
    gzthrow("Index too large");

  this->normals[_i] = _n;
}

//////////////////////////////////////////////////
math::Vector2d SubMesh::GetTexCoord(unsigned int _i) const
{
  if (_i >= this->texCoords.size())
    gzthrow("Index too large");

  return this->texCoords[_i];
}

//////////////////////////////////////////////////
NodeAssignment SubMesh::GetNodeAssignment(unsigned int _i) const
{
  if (_i >= this->nodeAssignments.size())
    gzthrow("Index too large");

  return this->nodeAssignments[_i];
}

//////////////////////////////////////////////////
void SubMesh::SetTexCoord(unsigned int _i, const math::Vector2d &_t)
{
  if (_i >= this->texCoords.size())
    gzthrow("Index too large");

  this->texCoords[_i] = _t;
}

//////////////////////////////////////////////////
unsigned int SubMesh::GetIndex(unsigned int _i) const
{
  if (_i > this->indices.size())
    gzthrow("Index too large");

  return this->indices[_i];
}

//////////////////////////////////////////////////
math::Vector3 SubMesh::GetMax() const
{
  math::Vector3 max;
  std::vector<math::Vector3>::const_iterator iter;

  max.x = -FLT_MAX;
  max.y = -FLT_MAX;
  max.z = -FLT_MAX;

  for (iter = this->vertices.begin(); iter != this->vertices.end(); ++iter)
  {
    max.x = std::max(max.x, (*iter).x);
    max.y = std::max(max.y, (*iter).y);
    max.z = std::max(max.z, (*iter).z);
  }

  return max;
}

//////////////////////////////////////////////////
math::Vector3 SubMesh::GetMin() const
{
  math::Vector3 min;
  std::vector<math::Vector3>::const_iterator iter;

  min.x = FLT_MAX;
  min.y = FLT_MAX;
  min.z = FLT_MAX;

  for (iter = this->vertices.begin(); iter != this->vertices.end(); ++iter)
  {
    min.x = std::min(min.x, (*iter).x);
    min.y = std::min(min.y, (*iter).y);
    min.z = std::min(min.z, (*iter).z);
  }

  return min;
}

//////////////////////////////////////////////////
unsigned int SubMesh::GetVertexCount() const
{
  return this->vertices.size();
}

//////////////////////////////////////////////////
unsigned int SubMesh::GetNormalCount() const
{
  return this->normals.size();
}

//////////////////////////////////////////////////
unsigned int SubMesh::GetIndexCount() const
{
  return this->indices.size();
}

//////////////////////////////////////////////////
unsigned int SubMesh::GetTexCoordCount() const
{
  return this->texCoords.size();
}

//////////////////////////////////////////////////
unsigned int SubMesh::GetNodeAssignmentsCount() const
{
  return this->nodeAssignments.size();
}

//////////////////////////////////////////////////
unsigned int SubMesh::GetMaxIndex() const
{
  std::vector<unsigned int>::const_iterator maxIter;
  maxIter = std::max_element(this->indices.begin(), this->indices.end());

  if (maxIter != this->indices.end())
    return *maxIter;

  return 0;
}

//////////////////////////////////////////////////
void SubMesh::SetMaterialIndex(unsigned int _index)
{
  this->materialIndex = _index;
}

//////////////////////////////////////////////////
unsigned int SubMesh::GetMaterialIndex() const
{
  return this->materialIndex;
}

//////////////////////////////////////////////////
bool SubMesh::HasVertex(const math::Vector3 &_v) const
{
  std::vector< math::Vector3 >::const_iterator iter;

  for (iter = this->vertices.begin(); iter != this->vertices.end(); ++iter)
    if (_v.Equal(*iter))
      return true;

  return false;
}

//////////////////////////////////////////////////
unsigned int SubMesh::GetVertexIndex(const math::Vector3 &_v) const
{
  std::vector< math::Vector3 >::const_iterator iter;

  for (iter = this->vertices.begin(); iter != this->vertices.end(); ++iter)
    if (_v.Equal(*iter))
      return iter - this->vertices.begin();

  return 0;
}

//////////////////////////////////////////////////
void SubMesh::FillArrays(float **_vertArr, int **_indArr) const
{
  if (this->vertices.empty() || this->indices.empty())
    gzerr << "No vertices or indices\n";

  std::vector< math::Vector3 >::const_iterator viter;
  std::vector< unsigned int >::const_iterator iiter;
  unsigned int i;

  if (*_vertArr)
    delete [] *_vertArr;

  if (*_indArr)
    delete [] *_indArr;

  *_vertArr = new float[this->vertices.size() * 3];
  *_indArr = new int[this->indices.size()];

  for (viter = this->vertices.begin(), i = 0; viter != this->vertices.end();
      ++viter)
  {
    (*_vertArr)[i++] = static_cast<float>((*viter).x);
    (*_vertArr)[i++] = static_cast<float>((*viter).y);
    (*_vertArr)[i++] = static_cast<float>((*viter).z);
  }

  for (iiter = this->indices.begin(), i = 0;
      iiter != this->indices.end(); ++iiter)
    (*_indArr)[i++] = (*iiter);
}

//////////////////////////////////////////////////
void SubMesh::RecalculateNormals()
{
  unsigned int i;
  if (normals.size() < 3)
    return;

  // Reset all the normals
  for (i = 0; i < this->normals.size(); i++)
    this->normals[i].Set(0, 0, 0);

  if (this->normals.size() != this->vertices.size())
    this->normals.resize(this->vertices.size());

  // For each face, which is defined by three indices, calculate the normals
  for (i = 0; i < this->indices.size(); i+= 3)
  {
    math::Vector3 v1 = this->vertices[this->indices[i]];
    math::Vector3 v2 = this->vertices[this->indices[i+1]];
    math::Vector3 v3 = this->vertices[this->indices[i+2]];
    math::Vector3 n = math::Vector3::GetNormal(v1, v2, v3);

    for (unsigned int j = 0; j< this->vertices.size(); j++)
    {
      if (this->vertices[j] == v1 ||
          this->vertices[j] == v2 ||
          this->vertices[j] == v3)
      {
        this->normals[j] += n;
      }
    }
  }

  // Normalize the results
  for (i = 0; i < this->normals.size(); i++)
  {
    this->normals[i].Normalize();
  }
}

//////////////////////////////////////////////////
void Mesh::GetAABB(math::Vector3 &_center, math::Vector3 &_min_xyz,
                   math::Vector3 &_max_xyz) const
{
  // find aabb center
  _min_xyz.x = 1e15;
  _max_xyz.x = -1e15;
  _min_xyz.y = 1e15;
  _max_xyz.y = -1e15;
  _min_xyz.z = 1e15;
  _max_xyz.z = -1e15;
  _center.x = 0;
  _center.y = 0;
  _center.z = 0;

  std::vector<SubMesh*>::const_iterator siter;
  for (siter = this->submeshes.begin(); siter != this->submeshes.end(); ++siter)
  {
    math::Vector3 max = (*siter)->GetMax();
    math::Vector3 min = (*siter)->GetMin();
    _min_xyz.x = std::min(_min_xyz.x, min.x);
    _max_xyz.x = std::max(_max_xyz.x, max.x);
    _min_xyz.y = std::min(_min_xyz.y, min.y);
    _max_xyz.y = std::max(_max_xyz.y, max.y);
    _min_xyz.z = std::min(_min_xyz.z, min.z);
    _max_xyz.z = std::max(_max_xyz.z, max.z);
  }
  _center.x = 0.5*(_min_xyz.x+_max_xyz.x);
  _center.y = 0.5*(_min_xyz.y+_max_xyz.y);
  _center.z = 0.5*(_min_xyz.z+_max_xyz.z);
}


//////////////////////////////////////////////////
void SubMesh::GenSphericalTexCoord(const math::Vector3 &_center)
{
  std::vector<math::Vector3>::const_iterator viter;
  for (viter = this->vertices.begin(); viter != this->vertices.end(); ++viter)
  {
    // generate projected texture coordinates, projected from center
    // get x, y, z for computing texture coordinate projections
    double x = (*viter).x - _center.x;
    double y = (*viter).y - _center.y;
    double z = (*viter).z - _center.z;

    double r = std::max(0.000001, sqrt(x*x+y*y+z*z));
    double s = std::min(1.0, std::max(-1.0, z/r));
    double t = std::min(1.0, std::max(-1.0, y/r));
    double u = acos(s) / M_PI;
    double v = acos(t) / M_PI;
    this->AddTexCoord(u, v);
  }
}

//////////////////////////////////////////////////
void SubMesh::Scale(double _factor)
{
  for (std::vector<math::Vector3>::iterator iter = this->vertices.begin();
       iter != this->vertices.end(); ++iter)
  {
    (*iter) *= _factor;
  }
}

//////////////////////////////////////////////////
void SubMesh::SetScale(const math::Vector3 &_factor)
{
  for (std::vector<math::Vector3>::iterator iter = this->vertices.begin();
       iter != this->vertices.end(); ++iter)
  {
    (*iter).x *= _factor.x;
    (*iter).y *= _factor.y;
    (*iter).z *= _factor.z;
  }
}

//////////////////////////////////////////////////
void SubMesh::Center(const math::Vector3 &_center)
{
  math::Vector3 min, max, half;
  min = this->GetMin();
  max = this->GetMax();
  half = (max - min) * 0.5;

  this->Translate(_center - (min + half));
}

//////////////////////////////////////////////////
void SubMesh::Translate(const math::Vector3 &_vec)
{
  for (std::vector<math::Vector3>::iterator iter = this->vertices.begin();
       iter != this->vertices.end(); ++iter)
  {
    (*iter) += _vec;
  }
}

//////////////////////////////////////////////////
void SubMesh::SetName(const std::string &_n)
{
  this->name = _n;
}

//////////////////////////////////////////////////
std::string SubMesh::GetName() const
{
  return this->name;
}
