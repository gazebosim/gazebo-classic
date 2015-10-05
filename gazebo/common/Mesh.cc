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
ignition::math::Vector3d Mesh::Max() const
{
  ignition::math::Vector3d max;
  std::vector<SubMesh*>::const_iterator iter;

  max.X(-FLT_MAX);
  max.Y(-FLT_MAX);
  max.Z(-FLT_MAX);

  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); ++iter)
  {
    if ((*iter)->GetVertexCount() <= 2)
      continue;

    ignition::math::Vector3d smax = (*iter)->Max();
    max.X(std::max(max.X(), smax.X()));
    max.Y(std::max(max.Y(), smax.Y()));
    max.Z(std::max(max.Z(), smax.Z()));
  }

  return max;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Mesh::Min() const
{
  ignition::math::Vector3d min;
  std::vector<SubMesh *>::const_iterator iter;

  min.X(FLT_MAX);
  min.Y(FLT_MAX);
  min.Z(FLT_MAX);

  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); ++iter)
  {
    if ((*iter)->GetVertexCount() <= 2)
      continue;

    ignition::math::Vector3d smin = (*iter)->Min();
    min.X(std::min(min.X(), smin.X()));
    min.Y(std::min(min.Y(), smin.Y()));
    min.Z(std::min(min.Z(), smin.Z()));
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
int Mesh::GetMaterialIndex(const Material *_mat) const
{
  for (unsigned int i = 0; i < this->materials.size(); ++i)
  {
    if (this->materials[i] == _mat)
      return i;
  }

  return -1;
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

    for (unsigned int i = 0; i < (*iter)->GetIndexCount(); ++i)
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
void Mesh::SetScale(const ignition::math::Vector3d &_factor)
{
  std::vector<SubMesh*>::iterator iter;
  for (iter = this->submeshes.begin(); iter != this->submeshes.end(); ++iter)
    (*iter)->SetScale(_factor);
}

//////////////////////////////////////////////////
void Mesh::GenSphericalTexCoord(const ignition::math::Vector3d &_center)
{
  std::vector<SubMesh*>::iterator siter;
  for (siter = this->submeshes.begin(); siter != this->submeshes.end(); ++siter)
    (*siter)->GenSphericalTexCoord(_center);
}

//////////////////////////////////////////////////
void Mesh::Center(const ignition::math::Vector3d &_center)
{
  ignition::math::Vector3d min, max, half;
  min = this->Min();
  max = this->Max();
  half = (max - min) * 0.5;

  this->Translate(_center - (min + half));
}

//////////////////////////////////////////////////
void Mesh::Translate(const ignition::math::Vector3d &_vec)
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
  if (!_mesh)
  {
    gzerr << "Submesh is NULL." << std::endl;
    return;
  }

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
void SubMesh::CopyVertices(const std::vector<ignition::math::Vector3d> &_verts)
{
  this->vertices.clear();
  this->vertices.resize(_verts.size());
  std::copy(_verts.begin(), _verts.end(), this->vertices.begin());
}

//////////////////////////////////////////////////
void SubMesh::CopyNormals(const std::vector<ignition::math::Vector3d> &_norms)
{
  this->normals.clear();
  this->normals.resize(_norms.size());
  for (unsigned int i = 0; i < _norms.size(); ++i)
  {
    this->normals[i] = _norms[i];
    this->normals[i].Normalize();
    if (ignition::math::equal(this->normals[i].Length(), 0.0))
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
void SubMesh::AddVertex(const ignition::math::Vector3d &_v)
{
  this->vertices.push_back(_v);
}

//////////////////////////////////////////////////
void SubMesh::AddVertex(double _x, double _y, double _z)
{
  this->AddVertex(ignition::math::Vector3d(_x, _y, _z));
}

//////////////////////////////////////////////////
void SubMesh::AddNormal(const ignition::math::Vector3d &_n)
{
  this->normals.push_back(_n);
}

//////////////////////////////////////////////////
void SubMesh::AddNormal(double _x, double _y, double _z)
{
  this->AddNormal(ignition::math::Vector3d(_x, _y, _z));
}

//////////////////////////////////////////////////
void SubMesh::AddTexCoord(double _u, double _v)
{
  this->texCoords.push_back(ignition::math::Vector2d(_u, _v));
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
ignition::math::Vector3d SubMesh::Vertex(unsigned int _i) const
{
  if (_i >= this->vertices.size())
    gzthrow("Index too large");

  return this->vertices[_i];
}

//////////////////////////////////////////////////
void SubMesh::SetVertex(unsigned int _i, const ignition::math::Vector3d &_v)
{
  if (_i >= this->vertices.size())
    gzthrow("Index too large");

  this->vertices[_i] = _v;
}

//////////////////////////////////////////////////
ignition::math::Vector3d SubMesh::Normal(unsigned int _i) const
{
  if (_i >= this->normals.size())
    gzthrow("Index too large");

  return this->normals[_i];
}

//////////////////////////////////////////////////
void SubMesh::SetNormal(unsigned int _i, const ignition::math::Vector3d &_n)
{
  if (_i >= this->normals.size())
    gzthrow("Index too large");

  this->normals[_i] = _n;
}

//////////////////////////////////////////////////
ignition::math::Vector2d SubMesh::TexCoord(unsigned int _i) const
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
void SubMesh::SetTexCoord(unsigned int _i, const ignition::math::Vector2d &_t)
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
ignition::math::Vector3d SubMesh::Max() const
{
  ignition::math::Vector3d max;
  std::vector<ignition::math::Vector3d>::const_iterator iter;

  max.X(-FLT_MAX);
  max.Y(-FLT_MAX);
  max.Z(-FLT_MAX);

  for (iter = this->vertices.begin(); iter != this->vertices.end(); ++iter)
  {
    max.X(std::max(max.X(), (*iter).X()));
    max.Y(std::max(max.Y(), (*iter).Y()));
    max.Z(std::max(max.Z(), (*iter).Z()));
  }

  return max;
}

//////////////////////////////////////////////////
ignition::math::Vector3d SubMesh::Min() const
{
  ignition::math::Vector3d min;
  std::vector<ignition::math::Vector3d>::const_iterator iter;

  min.X(FLT_MAX);
  min.Y(FLT_MAX);
  min.Z(FLT_MAX);

  for (iter = this->vertices.begin(); iter != this->vertices.end(); ++iter)
  {
    min.X(std::min(min.X(), (*iter).X()));
    min.Y(std::min(min.Y(), (*iter).Y()));
    min.Z(std::min(min.Z(), (*iter).Z()));
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
bool SubMesh::HasVertex(const ignition::math::Vector3d &_v) const
{
  std::vector< ignition::math::Vector3d >::const_iterator iter;

  for (iter = this->vertices.begin(); iter != this->vertices.end(); ++iter)
    if (_v.Equal(*iter))
      return true;

  return false;
}

//////////////////////////////////////////////////
unsigned int SubMesh::GetVertexIndex(const ignition::math::Vector3d &_v) const
{
  std::vector< ignition::math::Vector3d >::const_iterator iter;

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

  std::vector<ignition::math::Vector3d>::const_iterator viter;
  std::vector<unsigned int>::const_iterator iiter;
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
    (*_vertArr)[i++] = static_cast<float>((*viter).X());
    (*_vertArr)[i++] = static_cast<float>((*viter).Y());
    (*_vertArr)[i++] = static_cast<float>((*viter).Z());
  }

  for (iiter = this->indices.begin(), i = 0;
      iiter != this->indices.end(); ++iiter)
  {
    (*_indArr)[i++] = (*iiter);
  }
}

//////////////////////////////////////////////////
void SubMesh::RecalculateNormals()
{
  unsigned int i;
  if (normals.size() < 3)
    return;

  // Reset all the normals
  for (i = 0; i < this->normals.size(); ++i)
    this->normals[i].Set(0, 0, 0);

  if (this->normals.size() != this->vertices.size())
    this->normals.resize(this->vertices.size());

  // For each face, which is defined by three indices, calculate the normals
  for (i = 0; i < this->indices.size(); i+= 3)
  {
    ignition::math::Vector3d v1 = this->vertices[this->indices[i]];
    ignition::math::Vector3d v2 = this->vertices[this->indices[i+1]];
    ignition::math::Vector3d v3 = this->vertices[this->indices[i+2]];
    ignition::math::Vector3d n = ignition::math::Vector3d::Normal(v1, v2, v3);

    for (unsigned int j = 0; j< this->vertices.size(); ++j)
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
  for (i = 0; i < this->normals.size(); ++i)
  {
    this->normals[i].Normalize();
  }
}

//////////////////////////////////////////////////
void Mesh::GetAABB(ignition::math::Vector3d &_center,
                   ignition::math::Vector3d &_minXYZ,
                   ignition::math::Vector3d &_maxXYZ) const
{
  // find aabb center
  _minXYZ.X(1e15);
  _maxXYZ.X(-1e15);
  _minXYZ.Y(1e15);
  _maxXYZ.Y(-1e15);
  _minXYZ.Z(1e15);
  _maxXYZ.Z(-1e15);
  _center.X(0);
  _center.Y(0);
  _center.Z(0);

  std::vector<SubMesh*>::const_iterator siter;
  for (siter = this->submeshes.begin(); siter != this->submeshes.end(); ++siter)
  {
    ignition::math::Vector3d max = (*siter)->Max();
    ignition::math::Vector3d min = (*siter)->Min();

    _minXYZ.X(std::min(_minXYZ.X(), min.X()));
    _maxXYZ.X(std::max(_maxXYZ.X(), max.X()));
    _minXYZ.Y(std::min(_minXYZ.Y(), min.Y()));
    _maxXYZ.Y(std::max(_maxXYZ.Y(), max.Y()));
    _minXYZ.Z(std::min(_minXYZ.Z(), min.Z()));
    _maxXYZ.Z(std::max(_maxXYZ.Z(), max.Z()));
  }
  _center.X(0.5 * (_minXYZ.X() + _maxXYZ.X()));
  _center.Y(0.5 * (_minXYZ.Y() + _maxXYZ.Y()));
  _center.Z(0.5 * (_minXYZ.Z() + _maxXYZ.Z()));
}

//////////////////////////////////////////////////
void SubMesh::GenSphericalTexCoord(const ignition::math::Vector3d &_center)
{
  std::vector<ignition::math::Vector3d>::const_iterator viter;
  for (viter = this->vertices.begin(); viter != this->vertices.end(); ++viter)
  {
    // generate projected texture coordinates, projected from center
    // get x, y, z for computing texture coordinate projections
    double x = (*viter).X() - _center.X();
    double y = (*viter).Y() - _center.Y();
    double z = (*viter).Z() - _center.Z();

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
  for (auto &vert : this->vertices)
    vert *= _factor;
}

//////////////////////////////////////////////////
void SubMesh::SetScale(const ignition::math::Vector3d &_factor)
{
  for (auto &vert : this->vertices)
    vert *= _factor;
}

//////////////////////////////////////////////////
void SubMesh::Center(const ignition::math::Vector3d &_center)
{
  ignition::math::Vector3d min, max, half;
  min = this->Min();
  max = this->Max();
  half = (max - min) * 0.5;

  this->Translate(_center - (min + half));
}

//////////////////////////////////////////////////
void SubMesh::Translate(const ignition::math::Vector3d &_vec)
{
  for (auto &vert : this->vertices)
    vert += _vec;
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

//////////////////////////////////////////////////
NodeAssignment::NodeAssignment()
  : vertexIndex(0), nodeIndex(0), weight(0.0)
{
}
