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
#ifndef MESH_HH
#define MESH_HH

#include <vector>

#include "math/Vector3.hh"
#include "math/Vector2d.hh"

namespace gazebo
{
	namespace common
  {
    class Material;
    class SubMesh;
  
    class Mesh
    {
      /// \brief Constructor
      public: Mesh();
    
      /// \brief Destructor
      public: virtual ~Mesh();
  
      /// \brief Set the name of this mesh
      public: void SetName(const std::string &n);
  
      /// \brief Get the name of this mesh
      public: std::string GetName() const;
  
      /// \brief Get the maximun X,Y,Z values
      public: math::Vector3 GetMax() const;
  
      /// \brief Get the minimum X,Y,Z values
      public: math::Vector3 GetMin() const;
  
      /// \brief Return the number of vertices
      public: unsigned int GetVertexCount() const;
  
      /// \brief Return the number of normals
      public: unsigned int GetNormalCount() const;
  
      /// \brief Return the number of indicies
      public: unsigned int GetIndexCount() const;
  
      /// \brief Return the number of texture coordinates
      public: unsigned int GetTexCoordCount() const;
  
      /// \brief Add a submesh mesh
      public: void AddSubMesh(SubMesh *child);
  
      /// \brief Get the number of children
      public: unsigned int GetSubMeshCount() const;
  
      /// \brief Add a material to the mesh
      public: void AddMaterial( Material *mat );
  
      /// \brief Get the number of materials
      public: unsigned int GetMaterialCount() const;
  
      /// \brief Get a material
      public: const Material *GetMaterial(unsigned int index) const;
  
      /// \brief Get a child
      public: const SubMesh *GetSubMesh(unsigned int i) const;
  
      /// \brief Put all the data into flat arrays
      public: void FillArrays(float **vertArr, unsigned int **indArr) const;
  
      /// \brief Recalculate all the normals.
      public: void RecalculateNormals();
  
      /// \brief Get AABB coordinate
      public: void GetAABB(math::Vector3 &center,math::Vector3 &min_xyz,math::Vector3 &max_xyz);
  
      /// \brief Reset mesh center to geometric center
      public: void SetMeshCenter(math::Vector3 center);
  
      /// \brief Generate texture coordinates using spherical projection from center
      public: void GenSphericalTexCoord(math::Vector3 center);
  
  
      private: std::string name;
      private: std::vector<SubMesh *> submeshes;
      private: std::vector<Material *> materials;
    };
  
    class SubMesh
    {
      /// \brief Constructor
      public: SubMesh();
  
      /// \brief Destructor
      public: virtual ~SubMesh();
  
      /// \brief Add an index to the mesh
      public: void AddIndex( unsigned int i);
  
      /// \brief Add a vertex to the mesh
      public: void AddVertex( math::Vector3 v );
  
      /// \brief Add a vertex to the mesh
      public: void AddVertex(double x, double y, double z );
  
      /// \brief Add a normal to the mesh
      public: void AddNormal( math::Vector3 n );
  
      /// \brief Add a normal to the mesh
      public: void AddNormal(double x, double y, double z );
  
      /// \brief Add a texture coord to the mesh
      public: void AddTexCoord(double u, double v );
  
      /// \brief Get a vertex
      public: math::Vector3 GetVertex(unsigned int i) const;
  
      /// \brief Set a vertex
      public: void SetVertex(unsigned int i, const math::Vector3 &v);
  
      /// \brief Get a normal
      public: math::Vector3 GetNormal(unsigned int i) const;
  
      /// \brief Set a normal
      public: void SetNormal(unsigned int i, const math::Vector3 &n);
  
      /// \brief Get a tex coord
      public: math::Vector2d GetTexCoord(unsigned int i) const;
  
      /// \brief Set a tex coord
      public: void SetTexCoord(unsigned int i, const math::Vector2d &t);
  
      /// \brief Get an index
      public: unsigned int GetIndex(unsigned int i) const;
  
      /// \brief Get the maximun X,Y,Z values
      public: math::Vector3 GetMax() const;
  
      /// \brief Get the minimum X,Y,Z values
      public: math::Vector3 GetMin() const;
  
      /// \brief Return the number of vertices
      public: unsigned int GetVertexCount() const;
  
      /// \brief Return the number of normals
      public: unsigned int GetNormalCount() const;
  
      /// \brief Return the number of indicies
      public: unsigned int GetIndexCount() const;
  
      /// \brief Return the number of texture coordinates
      public: unsigned int GetTexCoordCount() const;
  
      /// \brief Get the highest index value
      public: unsigned int GetMaxIndex() const;
  
      /// \brief Set the material index. Relates to the parent mesh material list
      public: void SetMaterialIndex(unsigned int index);
  
      /// \brief Get the material index
      public: unsigned int GetMaterialIndex() const;
  
      /// \brief Return true if this submesh has the vertex
      public: bool HasVertex( const math::Vector3 &v ) const;
  
      /// \brief Get the index of the vertex
      public: unsigned int GetVertexIndex(const math::Vector3 &v) const;
  
      /// \brief Put all the data into flat arrays
      public: void FillArrays(float **vertArr, unsigned int **indArr) const;
  
      /// \brief Recalculate all the normals.
      public: void RecalculateNormals();
  
      /// \brief Reset mesh center to geometric center
      public: void SetSubMeshCenter(math::Vector3 center);
  
      /// \brief Generate texture coordinates using spherical projection from center
      public: void GenSphericalTexCoord(math::Vector3 center);
  
      private: std::vector< math::Vector3 > vertices;
      private: std::vector< math::Vector3 > normals;
      private: std::vector< math::Vector2d > texCoords;
      private: std::vector<unsigned int> indices;
  
  
      private: int materialIndex;
    };
  }

}
#endif
