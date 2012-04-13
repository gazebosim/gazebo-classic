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
#include <string>

#include "math/Vector3.hh"
#include "math/Vector2d.hh"

namespace gazebo
{
  namespace common
  {
    class Material;
    class SubMesh;
    class Skeleton;

    /// \addtogroup gazebo_common Common
    /// \{
    /// \brief A 3D mesh
    class Mesh
    {
      /// \brief Constructor
      public: Mesh();

      /// \brief Destructor
      public: virtual ~Mesh();

      /// \brief Set the path which contains the mesh resource
      public: void SetPath(const std::string &_path);

      /// \brief Get the path which contains the mesh resource
      public: std::string GetPath() const;

      /// \brief Set the name of this mesh
      public: void SetName(const std::string &_n);

      /// \brief Get the name of this mesh
      public: std::string GetName() const;

      /// \brief Get the maximun X, Y, Z values
      public: math::Vector3 GetMax() const;

      /// \brief Get the minimum X, Y, Z values
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
      public: void AddSubMesh(SubMesh *_child);

      /// \brief Get the number of children
      public: unsigned int GetSubMeshCount() const;

      /// \brief Add a material to the mesh
      /// \return Index of this material
      public: unsigned int AddMaterial(Material *_mat);

      /// \brief Get the number of materials
      public: unsigned int GetMaterialCount() const;

      /// \brief Get a material
      public: const Material *GetMaterial(int index) const;

      /// \brief Get a child
      public: const SubMesh *GetSubMesh(unsigned int i) const;

      /// \brief Put all the data into flat arrays
      public: void FillArrays(float **_vertArr, int **_indArr) const;

      /// \brief Recalculate all the normals.
      public: void RecalculateNormals();

      /// \brief Get AABB coordinate
      public: void GetAABB(math::Vector3 &_center, math::Vector3 &_min_xyz,
                           math::Vector3 &_max_xyz) const;

      /// \brief Generate texture coordinates using spherical projection
      ///        from center
      public: void GenSphericalTexCoord(const math::Vector3 &_center);

      /// \brief Get the skeleton to which this mesh is attached.
      /// \return pointer to skeleton, or NULL if none is present.
      public: Skeleton* GetSkeleton() const;

      /// \brief Set the mesh skeleton
      public: void SetSkeleton(Skeleton *_skel);

      /// \brief Return true if mesh is attached to a skeleton.
      public: bool HasSkeleton() const;

      private: std::string name;
      private: std::string path;
      private: std::vector<SubMesh *> submeshes;
      private: std::vector<Material *> materials;
      private: Skeleton *skeleton;
    };

    struct NodeAssignment
    {
      unsigned int vertexIndex;
      unsigned int nodeIndex;
      float weight;
    };

    /// \brief A child mesh
    class SubMesh
    {
      public: enum PrimitiveType {POINTS, LINES, LINESTRIPS, TRIANGLES,
                TRIFANS, TRISTRIPS};

      /// \brief Constructor
      public: SubMesh();

      /// \brief Destructor
      public: virtual ~SubMesh();

      /// \brief Set the primitive type
      public: void SetPrimitiveType(PrimitiveType _type);

      /// \brief Get the primitive type
      public: PrimitiveType GetPrimitiveType() const;

      /// \brief Copy vertices from a vector
      public: void CopyVertices(const std::vector<math::Vector3> &_verts);

      /// \brief Copy normals from a vector
      public: void CopyNormals(const std::vector<math::Vector3> &_norms);

      /// \brief Resize the vertex array
      public: void SetVertexCount(unsigned int _count);

      /// \brief Resize the index array
      public: void SetIndexCount(unsigned int _count);

      /// \brief Resize the normal array
      public: void SetNormalCount(unsigned int _count);

      /// \brief Resize the texture coordinate  array
      public: void SetTexCoordCount(unsigned int _count);

      /// \brief Add an index to the mesh
      public: void AddIndex(unsigned int _i);

      /// \brief Add a vertex to the mesh
      public: void AddVertex(const math::Vector3 &_v);

      /// \brief Add a vertex to the mesh
      public: void AddVertex(double _x, double _y, double _z);

      /// \brief Add a normal to the mesh
      public: void AddNormal(const math::Vector3 &_n);

      /// \brief Add a normal to the mesh
      public: void AddNormal(double _x, double _y, double _z);

      /// \brief Add a texture coord to the mesh
      public: void AddTexCoord(double _u, double _v);

      /// \brief Add a vertex - skeleton node assignment
      public: void AddNodeAssignment(unsigned int _vertex, unsigned int _node,
                                     float _weight);

      /// \brief Get a vertex
      public: math::Vector3 GetVertex(unsigned int _i) const;

      /// \brief Set a vertex
      public: void SetVertex(unsigned int _i, const math::Vector3 &_v);

      /// \brief Get a normal
      public: math::Vector3 GetNormal(unsigned int _i) const;

      /// \brief Set a normal
      public: void SetNormal(unsigned int _i, const math::Vector3 &_n);

      /// \brief Get a tex coord
      public: math::Vector2d GetTexCoord(unsigned int _i) const;

      /// \brief Get a vertex - skeleton node assignment
      public: NodeAssignment GetNodeAssignment(unsigned int _i) const;

      /// \brief Set a tex coord
      public: void SetTexCoord(unsigned int _i, const math::Vector2d &_t);

      /// \brief Get an index
      public: unsigned int GetIndex(unsigned int _i) const;

      /// \brief Get the maximun X, Y, Z values
      public: math::Vector3 GetMax() const;

      /// \brief Get the minimum X, Y, Z values
      public: math::Vector3 GetMin() const;

      /// \brief Return the number of vertices
      public: unsigned int GetVertexCount() const;

      /// \brief Return the number of normals
      public: unsigned int GetNormalCount() const;

      /// \brief Return the number of indicies
      public: unsigned int GetIndexCount() const;

      /// \brief Return the number of texture coordinates
      public: unsigned int GetTexCoordCount() const;

      /// \brief Return the number of vertex - skeleton node assignments
      public: unsigned int GetNodeAssignmentsCount() const;

      /// \brief Get the highest index value
      public: unsigned int GetMaxIndex() const;

      /// \brief Set the material index. Relates to the parent mesh material
      ///        list
      public: void SetMaterialIndex(unsigned int _index);

      /// \brief Get the material index
      public: unsigned int GetMaterialIndex() const;

      /// \brief Return true if this submesh has the vertex
      public: bool HasVertex(const math::Vector3 &_v) const;

      /// \brief Get the index of the vertex
      public: unsigned int GetVertexIndex(const math::Vector3 &_v) const;

      /// \brief Put all the data into flat arrays
      public: void FillArrays(float **_vertArr, int **_indArr) const;

      /// \brief Recalculate all the normals.
      public: void RecalculateNormals();

      /// \brief Reset mesh center to geometric center
      public: void SetSubMeshCenter(math::Vector3 _center);

      /// \brief Generate texture coordinates using spherical projection
      ///        from center
      public: void GenSphericalTexCoord(const math::Vector3 &_center);

      private: std::vector< math::Vector3 > vertices;
      private: std::vector< math::Vector3 > normals;
      private: std::vector< math::Vector2d > texCoords;
      private: std::vector<unsigned int> indices;
      private: std::vector<NodeAssignment> nodeAssignments;

      private: PrimitiveType primitiveType;

      private: int materialIndex;
    };
    /// \}
  }
}
#endif

