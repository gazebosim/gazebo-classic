/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef _MESH_HH_
#define _MESH_HH_

#include <vector>
#include <string>

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Vector2d.hh"

namespace gazebo
{
  namespace common
  {
    class Material;
    class SubMesh;
    class Skeleton;

    /// \addtogroup gazebo_common Common
    /// \{

    /// \class Mesh Mesh.hh common/common.hh
    /// \brief A 3D mesh
    class Mesh
    {
      /// \brief Constructor
      public: Mesh();

      /// \brief Destructor
      public: virtual ~Mesh();

      /// \brief Set the path which contains the mesh resource
      /// \param[in] _path the file path
      public: void SetPath(const std::string &_path);

      /// \brief Get the path which contains the mesh resource
      /// \return the path to the mesh resource
      public: std::string GetPath() const;

      /// \brief Set the name of this mesh
      /// \param[in] _n the name to set
      public: void SetName(const std::string &_n);

      /// \brief Get the name of this mesh
      /// \return the name
      public: std::string GetName() const;

      /// \brief Get the maximun X, Y, Z values
      /// \return the upper bounds of the bounding box
      public: math::Vector3 GetMax() const;

      /// \brief Get the minimum X, Y, Z values
      /// \return the lower bounds of the bounding box
      public: math::Vector3 GetMin() const;

      /// \brief Return the number of vertices
      /// \return the count
      public: unsigned int GetVertexCount() const;

      /// \brief Return the number of normals
      /// \return the count
      public: unsigned int GetNormalCount() const;

      /// \brief Return the number of indices
      /// \return the count
      public: unsigned int GetIndexCount() const;

      /// \brief Return the number of texture coordinates
      /// \return the count
      public: unsigned int GetTexCoordCount() const;

      /// \brief Add a submesh mesh.
      /// The Mesh object takes ownership of the submesh.
      /// \param[in] _child the submesh
      public: void AddSubMesh(SubMesh *_child);

      /// \brief Get the number of children
      /// \return the count
      public: unsigned int GetSubMeshCount() const;

      /// \brief Add a material to the mesh
      /// \param[in] _mat the material
      /// \return Index of this material
      public: int AddMaterial(Material *_mat);

      /// \brief Get the number of materials
      /// \return the count
      public: unsigned int GetMaterialCount() const;

      /// \brief Get a material
      /// \param[in] _index the index
      /// \return the material or NULL if the index is out of bounds
      public: const Material *GetMaterial(int _index) const;

      /// \brief Get a child mesh
      /// \param[in] _i the index
      /// \return the submesh. An exception is thrown if the index is out of
      /// bounds
      public: const SubMesh *GetSubMesh(unsigned int _i) const;

      /// \brief Get a child mesh by name.
      /// \param[in] _name Name of the submesh.
      /// \return The submesh, NULL if the _name is not found.
      public: const SubMesh *GetSubMesh(const std::string &_name) const;

      /// \brief Put all the data into flat arrays
      /// \param[out] _vertArr the vertex array
      /// \param[out] _indArr the index array
      public: void FillArrays(float **_vertArr, int **_indArr) const;

      /// \brief Recalculate all the normals of each face defined by three
      /// indices.
      public: void RecalculateNormals();

      /// \brief Get AABB coordinate
      /// \param[out] _center of the bounding box
      /// \param[out] _min_xyz bounding box minimum values
      /// \param[out] _max_xyz bounding box maximum values
      public: void GetAABB(math::Vector3 &_center, math::Vector3 &_min_xyz,
                           math::Vector3 &_max_xyz) const;

      /// \brief Generate texture coordinates using spherical projection
      /// from center
      /// \param[in] _center the center of the projection
      public: void GenSphericalTexCoord(const math::Vector3 &_center);

      /// \brief Get the skeleton to which this mesh is attached.
      /// \return pointer to skeleton, or NULL if none is present.
      public: Skeleton* GetSkeleton() const;

      /// \brief Set the mesh skeleton
      public: void SetSkeleton(Skeleton *_skel);

      /// \brief Return true if mesh is attached to a skeleton.
      public: bool HasSkeleton() const;

      /// \brief Scale all vertices by _factor
      /// \param _factor Scaling factor
      public: void Scale(double _factor);

      /// \brief Scale all vertices by the _factor vector
      /// \param[in] _factor Scaling vector
      public: void SetScale(const math::Vector3 &_factor);

      /// \brief Move the center of the mesh to the given coordinate. This
      /// will move all the vertices in all submeshes.
      /// \param[in] _center Location of the mesh center.
      public: void Center(const math::Vector3 &_center = math::Vector3::Zero);

      /// \brief Move all vertices in all submeshes by _vec.
      /// \param[in] _vec Amount to translate vertices.
      public: void Translate(const math::Vector3 &_vec);

      /// \brief The name of the mesh
      private: std::string name;

      /// \brief The path of the mesh resource
      private: std::string path;

      /// \brief The sub mesh array.
      private: std::vector<SubMesh *> submeshes;

      /// \brief The materials array.
      private: std::vector<Material *> materials;

      /// \brief The skeleton (for animation)
      private: Skeleton *skeleton;
    };

    /// \brief Vertex to node weighted assignement for skeleton animation
    /// visualization
    struct NodeAssignment
    {
      /// \brief index of the vertex
      unsigned int vertexIndex;

      /// \brief node (or bone) index
      unsigned int nodeIndex;

      /// \brief the weight (between 0 and 1)
      float weight;
    };

    /// \brief A child mesh
    class SubMesh
    {
      /// \brief An enumeration of the geometric mesh primitives
      public: enum PrimitiveType {POINTS, LINES, LINESTRIPS, TRIANGLES,
                TRIFANS, TRISTRIPS};

      /// \brief Constructor
      public: SubMesh();

      /// \brief Copy Constructor
      public: SubMesh(const SubMesh *_mesh);

      /// \brief Destructor
      public: virtual ~SubMesh();

      /// \brief Set the name of this mesh
      /// \param[in] _n the name to set
      public: void SetName(const std::string &_n);

      /// \brief Get the name of this mesh
      /// \return the name
      public: std::string GetName() const;

      /// \brief Set the primitive type
      /// \param[in] _type the type
      public: void SetPrimitiveType(PrimitiveType _type);

      /// \brief Get the primitive type
      /// \return the primitive type
      public: PrimitiveType GetPrimitiveType() const;

      /// \brief Copy vertices from a vector
      /// \param[in] _verts the vertices to copy from
      public: void CopyVertices(const std::vector<math::Vector3> &_verts);

      /// \brief Copy normals from a vector
      /// \param[in] _norms to copy from
      public: void CopyNormals(const std::vector<math::Vector3> &_norms);

      /// \brief Resize the vertex array
      /// \param[in] _count the new size of the array
      public: void SetVertexCount(unsigned int _count);

      /// \brief Resize the index array
      /// \param[in] _count the new size of the array
      public: void SetIndexCount(unsigned int _count);

      /// \brief Resize the normal array
      /// \param[in] _count the new size of the array
      public: void SetNormalCount(unsigned int _count);

      /// \brief Resize the texture coordinate  array
      /// \param[in] _count
      public: void SetTexCoordCount(unsigned int _count);

      /// \brief Add an index to the mesh
      /// \param[in] _i the new vertex index
      public: void AddIndex(unsigned int _i);

      /// \brief Add a vertex to the mesh
      /// \param[in] _v the new position
      public: void AddVertex(const math::Vector3 &_v);

      /// \brief Add a vertex to the mesh
      /// \param[in] _x position along x
      /// \param[in] _y position along y
      /// \param[in] _z position along z
      public: void AddVertex(double _x, double _y, double _z);

      /// \brief Add a normal to the mesh
      /// \param[in] _n the normal
      public: void AddNormal(const math::Vector3 &_n);

      /// \brief Add a normal to the mesh
      /// \param[in] _x position along x
      /// \param[in] _y position along y
      /// \param[in] _z position along z
      public: void AddNormal(double _x, double _y, double _z);

      /// \brief Add a texture coord to the mesh
      /// \param[in] _u position along u
      /// \param[in] _v position along v
      public: void AddTexCoord(double _u, double _v);

      /// \brief Add a vertex - skeleton node assignment
      /// \param[in] _vertex the vertex index
      /// \param[in] _node the node index
      /// \param[in] _weight the weight (between 0 and 1)
      public: void AddNodeAssignment(unsigned int _vertex, unsigned int _node,
                                     float _weight);

      /// \brief Get a vertex
      /// \param[in] _i the vertex index
      /// \return the position or throws an exception
      public: math::Vector3 GetVertex(unsigned int _i) const;

      /// \brief Set a vertex
      /// \param[in] _i the index
      /// \param[in] _v the position
      public: void SetVertex(unsigned int _i, const math::Vector3 &_v);

      /// \brief Get a normal
      /// \param[in] _i the normal index
      /// \return the orientation of the normal, or throws an exception
      public: math::Vector3 GetNormal(unsigned int _i) const;

      /// \brief Set a normal
      /// \param[in] _i the normal index
      /// \param[in] _n the normal direction
      public: void SetNormal(unsigned int _i, const math::Vector3 &_n);

      /// \brief Get a tex coord
      /// \param[in] _i the texture index
      /// \return the texture coordinates
      public: math::Vector2d GetTexCoord(unsigned int _i) const;

      /// \brief Get a vertex - skeleton node assignment
      /// \param[in] _i the index of the assignment
      public: NodeAssignment GetNodeAssignment(unsigned int _i) const;

      /// \brief Set a tex coord
      /// \param[in] _i
      /// \param[in] _t
      public: void SetTexCoord(unsigned int _i, const math::Vector2d &_t);

      /// \brief Get an index
      /// \param[in] _i
      public: unsigned int GetIndex(unsigned int _i) const;

      /// \brief Get the maximun X, Y, Z values
      /// \return
      public: math::Vector3 GetMax() const;

      /// \brief Get the minimum X, Y, Z values
      /// \return
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
      /// list
      /// \param[in] _index
      public: void SetMaterialIndex(unsigned int _index);

      /// \brief Get the material index
      public: unsigned int GetMaterialIndex() const;

      /// \brief Return true if this submesh has the vertex
      /// \param[in] _v
      public: bool HasVertex(const math::Vector3 &_v) const;

      /// \brief Get the index of the vertex
      /// \param[in] _v
      public: unsigned int GetVertexIndex(const math::Vector3 &_v) const;

      /// \brief Put all the data into flat arrays
      /// \param[in] _verArr
      /// \param[in] _indArr
      public: void FillArrays(float **_vertArr, int **_indArr) const;

      /// \brief Recalculate all the normals.
      public: void RecalculateNormals();

      /// \brief Reset mesh center to geometric center
      /// \param[in] _center
      public: void SetSubMeshCenter(math::Vector3 _center);

      /// \brief Generate texture coordinates using spherical projection
      /// from center
      /// \param[in] _center
      public: void GenSphericalTexCoord(const math::Vector3 &_center);

      /// \brief Scale all vertices by _factor
      /// \param[in] _factor Scaling factor
      public: void Scale(double _factor);

      /// \brief Move the center of the submesh to the given coordinate. This
      /// will move all the vertices.
      /// \param[in] _center Location of the mesh center.
      public: void Center(const math::Vector3 &_center = math::Vector3::Zero);

      /// \brief Move all vertices by _vec.
      /// \param[in] _vec Amount to translate vertices.
      public: void Translate(const math::Vector3 &_vec);

      /// \brief Scale all vertices by the _factor vector
      /// \param[in] _factor Scaling vector
      public: void SetScale(const math::Vector3 &_factor);

      /// \brief the vertex array
      private: std::vector< math::Vector3 > vertices;

      /// \brief the normal array
      private: std::vector< math::Vector3 > normals;

      /// \brief the texture coordinate array
      private: std::vector< math::Vector2d > texCoords;

      /// \brief the vertex index array
      private: std::vector<unsigned int> indices;

      /// \brief node assignment array
      private: std::vector<NodeAssignment> nodeAssignments;

      /// \brief primitive type for the mesh
      private: PrimitiveType primitiveType;

      /// \brief The material index for this mesh. Relates to the parent
      /// mesh material list.
      private: int materialIndex;

      /// \brief The name of the sub-mesh
      private: std::string name;
    };
    /// \}
  }
}

#endif
