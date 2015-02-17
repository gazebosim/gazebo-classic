/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifndef _MESHMANAGER_HH_
#define _MESHMANAGER_HH_

#include <map>
#include <string>
#include <vector>
#include <boost/thread/mutex.hpp>

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Vector2d.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/math/Plane.hh"
#include "gazebo/common/SingletonT.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    class ColladaLoader;
    class ColladaExporter;
    class STLLoader;
    class Mesh;
    class Plane;
    class SubMesh;

    /// \addtogroup gazebo_common Common
    /// \{

    /// \class MeshManager MeshManager.hh common/common.hh
    /// \brief Maintains and manages all meshes
    class GAZEBO_VISIBLE MeshManager : public SingletonT<MeshManager>
    {
      /// \brief Constructor
      private: MeshManager();

      /// \brief Destructor.
      ///
      /// Destroys the collada loader, the stl loader and all the meshes
      private: virtual ~MeshManager();

      /// \brief Load a mesh from a file
      /// \param[in] _filename the path to the mesh
      /// \return a pointer to the created mesh
      public: const Mesh *Load(const std::string &_filename);

      /// \brief Export a mesh to a file
      /// \param[in] _mesh Pointer to the mesh to be exported
      /// \param[in] _filename Exported file's path and name
      /// \param[in] _extension Exported file's format ("dae" for Collada)
      /// \param[in] _exportTextures True to export texture images to
      /// '../materials/textures' folder
      public: void Export(const Mesh *_mesh, const std::string &_filename,
          const std::string &_extension, bool _exportTextures = false);

      /// \brief Checks a path extension against the list of valid extensions.
      /// \return true if the file extension is loadable
      public: bool IsValidFilename(const std::string &_filename);

      /// \brief Get mesh aabb and center.
      /// \param[in] _mesh the mesh
      /// \param[out] _center the AAB center position
      /// \param[out] _min_xyz the bounding box minimum
      /// \param[out] _max_xyz the bounding box maximum
      public: void GetMeshAABB(const Mesh *_mesh,
                               math::Vector3 &_center,
                               math::Vector3 &_min_xyz,
                               math::Vector3 &_max_xyz);

      /// \brief generate spherical texture coordinates
      public: void GenSphericalTexCoord(const Mesh *_mesh,
                                        math::Vector3 _center);


      /// \brief Add a mesh to the manager.
      ///
      /// This MeshManager takes ownership of the mesh and will destroy it.
      /// See ~MeshManager.
      /// \param[in] the mesh to add.
      public: void AddMesh(Mesh *_mesh);

      /// \brief Get a mesh by name.
      /// \param[in] _name the name of the mesh to look for
      /// \return the mesh or NULL if not found
      public: const Mesh *GetMesh(const std::string &_name) const;

      /// \brief Return true if the mesh exists.
      /// \param[in] _name the name of the mesh
      public: bool HasMesh(const std::string &_name) const;

      /// \brief Create a sphere mesh.
      /// \param[in] _name the name of the mesh
      /// \param[in] _radius radius of the sphere in meter
      /// \param[in] _rings number of circles on th y axis
      /// \param[in] _segments number of segment per circle
      public: void CreateSphere(const std::string &_name, float _radius,
                                int _rings, int _segments);

      /// \brief Create a Box mesh
      /// \param[in] _name the name of the new mesh
      /// \param[in] _sides the x y x dimentions of eah side in meter
      /// \param[in] _uvCoords the texture coordinates
      public: void CreateBox(const std::string &_name,
                             const math::Vector3 &_sides,
                             const math::Vector2d &_uvCoords);

      /// \brief Create a Extruded Polyline mesh
      /// \param[in] _name the name of the new mesh
      /// \param[in] _vertices the x y  dimentions of eah vertex in meter
      /// \param[in] _height the height of the polyline
      /// \param[in] _uvCoords the texture coordinates
      public: void CreateExtrudedPolyline(const std::string &_name,
                  const std::vector<math::Vector2d> &_vertices,
                  const double &_height,
                  const math::Vector2d &_uvCoords);

      /// \brief Create a cylinder mesh
      /// \param[in] _name the name of the new mesh
      /// \param[in] _radius the radius of the cylinder in the x y plane
      /// \param[in] _height the height along z
      /// \param[in] _rings the number of circles along the height
      /// \param[in] _segments the number of segment per circle
      public: void CreateCylinder(const std::string &_name,
                                  float _radius,
                                  float _height,
                                  int _rings,
                                  int _segments);

      /// \brief Create a cone mesh
      /// \param[in] _name the name of the new mesh
      /// \param[in] _radius the radius of the cylinder in the x y plane
      /// \param[in] _height the height along z
      /// \param[in] _rings the number of circles along the height
      /// \param[in] _segments the number of segment per circle
      public: void CreateCone(const std::string &_name,
                              float _radius,
                              float _height,
                              int _rings,
                              int _segments);

      /// \brief Create a tube mesh.
      ///
      /// Generates rings inside and outside the cylinder
      /// Needs at least two rings and 3 segments
      /// \param[in] _name the name of the new mesh
      /// \param[in] _innerRadius the inner radius of the tube in the x y plane
      /// \param[in] _outterRadius the outer radius of the tube in the x y plane
      /// \param[in] _height the height along z
      /// \param[in] _rings the number of circles along the height
      /// \param[in] _segments the number of segment per circle
      /// \param[in] _arc the arc angle in radians
      public: void CreateTube(const std::string &_name,
                              float _innerRadius,
                              float _outterRadius,
                              float _height,
                              int _rings,
                              int _segments,
                              double _arc = 2.0 * M_PI);

      /// \brief Create mesh for a plane
      /// \param[in] _name
      /// \param[in] _plane plane parameters
      /// \param[in] _segments number of segments in x and y
      /// \param[in] _uvTile the texture tile size in x and y
      public: void CreatePlane(const std::string &_name,
                               const math::Plane &_plane,
                               const math::Vector2d &_segments,
                               const math::Vector2d &_uvTile);

      /// \brief Create mesh for a plane
      /// \param[in] _name the name of the new mesh
      /// \param[in] _normal the normal to the plane
      /// \param[in] _d distance from the origin along normal
      /// \param[in] _size the size of the plane in x and y
      /// \param[in] _segments the number of segments in x and y
      /// \param[in] _uvTile the texture tile size in x and y
      public: void CreatePlane(const std::string &_name,
                               const math::Vector3 &_normal,
                               double _d,
                               const math::Vector2d &_size,
                               const math::Vector2d &_segments,
                               const math::Vector2d &_uvTile);

      /// \brief Tesselate a 2D mesh
      ///
      /// Makes a zigzag pattern compatible with strips
      /// \param[in] _sm the mesh to tesselate
      /// \param[in] _meshWith mesh width
      /// \param[in] _meshHeight the mesh height
      /// \param[in] _doubleSided flag to specify single or double sided
      private: void Tesselate2DMesh(SubMesh *_sm,
                                    int _meshWidth,
                                    int _meshHeight,
                                    bool _doubleSided);

      /// \brief Create a Camera mesh
      /// \param[in] _name name of the new mesh
      /// \param[in] _scale scaling factor for the camera
      public: void CreateCamera(const std::string &_name, float _scale);

#ifdef HAVE_GTS
      /// \brief Create a boolean mesh from two meshes
      /// \param[in] _name the name of the new mesh
      /// \param[in] _m1 the parent mesh in the boolean operation
      /// \param[in] _m2 the child mesh in the boolean operation
      /// \param[in] _operation the boolean operation applied to the two meshes
      /// \param[in] _offset _m2's pose offset from _m1
      public: void CreateBoolean(const std::string &_name, const Mesh *_m1,
          const Mesh *_m2, const int _operation,
          const math::Pose &_offset = math::Pose::Zero);
#endif

      /// \brief 3D mesh loader for COLLADA files
      private: ColladaLoader *colladaLoader;

      /// \brief 3D mesh exporter for COLLADA files
      private: ColladaExporter *colladaExporter;

      /// \brief 3D mesh loader for STL files
      private: STLLoader *stlLoader;

      /// \brief Dictionary of meshes, indexed by name
      private: std::map<std::string, Mesh*> meshes;

      /// \brief supported file extensions for meshes
      private: std::vector<std::string> fileExtensions;

      private: boost::mutex mutex;

      /// \brief Singleton implementation
      private: friend class SingletonT<MeshManager>;
    };
    /// \}
  }
}
#endif
