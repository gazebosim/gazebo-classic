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
#ifndef MESHMANAGER_HH
#define MESHMANAGER_HH

#include <map>
#include <string>
#include <vector>

#include "common/Vector3.hh"
#include "common/Vector2d.hh"
#include "common/SingletonT.hh"

namespace gazebo
{
	namespace common
{
  class OgreLoader;
  class AssimpLoader;
  class STLLoader;
  class Mesh;
  class Plane;
  class SubMesh;
  
  class MeshManager : public SingletonT<MeshManager>
  {
    /// \brief Constructor
    private: MeshManager();
  
    /// \brief Destructor
    private: virtual ~MeshManager();

    /// \brief Load a mesh from a file
    public: const Mesh *Load(const std::string &filename);

    /// \brief Return true if the file extension is loadable
    public: bool IsValidFilename(const std::string &filename);

    /// \bridf set mesh center to be aabb center
    public: void SetMeshCenter(const Mesh *mesh,Vector3 center);

    /// \bridf get mesh aabb and center
    public: void GetMeshAABB(const Mesh *mesh,Vector3 &center, 
                             Vector3 &min_xyz, Vector3 &max_xyz);

    /// \brief generate spherical texture coordinates
    public: void GenSphericalTexCoord(const Mesh *mesh,Vector3 center);


    /// \brief Add a mesh to the manager
    public: void AddMesh(Mesh *mesh);

    /// \brief Get a mesh by name
    public: const Mesh *GetMesh(const std::string &name) const;

    /// \brief Return true if the mesh exists
    public: bool HasMesh(const std::string &name) const;

    /// \brief Create a sphere mesh
    public: void CreateSphere(const std::string &name, float radius, 
                              int rings, int segments);
  
    /// \brief Create a Box mesh
    public: void CreateBox(const std::string &name, const Vector3 &sides,
                           const Vector2d &uvCoords);
  
    /// \brief Create a cylinder mesh
    public: void CreateCylinder(const std::string &name, float radius, 
                                float height, int rings, int segments);
  
    /// \brief Create a cone mesh
    public: void CreateCone(const std::string &name, float radius, 
                            float height, int rings, int segments);
  
    /// \brief Create a tube mesh
    public: void CreateTube(const std::string &name, float innerRadius, 
                            float outterRadius, float height, int rings, 
                            int segments);

    public: void CreatePlane(const std::string &name, const Plane &plane,
                             const Vector2d &segments, 
                             const Vector2d uvTile);

    public: void CreatePlane(const std::string &name, const Vector3 &normal, 
                             double d, const Vector2d &size, 
                             const Vector2d &segments,
                             const Vector2d uvTile);

    private: void Tesselate2DMesh(SubMesh *sm, int meshWidth, int meshHeight,
                                  bool doubleSided);

    /// \brief Create a Camera mesh
    public: void CreateCamera(const std::string &name, float scale);
 
    private: OgreLoader *ogreLoader;
    private: AssimpLoader *assimpLoader;
    private: STLLoader *stlLoader;

    private: std::map<std::string, Mesh*> meshes;

    private: std::vector<std::string> fileExtensions;

    //Singleton implementation
    private: friend class SingletonT<MeshManager>;
  };
}
}
#endif
