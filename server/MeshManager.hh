#ifndef MESHMANAGER_HH
#define MESHMANAGER_HH

#include <map>
#include <string>

#include "Vector3.hh"
#include "Vector2.hh"
#include "SingletonT.hh"

namespace gazebo
{
  class OgreLoader;
  class AssimpLoader;
  class Mesh;
  
  class MeshManager : public SingletonT<MeshManager>
  {
    /// \brief Constructor
    private: MeshManager();
  
    /// \brief Destructor
    private: virtual ~MeshManager();

    /// \brief Load a mesh from a file
    public: const Mesh *Load(const std::string &filename);

    /// \brief Get a mesh by name
    public: const Mesh *GetMesh(const std::string &name) const;

    /// \brief Return true if the mesh exists
    public: bool HasMesh(const std::string &name) const;

    /// \brief Create a sphere mesh
    public: void CreateSphere(const std::string &name, float radius, 
                              int rings, int segments);
  
    /// \brief Create a Box mesh
    public: void CreateBox(const std::string &name, const Vector3 &sides,
                           const Vector2<double> &uvCoords);
  
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
 
    private: OgreLoader *ogreLoader;
    private: AssimpLoader *assimpLoader;

    private: std::map<std::string, Mesh*> meshes;

    //Singleton implementation
    private: friend class DestroyerT<MeshManager>;
    private: friend class SingletonT<MeshManager>;
  };
}
#endif
