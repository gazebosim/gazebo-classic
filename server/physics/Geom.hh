#ifndef GEOM_HH
#define GEOM_HH

#include <ode/ode.h>

#include "Entity.hh"
#include "Pose3d.hh"
#include "Vector3.hh"

namespace Ogre
{
  class Entity;
  class MaterialPtr;
}

namespace gazebo
{

  class Body;
  class ContactParams;


class Geom : public Entity
{
  public: Geom(Body *body);
  public: virtual ~Geom();

  // Set the encapsulated geometry object
  public: void SetGeom(dGeomID geomId, bool placeable);

  // Return the geom id
  public: dGeomID GetGeomId() const;

  // Return whether this geom is placeable
  public: bool IsPlaceable() const;

  /// Set the pose
  /// \param pose New pose
  public: void SetPose(const Pose3d &pose);

  /// Return the pose of the geom
  public: Pose3d GetPose() const;

  /// Set the position
  /// \param pos Vector3 position
  public: void SetPosition(const Vector3 &pos);

  /// Set the rotation
  /// \param rot Quaternion rotation
  public: void SetRotation(const Quatern &rot);

  /// Attach a mesh to the geom
  /// \param meshName Name of the mesh
  public: void AttachMesh(const std::string &meshName);

  /// Set the scale of the mesh
  /// \param scale New scale of the mesh
  public: void ScaleMesh(const Vector3 &scale);

  /// Set the mesh pose
  /// \param pose Pose of the mesh.
  public: void SetMeshPose(const Pose3d &pose);

  /// Set the mesh position
  /// \param pos Position of the mesh
  public: void SetMeshPosition(const Vector3 &pos);

  /// Set whether the mesh casts shadows
  /// \param enable True=cast shadows
  public: void SetCastShadows(bool enable);

  /// Set the material to apply to the mesh
  public: void SetMeshMaterial(const std::string &materialName);

  /// Contact parameters
  public: ContactParams *contact; 

  /// The body this geom belongs to
  protected: Body *body;

  private: bool placeable;

  private: Ogre::Entity *meshEntity;

  protected: dGeomID geomId;

  private: static int geomIdCounter;

  // Mass of this geometry
  protected: dMass mass;
};

}
#endif
