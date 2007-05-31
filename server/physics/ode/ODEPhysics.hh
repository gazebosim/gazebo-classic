#ifndef ODEPHYSICS_HH
#define ODEPHYSICS_HH

#include <ode/ode.h>
#include <map>

#include "PhysicsEngine.hh"


namespace gazebo
{
  class SphereGeom;
  class PlaneGeom;
  class BoxGeom;
  class CylinderGeom;
  //class Body;
  class Entity;

class ODEPhysics : public PhysicsEngine
{
  /// Constructor
  public: ODEPhysics();

  /// Destructor
  public: virtual ~ODEPhysics();

  /// Load the ODE engine
  public: virtual int Load();

  /// Initialize the ODE engine
  public: virtual int Init();

  /// Update the ODE engine
  public: virtual int Update();

  /// Finilize the ODE engine
  public: virtual int Fini();

  /// Add an entity
  public: virtual int AddEntity(Entity *entity);

  /// Create a new body
  public: virtual Body *CreateBody(Entity *parent);

  /// Create a new joint
  public: virtual Joint *CreateJoint(Joint::Type type);

  /// Return the space id 
  public: dSpaceID GetSpaceId() const;

  /// Do collision detection
  private: static void CollisionCallback( void *data, dGeomID o1, dGeomID o2);

  /// Top-level world for all bodies
  private: dWorldID worldId;

  /// Top-level space for all sub-spaces/geoms
  private: dSpaceID spaceId;

  /// Collision attributes
  private: dJointGroupID contactGroup;
           
  protected: std::map<int, Entity* > entities;
};

}

#endif
