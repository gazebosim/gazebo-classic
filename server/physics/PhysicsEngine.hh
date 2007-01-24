#ifndef PHYSICSENGINE_HH
#define PHYSICSENGINE_HH

#include "Joint.hh"

class Entity;
class Body;


class PhysicsEngine
{
  public: PhysicsEngine();
  public: virtual ~PhysicsEngine();

  // Load the physics engine
  public: virtual int Load() = 0;

  // Initialize the physics engine
  public: virtual int Init() = 0;

  //Update the physics engine
  public: virtual int Update() = 0;

  //Finilize the physics engine
  public: virtual int Fini() = 0;

  // Add an entity
  public: virtual int AddEntity(Entity *entity) = 0;

  // Create a new body
  public: virtual Body *CreateBody(Entity *parent) = 0;

  // Create a new joint
  public: virtual Joint *CreateJoint(Joint::Type type) = 0;

};

#endif
