#ifndef WORLD_HH
#define WORLD_HH

#include <vector>


// Forward declarations
class PhysicsEngine;
class Model;
class XMLConfigNode;
class XMLConfig;
class Server;
class SimIface;

class World
{
  // Private constructor
  private: World();
  // Private destructor
  private: ~World();

  //Get an instance of this World
  public: static World *Instance();

  // Load the world
  public: int Load(XMLConfig *node, int serverId);

  // Initialize the world
  public: int Init();

  // Update the world
  public: int Update();

  // Finilize the world
  public: int Fini();

  // Retun the libgazebo server
  public: Server *GetGzServer() const;

  // Return the physics engine
  public: PhysicsEngine *GetPhysicsEngine() const;

  // Load all models
  private: int LoadModel(XMLConfigNode *node, Model *parent);

  // Pointer to myself
  private: static World *myself;

  // Pointer the physics engine
  private: PhysicsEngine *physicsEngine;

  // List of all the models
  private: std::vector<Model*> models;

  // Simulator control interface
  private: Server *server;
  private: SimIface *simIface;

  // Flag se if simulation is paused
  private: bool pause;
};

#endif
