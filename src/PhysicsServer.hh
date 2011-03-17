#ifndef PHYSICSSERVER_HH
#define PHYSICSSERVER_HH

#include <string>
#include <vector>

namespace gazebo
{
  namespace physics
  {
    class World;
  }
  namespace transport
  {
    class Server;
  }

  class PhysicsServer
  {
    public: PhysicsServer();
    public: virtual ~PhysicsServer();

    public: void Load( const std::string &filename );
    public: void Init();
    public: void Run();

    private: std::vector< physics::World* > worlds;
    private: transport::Server *server;
  };
}
#endif
