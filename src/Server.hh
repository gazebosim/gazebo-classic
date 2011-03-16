#ifndef SERVER_HH
#define SERVER_HH

#include <string>
#include <vector>

namespace gazebo
{
  namespace physics
  {
    class World;
  }

  class Server
  {
    public: Server();
    public: virtual ~Server();

    public: void Load( const std::string &filename );
    public: void Init();
    public: void Run();

    private: std::vector< physics::World* > worlds;
  };
}
#endif
