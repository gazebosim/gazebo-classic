#ifndef COMBINED_HH
#define COMBINED_HH

#include <string>
#include <vector>

#include <boost/thread.hpp>

#include "common/CommonTypes.hh"
#include "physics/PhysicsTypes.hh"


namespace gazebo
{
  class Master;

  class Combined
  {
    public: Combined();
    public: virtual ~Combined();

    public: void Load(const std::string &filename);
    public: void Init();
    public: void Run();
    public: void Quit();

    public: void SetParams( const common::StrStr_M &params );

    private: std::vector< physics::WorldPtr > worlds;
    private: bool quit;

    private: Master *master;
    private: boost::thread *masterThread;
  };
}

#endif
