#ifndef COMBINED_HH
#define COMBINED_HH

#include <string>
#include <vector>

#include "physics/PhysicsTypes.hh"


namespace gazebo
{
  class Combined
  {
    public: Combined();
    public: virtual ~Combined();

    public: void Load();
    public: void Init();
    public: void Run();
    public: void Quit();

    private: std::vector< physics::WorldPtr > worlds;
    private: bool quit;
  };
}

#endif
