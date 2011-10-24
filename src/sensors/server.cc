#include <signal.h>
#include <iostream>
#include "common/SystemPaths.hh"
#include "common/Time.hh"
#include "transport/Transport.hh"
#include "sensors/Sensors.hh"
#include "rendering/Rendering.hh"
#include "rendering/RenderEngine.hh"

#include "gazebo.h"

bool quit = false;

////////////////////////////////////////////////////////////////////////////////
// sighandler to shut everything down properly
void SignalHandler( int )
{
  quit = true;
}

void Load()
{
  gazebo::load();

  /// Init the sensors library
  gazebo::sensors::load();
  gazebo::sensors::init();

  gazebo::rendering::create_scene("world_1");
  gazebo::common::Time::MSleep(10);
}

void Run()
{
  gazebo::run();
  while (!quit)
  {
    gazebo::sensors::run_once(true);
    gazebo::common::Time::MSleep(10);
  }
}

int main(int /*argc*/, char ** /*argv*/)
{
  if (signal(SIGINT, SignalHandler) == SIG_ERR)
  {
    std::cerr << "signal(2) failed while setting up for SIGINT" << std::endl;
    return -1;
  }

  Load();
  Run();

  return 0;
}
