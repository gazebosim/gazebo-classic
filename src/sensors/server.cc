#include <signal.h>
#include <iostream>
#include "common/SystemPaths.hh"
#include "transport/Transport.hh"
#include "sensors/Sensors.hh"
#include "rendering/Rendering.hh"

bool quit = false;

////////////////////////////////////////////////////////////////////////////////
// sighandler to shut everything down properly
void SignalHandler( int )
{
  quit = true;
}

void Load(const std::string &/*filename*/)
{
  // load the configuration options 
  try
  {
    gazebo::common::SystemPaths::Instance()->Load();
  }
  catch (gazebo::common::Exception e)
  {
    gzthrow("Error loading the Gazebo configuration file, check the .gazeborc file on your HOME directory \n" << e); 
  }

  // Start the transport system by connecting to the master.
  gazebo::transport::init();

  /// Init the sensors library
  gazebo::sensors::init("default");

  // Load the rendering system
  if (!gazebo::rendering::load())
    gzthrow("Unable to load the rendering engine");

  // The rendering engine will run headless 
  if (!gazebo::rendering::init())
    gzthrow("Unable to intialize the rendering engine");

  gazebo::rendering::create_scene("default");
}

void Run()
{
  while (!quit)
  {
    gazebo::sensors::run_once(true);
  }
}

int main(int /*argc*/, char ** /*argv*/)
{
  if (signal(SIGINT, SignalHandler) == SIG_ERR)
  {
    std::cerr << "signal(2) failed while setting up for SIGINT" << std::endl;
    return -1;
  }

  Load("");
  Run();

  return 0;
}
