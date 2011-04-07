#include <signal.h>
#include <iostream>
#include "SensorServer.hh"

gazebo::SensorServer *server;

////////////////////////////////////////////////////////////////////////////////
// sighandler to shut everything down properly
void SignalHandler( int )
{
  server->Quit();
}

int main(int argc, char **argv)
{
  if (signal(SIGINT, SignalHandler) == SIG_ERR)
  {
    std::cerr << "signal(2) failed while setting up for SIGINT" << std::endl;
    return -1;
  }

   server = new gazebo::SensorServer();

  server->Load("");
  server->Run();

  delete server;
  return 0;
}
