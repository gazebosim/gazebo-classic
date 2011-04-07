#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <iostream>

#include "gazebo_config.h"
#include "Combined.hh"

gazebo::Combined *server = NULL;

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

  server = new gazebo::Combined();
  server->Load();
  server->Init();
  server->Run();
  
  delete server;
  server = NULL;
}
