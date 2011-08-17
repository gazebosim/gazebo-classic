#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <iostream>
#include <valgrind/callgrind.h>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>

#include "gazebo_config.h"
#include "common/CommonTypes.hh"
#include "rendering/Rendering.hh"
#include "common/SystemPaths.hh"
#include "Server.hh"

gazebo::Server *server = NULL;

std::string config_filename = "";
gazebo::common::StrStr_M params;

boost::interprocess::interprocess_semaphore sem(0);

////////////////////////////////////////////////////////////////////////////////
// TODO: Implement these options
void PrintUsage()
{
  std::cerr << "Usage: gzserver\n";
}

////////////////////////////////////////////////////////////////////////////////
// Print the version/licence string
void PrintVersion()
{
  fprintf(stderr, "%s", GAZEBO_VERSION_HEADER);
}

////////////////////////////////////////////////////////////////////////////////
// Parse the argument list.  Options are placed in static variables.
int ParseArgs(int argc, char **argv)
{
  //FILE *tmpFile;
  int ch;

  char *flags = (char*)("u");
  // Get letter options
  while ((ch = getopt(argc, argv, flags)) != -1)
  {
    switch (ch)
    {
      case 'u':
        params["pause"] = "true";
        break;
      default:
        PrintUsage();
        return -1;
    }
  }

  argc -= optind;
  argv += optind;

  // Get the world file name
  if (argc >= 1)
    config_filename = argv[0];

  return 0;
}


////////////////////////////////////////////////////////////////////////////////
// sighandler to shut everything down properly
void SignalHandler( int )
{
  server->Stop();
}

int main(int argc, char **argv)
{
  CALLGRIND_STOP_INSTRUMENTATION;

  //Application Setup
  if (ParseArgs(argc, argv) != 0)
    return -1;

  PrintVersion();

  if (signal(SIGINT, SignalHandler) == SIG_ERR)
  {
    std::cerr << "signal(2) failed while setting up for SIGINT" << std::endl;
    return -1;
  }

  server = new gazebo::Server();
  server->Load(config_filename);
  server->SetParams( params );
  server->Init();
  CALLGRIND_DUMP_STATS;
  CALLGRIND_ZERO_STATS;
  CALLGRIND_START_INSTRUMENTATION;

  printf("RUNNING!\n");
  server->Run();

  CALLGRIND_DUMP_STATS;
  CALLGRIND_ZERO_STATS;
  CALLGRIND_STOP_INSTRUMENTATION;


  server->Fini();

  delete server;
  server = NULL;
  return 0;
}
