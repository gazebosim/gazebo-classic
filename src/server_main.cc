#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <iostream>
#include <boost/interprocess/sync/interprocess_semaphore.hpp>

#include "gazebo_config.h"
#include "common/CommonTypes.hh"
#include "rendering/Rendering.hh"
#include "common/SystemPaths.hh"
#include "Server.hh"

gazebo::Server *server = NULL;

std::string config_filename = "";
gazebo::common::StrStr_M params;
std::vector<std::string> plugins;

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

  char *flags = (char*)("up:");
  // Get letter options
  while ((ch = getopt(argc, argv, flags)) != -1)
  {
    switch (ch)
    {
      case 'p':
        {
          if (optarg != NULL)
            plugins.push_back( std::string(optarg) );
          else
            gzerr << "Missing plugin filename with -p argument\n";
          break;
        }

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
  if (config_filename.empty())
  {
    printf("Warning: no world filename specified, using default world\n");
    config_filename = "worlds/empty.world";
  }

  // Construct plugins
  /// Load all the plugins specified on the command line
  for (std::vector<std::string>::iterator iter = plugins.begin(); 
       iter != plugins.end(); iter++)
  {
    server->LoadPlugin(*iter);
  }

  if (!server->Load(config_filename))
  {
    gzerr << "Could not open file[" << config_filename << "]\n";
    return -1;
  }

  server->SetParams( params );
  server->Init();

  server->Run();

  server->Fini();

  delete server;
  server = NULL;

  printf("\n");  
  return 0;
}
