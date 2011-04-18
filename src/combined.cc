#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <iostream>

#include "gazebo_config.h"
#include "common/CommonTypes.hh"
#include "Combined.hh"

gazebo::Combined *combined = NULL;

std::string config_filename = "";
gazebo::common::StrStr_M params;

////////////////////////////////////////////////////////////////////////////////
// TODO: Implement these options
void PrintUsage()
{
  std::cerr << "Usage: gzcombined\n";
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
  FILE *tmpFile;
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
  combined->Quit();
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

  combined = new gazebo::Combined();
  combined->Load(config_filename);
  combined->SetParams( params );
  combined->Init();
  combined->Run();
  
  delete combined;
  combined = NULL;
}
