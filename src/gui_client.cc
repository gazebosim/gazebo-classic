#include <stdio.h>
#include <signal.h>

#include "GuiClient.hh"
#include "gazebo_config.h"

std::string config_file = "";
gazebo::GuiClient *client = NULL;

int g_argc;
char **g_argv;

////////////////////////////////////////////////////////////////////////////////
// Print the version/licence string
void PrintVersion()
{
  fprintf(stderr, "%s", GAZEBO_VERSION_HEADER);
}

////////////////////////////////////////////////////////////////////////////////
// TODO: Implement these options
void PrintUsage()
{
  fprintf(stderr, "Usage: gzclient [-h] <config_file>\n");
  fprintf(stderr, "  -h            : Print this message.\n");
  fprintf(stderr, "  <config_file> : Load a GUI configuration file\n");
}

////////////////////////////////////////////////////////////////////////////////
// Parse the argument list.  Options are placed in static variables.
int ParseArgs(int argc, char **argv)
{
  g_argc = argc;
  g_argv = argv;

  int ch;

  char *flags = (char*)("h");

  // Get letter options
  while ((ch = getopt(argc, argv, flags)) != -1)
  {
    switch (ch)
    {
      case 'h':
      default:
        PrintUsage();
        return -1;
    }
  }

  argc -= optind;
  argv += optind;

  // Get the world file name
  if (argc >= 1)
    config_file = argv[0];

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// sighandler to shut everything down properly
void SignalHandler( int )
{
  client->Quit();
  return;
}

////////////////////////////////////////////////////////////////////////////////
// Main function
int main(int argc, char **argv)
{
  if (ParseArgs(argc, argv) != 0)
    return -1;

  PrintVersion();

  if (signal(SIGINT, SignalHandler) == SIG_ERR)
  {
    std::cerr << "signal(2) failed while setting up for SIGINT" << std::endl;
    return -1;
  }

  client = new gazebo::GuiClient();
  client->Load(config_file);
  client->Init();
  client->Run();

  client->Quit();
  delete client;
  client = NULL;

  return 0;
}
