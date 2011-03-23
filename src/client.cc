#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <iostream>

#include "GuiClient.hh"

std::string config_file = "";
gazebo::GuiClient *client = NULL;

////////////////////////////////////////////////////////////////////////////////
// Print the version/licence string
void PrintVersion()
{
  fprintf(stderr, "Gazebo multi-robot simulator, version %s\n\n", GAZEBO_VERSION);
  fprintf(stderr, "Part of the Player/Stage Project "
          "[http://playerstage.sourceforge.net].\n");
  fprintf(stderr, "Copyright (C) 2003 Nate Koenig, Andrew Howard, and contributors.\n");
  fprintf(stderr, "Released under the GNU General Public License.\n\n");
  return;
}

////////////////////////////////////////////////////////////////////////////////
// TODO: Implement these options
void PrintUsage()
{
  fprintf(stderr, "Usage: gazebo [-hv] <worldfile>\n");
  fprintf(stderr, "  -h            : Print this message.\n");
  fprintf(stderr, "  -d <-1:9>      : Verbose mode: -1 = none, 0 = critical (default), 9 = all)\n");
  fprintf(stderr, "  -t <sec>      : Timeout and quit after <sec> seconds\n");
  fprintf(stderr, "  -g            : Run without a GUI\n");
  fprintf(stderr, "  -r            : Run without a rendering engine\n");
  fprintf(stderr, "  -l <logfile>  : Log to indicated file.\n");
  fprintf(stderr, "  -n            : Do not do any time control\n");
  fprintf(stderr, "  -p            : Run without physics engine\n");
  fprintf(stderr, "  -u            : Start the simulation paused\n");
  fprintf(stderr, "  --add_plugin  : Add a plugin to the running gazebo\n");
  fprintf(stderr, "  --remove_plugin  : Remove a plugin from the running gazebo\n");
  fprintf(stderr, "  <worldfile>   : load the the indicated world file\n");
  return;
}

////////////////////////////////////////////////////////////////////////////////
// Parse the argument list.  Options are placed in static variables.
int ParseArgs(int argc, char **argv)
{
  int ch;

  char *flags = (char*)("l:hd:gxt:nqperu");

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
void SignalHandler( int /*dummy*/ )
{
  client->Quit();
  return;
}

/*
void Init()
{
  RenderState::Init();
}

void Fini()
{
  RenderEngine::Instance()->Fini();
}

void Save()
{
  // NATY: make save work again
  //this->GetRenderEngine()->Save(prefix, output);
}

void Update()
{
  RenderEngine::Instance()->UpdateScenes();
}
*/

////////////////////////////////////////////////////////////////////////////////
// Main function
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

  client = new gazebo::GuiClient();
  client->Load(config_file);
  client->Run();

  delete client;
  client = NULL;

  return 0;
}
