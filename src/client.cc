#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <iostream>

#include "rendering/RenderEngine.hh"
#include "rendering/RenderState.hh"
#include "gazebo_config.h"
#include "transport/IOManager.hh"

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
  FILE *tmpFile;
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

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// sighandler to shut everything down properly
void SignalHandler( int /*dummy*/ )
{
  //gazebo::event::Events::quitSignal();
  return;
}

void Load()
{

  // Load the Ogre rendering system
  RenderEngine::Instance()->Load(configNode);

  // Create and initialize the Gui
  if (this->renderEngineEnabled && this->guiEnabled)
  {
    try
    {
      XMLConfigNode *childNode = NULL;
      if (rootNode)
        childNode = configNode->GetChild("gui");

      int width=0;
      int height=0;
      int x = 0;
      int y = 0;

      if (childNode)
      {
        width = childNode->GetTupleInt("size", 0, 800);
        height = childNode->GetTupleInt("size", 1, 600);
        x = childNode->GetTupleInt("pos",0,0);
        y = childNode->GetTupleInt("pos",1,0);
      }

      // Create the GUI
      if (!this->gui && (childNode || !rootNode))
      {
        this->gui = new SimulationApp();
        this->gui->Load();
      }
    }
    catch (GazeboError e)
    {
      gzthrow( "Error loading the GUI\n" << e);
    }
  }
  else
  {
    this->gui = NULL;
  }

//Initialize RenderEngine
  if (this->renderEngineEnabled)
  {
    try
    {
      RenderEngine::Instance()->Init(configNode);
    }
    catch (gazebo::GazeboError e)
    {
      gzthrow("Failed to Initialize the Rendering engine subsystem\n" << e );
    }
  }

  // Initialize the GUI
  if (this->gui)
  {
    this->gui->Init();
  }
}

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

  Scene *scene = new Scene("scene");
  this->scene->SetType(Scene::GENERIC);
  //this->scene->CreateGrid( 10, 1, 0.03, Color(1,1,1,1));
  //this->scene->Init();
}
