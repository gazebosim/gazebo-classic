#include <python2.4/Python.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include "XMLConfig.hh"
#include "ModelFactory.hh"
#include "World.hh"

// Command line options
const char *worldFileName;
const char *optLogFileName = NULL;
int optServerId = 0;
bool optServerForce = true;
double optTimeout = -1;
int optMsgLevel = 1;
int optTimeControl = 1;

// Set to true when the user wants to leave the application
bool userQuit;

////////////////////////////////////////////////////////////////////////////////
// TODO: Implement these options
void PrintUsage()
{
  fprintf(stderr, "Usage: gazebo [-hv] <worldfile>\n");
  fprintf(stderr, "  -h            : Print this message.\n");
  fprintf(stderr, "  -s <id>       : Use server id <id> (an integer); default is 0.\n");
  fprintf(stderr, "  -f            : Force usage of the server id (use with caution)\n");
  fprintf(stderr, "  -d <-1:9>      : Verbose mode: -1 = none, 0 = critical (default), 9 = all)\n");
  fprintf(stderr, "  -t <sec>      : Timeout and quit after <sec> seconds\n");
  fprintf(stderr, "  -l <logfile>  : Log to indicated file.\n");
  fprintf(stderr, "  -n            : Do not do any time control\n");
  fprintf(stderr, "  <worldfile>   : load the the indicated world file\n");
  return;
}

////////////////////////////////////////////////////////////////////////////////
// Print the version/licence string
void PrintVersion()
{
  //TODO: Fix the version number
  fprintf(stderr, "Gazebo multi-robot simulator, version %s\n\n", "0.8");
  fprintf(stderr, "Part of the Player/Stage Project "
          "[http://playerstage.sourceforge.net].\n");
  fprintf(stderr, "Copyright (C) 2003 Nate Koenig, Andrew Howard, and contributors.\n");
  fprintf(stderr, "Released under the GNU General Public License.\n\n");
  return;
}

////////////////////////////////////////////////////////////////////////////////
// Parse the argument list.  Options are placed in static variables.
int ParseArgs(int argc, char **argv)
{
  FILE *tmpFile;
  int ch;
  char *flags = "l:hd:s:fg:xt:nq";

  // Get letter options
  while ((ch = getopt(argc, argv, flags)) != -1)
  {
    switch (ch)
    {
      case 'd':
        // Verbose mode
        optMsgLevel = atoi(optarg);
        break;
 
      case 'f':
        // Force server id
        optServerForce = true;
        break;

      case 'l':
        optLogFileName = optarg;
        break;

      case 's':
        // Server id
        optServerId = atoi(optarg);
        optServerForce = false;
        break;
       
      case 't':
        // Timeout and quit after x seconds
        optTimeout = atof(optarg);
        break;
      case 'n':
        optTimeControl = 0;
      break;

      case 'h':
      default:
        PrintUsage();
        return -1;
    }
  }

  argc -= optind;
  argv += optind;

  if (argc < 1)
  {
    PrintUsage();
    return -1;
  }

  // Get the world file name
  worldFileName = argv[0];
  tmpFile = fopen(worldFileName, "r");

  if (tmpFile == NULL)
  {
    char tmpStr[256];
    sprintf(tmpStr, "WorldFile [%s]", worldFileName);

    perror(tmpStr);
    return  -1;
  }
  fclose(tmpFile);
 
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// sighandler to shut everything down properly
void SignalHandler( int /*dummy*/ ) 
{
  userQuit = true;
  return;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the sim
int Init()
{
  Py_Initialize();

  //struct tms cpu;

  // TODO: Fix the version number
  // Print banner
  printf("** Gazebo %s **\n", "0.8");
  printf("* Part of the Player/Stage Project [http://playerstage.sourceforge.net].\n");
  printf("* Copyright 2000-2005 Copyright (C) 2003 Nate Koenig, Andrew Howard, and contributors.\n");
  printf("* Released under the GNU General Public License.\n");

  // Establish signal handlers
  if (signal(SIGINT, SignalHandler) == SIG_ERR)
  {
    printf("signal(2) failed while setting up for SIGINT\n");
    return -1;
  }

  // Register static models
  ModelFactory::RegisterAll();
  
  // Load the world file
  XMLConfig *xmlFile = new XMLConfig();
  if (xmlFile->Load(worldFileName) != 0)
    return -1;

  // Load the world
  if (World::Instance()->Load(xmlFile, optServerId) != 0)
    return -1;

  // Initialize
  if (World::Instance()->Init() != 0)
    return -1;

  // Check for unused attributes
  //xmlFile->WarnUnused();

  // Our server id
  //printf("server id [%d]\n", world->server_id);

  // Record the start times for accurate CPU loads
  /*times(&cpu);
  cpuStartTime = (double) (cpu.tms_utime + cpu.tms_stime) / sysconf(_SC_CLK_TCK);    
  */
    
  return 0;
}

// Finalize the sim
int Fini()
{
  World::Instance()->Fini();

  Py_Finalize();

  return 0;
}

// Idle-time processing
bool MainIdle()
{
  // Advance the world 
  World::Instance()->Update();

  // Exit if the user has decided to end the simulation
  if (userQuit)
    return false;

  return true;
}


int main(int argc, char **argv)
{
  if (ParseArgs(argc, argv) != 0)
    return -1;

  if (Init() != 0)
  {
    fprintf(stderr,"Initialization failed\n");
    return -1;
  }

  userQuit = false;

  // Run the sim
  while (MainIdle());

  // Finalize
  Fini();
}
