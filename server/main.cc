/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Desc: External interfaces for Gazebo
 * Author: Nate Koenig
 * Date: 3 Apr 2007
 * SVN: $Id$
 */

/** @page gazebo_server Console-mode server (gazebo)

The basic gazebo server is a console-mode application: it creates no
windows and accepts no user input.  The console-mode server is useful
for running automated tests and batch experiments.

The server is run as follows:

@verbatim
$ gazebo [options] <worldfile>
@endverbatim

where [options] is one or more of the following:

- -h            : Print usage message.
- -s &lt;id&gt;       : Use server id &lt;id&gt; (an integer); default is 0
- -f                  : Force usage of the server id (use with caution)
- -d &lt;level&gt;    : Verbose mode: -1 = none, 0 = critical messages (default), 9 = all messages
- -t &lt;sec&gt;      : Timeout and quit after &lt;sec&gt; seconds
- -l &lt;logfile&gt;  : Log messages to &lt;logfile&gt 
- -n                  : Do not do any time control

The server prints some diagnostic information to the console before
starting the main simulation loop.  Check carefully for any warnings
that are printed at this stage; common warnings include:

- Invalid tags or attributes in the world file, which yield "unused
  attribute" warnings:
@verbatim
warning : in worlds/pioneer2at.world:14 unused attribute [xuz]
@endverbatim
  To remove these warnings, fix the world file.

- Left-over files from an earlier instance of the server (that
  crashed).
@verbatim
gz_server.c:111 directory [/tmp/gazebo-ahoward-0] already exists
@endverbatim
  To remove this warning, delete the indicated directory.

While the simulation loop is running, basic status information is
printed on the console:
  
@verbatim
Time 1.542 1.560 0.000 [1.000] 0.220 [ 14%]
@endverbatim

These five fields specify, in order:

- The elapsed real time (sec).

- The elapsed simulation time (sec).

- The accumulated pause time (sec).

- The ratio of elapsed simulation to elaped real time; i.e., the
effective speed of the simulator.  This should hover around target
speed as specified in the world file, but may fall below this value if
your processor is too slow.

- The total CPU time used by the server; useful for performance
monitoring.

- The CPU utilization, measured as ratio of total CPU time to elapsed
real time.  Note that this measures the utilization by the server process
only; it does not measure the processor utilization of the X server,
which may be significant.

The server can be terminated with @c control-C.  Errors, warnings and
messages are appended by default to a file called @c .gazebo located in your
home directory, or to the log file specified with the -l command line option.
*/

//#include <python2.4/Python.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <iostream>
#include <FL/Fl.H>
#include <boost/thread/thread.hpp>

#include "Global.hh"
#include "Gui.hh"
#include "XMLConfig.hh"
#include "SensorFactory.hh"
#include "IfaceFactory.hh"
#include "ControllerFactory.hh"
#include "GuiFactory.hh"
#include "World.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "OgreAdaptor.hh"

// Command line options
const char *worldFileName;
const char *optLogFileName = NULL;
int optServerId = 0;
bool optServerForce = true;
double optTimeout = -1;
int optMsgLevel = 1;
int optTimeControl = 1;

boost::mutex mutex;

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
  char *flags = (char*)("l:hd:s:fg:xt:nq");

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
  gazebo::Global::userQuit = true;
  return;
}

void LoadConfigFile()
{
  std::ifstream cfgFile;

  std::string rcFilename = getenv("HOME");
  rcFilename += "/.gazeborc";

  cfgFile.open(rcFilename.c_str(), std::ios::in);

  if (cfgFile)
  {
    gazebo::XMLConfig rc;
    gazebo::XMLConfigNode *node;
    rc.Load(rcFilename);

    node = rc.GetRootNode()->GetChild("gazeboPath");
    while (node)
    {
      std::cout << "Gazebo Path[" << node->GetValue() << "]\n";
      gazebo::Global::gazeboPaths.push_back(node->GetValue());
      node = node->GetNext("gazeboPath");
    }

    node = rc.GetRootNode()->GetChild("ogrePath");
    while (node)
    {
      std::cout << "Ogre Path[" << node->GetValue() << "]\n";
      gazebo::Global::ogrePaths.push_back( node->GetValue() );
      node = node->GetNext("ogrePath");
    }

  }
  else
  {
    std::cout << "Unable to file ~/.gazeborc. Using default paths. This may cause OGRE to fail.\n";
    gazebo::Global::gazeboPaths.push_back("/usr/local/share/gazebo");
    gazebo::Global::ogrePaths.push_back("/usr/local/lib/OGRE");
    gazebo::Global::ogrePaths.push_back("/usr/lib/OGRE");
  }


}

////////////////////////////////////////////////////////////////////////////////
// Initialize the sim
int Init()
{
  gazebo::XMLConfigNode *childNode = NULL;

  //Py_Initialize();

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

  // Load the world file
  gazebo::XMLConfig *xmlFile(new gazebo::XMLConfig());

  if (xmlFile->Load(worldFileName) != 0)
    return -1;

  // Load the messaging system
  gazebo::GazeboMessage::Instance()->Load(xmlFile->GetRootNode());

  // Register all the factories
  gazebo::SensorFactory::RegisterAll();
  gazebo::IfaceFactory::RegisterAll();
  gazebo::ControllerFactory::RegisterAll();
  gazebo::GuiFactory::RegisterAll();

  childNode = xmlFile->GetRootNode()->GetChild("gui");

  if (childNode)
  {
    int width = childNode->GetTupleInt("size",0,640);
    int height = childNode->GetTupleInt("size",1,480);
    int x = childNode->GetTupleInt("pos",0,0);
    int y = childNode->GetTupleInt("pos",1,0);
    std::string type = childNode->GetString("type","fltk",1);

    gzmsg(1) << "Creating GUI:\n\tType[" << type << "] Pos[" << x << " " << y << "] Size[" << width << " " << height << "]\n";

    // Create the GUI
    gazebo::Global::gui = gazebo::GuiFactory::NewGui(type, x, y, width, height, type+"::Gazebo");

    // Initialize the GUI
    gazebo::Global::gui->Init();
  }
  else
  {
    gzthrow("XML file must contain a <rendering:gui> section\n");
  }

  LoadConfigFile();

  // Initialize Ogre
  try
  {
    gazebo::OgreAdaptor::Instance()->Init(xmlFile->GetRootNode()->GetChild("ogre"));
  }
  catch (gazebo::GazeboError e)
  {
    std::ostringstream stream;
    stream << "Failed to Initialize the OGRE Rendering system\n" 
              << e << "\n";
    gzthrow(stream.str());
  }

  // Load the world
  gazebo::World::Instance()->Load(xmlFile, optServerId);

  // Initialize the world
  if (gazebo::World::Instance()->Init() != 0)
    return -1;

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the sim
int Fini()
{
  gazebo::World::Instance()->Fini();

  //Py_Finalize();

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Idle-time processing
void MainCallback()
{
  boost::mutex::scoped_lock lock(mutex);

  // Advance the world 
  gazebo::World::Instance()->Update();
}

////////////////////////////////////////////////////////////////////////////////
// Run the FLTK main loop
void FLTKLoop()
{
  Fl::run();
}

////////////////////////////////////////////////////////////////////////////////
// Main function
int main(int argc, char **argv)
{
  if (ParseArgs(argc, argv) != 0)
    return -1;

  try
  {
    if (Init() != 0)
    {
      fprintf(stderr,"Initialization failed\n");
      return -1;
    }
  }
  catch (gazebo::GazeboError e)
  {
    std::cerr << e << std::endl;
    return -1;
  }

  // Run FLTK main loop in a separate thread
  boost::thread fltkThread(FLTKLoop);

  //Fl::add_idle(MainCallback);
  //Fl::run();

  try
  {
    while (!gazebo::Global::userQuit)
    {
      MainCallback();
      usleep(1000);
    }
  }
  catch (gazebo::GazeboError e)
  {
    std::cerr << "MainIdle Failed[" << e << "]\n";
    return -1;
  }
  

  try
  {
    // Finalize
    Fini();
  }
  catch (gazebo::GazeboError e)
  {
    std::cerr << "Finalized Failed[" << e << "]\n";
    return -1;
  }

  //fltkThread.join();

  return 0;
}
