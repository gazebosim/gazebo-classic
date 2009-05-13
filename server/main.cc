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
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <iostream>

#include "config.h"
#include "Simulator.hh"
#include "GazeboError.hh"
#include "Global.hh"

// Command line options
const char *worldFileName;
const char *optLogFileName = NULL;
unsigned int optServerId = 0;
bool optServerForce = true;
bool optGuiEnabled = true;
bool optRenderEngineEnabled = true;
double optTimeout = -1;
unsigned int optMsgLevel = 1;
int optTimeControl = 1;
bool optPhysicsEnabled  = true;
bool optPaused = false;

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
  fprintf(stderr, "  -g            : Run without a GUI\n");
  fprintf(stderr, "  -r            : Run without a rendering engine\n");
  fprintf(stderr, "  -l <logfile>  : Log to indicated file.\n");
  fprintf(stderr, "  -n            : Do not do any time control\n");
  fprintf(stderr,"   -p            : Run without physics engine\n");
  fprintf(stderr, "  <worldfile>   : load the the indicated world file\n");
  return;
}

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
// Parse the argument list.  Options are placed in static variables.
int ParseArgs(int argc, char **argv)
{
  FILE *tmpFile;
  int ch;

  char *flags = (char*)("l:hd:s:fgxt:nqper");

  // Get letter options
  while ((ch = getopt(argc, argv, flags)) != -1)
  {
    switch (ch)
    {
      case 'e':
        std::cout << "Running in federation mode!\n";
        optPaused = true;
        break;

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

      case 'g':
        optGuiEnabled = false;
        break;

      case 'r':
        optRenderEngineEnabled = false;
        break;

      case 'p':
        optPhysicsEnabled = false;
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
  //TODO: use a boost::signal
  gazebo::Simulator::Instance()->SetUserQuit();
  return;
}

////////////////////////////////////////////////////////////////////////////////
// Main function
int main(int argc, char **argv)
{


  // force a cpu affinity for CPU 0, this slow down sim by about 4X
  // cpu_set_t cpuSet;
  // CPU_ZERO(&cpuSet);
  // CPU_SET(0, &cpuSet);
  // sched_setaffinity( 0, sizeof(cpuSet), &cpuSet);


  //Application Setup
  if (ParseArgs(argc, argv) != 0)
    return -1;

  PrintVersion();

  if (signal(SIGINT, SignalHandler) == SIG_ERR)
  {
    std::cerr << "signal(2) failed while setting up for SIGINT" << std::endl;
    return -1;
  }

  gazebo::Simulator::Instance()->SetGuiEnabled( optGuiEnabled );
  gazebo::Simulator::Instance()->SetRenderEngineEnabled( optRenderEngineEnabled );

  //Load the simulator
  try
  {
    gazebo::Simulator::Instance()->Load(worldFileName, optServerId);
    gazebo::Simulator::Instance()->SetTimeout(optTimeout);
    gazebo::Simulator::Instance()->SetPhysicsEnabled(optPhysicsEnabled);
    gazebo::Simulator::Instance()->SetPaused(optPaused);
  }
  catch (gazebo::GazeboError e)
  {
    std::cerr << "Error Loading Gazebo" << std::endl;
    std::cerr << e << std::endl;
    gazebo::Simulator::Instance()->Fini();
    return -1;
  }

  // Initialize the simulator
  try
  {
    gazebo::Simulator::Instance()->Init();
  }
  catch (gazebo::GazeboError e)
  {
    std::cerr << "Initialization failed" << std::endl;
    std::cerr << e << std::endl;
    gazebo::Simulator::Instance()->Fini();
    return -1;
  }

  // Main loop of the simulator
  try
  {
    gazebo::Simulator::Instance()->MainLoop();
  }
  catch (gazebo::GazeboError e)
  {
    std::cerr << "Main simulation loop failed" << std::endl;
    std::cerr << e << std::endl;
    gazebo::Simulator::Instance()->Fini();
    return -1;
  }

  // Finalization and clean up
  try
  {
    gazebo::Simulator::Instance()->Fini();
  }
  catch (gazebo::GazeboError e)
  {
    std::cerr << "Finalization failed" << std::endl;
    std::cerr << e << std::endl;
    return -1;
  }

  printf("Done.\n");
  return 0;
}
