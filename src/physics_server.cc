/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
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

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <iostream>

#include "gazebo_config.h"
#include "PhysicsServer.hh"

// Command line options
std::string config_filename = "";
const char *optLogFileName = NULL;
bool optGuiEnabled = true;
bool optRenderEngineEnabled = true;
double optTimeout = -1;
unsigned int optMsgLevel = 1;
int optTimeControl = 1;
bool optPhysicsEnabled  = true;
bool optPaused = false;

gazebo::PhysicsServer *server = NULL;

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

  char *flags = (char*)("l:hd:gxt:nqperu");

  // Get letter options
  while ((ch = getopt(argc, argv, flags)) != -1)
  {
    switch (ch)
    {
      case 'u':
        optPaused = true;
        break;

      case 'd':
        // Verbose mode
        optMsgLevel = atoi(optarg);
        break;

      case 'l':
        optLogFileName = optarg;
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

  // Get the world file name
  if (argc >= 1)
    config_filename = argv[0];

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// sighandler to shut everything down properly
void SignalHandler( int )
{
  server->Quit();
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

  server = new gazebo::PhysicsServer();
  server->Load(config_filename);
  server->Init();
  server->Run();
  
  delete server;
  server = NULL;

  return 0;
}
