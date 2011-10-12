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
#include <stdio.h>
#include <signal.h>
#include <string>

#include "transport/Transport.hh"
#include "gui/Gui.hh"
#include "gazebo.h"

std::string config_file = "";
std::vector<std::string> plugins;


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
  int ch;

  char *flags = (char*)("hp:");

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
  gazebo::stop();
  gazebo::gui::stop();
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

  gazebo::load();
  gazebo::init();
  gazebo::run();

  gazebo::gui::load();

  /// Load all the plugins specified on the command line
  for (std::vector<std::string>::iterator iter = plugins.begin(); 
       iter != plugins.end(); iter++)
  {
    gazebo::gui::load_plugin(*iter);
  }

  gazebo::gui::init();
  gazebo::gui::run();

  gazebo::fini();
  gazebo::gui::fini();

  return 0;
}
