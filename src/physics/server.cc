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
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <iostream>
#include <vector>

#include "physics/Physics.hh"
#include "transport/Transport.hh"
#include "sdf/sdf.h"
#include "sdf/sdf_parser.h"
#include "common/SystemPaths.hh"
#include "gazebo_config.h"

// Command line options
std::string config_filename = "";

bool quit = false;
std::vector< gazebo::physics::WorldPtr > worlds;

////////////////////////////////////////////////////////////////////////////////
// TODO: Implement these options
void PrintUsage()
{
  fprintf(stderr, "Usage: gzphysics [-hv] <worldfile>\n");
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
    config_filename = argv[0];

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
// sighandler to shut everything down properly
void SignalHandler( int )
{
  quit = true;
}

void Load()
{
  // load the configuration options 
  try
  {
    gazebo::common::SystemPaths::Instance()->Load();
  }
  catch (gazebo::common::Exception e)
  {
    gzthrow("Error loading the Gazebo configuration file, check the .gazeborc file on your HOME directory \n" << e); 
  }

  gazebo::transport::init();
  gazebo::physics::init();


  // Load the world file
  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::initFile(config_filename, sdf);
  sdf::readFile(config_filename, sdf);

  sdf::ElementPtr worldElem = sdf->root->GetElement("world");

  while(worldElem)
  {
    gazebo::physics::WorldPtr world = gazebo::physics::create_world("default");

    worlds.push_back(world);

    //Create the world
    try
    {
      gazebo::physics::load_world(world, worldElem);
    }
    catch (gazebo::common::Exception e)
    {
      gzthrow("Failed to load the World\n"  << e);
    }

    worldElem = sdf->root->GetNextElement("world", worldElem);
  }

  for (unsigned int i=0; i < worlds.size(); i++)
    gazebo::physics::init_world(worlds[i]);
}

void Run()
{
  for (unsigned int i=0; i < worlds.size(); i++)
    gazebo::physics::run_world(worlds[i]);

  while (!quit)
  {
    usleep(1000000);
  }
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

  Load();
  Run();
  
  return 0;
}
