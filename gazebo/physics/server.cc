/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include <sdf/sdf.hh>

#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/transport/TransportIface.hh"

#include "gazebo/gazebo_config.h"

// Command line options
std::string config_filename = "";

bool quit = false;

//////////////////////////////////////////////////
void PrintUsage()
{
  fprintf(stderr, "Usage: gzphysics [-hv] <worldfile>\n");
  return;
}

//////////////////////////////////////////////////
void PrintVersion()
{
  fprintf(stderr, "%s", GAZEBO_VERSION_HEADER);
}

//////////////////////////////////////////////////
int ParseArgs(int _argc, char **_argv)
{
  // FILE *tmpFile;
  int ch;

  char *flags = const_cast<char*>("h");

  // Get letter options
  while ((ch = getopt(_argc, _argv, flags)) != -1)
  {
    switch (ch)
    {
      case 'h':
      default:
        PrintUsage();
        return -1;
    }
  }

  _argc -= optind;
  _argv += optind;

  // Get the world file name
  if (_argc >= 1)
    config_filename = _argv[0];

  return 0;
}

//////////////////////////////////////////////////
void SignalHandler(int)
{
  quit = true;
}

void Load()
{
  gazebo::transport::init();
  gazebo::physics::load();


  // Load the world file
  sdf::SDFPtr sdf(new sdf::SDF());
  sdf::initFile(config_filename, sdf);
  sdf::readFile(config_filename, sdf);

  sdf::ElementPtr worldElem = sdf->root->GetElement("world");

  while (worldElem)
  {
    gazebo::physics::WorldPtr world =
      gazebo::physics::create_world(worldElem->Get<std::string>("name"));

    // Create the world
    try
    {
      gazebo::physics::load_world(world, worldElem);
    }
    catch(gazebo::common::Exception &e)
    {
      gzthrow("Failed to load the World\n"  << e);
    }

    worldElem = worldElem->GetNextElement("world");
  }

  gazebo::physics::init_worlds();
}

void Run()
{
  gazebo::physics::run_worlds();

  while (!quit)
  {
    gazebo::common::Time::MSleep(50);
  }
}

//////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Application Setup
  if (ParseArgs(_argc, _argv) != 0)
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


