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
#include <boost/program_options.hpp>

#include "gazebo_config.h"
#include "common/CommonTypes.hh"
#include "rendering/Rendering.hh"
#include "common/SystemPaths.hh"
#include "Server.hh"

gazebo::Server *server = NULL;

std::string config_filename = "";
gazebo::common::StrStr_M params;
std::vector<std::string> plugins;

namespace po = boost::program_options;

//////////////////////////////////////////////////
void PrintUsage()
{
  std::cerr << "Run the Gazebo server.\n\n"
    << "Usage: gzserver [options] <world_file>\n\n";
}

//////////////////////////////////////////////////
void PrintVersion()
{
  fprintf(stderr, "%s", GAZEBO_VERSION_HEADER);
}

//////////////////////////////////////////////////
bool ParseArgs(int argc, char **argv)
{
  po::options_description v_desc("Allowed options");
  v_desc.add_options()
    ("help,h", "Produce this help message.")
    ("pause,u", "Start the server in a paused state.")
    ("plugin,p", po::value<std::string>(), "Load a plugin.");

  po::options_description h_desc("Hidden options");
  h_desc.add_options()
    ("world_file", po::value<std::string>(), "SDF world to load.");

  po::options_description desc("Allowed options");
  desc.add(v_desc).add(h_desc);

  po::positional_options_description p_desc;
  p_desc.add("world_file", 1);

  po::variables_map vm;
  try
  {
    po::store(po::command_line_parser(argc,
          argv).options(desc).positional(p_desc).run(), vm);
    po::notify(vm);
  } catch(boost::exception &_e)
  {
    std::cerr << "Error. Invalid arguments\n";
    // std::cerr << boost::diagnostic_information(_e) << "\n";
    return false;
  }

  if (vm.count("help"))
  {
    PrintUsage();
    std::cerr << v_desc << "\n";
    return false;
  }

  if (vm.count("pause"))
    params["pause"] = "true";
  else
    params["pause"] = "false";

  if (vm.count("plugin"))
    plugins.push_back(vm["plugin"].as<std::string>());

  if (vm.count("world_file"))
    config_filename = vm["world_file"].as<std::string>();
  else
    config_filename = "worlds/empty.world";

  return true;
}

//////////////////////////////////////////////////
void SignalHandler(int)
{
  server->Stop();
}


//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Application Setup
  if (!ParseArgs(argc, argv))
  {
    return -1;
  }

  PrintVersion();

  if (signal(SIGINT, SignalHandler) == SIG_ERR)
  {
    std::cerr << "signal(2) failed while setting up for SIGINT" << std::endl;
    return -1;
  }

  server = new gazebo::Server();
  if (config_filename.empty())
  {
    printf("Warning: no world filename specified, using default world\n");
    config_filename = "worlds/empty.world";
  }

  // Construct plugins
  /// Load all the plugins specified on the command line
  for (std::vector<std::string>::iterator iter = plugins.begin();
       iter != plugins.end(); ++iter)
  {
    server->LoadPlugin(*iter);
  }

  if (!server->Load(config_filename))
  {
    gzerr << "Could not open file[" << config_filename << "]\n";
    return -1;
  }

  server->SetParams(params);
  server->Init();

  server->Run();

  server->Fini();

  delete server;
  server = NULL;

  printf("\n");
  return 0;
}
