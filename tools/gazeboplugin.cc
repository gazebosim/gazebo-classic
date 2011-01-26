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
#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "Toolbase.hh"

class PluginTool : public Toolbase
{

  //////////////////////////////////////////////////////////////////////////////
  // Print out a list of all the plugins
  public: void List()
  {
    unsigned int numPlugins = 0;

    std::string prefix = "";
    if (!this->simIface->GetPluginCount(numPlugins))
      std::cerr << "Unable to get the plugin count\n";

    for (unsigned int i=0; i < numPlugins; i++)
    {
      std::string name;
      if (!this->simIface->GetPluginName(i, name))
        std::cerr << "Unable to get plugin[" << i << "]\n";
      else
        std::cout << name << "\n";
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  // Remove a plugin from the simulation
  public: void RemovePlugin()
  {
    if (this->params.size() < 2)
      std::cerr << "Missing pluing filename\n";
    else
    {
      std::string plugin;
      struct stat buf;
      if (stat(params[1].c_str(),&buf) == 0)
        plugin = std::string(getenv("PWD")) + "/" + params[1];
      else
        plugin = params[1];

      this->simIface->RemovePlugin(plugin);
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  // Add a handler
  public: void AddPlugin()
  {
    if (this->params.size() < 3)
      std::cerr << "Must specify a filename and a handle name\n";
    else
    {
      std::string filename, handle;
      struct stat buf;
      if (stat(params[1].c_str(),&buf) == 0)
        filename = std::string(getenv("PWD")) + "/" + params[1];
      else
        filename = params[1];

      handle = params[2];

      this->simIface->AddPlugin(filename, handle);
    }

  }

  public: bool Run()
  {
    if (!Toolbase::Run())
    {
      if (this->params[0] == "list")
        this->List();
      else if (params[0] == "add")
        this->AddPlugin();
      else if (params[0] == "remove")
        this->RemovePlugin();
      else
        std::cerr << "Unknown command[" << this->params[0] << "]\n";
    }

    return true;
  }


  //////////////////////////////////////////////////////////////////////////////
  // Print out help information
  public: void Help()
  {
    std::cout << "gazeboplugin is a command-line tool for manipulating plugins in a gazebo world.\n";
    std::cout << "\n";
    std::cout << "Usage: gazeboplugin <command> <option_1> ... <option_n>\n";
    std::cout << "\n";

    std::cout << "Commands:\n";

    this->PrintCommands("gazeboplugin");

    std::cout << "\tgazeboplugin list   \t List all the models\n";
    std::cout << "\tgazeboplugin add    \t  Add a plugin to the world\n";
    std::cout << "\tgazeboplugin remove \t Remove a plugin from the world\n";
  }
};

////////////////////////////////////////////////////////////////////////////////
// Main
int main(int argc, char **argv)
{
  PluginTool tool;
  tool.Init(argc, argv);
  tool.Run();

  return 1;
}
