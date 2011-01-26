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
#ifndef TOOLBASE_HH
#define TOOLBASE_HH

#include <map>
#include <vector>
#include <yaml.h>

#include "libgazebo/gz.h"

class Toolbase
{
  /// \brief Constructor
  public: Toolbase();

  /// \brief Destructor
  public: virtual ~Toolbase();

  /// \brief Init the tool
  public: bool Init(int argc, char **argv);

  /// \brief Run the tool
  public: virtual bool Run();

  /// \brief Pause the simulation
  public: void Pause();

  /// \brief Run the simulation
  public: void Unpause();

  /// \brief Step the simulation
  public: void Step();

  /// \brief Pause the simulation
  public: void Reset();

  public: virtual void Help() = 0;

  /// \brief Parse yaml parameters
  private: void ParseYAML();

  /// \brief Print all the commands to stdout
  protected: void PrintCommands(std::string prefix);

  protected: libgazebo::Client *client;
  protected: libgazebo::SimulationIface *simIface;
  protected: libgazebo::FactoryIface *factoryIface;

  protected: std::map<std::string, std::string> yamlValues;
  protected: std::vector<std::string> params;

  protected: yaml_parser_t parser;
  protected: yaml_event_t event;


};

#endif
