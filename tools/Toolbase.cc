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
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "Toolbase.hh"

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Toolbase::Toolbase()
  : client(NULL), simIface(NULL), factoryIface(NULL),
{
  this->yamlValues.clear();
  this->params.clear();
}

////////////////////////////////////////////////////////////////////////////////
// Init the tool
bool Toolbase::Init(int argc, char **argv)
{
  if (argc == 1 || std::string(argv[1]) == "help")
  {
    this->Help();
    return false;
  }

  // Get parameters from command line
  for (int i=1; i < argc; i++)
  {
    std::string p = argv[i];
    boost::trim(p);
    this->params.push_back( p );
  }

  // Get parameters from stdin 
  if (!isatty(fileno(stdin)))
  {
    char str[1024];
    while (!feof(stdin))
    {
      if (fgets(str, 1024, stdin)==NULL)
        break;

      if (feof(stdin))
        break;
      std::string p = str;
      boost::trim(p);
      this->params.push_back(p);
    }
  }

  this->ParseYAML();

  this->client = new libgazebo::Client();
  this->simIface = new libgazebo::SimulationIface();
  this->factoryIface = new libgazebo::FactoryIface();

  try
  {
    this->client->ConnectWait(0, GZ_CLIENT_ID_USER_FIRST);
  }
  catch(std::string e)
  {
    std::cerr << "Gazebo Error: Unable to connect: " << e << "\n";
    return false;
  }

  /// Open the sim iface
  try
  {
    this->simIface->Open(client, "default");
  }
  catch (std::string e)
  {
    std::cerr << "Gazebo error: Unable to connect to sim iface:" << e << "\n";
    return false;
  }

  // Open the factory iface
  try
  {
    this->factoryIface->Open(client, "default");
  }
  catch( std::string e)
  {
    std::cerr << "Gazebo error: Unable to connect to the factory Iface:" 
      << e << "\n";
    return false;
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Toolbase::~Toolbase()
{
}

////////////////////////////////////////////////////////////////////////////////
// Run
bool Toolbase::Run()
{
  bool handeled = true;

  if (this->params[0] == "pause")
    this->Pause();
  else if (this->params[0] == "go")
    this->Unpause();
  else if (this->params[0] == "reset")
    this->Reset();
  else if (this->params[0] == "step")
    this->Step();
  else
    handeled = false;

  return handeled;
}

////////////////////////////////////////////////////////////////////////////////
/// Pause the simulation
void Toolbase::Pause()
{
  this->simIface->Pause();
}

////////////////////////////////////////////////////////////////////////////////
/// Run the simulation
void Toolbase::Unpause()
{
  this->simIface->Unpause();
}

////////////////////////////////////////////////////////////////////////////////
/// Step the simulation
void Toolbase::Step()
{
  int count = 1;

  if (params.size() >= 2)
  {
    try
    {
      count = boost::lexical_cast<int>(params[1]);
    }
    catch (boost::bad_lexical_cast &e)
    {
      std::cerr << "step can take only an integer parameter\n";
    }
  }

  std::cout << "COUNT[" << count << "]\n";
  for (int i=0; i < count; i++)
  {
    this->simIface->Step();
    usleep(1000);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Pause the simulation
void Toolbase::Reset()
{
  this->simIface->Reset();
}

////////////////////////////////////////////////////////////////////////////////
// Parse yaml parameters
void Toolbase::ParseYAML()
{

  // Create the yaml parser 
  yaml_parser_initialize(&parser);

  std::string input = params[params.size()-1];
  yaml_parser_set_input_string(&parser, 
      (const unsigned char *)input.c_str(), input.size());

  bool done = false;
  std::string name;
  while (!done)
  {
    if (!yaml_parser_parse(&parser, &event))
    {
      std::cerr << "YAML error: Bad syntax for '" << input << "'\n";
      break;
    }

    if (event.type == YAML_SCALAR_EVENT)
    {
      if (name.size() == 0)
        name = (char*)(event.data.scalar.value);
      else
      {
        yamlValues[name] = (char*)event.data.scalar.value;
        name = "";
      }
    }

    done = (event.type == YAML_STREAM_END_EVENT);
    yaml_event_delete(&event);
  }

  yaml_parser_delete(&parser);

  std::map<std::string, std::string>::iterator iter;
  for (iter = yamlValues.begin(); iter != yamlValues.end(); iter++)
  {
    std::cout << "Key[" << iter->first << "] Value[" << iter->second << "]\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
// Print all the commands to stdout
void Toolbase::PrintCommands(const std::string &_prefix)
{
  std::cout << "\t" << _prefix <<  " pause \t Pause the simulation\n";
  std::cout << "\t" << _prefix <<  " go \t\t Unpause the simulation\n";
  std::cout << "\t" << _prefix <<  " step \t Increment the simulation time\n";
  std::cout << "\t" << _prefix <<  " reset \t Reset the simulation\n";
}
