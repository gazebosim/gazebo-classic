#include <iostream>
#include <stdio.h>
#include <sys/select.h>
#include <boost/algorithm/string.hpp>
#include "libgazebo/gz.h"

#include <yaml.h>

gazebo::Client *client = NULL;
gazebo::SimulationIface *simIface = NULL;
gazebo::FactoryIface *factoryIface = NULL;

std::map<std::string, std::string> yamlValues;
std::vector<std::string> params;

yaml_parser_t parser;
yaml_event_t event;

////////////////////////////////////////////////////////////////////////////////
// Print out info for one model. Recurses through all child models
void print_model(std::string name, std::string prefix)
{
  std::string type;
  gazebo::Pose pose;

  std::cout << prefix << name << "\n";

  unsigned int children;
  if (!simIface->GetNumChildren(name, children))
    std::cerr << "Unable to get the number of children for model[" 
      << name << "]\n";

  for (unsigned int i=0; i < children; i++)
  {
    std::string childName;
    if (!simIface->GetChildName(name, i, childName))
      std::cerr << "Unable to get model[" << name << "] child name.\n";
    else
    {
      std::string type;
      simIface->GetEntityType(childName, type);
      if (type == "model")
        print_model(childName, prefix+"  ");
    }
  }

}

////////////////////////////////////////////////////////////////////////////////
// Print out a list of all the models
void list()
{
  unsigned int numModels = 0;

  std::string prefix = "";
  if (!simIface->GetNumModels(numModels))
    std::cerr << "Unable to get the model count\n";

  for (unsigned int i=0; i < numModels; i++)
  {
    std::string name;
    if (!simIface->GetModelName(i, name))
      std::cerr << "Unable to get model[" << i << "]\n";

    print_model(name, "");
  }
}

////////////////////////////////////////////////////////////////////////////////
// Show info for a model
void show()
{
  if (params.size() < 2)
    std::cerr << "Missing model name\n";
  else
  {
    for (unsigned int i=1; i < params.size(); i++)
    {
      std::string name = params[i];
      std::string type;
      gazebo::Pose pose;
      unsigned int paramCount;

      if (!simIface->GetPose3d(name, pose))
        std::cerr << "Unable to get model[" << name << "] pose\n";
      if (!simIface->GetModelType(name, type))
        std::cerr << "Unable to get model[" << name << "] type\n";
      if (!simIface->GetEntityParamCount(name, paramCount))
        std::cerr << "Unable to get model[" << name << "] param count\n";

      for (unsigned int i=0; i < paramCount; i++)
      {
        std::string paramKey;
        std::string paramValue;

        if (!simIface->GetEntityParamKey(name, i, paramKey))
          std::cerr << "Unable to get model[" << name << "] param key\n";
        if (!simIface->GetEntityParamValue(name, i, paramValue))
          std::cerr << "Unable to get model[" << name << "] param value\n";

        std::cout << paramKey << ": " << paramValue << "\n";
      }
      std::cout << "\n";
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Remove a model from the world
void kill()
{
  if (params.size() < 2)
    std::cerr << "Missing model name\n";
  else
  {
    // Kill all the passed in models
    for (unsigned int i=1; i < params.size(); i++)
    {
      factoryIface->DeleteModel( params[i] );

      while (strcmp((const char*)factoryIface->data->deleteModel,"") != 0)
        usleep(10000);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Spawn a new model into the world
void spawn()
{
  //TODO
  //PROCESS YAML PARAMETER!!!
  if (params.size() < 2)
    std::cerr << "Missing model filename\n";
  else
  {
    FILE *file = fopen(params[1].c_str(),"r");
    if (file)
    {
      std::ostringstream stream;
      while (!feof(file))
      {
        char buffer[256];
        if (fgets(buffer, 256, file) == NULL)
          std::cerr << "Unable to read file\n";

        if (feof(file))
            break;
        stream << buffer;
      }
      strcpy((char*)factoryIface->data->newModel, stream.str().c_str());
    }
    else
      std::cerr << "Unable to open file[" << params[1] << "]\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
// Print out help information
void help()
{
  std::cout << "gazebomodel is a command-line tool for printing out information about models in a gazebo world.\n";
  std::cout << "\n";
  std::cout << "Usage: gazebomodel <command> <option_1> ... <option_n>\n";
  std::cout << "\n";

  std::cout << "Commands:\n";
  std::cout << "\tgazebomodel list \t List all the models\n";
  std::cout << "\tgazebomodel show \t Show info about a model(s)\n";
  std::cout << "\tgazebomodel kill \t Remove a model(s) from the world\n";
  std::cout << "\tgazebomodel spawn \t Insert a model into the world\n";
}

////////////////////////////////////////////////////////////////////////////////
// Parse yaml parameters
void parseYAML()
{
  std::map<std::string, std::string>::iterator iter;

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

  /*for (iter = yamlValues.begin(); iter != yamlValues.end(); iter++)
  {
    std::cout << "Key[" << iter->first << "] Value[" << iter->second << "]\n";
  }*/
}

////////////////////////////////////////////////////////////////////////////////
// Main
int main(int argc, char **argv)
{

  if (argc == 1 || std::string(argv[1]) == "help")
  {
    help();
    return 1;
  }

  // Get parameters from command line
  for (int i=1; i < argc; i++)
  {
    std::string p = argv[i];
    boost::trim(p);
    params.push_back( p );
  }

  // Get parameters from stdin 
  if (!isatty(fileno(stdin)))
  {
    char str[1024];
    while (!feof(stdin))
    {
      if (fgets(str, 1024, stdin)==NULL)
        std::cerr << "Unable to read file\n";
      if (feof(stdin))
        break;
      std::string p = str;
      boost::trim(p);
      params.push_back(p);
    }
  }

  parseYAML();

  /*
  client = new gazebo::Client();
  simIface = new gazebo::SimulationIface();
  factoryIface = new gazebo::FactoryIface();

  try
  {
    client->ConnectWait(0, GZ_CLIENT_ID_USER_FIRST);
  }
  catch(std::string e)
  {
    std::cerr << "Gazebo Error: Unable to connect: " << e << "\n";
    return -1;
  }

  /// Open the sim iface
  try
  {
    simIface->Open(client, "default");
  }
  catch (std::string e)
  {
    std::cerr << "Gazebo error: Unable to connect to sim iface:" << e << "\n";
    return -1;
  }

  // Open the factory iface
  try
  {
    factoryIface->Open(client, "default");
  }
  catch( std::string e)
  {
    std::cerr << "Gazebo error: Unable to connect to the factory Iface:" 
              << e << "\n";
    return -1;
  }

  if (params[0] == "list")
    list();
  else if (params[0] == "show")
    show();
  else if (params[0] == "kill")
    kill();
  else if (params[0] == "spawn")
    spawn();
  else
    std::cerr << "Unknown command[" << params[0] << "]\n";
  */
}
