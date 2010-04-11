#include <iostream>
//#include <gazebo/gazebo.h>
#include "libgazebo/gz.h"

gazebo::Client *client = NULL;
gazebo::SimulationIface *simIface = NULL;
gazebo::FactoryIface *factoryIface = NULL;


////////////////////////////////////////////////////////////////////////////////
// Print out info for one model. Recurses through all child models
void print_model(std::string name, std::string prefix)
{
  std::string type;
  gazebo::Pose pose;

  /*if (!simIface->GetPose3d(name, pose))
    std::cerr << "Unable to get model[" << name << "] pose\n";
  if (!simIface->GetModelType(name, type))
    std::cerr << "Unable to get model[" << name << "] type\n";
    */

  std::cout << prefix << name << "\n";
  /*std::cout << prefix << "  Type: " << type << "\n";
  std::cout << prefix << "  XYZ: " << pose.pos.x << " " << pose.pos.y 
            << " " << pose.pos.z << "\n";
  std::cout << prefix << "  RPY: " << pose.roll << " " << pose.pitch 
            << " " << pose.yaw << "\n";
            */

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
void list(int argc, char **argv)
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
void show(int argc, char **argv)
{
  if (argc < 3)
    std::cerr << "Missing model name\n";
  else
    print_model(argv[2], "");
}

////////////////////////////////////////////////////////////////////////////////
// Remove a model from the world
void kill(int argc, char **argv)
{
  if (argc < 3)
    std::cerr << "Missing model name\n";
  else
  {
    factoryIface->DeleteModel( argv[2] );
  }
}

void help()
{
  std::cout << "gazebomodel is a command-line tool for printing out information about models in a gazebo world.\n";
  std::cout << "\n";
  std::cout << "Usage: gazebomodel <command> <option_1> ... <option_n>\n";
  std::cout << "\n";

  std::cout << "Commands:\n";
  std::cout << "\tgazebomodel list \t List all the models\n";
  std::cout << "\tgazebomodel show \t Show info about a model\n";
}

int main(int argc, char **argv)
{

  if (argc == 1 || std::string(argv[1]) == "help")
  {
    help();
    return 1;
  }

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

  std::string cmd = argv[1];
  if (cmd == "list")
    list(argc, argv);
  else if (cmd == "show")
    show(argc, argv);
  else if (cmd == "kill")
    kill(argc, argv);
  else
    std::cerr << "Unknown command[" << cmd << "]\n";
}
