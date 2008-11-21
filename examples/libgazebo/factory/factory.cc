#include <sstream>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <string.h>

int main()
{
  gazebo::Client *client = new gazebo::Client();
  gazebo::SimulationIface *simIface = new gazebo::SimulationIface();
  gazebo::FactoryIface *factoryIface = new gazebo::FactoryIface();

  int serverId = 0;

  /// Connect to the libgazebo server
  try
  {
    client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
  }
  catch (gazebo::GazeboError e)
  {
    std::cout << "Gazebo error: Unable to connect\n" << e << "\n";
    return -1;
  }

  /// Open the Simulation Interface
  try
  {
    simIface->Open(client, "default");
  }
  catch (gazebo::GazeboError e)
  {
    std::cout << "Gazebo error: Unable to connect to the sim interface\n" << e << "\n";
    return -1;
  }

  /// Open the Factory interface
  try
  {
    factoryIface->Open(client, "factory_iface");
  }
  catch (gazebo::GazeboError e)
  {
    std::cout << "Gazebo error: Unable to connect to the factory interface\n"
    << e << "\n";
    return -1;
  }

  std::string sphereBegin = "<model:physical name='sphere_model'> <xyz>0 0 5.5</xyz> <body:sphere name='sphere_body'> <geom:sphere name='sphere_geom'> <mesh>default</mesh> <size>0.5</size> <density>1.0</density> <material>Gazebo/Rocky</material></geom:sphere> </body:sphere> </model:physical>";


  for (int i=0; i<3; i++)
  {
    factoryIface->Lock(1);
    if (!strcmp((const char*)factoryIface->data->newModel,""))
    {
      std::ostringstream stream;
      stream << "<model:physical name='sphere_model_" << i << "'>";
      stream <<   "<xyz>" << i*0.5 << " 0 1</xyz>";
      stream <<   "<body:sphere name='sphere_body_" << i << "'>";
      stream <<     "<geom:sphere name='sphere_geom_" << i << "'>";
      stream <<       "<size>0.1</size>";
      stream <<       "<density>1.0</density>";
      stream <<       "<visual>";
      stream <<         "<size>0.1 0.1 0.1</size>";
      stream <<         "<material>Gazebo/Rocky</material>";
      stream <<         "<mesh>unit_sphere</mesh>";
      stream <<       "</visual>";
      stream <<     "</geom:sphere>";
      stream <<   "</body:sphere>";
      stream << "</model:physical>";

      printf("Creating[%d]\n",i);
      strcpy((char*)factoryIface->data->newModel, stream.str().c_str());
    }
    factoryIface->Unlock();
    usleep(1000000);
  }

  for (int i=0; i<3; i++)
  {
    factoryIface->Lock(1);
    if (!strcmp((const char*)factoryIface->data->deleteModel,""))
    {
      std::ostringstream stream;

      stream << "sphere_model_" << i;

      printf("Deleting[%d]\n",i);
      strcpy((char*)factoryIface->data->deleteModel, stream.str().c_str());
    }
    factoryIface->Unlock();
    usleep(1000000);

  }
  return 0;
}

