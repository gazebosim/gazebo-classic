#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>

int main()
{
  gazebo::Client *client = new gazebo::Client();
  gazebo::SimulationIface *simIface = new gazebo::SimulationIface();
  gazebo::ActarrayIface *actarrayIface = new gazebo::ActarrayIface();

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

  /// Open the Actuator Array interface
  try
  {
    actarrayIface->Open(client, "bandit_actarray_iface");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the actarray interface\n"
    << e << "\n";
    return -1;
  }

  while (true)
  {
    actarrayIface->Lock(1);
    actarrayIface->data->cmd_pos[16] = 0.3;
    actarrayIface->data->cmd_pos[17] = -0.3;
    actarrayIface->data->cmd_pos[18] = 0.3;
    actarrayIface->data->cmd_pos[19] = -0.3;
    actarrayIface->Unlock();

    usleep(100000);
  }
  return 0;
}

