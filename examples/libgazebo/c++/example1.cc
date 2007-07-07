#include <gazebo/gazebo.h>

int main()
{
  gazebo::Client *client = new gazebo::Client();
  gazebo::SimulationIface *simIface = new gazebo::SimulationIface();
  gazebo::PositionIface *posIface = new gazebo::PositionIface();

  int serverId = 0;

  /// Connect to the libgazebo server
  if (client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST))
  {
    printf("Gazebo error: Unable to connect\n");
    return -1;
  }

  /// Open the Simulation Interface
  if (simIface->Open(client, "default"))
  {
    printf("Gazebo error: Unable to connect to the sim interface\n");
    return -1;
  }

  /// Open the Position interface
  if (posIface->Open(client, "pioneer1_model"))
  {
    printf("Gazebo error: Unable to connect to the position interface\n");
    return -1;
  }

  posIface->Lock(1);
  posIface->data->cmdEnableMotors = 1;
  posIface->Unlock();

  while (true)
  {
    posIface->Lock(1);
    printf("Enables[%d]\n", posIface->data->cmdEnableMotors);
    posIface->Unlock();

    usleep(100000);
  }
  return 0;
}

