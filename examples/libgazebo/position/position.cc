#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>

int main()
{
  libgazebo::Client *client = new libgazebo::Client();
  libgazebo::SimulationIface *simIface = new libgazebo::SimulationIface();
  libgazebo::PositionIface *posIface = new libgazebo::PositionIface();

  int serverId = 0;

  /// Connect to the libgazebo server
  try
  {
    client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect\n" << e << "\n";
    return -1;
  }

  /// Open the Simulation Interface
  try
  {
    simIface->Open(client, "default");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the sim interface\n" << e << "\n";
    return -1;
  }

  /// Open the Position interface
  try
  {
    posIface->Open(client, "turtlebot1::position_iface_0");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the position interface\n"
    << e << "\n";
    return -1;
  }

  // Enable the motor
  posIface->Lock(1);
  posIface->data->cmdEnableMotors = 1;
  posIface->Unlock();

  posIface->data->cmdVelocity.yaw = -0.1;
  while (true)
  {
    posIface->Lock(1);
    posIface->data->cmdVelocity.pos.x = 0.2;
    posIface->Unlock();

    usleep(100000);
  }
  return 0;
}

