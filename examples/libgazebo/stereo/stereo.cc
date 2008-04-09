#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>

int main()
{
  gazebo::Client *client = new gazebo::Client();
  gazebo::SimulationIface *simIface = new gazebo::SimulationIface();
  gazebo::StereoCameraIface *stereoIface = new gazebo::StereoIface();

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

  /// Open the Stereo interface
  try
  {
    posIface->Open(client, "stereo_iface_0");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the position interface\n"
    << e << "\n";
    return -1;
  }

  // Enable the motor
  stereoIface->Lock(1);
  stereoIface->Unlock();

  /*while (true)
  {
    posIface->Lock(1);
    posIface->data->cmdVelocity.pos.x += 10;
    posIface->Unlock();

    usleep(100000);
  }*/
  return 0;
}

