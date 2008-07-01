#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>

int main()
{
  gazebo::Client *client = new gazebo::Client();
  gazebo::SimulationIface *simIface = new gazebo::SimulationIface();

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
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the sim interface\n" << e << "\n";
    return -1;
  }

  //simIface->data->reset = 1;

  // Example of how to move a model (box1_model)
  uint8_t name[512] = "pioneer2dx_model1";
  uint8_t cmd[32] = "set_pose3d";

  for (int i=0; i< 10; i++)
  {
    simIface->Lock(1);

    memcpy(simIface->data->model_name, name, 512);
    memcpy(simIface->data->model_req,cmd, 32);

    simIface->data->model_pose.pos.x = i+.1;
    simIface->data->model_pose.pos.y = 0;
    simIface->data->model_pose.pos.z = 0.2;

    simIface->Unlock();

    usleep(100000);
  }


  // Example of resetting the simulator
  /*simIface->Lock(1);
  simIface->data->reset = 1;
  simIface->Unlock();
  */

  usleep(1000000);

  simIface->Close();
  delete simIface;
  return 0;
}

