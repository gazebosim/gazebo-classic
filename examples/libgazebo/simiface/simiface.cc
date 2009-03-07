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

  for (int i=0; i< 10; i++)
  {
    simIface->Lock(1);

    gazebo::SimulationRequestData *request = &(simIface->data->requests[simIface->data->requestCount++]);

    request->type = gazebo::SimulationRequestData::SET_STATE;
    memcpy(request->modelName, name, 512);

    request->modelPose.pos.x = i+1;
    request->modelPose.pos.y = 0;
    request->modelPose.pos.z = 0;
    request->modelPose.roll = 0;
    request->modelPose.pitch = 0;
    request->modelPose.yaw = 0;

    request->modelLinearVel.x = 0.1;
    request->modelLinearVel.y = 0;
    request->modelLinearVel.z = 0;

    request->modelAngularVel.x = 0.1;
    request->modelAngularVel.y = 0;
    request->modelAngularVel.z = 0;

    /*request->type = gazebo::SimulationRequestData::SET_POSE2D;
    memcpy(request->modelName, name, 512);

    request->modelPose.pos.x = i+.1;
    request->modelPose.pos.y = 0;
    */

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

