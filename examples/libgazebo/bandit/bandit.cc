#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>

#include <math.h>

#define DTOR(d) ((d) * M_PI / 180)
#define RTOD(r) ((r) * 180 / M_PI)

enum Joint {HEAD, NECK, R_SHOULDER, R_SHOULDER2, R_ELBOW, R_ELBOW2, 
  R_WRIST, R_WRIST2, R_HAND, 
  L_SHOULDER, L_SHOULDER2, L_ELBOW, L_ELBOW2, L_WRIST, 
  L_WRIST2, L_HAND, NUM_JOINTS};


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

  printf("Shoulder2[%f]\n", RTOD(actarrayIface->data->actuators[R_SHOULDER2].position));

  actarrayIface->data->cmd_pos[R_SHOULDER2] = DTOR(80);
  actarrayIface->data->cmd_pos[R_ELBOW] = DTOR(90);
  usleep(1000000);
  actarrayIface->data->cmd_pos[R_ELBOW2] = DTOR(100);
  usleep(500000);
  actarrayIface->data->cmd_pos[R_ELBOW2] = DTOR(0);
  usleep(500000);

  actarrayIface->data->cmd_pos[R_ELBOW2] = DTOR(100);
  usleep(500000);
  actarrayIface->data->cmd_pos[R_ELBOW2] = DTOR(0);
  usleep(500000);
  actarrayIface->data->cmd_pos[R_ELBOW2] = DTOR(100);

  actarrayIface->data->cmd_pos[R_ELBOW2] = DTOR(0);
  actarrayIface->data->cmd_pos[R_SHOULDER2] = DTOR(0);
  sleep(2);

  /*while (true)
  {
    actarrayIface->Lock(1);
    actarrayIface->data->cmd_pos[16] = 0.3;
    actarrayIface->data->cmd_pos[17] = -0.3;
    actarrayIface->data->cmd_pos[18] = 0.3;
    actarrayIface->data->cmd_pos[19] = -0.3;
    actarrayIface->Unlock();

  }*/
  return 0;
}

