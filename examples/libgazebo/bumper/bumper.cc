#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>

int main()
{
  gazebo::Client *client = new gazebo::Client();
  gazebo::BumperIface *bumperIface = new gazebo::BumperIface();

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

  /// Open the bumper Interface
  try
  {
    bumperIface->Open(client, "bumper_iface");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the bumper interface\n" << e << "\n";
    return -1;
  }

  for (int i=0; i< 10; i++)
  {
    bumperIface->Lock(1);
    int bump_count = bumperIface->data->bumper_count;

    printf("Bump Count[%d]\n", bump_count);

    for (int c = 0; c < bump_count; c++)
    {
      printf("  State[%d]\n", bumperIface->data->bumpers[c]);
    }

    bumperIface->Unlock();

    usleep(100000);
  }


  bumperIface->Close();
  delete bumperIface;
  return 0;
}

