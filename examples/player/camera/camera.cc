#include <libplayerc++/playerc++.h>
#include <iostream>

using namespace PlayerCc;
using namespace std;


int main()
{
  // We throw exceptions on creation if we fail
  try
  {
    // Create a player client object, using the variables assigned by the
    // call to parse_args()
    PlayerClient robot(PlayerCc::PLAYER_HOSTNAME, PlayerCc::PLAYER_PORTNUM);

    // Subscribe to the camera proxy
    CameraProxy cp(&robot, 0);
    BlobfinderProxy bp (&robot, 0);

    while (true)
    {
      robot.Read();

      printf("Blobs[%d]\n",bp.GetCount());
      usleep(10000);
    }

  }
  catch (PlayerCc::PlayerError e)
  {
    std::cerr << "Error:" << e << std::endl;
    return -1;
  }

}
