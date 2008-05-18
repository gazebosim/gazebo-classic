#include <libplayerc++/playerc++.h>
#include <iostream>

int main()
{
  // We throw exceptions on creation if we fail
  try
  {
    using namespace PlayerCc;
    using namespace std;

    // Create a player client object, using the variables assigned by the
    // call to parse_args()
    PlayerClient robot(PlayerCc::PLAYER_HOSTNAME, PlayerCc::PLAYER_PORTNUM);

    // Subscribe to the camera proxy
    PtzProxy ptzp(&robot, 0);

  }
  catch (PlayerCc::PlayerError e)
  {
    std::cerr << "Error:" << e << std::endl;
    return -1;
  }
}
