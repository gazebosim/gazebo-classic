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

    // Subscribe to the simulation proxy
    ActArrayProxy ap(&robot, 0);

    // Print out some stuff
    std::cout << robot << std::endl;

    for (;;)
    {
      // This blocks until new data comes
      robot.Read();

      ap.MoveTo(3, -M_PI/2.0);
      player_actarray_actuator_t data = ap.GetActuatorData(3);
      printf("Pos[%f] Speed[%f]\n",data.position, data.speed);
    }
  }
  catch (PlayerCc::PlayerError e)
  {
    std::cerr << "Error:" << e << std::endl;
    return -1;
  }
}
