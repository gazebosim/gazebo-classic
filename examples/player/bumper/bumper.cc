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
    SimulationProxy sp(&robot, 0);

    BumperProxy bp(&robot, 0);

    // Print out some stuff
    std::cout << robot << std::endl;

    for (int i=0; i< 20; i++)
    {
      robot.Read();

      printf("Bumper Count[%d]\n", bp.GetCount());
      
      for (unsigned int j=0; j < bp.GetCount(); j++)
      {
        printf("  State[%d] = %d\n", j, bp.IsBumped(j));
      }

      usleep(100000);
    }

  }
  catch (PlayerCc::PlayerError e)
  {
    std::cerr << "Error:" << e << std::endl;
    return -1;
  }
}
