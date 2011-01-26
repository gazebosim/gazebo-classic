/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
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

    // Subscribe to the laser proxy
    LaserProxy lp(&robot, 0);

    robot.Read();

    // Print out some stuff
    std::cout << robot << std::endl;

    lp.RequestGeom();

    player_pose3d_t laserPose = lp.GetPose();

    // Print out laser stuff
    std::cout << "Laser Pose[" << laserPose.px << " " <<
      laserPose.py << " " << laserPose.pyaw << "]\n";

    for (;;)
    {
      // This blocks until new data comes
      robot.Read();

      for (unsigned int i=0; i < lp.GetCount(); i++)
      {
        printf("[%f %d]", lp.GetRange(i), lp.GetIntensity(i) );
      }
      printf("\n");
    }
  }
  catch (PlayerCc::PlayerError e)
  {
    std::cerr << "Error:" << e << std::endl;
    return -1;
  }
}
