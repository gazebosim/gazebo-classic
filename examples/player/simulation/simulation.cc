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

    // Subscribe to the simulation proxy
    SimulationProxy sp(&robot, 0);

    // Print out some stuff
    std::cout << robot << std::endl;

    double x,y,z,roll,pitch,yaw, time;

    // Get the robot pose
    sp.GetPose3d((char*)"pioneer2dx_model1", x, y, z, roll, pitch, yaw, time);
    printf("Robot Pose3d: XYZ[%f %f %f] RPY[%f %f %f]\n",x, y, z, roll, pitch, yaw); 

    sp.GetPose2d((char*)"pioneer2dx_model1", x, y, roll );
    printf("Robot Pose2d: XY[%f %f] Yaw[%f]\n", x, y, roll );

    // Set the robot pose
    //sp.SetPose3d((char*)"pioneer2dx_model1", 2.0, 0.0, 5.0, 0, 0, 0);

    printf("Get Sim Time\n");
    /// Get the simulation time
    sp.GetProperty((char*)"world",(char*)"sim_time", &time, sizeof(double));
    printf("Simulation Time[%f]\n",time);

    /// Get the time the simulation has been pause
    sp.GetProperty((char*)"world",(char*)"pause_time", &time, sizeof(time));
    printf("Pause Time[%f]\n",time);

    /// Get the realtime
    sp.GetProperty((char*)"world",(char*)"real_time", &time, sizeof(time));
    printf("Real Time[%f]\n",time);

  }
  catch (PlayerCc::PlayerError e)
  {
    std::cerr << "Error:" << e << std::endl;
    return -1;
  }
}
