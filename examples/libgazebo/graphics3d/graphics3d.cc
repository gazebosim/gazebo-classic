#include <string.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>
#include <math.h>

#include <gazebo/gazebo.h>

// All the interfaces
libgazebo::Client *client;
libgazebo::SimulationIface *simIface;
libgazebo::Graphics3dIface *g3dIface;

// Stuff to draw the square
std::string squareName = "square";
libgazebo::Vec3 squareSize;
int dir = 1;

// Stuff to draw the sphere
std::string sphereName = "sphere";
libgazebo::Vec3 spherePos, sphereSize;
float radius = 0.2;
float theta = 0;

// Stuff to draw the robot's path
std::string pathName = "path";
std::vector<libgazebo::Vec3> positions;

// Stuff to draw the text
std::string textName = "velocities";

void UpdatePath()
{
  libgazebo::Pose rPos;
  libgazebo::Vec3 blockSize;
  libgazebo::Color clr;
  std::ostringstream blockName;

  blockSize.x = 0.05;
  blockSize.y = 0.05;
  blockSize.z = 0.05;

  // Get the simulation pose of the robot
  simIface->Lock(1);
  simIface->data->requestCount = 0;
  simIface->Unlock();
  simIface->GetPose2d("pr2", rPos);

  rPos.pos.z = 0.15;

  // Draw the robot's path
  if (positions.size() == 0 ||
      sqrt( pow(positions[positions.size()-1].x - rPos.pos.x,2) +
            pow(positions[positions.size()-1].y - rPos.pos.y,2)) > 0.5)
  {

    // Store the new position
    positions.push_back(rPos.pos);

    libgazebo::Vec3 tmpvec[positions.size()];
    for (unsigned int j =0; j < positions.size(); j++)
    {
      tmpvec[j].x = positions[j].x;
      tmpvec[j].y = positions[j].y;
      tmpvec[j].z = positions[j].z;
    }

    clr.r = 0.0;
    clr.g = 1.0;
    clr.b = 0.0;

    // Draw the line
    g3dIface->DrawSimple(pathName.c_str(),
        libgazebo::Graphics3dDrawData::LINE_STRIP,
        tmpvec, positions.size(), clr);

    blockName << "path_block:" << positions.size();

    clr.r = 1.0;
    clr.g = 1.0;
    clr.b = 0.0;
    clr.a = 1.0;

    g3dIface->DrawShape(blockName.str().c_str(),
        libgazebo::Graphics3dDrawData::CUBE, tmpvec[positions.size()-1], 
        blockSize, clr);
  }
}


int main()
{
  client = new libgazebo::Client();
  simIface = new libgazebo::SimulationIface();
  g3dIface = new libgazebo::Graphics3dIface();

  int serverId = 0;

  /// Connect to the libgazebo server
  try
  {
    client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect\n" << e << "\n";
    return -1;
  }

  /// Open the global graphics Interface
  try
  {
    simIface->Open(client, "default");

    g3dIface->Open(client, "default");
  }
  catch (std::string e)
  {
    std::cerr << "Gazebo error: Unable to connect to an  interface\n" 
              << e << "\n";
    return -1;
  }

  // Set the size of the square
  squareSize.x = 0.25;
  squareSize.y = 0.25;
  squareSize.z = 0.0;

  // Set the size of the sphere
  sphereSize.x = 0.1;
  sphereSize.y = 0.1;
  sphereSize.z = 0.1;

  libgazebo::Color barClr;
  barClr.r = 1.0;
  barClr.g = 1.0;
  barClr.b = 0.0;
  barClr.a = 0.5;

  float percent = 0.0;
  int dir = 1.0;

  // Update all the drawables
  while (true)
  {
    UpdatePath();

    usleep(20000);
  }

  simIface->Close();
  g3dIface->Close();

  delete simIface;
  delete g3dIface;
  delete client;
  return 0;
}

