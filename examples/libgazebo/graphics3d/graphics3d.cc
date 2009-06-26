#include <string.h>
#include <stdio.h>
#include <string>
#include <iostream>
#include <vector>
#include <math.h>

#include <gazebo/gazebo.h>

// All the interfaces
gazebo::Client *client;
gazebo::SimulationIface *simIface;
gazebo::PositionIface *posIface;
gazebo::Graphics3dIface *g3dIface;
gazebo::Graphics3dIface *pioneerG3DIface;

// Stuff to draw the square
std::string squareName = "square";
gazebo::Vec3 squareSize;
int dir = 1;

// Stuff to draw the sphere
std::string sphereName = "sphere";
gazebo::Vec3 spherePos, sphereSize;
float radius = 0.2;
float theta = 0;

// Stuff to draw the robot's path
std::string pathName = "path";
std::vector<gazebo::Vec3> positions;

// Stuff to draw the text
std::string textName = "velocities";

void UpdateSquare()
{
  gazebo::Color clr;
  gazebo::Vec3 vec[5];

  vec[0].x = squareSize.x;
  vec[0].y = squareSize.y;
  vec[0].z = squareSize.z;

  vec[1].x = -squareSize.x;
  vec[1].y = squareSize.y;
  vec[1].z = squareSize.z;

  vec[2].x = -squareSize.x;
  vec[2].y = -squareSize.y;
  vec[2].z = squareSize.z;

  vec[3].x = squareSize.x;
  vec[3].y = -squareSize.y;
  vec[3].z = squareSize.z;

  vec[4].x = squareSize.x;
  vec[4].y = squareSize.y;
  vec[4].z = squareSize.z;

  squareSize.z += dir*0.01;
  if (squareSize.z >= 0.2 || squareSize.z < 0)
      dir *= -1;

  clr.r = 1.0;
  clr.g = 0.0;
  clr.b = 0.0;
  clr.a = 1.0;

  // Draw the bouncing square
  pioneerG3DIface->DrawSimple(sphereName.c_str(), 
        gazebo::Graphics3dDrawData::LINE_STRIP, vec, 5, clr);

}

void UpdateSphere()
{
  gazebo::Color clr;

  clr.r = 1.0;
  clr.g = 0.0;
  clr.b = 0.0;
  clr.a = 1.0;

  spherePos.x = radius * cos(theta);
  spherePos.y = radius * sin(theta);
  spherePos.z = 0.8;

  pioneerG3DIface->DrawShape("mysphere",
      gazebo::Graphics3dDrawData::SPHERE, spherePos, sphereSize, clr);

  theta += 0.1;

}

void UpdatePath()
{
  gazebo::Pose rPos;
  gazebo::Vec3 blockSize;
  gazebo::Color clr;
  std::ostringstream blockName;

  blockSize.x = 0.05;
  blockSize.y = 0.05;
  blockSize.z = 0.05;

  // Get the simulation pose of the robot
  simIface->Lock(1);
  simIface->data->requestCount = 0;
  simIface->Unlock();
  simIface->GetPose2d("pioneer2dx_model1", rPos);

  rPos.pos.z = 0.15;

  // Draw the robot's path
  if (positions.size() == 0 ||
      sqrt( pow(positions[positions.size()-1].x - rPos.pos.x,2) +
            pow(positions[positions.size()-1].y - rPos.pos.y,2)) > 0.5)
  {

    // Store the new position
    positions.push_back(rPos.pos);

    gazebo::Vec3 tmpvec[positions.size()];
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
        gazebo::Graphics3dDrawData::LINE_STRIP,
        tmpvec, positions.size(), clr);

    blockName << "path_block:" << positions.size();

    clr.r = 1.0;
    clr.g = 1.0;
    clr.b = 0.0;
    clr.a = 1.0;

    g3dIface->DrawShape(blockName.str().c_str(),
        gazebo::Graphics3dDrawData::CUBE, tmpvec[positions.size()-1], 
        blockSize, clr);
  }
}

void UpdateText()
{
  gazebo::Vec3 pos;
  char vel[50];
  float fontSize = 0.1;

  pos.x = 0;
  pos.y = 0;
  pos.z = 0.2;

  sprintf(vel,"Linear %4.2f Angular %4.2f",
      posIface->data->velocity.pos.x,
      posIface->data->velocity.yaw);

  // Draw some text on the robot
  pioneerG3DIface->DrawText(textName.c_str(), vel, pos, fontSize);
}

int main()
{
  client = new gazebo::Client();
  simIface = new gazebo::SimulationIface();
  posIface = new gazebo::PositionIface();
  g3dIface = new gazebo::Graphics3dIface();
  pioneerG3DIface = new gazebo::Graphics3dIface();

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
    posIface->Open(client, "pioneer2dx_model1::position_iface_0");
    pioneerG3DIface->Open(client,"pioneer2dx_model1");
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

  // Draw a billboard
  pioneerG3DIface->DrawBillboard("mybillboard", "Gazebo/SmileySad",
      gazebo::Vec3(0.4,0.0,0.4), gazebo::Vec2(0.2, 0.2)  );

  // Update all the drawables
  while (true)
  {
    UpdateSquare();
    UpdateSphere();
    UpdatePath();
    UpdateText();

    usleep(20000);
  }

  simIface->Close();
  posIface->Close();
  pioneerG3DIface->Close();
  g3dIface->Close();

  delete simIface;
  delete posIface;
  delete pioneerG3DIface;
  delete g3dIface;
  delete client;
  return 0;
}

