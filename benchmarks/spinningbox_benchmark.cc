#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <libgazebo/gz.h>

libgazebo::Client *client = NULL;
libgazebo::SimulationIface *simIface = NULL;
libgazebo::FactoryIface *factoryIface = NULL;

std::string test_name="Spinning Box Benchmark";
std::string xlabel = "Spinning Box Count";
std::string ylabel = "Simtime / Realtime";
std::string data_filename = "/tmp/" + test_name + ".data";

void make_plot()
{
  std::ostringstream cmd;

  cmd << "echo \"";
  cmd << "set xlabel '" << xlabel << "'\n";
  cmd << "set ylabel '" << ylabel << "'\n";
  cmd << "set title '" << test_name << "'\n";
  cmd << "set terminal png\n";
  cmd << "set output '" << test_name << ".png'\n";
  cmd << "plot '" << data_filename << "' with lines\n";
  cmd << "\" | gnuplot";

  if (system(cmd.str().c_str() ) < 0)
    std::cerr << "Error\n";
}

void spawn_box()
{
  std::ostringstream model;

  model << "<model:physical name='box'>";
  model << "  <static>false</static>";
  model << "  <xyz>0 0 0.5</xyz>";
  model << "  <rpy>00 00 0</rpy>";
  model << "  <body:box name='body'>";
  model << "    <geom:box name='geom'>";
  model << "      <size>1 1 1</size>";
  model << "      <mass>1</mass>";
  model << "      <mu1>.1</mu1>";
  model << "      <mu2>.1</mu2>";
  model << "      <visual>";
  model << "        <size>1 1 1</size>";
  model << "        <mesh>unit_box</mesh>";
  model << "        <material>Gazebo/Rocky</material>";
  model << "      </visual>";
  model << "    </geom:box>";
  model << "  </body:box>";
  model << "</model:physical>";

  factoryIface->Lock(1);
  strcpy( (char*)factoryIface->data->newModel, model.str().c_str() );
  factoryIface->Unlock();
}

int main(int argc, char **argv)
{
  client = new libgazebo::Client();
  simIface = new libgazebo::SimulationIface();
  factoryIface = new libgazebo::FactoryIface();

  float vel = 1.0;
  if (argc > 1)
    vel = atof(argv[1]);

  try
  {
    client->ConnectWait(0, GZ_CLIENT_ID_USER_FIRST);
  }
  catch(std::string e)
  {
    std::cerr << "Gazebo Error: Unable to connect: " << e << "\n";
    return -1;
  }

  /// Open the sim iface
  try
  {
    simIface->Open(client, "default");
  }
  catch (std::string e)
  {
    std::cerr << "Gazebo error: Unable to connect to sim iface:" << e << "\n";
    return -1;
  }

  // Open the factory iface
  try
  {
    factoryIface->Open(client, "default");
  }
  catch( std::string e)
  {
    std::cerr << "Gazebo error: Unable to connect to the factory Iface:" 
              << e << "\n";
    return -1;
  }
  spawn_box();

  usleep(5000000);
  //FILE *out = fopen(data_filename.c_str(), "w");

  /*if (!out)
    std::cerr << "Unable to open log file";
    */

  //double simTime = 0;
  //double realTime = 0;


  libgazebo::Vec3 linearVel, angularVel, linearAccel, angularAccel;
  libgazebo::Pose modelPose;


  bool done = false;
  while (!done)
  {
    simIface->SetAngularVel("box", libgazebo::Vec3(0,0,vel));
    
    simIface->GetState("box", modelPose, linearVel, angularVel, 
                       linearAccel, angularAccel);
    printf("Pos[%4.2f %4.2f %4.2f] RPY[%4.2f %4.2f %4.2f] ", modelPose.pos.x, modelPose.pos.y, modelPose.pos.z, modelPose.roll, modelPose.pitch, modelPose.yaw);
    printf("LV[%4.2f %4.2f %4.2f] ",linearVel.x, linearVel.y, linearVel.z );
    printf("AV[%4.2f %4.2f %4.2f] ",angularVel.x, angularVel.y, angularVel.z);
    printf("LA[%4.2f %4.2f %4.2f] ",linearAccel.x, linearAccel.y, linearAccel.z);
    printf("AA[%4.2f %4.2f %4.2f]\n",angularAccel.x, angularAccel.y, angularAccel.z );
  }

  //double percent = simTime / realTime;
  //fprintf(out,"%f\n",percent);

  //fclose(out);

  //make_plot();

  simIface->Close();
  client->Disconnect();
}
