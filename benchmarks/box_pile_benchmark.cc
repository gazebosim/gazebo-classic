#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <libgazebo/gz.h>

gazebo::Client *client = NULL;
gazebo::SimulationIface *simIface = NULL;
gazebo::FactoryIface *factoryIface = NULL;
int laser_count = 0;

std::string test_name="Box Pile Benchmark";
std::string xlabel = "Box Count";
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

void spawn_box(double x, double y, double z=10)
{
  std::ostringstream model;

  model << "<model:physical name='box" << laser_count++ << "'>";
  model << "  <xyz>" << x << " " << y << " " << z << "</xyz>";
  model << "  <rpy>30 40 0</rpy>";
  model << "  <body:box name='body'>";

  model << "    <geom:box name='geom'>";
  model << "      <size>0.5 0.5 0.5</size>";
  model << "      <mass>1</mass>";
  model << "      <visual>";
  model << "        <size>0.5 0.5 0.5</size>";
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

int main()
{
  client = new gazebo::Client();
  simIface = new gazebo::SimulationIface();
  factoryIface = new gazebo::FactoryIface();

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

  FILE *out = fopen(data_filename.c_str(), "w");

  for (unsigned int j=0; j < 400; j++)
  {
    spawn_box(0, 0);

    double simTime = 0;
    double realTime = 0;

    for (unsigned int i=0; i < 30; i++)
    {
      simTime += simIface->data->simTime;
      realTime += simIface->data->realTime;

      /// Wait .1 seconds 
      usleep(100000);
    }

    double percent = simTime / realTime;
    fprintf(out,"%f\n",percent);

    std::cout << "Index[" << j << "] = " << percent << "\n";
  }
  fclose(out);

  make_plot();

  factoryIface->Close();
  simIface->Close();
  client->Disconnect();
}
