#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <libgazebo/gz.h>

libgazebo::Client *client = NULL;
libgazebo::SimulationIface *simIface = NULL;
libgazebo::FactoryIface *factoryIface = NULL;
int laser_count = 0;

std::string test_name="Laser Benchmark";
std::string xlabel = "Laser Count";
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

void spawn_laser(double x, double y, double z=0.05)
{
  std::ostringstream model;

  model << "<model:physical name='laser" << laser_count++ << "'>";
  model << "  <xyz>" << x << " " << y << " " << z << "</xyz>";
  model << "  <static>true</static>";
  model << "  <body:box name='body'>";

  model << "    <geom:box name='geom'>";
  model << "      <size>0.1 0.1 0.1</size>";
  model << "      <mass>0.1</mass>";
  model << "      <visual>";
  model << "        <size>0.1 0.1 0.1</size>";
  model << "        <mesh>unit_box</mesh>";
  model << "      </visual>";
  model << "    </geom:box>";

  model << "    <sensor:ray name='laser'>";
  model << "      <alwaysActive>true</alwaysActive>";
  model << "      <rayCount>260</rayCount>";
  model << "      <rangeCount>260</rangeCount>";
  model << "      <origin>0.02 0 0</origin>";
  model << "      <displayRays>fan</displayRays>";
  model << "      <minAngle>-90</minAngle>";
  model << "      <maxAngle>90</maxAngle>";
  model << "      <minRange>0.1</minRange>";
  model << "      <maxRange>8.0</maxRange>";
  model << "      <resRange>0.1</resRange>";
  model << "    </sensor:ray>";

  model << "  </body:box>";
  model << "</model:physical>";

  factoryIface->Lock(1);
  strcpy( (char*)factoryIface->data->newModel, model.str().c_str() );
  factoryIface->Unlock();
}

int main()
{
  client = new libgazebo::Client();
  simIface = new libgazebo::SimulationIface();
  factoryIface = new libgazebo::FactoryIface();

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

  if (!out)
    std::cerr << "Unable to open log file";

  for (int y = -10; y < 10; y++)
  {
    for (int x = -10; x < 10; x++)
    {
      spawn_laser(x,y);

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
      std::cout << "Index[" << y << " " << x << "] = " << percent << "\n";
    }
  }
  fclose(out);

  make_plot();

  factoryIface->Close();
  simIface->Close();
  client->Disconnect();
}
