#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <libgazebo/gz.h>

libgazebo::Client *client = NULL;
libgazebo::SimulationIface *simIface = NULL;
libgazebo::FactoryIface *factoryIface = NULL;
int camera_count = 0;

std::string test_name="Camera Benchmark";
std::string xlabel = "Camera Count";
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

void spawn_camera(double x, double y, double z=1)
{
  std::ostringstream model;

  model << "<model:physical name='camera" << camera_count++ << "'>";
  model << "  <xyz>" << x << " " << y << " " << z << "</xyz>";
  model << "  <static>true</static>";
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

  model << "   <sensor:camera name='camera" << camera_count << "'>";
  model << "     <alwaysActive>true</alwaysActive>";
  model << "     <imageSize>640 480</imageSize>";
  model << "     <hfov>60</hfov>";
  model << "     <nearClip>0.1</nearClip>";
  model << "     <farClip>50</farClip>";
  model << "   </sensor:camera>";

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

  for (int y=-10; y < 10; y++)
  {
    for ( int x=-10; x < 10; x++)
    {
      spawn_camera(x, y);

      double simTime = 0;
      double realTime = 0;

      for (unsigned int i=0; i < 50; i++)
      {
        simTime += simIface->data->simTime;
        realTime += simIface->data->realTime;

        /// Wait .1 seconds 
        usleep(100000);
      }

      double percent = simTime / realTime;
      fprintf(out,"%f\n",percent);

    }
  }
  fclose(out);

  make_plot();

  factoryIface->Close();
  simIface->Close();
  client->Disconnect();
}
