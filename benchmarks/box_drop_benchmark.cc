#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <libgazebo/gz.h>
#include <boost/lexical_cast.hpp>

libgazebo::Client *client = NULL;
libgazebo::SimulationIface *simIface = NULL;
libgazebo::FactoryIface *factoryIface = NULL;
int laser_count = 0;

std::string test_name="Box Drop Benchmark";
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

void spawn_box(double x, double y, double z=1)
{
  std::ostringstream model;

  model << "<model:physical name='box'>";
  model << "  <xyz>" << x << " " << y << " " << z << "</xyz>";
  model << "  <rpy>0 0 0</rpy>";
  model << "  <body:box name='body'>";
  model << "    <geom:box name='geom'>";
  model << "      <size>0.5 0.5 0.5</size>";
  model << "      <mass>1</mass>";
  model << "      <kp>100000000.0</kp>";
  model << "      <kd>1.0</kd>";
  model << "      <bounce>0</bounce>";
  model << "      <bounceVel>100000</bounceVel>";
  model << "      <slip1>0.01</slip1>";
  model << "      <slip2>0.01</slip2>";
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

void RunSim()
{
  simIface->StartLogEntity("box", "/tmp/box.log");
  simIface->Unpause();
  /*libgazebo::Vec3 linearVel, angularVel, linearAccel, angularAccel;
  libgazebo::Pose modelPose;

  double time = simIface->data->realTime;
  while ( simIface->data->realTime - time < 10.0 )
  {
    //simIface->GetState("box", modelPose, linearVel, angularVel, linearAccel, angularAccel); 

    //if ( fabs(prev_z - modelPose.pos.z) >= 1e-5)
      //time = simIface->data->realTime;

    //prev_z = modelPose.pos.z;
  }
  */
  usleep(10000000);

  simIface->StopLogEntity("box");
  simIface->Pause();
  simIface->Reset();
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
  double prev_z = 5;
  spawn_box(0, 0, prev_z);
  usleep(10000);


  std::vector<std::string > step_types;
  std::vector<std::string >::iterator iter;
  //step_types.push_back("robust");
  step_types.push_back("world");
  step_types.push_back("quick");

  for (iter = step_types.begin(); iter != step_types.end(); iter++)
  {
    std::string path = std::string("/home/nate/work/simpar/data/box_drop/") + *iter + "/";

    system((std::string("mkdir -p ")+path).c_str());

    FILE *out = fopen(std::string(path+"index.txt").c_str(), "w");
    fprintf(out,"# index step_time\n");

    int i = 0;

    simIface->SetStepType(*iter);
    for (double step=0.1; step > 1e-5; step *= 0.5)
    {
      unsigned int iterations = 10;
      if (*iter == "world")
        iterations = 199;
      for (; iterations < 200; iterations +=20, i++)
      {
        simIface->SetStepTime(step);
        simIface->SetStepIterations(iterations);

        fprintf(out,"%d %f %d\n",i,step, iterations);
        std::cout << "Type[" << *iter << "] Step[" << step << "] Iterations[" << iterations << "]\n";
        RunSim();
        std::string mv_cmd = std::string("mv /tmp/box.log ") + path + "box_drop_benchmark_" + *iter + "_" + boost::lexical_cast<std::string>(i) + ".data";
        system(mv_cmd.c_str());
      }
    }
    fclose(out);
  }

  factoryIface->Close();
  simIface->Close();
  client->Disconnect();
}


