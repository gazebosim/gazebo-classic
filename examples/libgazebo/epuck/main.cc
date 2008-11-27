#include <sstream>
#include <string.h>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <math.h>

#include "EpuckModel.hh"

#define DTOR(x) x*3.1415926/180.0
const int swarm_size = 60;

int main()
{
  gazebo::Client *client = new gazebo::Client();
  gazebo::SimulationIface *simIface = new gazebo::SimulationIface();
  gazebo::FactoryIface *factoryIface = new gazebo::FactoryIface();

 
  int serverId = 0;

  /// Connect to the libgazebo server
  try
  {
    client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
  }
  catch (gazebo::GazeboError e)
  {
    std::cout << "Gazebo error: Unable to connect\n" << e << "\n";
    return -1;
  }
 

  /// Open the Simulation Interface
  try
  {
    simIface->Open(client, "default");
  }
  catch (gazebo::GazeboError e)
  {
    std::cout << "Gazebo error: Unable to connect to the sim interface\n" << e << "\n";
    return -1;
  }
  
  

  /// Open the Factory interface
  try
  {
    factoryIface->Open(client, "factory_model-factory_iface");
  }
  catch (gazebo::GazeboError e)
  {
    std::cout << "Gazebo error: Unable to connect to the factory interface\n"
    << e << "\n";
    return -1;
  }
  
  //Deploy 60 robots in the simulator
  Pose pose;
  pose.pos.y = 0.6;
  pose.pos.x = 0;
  pose.pos.z = 0.01;
  pose.roll = 0;
  pose.pitch = 0;
  pose.yaw = 0;
  Epuck *robots = new Epuck[swarm_size];
  double angle=15;
  double radius=0.8;
  
  
  for (int i=0; i<swarm_size; i++)
  {
    factoryIface->Lock(1);
    if (!strcmp((const char*)factoryIface->data->newModel,""))
    {
     // std::ostringstream stream;
      /*
      if(i==8)
        pose.pos.y = 0.4;
      else if(i==16)
        pose.pos.y = 0.2;
      else if(i==24)
        pose.pos.y = -0.0;
      else if(i==32)
        pose.pos.y = -0.2;
      else if(i==40)
        pose.pos.y = -0.4;
      else if(i==48)
        pose.pos.y = -0.6;
      else if(i==56)
        pose.pos.y = -0.8;
         
      pose.pos.x = -0.7 + (i%8) * 0.2;*/
      
      if(i<24)
      {
      	radius=0.8;
      	angle = 15;
        pose.pos.x = radius * cos(DTOR(i*angle));
        pose.pos.y = radius * sin(DTOR(i*angle));
        pose.yaw = i * angle + 90;

      }
      else if(i<48)
      {
      	radius=0.6;
      	angle = 15;
      	pose.pos.x = radius * cos(DTOR(i*angle));
        pose.pos.y = radius * sin(DTOR(i*angle));
        pose.yaw = i * angle -90;
      }
      else
      {
      	radius=0.4;
      	angle = 30;
      	pose.pos.x = radius * cos(DTOR(i*angle));
        pose.pos.y = radius * sin(DTOR(i*angle));
        pose.yaw = i * angle +90;
      }
      
     //CreateEpuckModel( i, pose);
      
      //std::cout<<stream;

      printf("Creating[%d]\n",i);
      //if(i==10 || i==8)
      //  strcpy((char*)factoryIface->data->newModel, robots[i].CreateEpuckModel( i+1, pose,true));//stream.str().c_str());
     // else
        strcpy((char*)factoryIface->data->newModel, robots[i].CreateEpuckModel( i+1, pose, true));//stream.str().c_str());
      
      
    }
    factoryIface->Unlock();
    usleep(200000);
  }
  
  for(int i=0;i<swarm_size;i++)
  {
  	robots[i].Subscribe(client);
  	robots[i].EnableMotor();
  }
  
  double lastupdateTime=0;
  double currentTime=0;
  
  while(true)
  {
  	simIface->Lock(1);
  	currentTime = simIface->data->simTime;
  	simIface->Unlock();
  	
  	if(currentTime - lastupdateTime >=0.01)
  	{
  	   lastupdateTime = currentTime;
  	   if(currentTime < 30)
  	   for(int i=0;i<swarm_size;i++)
  	   {   //robots[i].Update();
  	   	  if(i<24)
  	        robots[i].DoCircle(0.15, 0.8);
  	      else if(i<48)
  	        robots[i].DoCircle(0.15, 0.6, false);
  	      else
  	        robots[i].DoCircle(0.15,0.4);
  	      
  	   }
  	   else
  	   for(int i=0;i<swarm_size;i++)
  	   {   
         robots[i].Update();
  	   }         
  	}  	
  }

  
  return 0;
}

