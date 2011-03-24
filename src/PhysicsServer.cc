#include "common/GazeboConfig.hh"
#include "common/XMLConfig.hh"
#include "common/GazeboError.hh"

#include "physics/World.hh"
#include "physics/PhysicsFactory.hh"

#include "transport/Publisher.hh"
#include "transport/IOManager.hh"
#include "transport/TopicManager.hh"

#include "PhysicsServer.hh"

using namespace gazebo;

const std::string default_config =
"<?xml version='1.0'?>\
<gazebo>\
  <world name='default'>\
    <scene>\
      <ambient>0.1 0.1 0.1 1</ambient>\
      <background_color>.1 .1 .1 1.0</background_color>\
      <shadows enabled='false' color='0.2 0.2 0.2 1.0' type='texture_modulative'/>\
      <grid>false</grid>\
    </scene>\
    <physics type='ode'>\
      <stepTime>0.001</stepTime>\
      <gravity>0 0 -9.8</gravity>\
      <cfm>0.0000000001</cfm>\
      <erp>0.2</erp>\
      <stepType>quick</stepType>\
      <stepIters>10</stepIters>\
      <stepW>1.3</stepW>\
      <contactMaxCorrectingVel>100.0</contactMaxCorrectingVel>\
      <contactSurfaceLayer>0.0</contactSurfaceLayer>\
    </physics>\
    <model name='plane1_model'>\
      <static>true</static>\
      <link name='body'>\
        <collision name='geom'>\
          <geometry>\
            <plane normal='0 0 1'/>\
          </geometry>\
          <contact_coefficients mu1='109999.0' mu2='1000.0'/>\
        </collision>\
        <visual>\
          <geometry>\
            <plane normal='0 0 1' size='100 100' offset='0'/>\
          </geometry>\
          <material name='Gazebo/GreyGrid' uv_tile='100 100'/>\
          <cast_shadows>false</cast_shadows>\
        </visual>\
      </link>\
    </model>\
  </world>\
</gazebo>";

PhysicsServer::PhysicsServer()
  : node(new common::Node())
{
  this->quit = false;

  // load the configuration options 
  try
  {
    common::GazeboConfig::Instance()->Load();
  }
  catch (common::GazeboError e)
  {
    gzthrow("Error loading the Gazebo configuration file, check the .gazeborc file on your HOME directory \n" << e); 
  }

  this->node->Init("localhost", 11345);

  physics::PhysicsFactory::RegisterAll();
}

PhysicsServer::~PhysicsServer()
{
}

void PhysicsServer::Load( const std::string &filename )
{
  // Load the world file
  gazebo::common::XMLConfig *xmlFile = new gazebo::common::XMLConfig();

  try
  {
    if (!filename.empty())
      xmlFile->Load(filename);
    else
      xmlFile->LoadString(default_config);
  }
  catch (common::GazeboError e)
  {
    gzthrow("The XML config file can not be loaded, please make sure is a correct file\n" << e); 
  }

  common::XMLConfigNode *rootNode(xmlFile->GetRootNode());
  if (!rootNode || rootNode->GetName() != "gazebo")
    gzthrow("Invalid xml. Needs a root node with the <gazebo> tag");

  common::XMLConfigNode *worldNode = rootNode->GetChild("world");

  while(worldNode)
  {
    physics::World *world = new physics::World();
    this->worlds.push_back(world);

    //Create the world
    try
    {
      world->Load(worldNode);
    }
    catch (common::GazeboError e)
    {
      gzthrow("Failed to load the World\n"  << e);
    }

    //common::XMLConfigNode *pluginNode = worldNode->GetChild("plugin");
    //while (pluginNode != NULL)
    //{
    //  this->AddPlugin( pluginNode->GetString("filename","",1), 
    //      pluginNode->GetString("handle","",1) );
    //  pluginNode = pluginNode->GetNext("plugin");
    //}

    worldNode = worldNode->GetNext("world");
  }
}

void PhysicsServer::Init()
{
}

void PhysicsServer::Run()
{
  //msgs::String msg;
  //msg.set_data("Hello");

  //transport::PublisherPtr pub = this->node->Advertise<msgs::String>("/gazebo/test");

  while (!this->quit)
  {
    //pub->Publish(msg);
    //for (int i=0; i < this->worlds.size(); i++)
      //this->worlds[i]->Update();
    usleep(1000000);
  }
}

void PhysicsServer::Quit()
{
  this->quit = true;
}
