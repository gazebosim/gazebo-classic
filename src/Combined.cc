#include "common/Timer.hh"
#include "common/GazeboConfig.hh"
#include "common/XMLConfig.hh"

#include "transport/Transport.hh"

#include "sensors/Sensors.hh"

#include "physics/Physics.hh"
#include "rendering/Rendering.hh"

#include "Combined.hh"

using namespace gazebo;

const std::string default_config =
"<?xml version='1.0'?>\
<gazebo>\
  <config>\
    <verbosity>4</verbosity>\
    <gui>\
      <size>800 600</size>\
      <pos>0 0</pos>\
    </gui>\
  </config>\
  <world name='default'>\
    <scene>\
      <ambient>0.1 0.1 0.1 1</ambient>\
      <background_color>.1 .1 .1 1.0</background_color>\
      <shadows enabled='false' color='0.2 0.2 0.2 1.0' type='texture_modulative'/>\
      <grid>false</grid>\
    </scene>\
    <physics type='ode'>\
      <step_time>0.001</step_time>\
      <gravity>0 0 -9.8</gravity>\
      <cfm>0.0000000001</cfm>\
      <erp>0.2</erp>\
      <step_type>quick</step_type>\
      <step_iters>10</step_iters>\
      <stepW>1.3</stepW>\
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>\
      <contact_surface_layer>0.0</contact_surface_layer>\
    </physics>\
    <model name='box1'>\
      <static>true</static>\
      <origin xyz='0 0 1'/>\
      <link name='body'>\
        <collision name='geom'>\
          <geometry>\
            <box size='1.0 1.0 1.0'/>\
          </geometry>\
          <mass>1.0</mass>\
        </collision>\
        <visual>\
          <geometry>\
            <box size='1.0 1.0 1.0'/>\
          </geometry>\
          <mesh filename='unit_box'/>\
        </visual>\
      </link>\
    </model>\
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


Combined::Combined()
{
  this->quit = false;

  // load the configuration options 
  try
  {
    common::GazeboConfig::Instance()->Load();
  }
  catch (common::Exception e)
  {
    gzthrow("Error loading the Gazebo configuration file, check the .gazeborc file on your HOME directory \n" << e); 
  }

  transport::init();

  physics::init();

  /// Init the sensors library
  sensors::init("default");
}

Combined::~Combined()
{
}

void Combined::Load()
{
  // Load the world file
  gazebo::common::XMLConfig *xmlFile = new gazebo::common::XMLConfig();

  try
  {
    xmlFile->LoadString(default_config);
  }
  catch (common::Exception e)
  {
    gzthrow("The XML config file can not be loaded, please make sure is a correct file\n" << e); 
  }

  common::XMLConfigNode *rootNode(xmlFile->GetRootNode());
  if (!rootNode || rootNode->GetName() != "gazebo")
    gzthrow("Invalid xml. Needs a root node with the <gazebo> tag");

  common::XMLConfigNode *worldNode = rootNode->GetChild("world");

  while(worldNode)
  {
    physics::WorldPtr world = physics::create_world("default");

    this->worlds.push_back(world);

    //Create the world
    try
    {
      physics::load_world(world, worldNode);
    }
    catch (common::Exception e)
    {
      gzthrow("Failed to load the World\n"  << e);
    }

    worldNode = worldNode->GetNext("world");
  }



  common::XMLConfigNode *configNode = rootNode->GetChild("config");
  if (!configNode)
    gzthrow("Invalid xml. Needs a <config> tag");

  // Load the rendering system
  if (!rendering::load(configNode))
    gzthrow("Unable to load the rendering engine");

  // The rendering engine will run headless 
  if (!rendering::init(true))
    gzthrow("Unable to intialize the rendering engine");

  rendering::create_scene("default");

  sensors::SensorPtr sensor1 = sensors::create_sensor("camera");
  sensors::SensorPtr sensor2 = sensors::create_sensor("camera");

}

void Combined::Init()
{
  for (int i=0; i < this->worlds.size(); i++)
    physics::init_world(this->worlds[i]);
}

void Combined::Run()
{
  for (int i=0; i < this->worlds.size(); i++)
    physics::run_world(this->worlds[i]);

  while (!this->quit)
  {
    //timer.Start();
    sensors::run_once(true);
    //std::cout << "Render Time[" << timer.GetElapsed() << "]\n";
  }
}

void Combined::Quit()
{
  this->quit = true;
}
