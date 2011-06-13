#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include "common/Timer.hh"
#include "common/Exception.hh"
#include "common/SystemPaths.hh"
#include "common/XMLConfig.hh"

#include "sensors/Sensors.hh"
#include "transport/Transport.hh"
#include "transport/IOManager.hh"

#include "physics/Physics.hh"
#include "rendering/Rendering.hh"

#include "Master.hh"
#include "Server.hh"

using namespace gazebo;

const std::string default_config =
"<?xml version='1.0'?>\
<gazebo>\
  <config>\
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
  </world>\
</gazebo>";


Server::Server()
{
  this->stop = false;

  // load the configuration options 
  try
  {
    common::SystemPaths::Instance()->Load();
  }
  catch (common::Exception e)
  {
    gzthrow("Error loading the Gazebo configuration file, check the .gazeborc file on your HOME directory \n" << e); 
  }
}

Server::~Server()
{
  delete this->master;

  // TODO: probably should clean this up. Make sure this is needed.
  /*while (transport::IOManager::Instance()->GetCount() > 0)
  {
    usleep(100000);
    gzdbg << "Connectionmanager::Destructor IOCount[" << transport::IOManager::Instance()->GetCount() << "]\n";
  }
  */
}

void Server::Load(const std::string &filename)
{
  std::string host = "";
  unsigned short port = 0;

  gazebo::transport::get_master_uri(host,port);

  this->master = new gazebo::Master();
  this->master->Init(port);
  this->master->Run();

  transport::init();
  physics::init();

  /// Init the sensors library
  sensors::init("default");

  // Load the world file
  gazebo::common::XMLConfig *xmlFile = new gazebo::common::XMLConfig();

  try
  {
    if (!filename.empty())
      xmlFile->Load(filename);
    else
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

  // Load the rendering system
  if (!rendering::load(filename))
    gzthrow("Unable to load the rendering engine");

  // The rendering engine will run headless 
  if (!rendering::init())
    gzthrow("Unable to intialize the rendering engine");

  rendering::create_scene("default");
}

void Server::Init()
{
  for (unsigned int i=0; i < this->worlds.size(); i++)
    physics::init_world(this->worlds[i]);
}

void Server::Stop()
{
  this->stop = true;
}

void Server::Fini()
{
  transport::fini();
  physics::fini();
  rendering::fini();

  this->master->Fini();
  delete this->master;
  this->master = NULL;
}

void Server::Run()
{
  // Run the transport loop: starts a new thread
  transport::run();

  // Run each world. Each world starts a new thread
  for (unsigned int i=0; i < this->worlds.size(); i++)
    physics::run_world(this->worlds[i]);

  // Update the sensors.
  this->stop = false;
  while (!this->stop)
  {
    sensors::run_once(true);
    usleep(10000);
  }

  // Stop all the worlds
  for (unsigned int i=0; i < this->worlds.size(); i++)
    physics::stop_world(this->worlds[i]);

  // Stop the transport loop
  transport::stop();

  // Stop the master 
  this->master->Stop();
}

void Server::SetParams( const common::StrStr_M &params )
{
  common::StrStr_M::const_iterator iter;
  for (iter = params.begin(); iter != params.end(); iter++)
  {
    if (iter->first == "pause")
    {
      bool p = false;
      try
      {
        p = boost::lexical_cast<bool>(iter->second);
      }
      catch (...)
      {
        // Unable to convert via lexical_cast, so try "true/false" string
        std::string str = iter->second;
        boost::to_lower(str);

        if (str == "true")
          p = true;
        else if (str == "false")
          p = false;
        else
          gzerr << "Invalid param value[" << iter->first << ":" 
                << iter->second << "]\n";
      }

      for (unsigned int i=0; i < this->worlds.size(); i++)
        physics::pause_world(this->worlds[i], p);
    }
  }
}
