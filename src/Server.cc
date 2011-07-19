#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include "common/Timer.hh"
#include "common/Exception.hh"

#include "sdf/sdf.h"
#include "sdf/sdf_parser.h"

#include "sensors/Sensors.hh"
//#include "transport/Transport.hh"
//#include "transport/IOManager.hh"

#include "physics/Physics.hh"
#include "physics/World.hh"
#include "physics/Base.hh"
//#include "rendering/Rendering.hh"

#include "gazebo.h"
#include "Master.hh"
#include "Server.hh"

using namespace gazebo;


Server::Server()
{
  this->stop = false;

  std::string host = "";
  unsigned short port = 0;

  gazebo::transport::get_master_uri(host,port);

  this->master = new gazebo::Master();
  this->master->Init(port);
  this->master->Run();
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
  // Load gazebo
  gazebo::load();

  // Load the world file
  sdf::SDFPtr sdf(new sdf::SDF);
  sdf::init(sdf);
  sdf::readFile(filename, sdf);

  /// Load the physics library
  physics::load();

  /// Load the sensors library
  sensors::load();

  sdf::ElementPtr worldElem = sdf->root->GetElement("world");
  while(worldElem)
  {
    physics::WorldPtr world = physics::create_world("default");

    this->worlds.push_back(world);

    //Create the world
    try
    {
      physics::load_world(world, worldElem);
    }
    catch (common::Exception e)
    {
      gzthrow("Failed to load the World\n"  << e);
    }

    worldElem = sdf->root->GetNextElement("world", worldElem);
  }

  /*// Load the rendering system
  if (!rendering::load())
    gzthrow("Unable to load the rendering engine");

  // The rendering engine will run headless 
  if (!rendering::init())
    gzthrow("Unable to intialize the rendering engine");

  rendering::create_scene("default");
  */
}

void Server::Init()
{
  sensors::init();

  for (unsigned int i=0; i < this->worlds.size(); i++)
    physics::init_world(this->worlds[i]);
}

void Server::Stop()
{
  this->stop = true;
}

void Server::Fini()
{
  gazebo::fini();

  physics::fini();
  sensors::fini();

  //rendering::fini();

  this->master->Fini();
  delete this->master;
  this->master = NULL;
}

void Server::Run()
{
  // Run the transport loop: starts a new thread
  gazebo::run();

  // Run each world. Each world starts a new thread
  for (unsigned int i=0; i < this->worlds.size(); i++)
    physics::run_world(this->worlds[i]);

  // Update the sensors.
  sensors::run();

  this->stop = false;
  while (!this->stop)
    usleep(1000000);

  // Stop all the worlds
  for (unsigned int i=0; i < this->worlds.size(); i++)
    physics::stop_world(this->worlds[i]);

  sensors::stop();

  // Stop the transport loop
  gazebo::stop();

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


physics::BasePtr Server::GetByName(const std::string &name)
{
  physics::BasePtr result;
  result.reset();
  for (unsigned int i=0; i < this->worlds.size(); i++)
  {
    std::cout << "world " << i << " : " << "\n";
    result = this->worlds[i]->GetByName(name);
    if (result) break;
  }
  return result;
}
