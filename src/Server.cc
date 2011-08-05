#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include "common/Timer.hh"
#include "common/Exception.hh"

#include "sdf/sdf.h"
#include "sdf/sdf_parser.h"

#include "sensors/Sensors.hh"

#include "physics/Physics.hh"
#include "physics/World.hh"
#include "physics/Base.hh"

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
}

void Server::Load(const std::string &filename)
{
  // Load gazebo
  gazebo::load();

  // Load the world file
  sdf::SDFPtr sdf(new sdf::SDF);
  sdf::init(sdf);
  sdf::readFile(filename, sdf);

  /// Load the sensors library
  sensors::load();

  /// Load the physics library
  physics::load();

  sdf::ElementPtr worldElem = sdf->root->GetElement("world");
  while(worldElem)
  {
    physics::WorldPtr world = physics::create_world();

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
}

void Server::Init()
{
  sensors::init();

  physics::init_worlds();
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

  this->master->Fini();
  delete this->master;
  this->master = NULL;
}

void Server::Run()
{
  // Run the gazebo, starts a new thread
  gazebo::run();

  // Run each world. Each world starts a new thread
  physics::run_worlds();

  // Update the sensors.
  this->stop = false;
  while (!this->stop)
  {
    sensors::run_once(true);
    usleep(100000);
  }

  // Stop all the worlds
  physics::stop_worlds();

  sensors::stop();

  // Stop gazebo
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

      physics::pause_worlds(p);
    }
  }
}

