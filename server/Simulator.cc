/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/* Desc: The Simulator; Top level managing object
 * Author: Nate Koenig, Jordi Polo
 * Date: 3 Jan 2008
 */

#include <assert.h>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>

#include "IOManager.hh"
#include "Events.hh"
#include "RenderState.hh"
#include "PhysicsFactory.hh"
#include "gazebo_config.h"
#include "Plugin.hh"
#include "Timer.hh"
#include "Body.hh"
#include "Geom.hh"
#include "Model.hh"
#include "Entity.hh"
#include "Visual.hh"
#include "World.hh"
#include "XMLConfig.hh"
#include "SimulationApp.hh"
#include "GazeboConfig.hh"
#include "gz.h"
#include "PhysicsEngine.hh"
#include "OgreAdaptor.hh"
#include "GazeboMessage.hh"
#include "GazeboError.hh"
#include "Global.hh"

#include "Simulator.hh"

using namespace gazebo;

std::string Simulator::defaultConfigXML =
"<?xml version='1.0'?>\
<gazebo>\
  <config>\
    <verbosity>4</verbosity>\
    <gui>\
      <size>800 600</size>\
      <pos>0 0</pos>\
    </gui>\
    <rendering>\
      <ambient>.1 .1 .1 1</ambient>\
      <shadows>true</shadows>\
      <grid>false</grid>\
    </rendering>\
  </config>\
</gazebo>";

std::string Simulator::defaultWorldXML =
"<?xml version='1.0'?>\
<gazebo>\
  <world name='default'>\
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
    <!-- Ground Plane -->\
    <model name='plane1_model'>\
      <xyz>0 0 0</xyz>\
      <rpy>0 0 0</rpy>\
      <static>true</static>\
      <body name='plane1_body'>\
        <geom type='plane' name='plane1_geom'>\
          <normal>0 0 1</normal>\
          <size>100 100</size>\
          <segments>1  1</segments>\
          <uv_tile>100 100</uv_tile>\
          <mu1>109999.0</mu1>\
          <mu2>1000.0</mu2>\
          <material>Gazebo/Grey</material>\
        </geom>\
      </body>\
    </model>\
    <!-- White Point light -->\
    <light name='point_white'>\
      <xyz>0.0 0 1</xyz>\
      <rpy>0 0 0</rpy>\
      <type>point</type>\
      <diffuse_color>0.6 0.6 0.6 1.0</diffuse_color>\
      <specular_color>.1 .1 .1 1.0</specular_color>\
      <attenuation>.2 0.1 0.0</attenuation>\
      <range>20</range>\
      <direction>0 0 -1.0</direction>\
      <cast_shadows>true</cast_shadows>\
    </light>\
  </world>\
</gazebo>";

////////////////////////////////////////////////////////////////////////////////
// Constructor
Simulator::Simulator()
: gui(NULL),
  renderEngine(NULL),
  gazeboConfig(NULL),
  physicsUpdates(0),
  renderUpdates(0),
  stepInc(false),
  userQuit(false),
  physicsQuit(false),
  guiEnabled(true),
  renderEngineEnabled(true),
  physicsEnabled(true)
{
  PhysicsFactory::RegisterAll();
  this->activeWorldIndex = 0;

  //this->render_mutex = new boost::recursive_mutex();
  //this->model_delete_mutex = new boost::recursive_mutex();
  this->gazeboConfig=new gazebo::GazeboConfig();

  this->quitConnection = Events::ConnectQuitSignal( boost::bind(&Simulator::SetUserQuit, this) );
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Simulator::~Simulator()
{
  this->Fini();
  if (this->gazeboConfig)
  {
    delete this->gazeboConfig;
    this->gazeboConfig = NULL;
  }

  /* NATY
  if (this->render_mutex)
  {
    delete this->render_mutex;
    this->render_mutex = NULL;
  }

  if (this->model_delete_mutex)
  {
    delete this->model_delete_mutex;
    this->model_delete_mutex = NULL;
  }
  */
}

////////////////////////////////////////////////////////////////////////////////
/// Finishes the Simulator and frees everything
void Simulator::Fini()
{
  if (this->renderEngineEnabled)
    gazebo::OgreAdaptor::Instance()->Fini();

  for (unsigned int i=0; i < this->worlds.size(); i++)
  {
    this->worlds[i]->Fini();
    delete this->worlds[i];
  }
  this->worlds.clear();

  if (this->gui)
  {
    delete this->gui;
    this->gui = NULL;
  }

  google::protobuf::ShutdownProtobufLibrary();
  IOManager::Instance()->Stop();
}

////////////////////////////////////////////////////////////////////////////////
/// Load the world configuration file
/// Any error that reach this level must make the simulator exit
void Simulator::Load(const std::string &fileName)
{
  // Load the world file
  XMLConfig *xmlFile = new gazebo::XMLConfig();

  // load the configuration options 
  try
  {
    this->gazeboConfig->Load();
  }
  catch (GazeboError e)
  {
    gzthrow("Error loading the Gazebo configuration file, check the .gazeborc file on your HOME directory \n" << e); 
  }

  try
  {
    if (!fileName.empty())
      xmlFile->Load(fileName);
    else
      xmlFile->LoadString(defaultConfigXML);
  }
  catch (GazeboError e)
  {
    gzthrow("The XML config file can not be loaded, please make sure is a correct file\n" << e); 
  }

  XMLConfigNode *rootNode(xmlFile->GetRootNode());
  XMLConfigNode *configNode = rootNode->GetChild("config");

  // Load the messaging system
  gazebo::GazeboMessage::Instance()->Load(configNode);

  // Load the Ogre rendering system
  if (this->renderEngineEnabled)
    OgreAdaptor::Instance()->Load(configNode);

  // Create and initialize the Gui
  if (this->renderEngineEnabled && this->guiEnabled)
  {
    try
    {
      XMLConfigNode *childNode = NULL;
      if (rootNode)
       childNode = configNode->GetChild("gui");

      int width=0;
      int height=0;
      int x = 0;
      int y = 0;

      if (childNode)
      {
        width = childNode->GetTupleInt("size", 0, 800);
        height = childNode->GetTupleInt("size", 1, 600);
        x = childNode->GetTupleInt("pos",0,0);
        y = childNode->GetTupleInt("pos",1,0);
      }

      // Create the GUI
      if (!this->gui && (childNode || !rootNode))
      {
        this->gui = new SimulationApp();
        this->gui->Load();
      }
    }
    catch (GazeboError e)
    {
      gzthrow( "Error loading the GUI\n" << e);
    }
  }
  else
  {
    this->gui = NULL;
  }

  //Initialize RenderEngine
  if (this->renderEngineEnabled)
  {
    try
    {
      OgreAdaptor::Instance()->Init(configNode);
      this->renderEngine = OgreAdaptor::Instance();
    }
    catch (gazebo::GazeboError e)
    {
      gzthrow("Failed to Initialize the Rendering engine subsystem\n" << e );
    }
  }

  // Initialize the GUI
  if (this->gui)
  {
    this->gui->Init();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Create a new world 
World *Simulator::CreateWorld(const std::string &fileName)
{
  World *world = NULL;

  // Load the world file
  XMLConfig *xmlFile=new gazebo::XMLConfig();

  try
  {
    if (!fileName.empty())
      xmlFile->Load(fileName);
    else
      xmlFile->LoadString(defaultWorldXML);
  }
  catch (GazeboError e)
  {
    gzthrow("The XML config file can not be loaded, please make sure is a correct file\n" << e); 
  }

  XMLConfigNode *rootNode(xmlFile->GetRootNode());

  XMLConfigNode *worldNode = rootNode->GetChild("world");

  while(worldNode)
  {
    world = new World();
    this->worlds.push_back(world);

    //Create the world
    try
    {
      world->Load(worldNode);
    }
    catch (GazeboError e)
    {
      gzthrow("Failed to load the World\n"  << e);
    }

    XMLConfigNode *pluginNode = worldNode->GetChild("plugin");
    while (pluginNode != NULL)
    {
      this->AddPlugin( pluginNode->GetString("filename","",1), 
          pluginNode->GetString("handle","",1) );
      pluginNode = pluginNode->GetNext("plugin");
    }

    worldNode = worldNode->GetNext("world");
  }

  if (this->gui)
    this->gui->ViewScene(world->GetScene());

  return world;
}

////////////////////////////////////////////////////////////////////////////////
// Remove a world by name
void Simulator::RemoveWorld( const std::string &name )
{
  std::vector<World*>::iterator iter;

  for (iter = this->worlds.begin(); iter != this->worlds.end(); iter++)
    if ((*iter)->GetName() == name)
      break;

  if (iter != this->worlds.end())
  {
    delete *iter;
    this->worlds.erase(iter);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get the number of world
unsigned int Simulator::GetWorldCount() const
{
  return this->worlds.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Get a world by number
World *Simulator::GetWorld(unsigned int i) const
{
  if (i < this->worlds.size())
    return this->worlds[i];
  else
    return NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the active world
void Simulator::SetActiveWorld(unsigned int i)
{
  if (i < this->worlds.size())
    this->activeWorldIndex = i;
  else
    gzerr(0) << "Invalid world index[" << i << "]\n";
}

////////////////////////////////////////////////////////////////////////////////
/// Set the active world
void Simulator::SetActiveWorld(World *world)
{
  unsigned int i=0;
  for (; i < this->worlds.size(); i++)
    if (this->worlds[i]->GetName() == world->GetName())
      break;

  if (i < this->worlds.size())
    this->activeWorldIndex = i;
  else
    gzerr(0) << "Invalid world [" << world->GetName() << "]\n";
}

////////////////////////////////////////////////////////////////////////////////
/// Get the currently active world
World *Simulator::GetActiveWorld() const
{
  return this->worlds[this->activeWorldIndex];
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize the simulation
void Simulator::Init()
{
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  IOManager::Instance()->Start();

  RenderState::Init();

  //Initialize the world
  try
  {
    for (unsigned int i=0; i < this->worlds.size(); i++)
      this->worlds[i]->Init();
  }
  catch (GazeboError e)
  {
    gzthrow("Failed to Initialize the World\n"  << e);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Save the world configuration file
void Simulator::Save(const std::string& filename)
{
  std::fstream output;

  output.open(filename.c_str(), std::ios::out);

  if (!output.is_open())
  {
    gzerr(0) << "Unable to save file to[" << filename << "]\n";
    return;
  }

  std::string prefix = "  ";

  // Write out the xml header
  output << "<?xml version=\"1.0\"?>\n";
  output << "<gazebo>\n\n";

  output << "  <config>";
  GazeboMessage::Instance()->Save(prefix, output);
  output << "\n";

  this->GetRenderEngine()->Save(prefix, output);
  output << "\n";

  this->gui->Save(prefix, output);
  output << "\n";

  output << "  </config>";

  for (unsigned int i=0; i < this->worlds.size(); i++)
    this->worlds[i]->Save(prefix, output);
}

////////////////////////////////////////////////////////////////////////////////
/// Main simulation loop, when this loop ends the simulation finish
void Simulator::Run()
{
  Time currTime = 0;
  Time lastTime = 0;
  struct timespec timeSpec;
  double freq = 80.0;

  for (unsigned int i=0; i < this->worlds.size(); i++)
    this->worlds[i]->Start();

  if (this->gui)
    this->gui->Run();
  else
  {
    while (!this->userQuit)
    {
      currTime = Time::GetWallTime();
      if ( currTime - lastTime > 1.0/freq)
      {
        lastTime = Time::GetWallTime();
        this->GraphicsUpdate();
        currTime = Time::GetWallTime();
        if (currTime - lastTime < 1/freq)
        {
          Time sleepTime = ( Time(1.0/freq) - (currTime - lastTime));
          timeSpec.tv_sec = sleepTime.sec;
          timeSpec.tv_nsec = sleepTime.nsec;

          nanosleep(&timeSpec, NULL);
        }
      }
      else
      {
        Time sleepTime = ( Time(1.0/freq) - (currTime - lastTime));
        timeSpec.tv_sec = sleepTime.sec;
        timeSpec.tv_nsec = sleepTime.nsec;
        nanosleep(&timeSpec, NULL);
      }
    }
  }

  for (unsigned int i=0; i < this->worlds.size(); i++)
    this->worlds[i]->Stop();
}

////////////////////////////////////////////////////////////////////////////////
void Simulator::GraphicsUpdate()
{
  if (this->renderEngineEnabled)
  {
    OgreAdaptor::Instance()->UpdateScenes();
    for (unsigned int i=0; i < this->worlds.size(); i++)
      this->worlds[i]->GraphicsUpdate();
  }

  for (unsigned int i=0; i < this->worlds.size(); i++)
  {
    this->worlds[i]->ProcessEntitiesToLoad();
    this->worlds[i]->ProcessEntitiesToDelete();
  }

}

////////////////////////////////////////////////////////////////////////////////
/// Gets local configuration for this computer
GazeboConfig *Simulator::GetGazeboConfig() const
{
  return this->gazeboConfig;
}

////////////////////////////////////////////////////////////////////////////////
OgreAdaptor *Simulator::GetRenderEngine() const
{
  if (this->renderEngineEnabled)
    return this->renderEngine;
  else
    return NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Set the user quit flag
void Simulator::SetUserQuit()
{
  this->userQuit = true;
}

////////////////////////////////////////////////////////////////////////////////
// True if the gui is to be used
void Simulator::SetGuiEnabled( bool enabled )
{
  this->guiEnabled = enabled;
}

////////////////////////////////////////////////////////////////////////////////
/// Return true if the gui is enabled
bool Simulator::GetGuiEnabled() const
{
  return this->guiEnabled;
}

////////////////////////////////////////////////////////////////////////////////
// True if the gui is to be used
void Simulator::SetRenderEngineEnabled( bool enabled )
{
  this->renderEngineEnabled = enabled;
}

////////////////////////////////////////////////////////////////////////////////
// Set the physics enabled/disabled
void Simulator::SetPhysicsEnabled( bool enabled )
{
  this->physicsEnabled = enabled;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the number of plugins
unsigned int Simulator::GetPluginCount() const
{
  return this->plugins.size();
}

////////////////////////////////////////////////////////////////////////////////
// Get the name of a plugin
std::string Simulator::GetPluginName(unsigned int i) const
{
  std::string result;
  if (i < this->plugins.size())
    result = this->plugins[i]->GetHandle();

  return result;
}

////////////////////////////////////////////////////////////////////////////////
// Add a plugin
void Simulator::AddPlugin(const std::string &filename, const std::string &handle)
{
  Plugin *plugin = Plugin::Create(filename, handle);
  if (plugin)
  {
    plugin->Load();
    this->plugins.push_back(plugin);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Add a plugin
void Simulator::RemovePlugin(const std::string &name)
{
  std::vector<Plugin*>::iterator iter;
  for (iter = this->plugins.begin(); iter != this->plugins.end(); iter++)
  {
    if ((*iter)->GetHandle() == name)
    {
      delete (*iter);
      this->plugins.erase(iter);
      break;
    }
  }
}
