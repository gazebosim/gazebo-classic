/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: The Simulator; Top level managing object
 * Author: Jordi Polo
 * Date: 3 Jan 2008
 */

#include <assert.h>
#include <iostream>
#include <fstream>
#include <sys/time.h>

#include "Body.hh"
#include "Geom.hh"
#include "Model.hh"
#include "Entity.hh"
#include "OgreVisual.hh"
#include "World.hh"
#include "Gui.hh"
#include "XMLConfig.hh"
#include "Gui.hh"
#include "GazeboConfig.hh"
#include "gazebo.h"
#include "PhysicsEngine.hh"
#include "OgreAdaptor.hh"
#include "GazeboMessage.hh"
#include "Global.hh"

#include "Simulator.hh"

#define MAX_FRAME_RATE 60

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Simulator::Simulator()
: xmlFile(NULL),
  gui(NULL),
  renderEngine(NULL),
  gazeboConfig(NULL),
  loaded(false),
  pause(false),
  iterations(0),
  simTime(0.0),
  pauseTime(0.0),
  startTime(0.0),
  physicsUpdates(0),
  checkpoint(0.0),
  renderUpdates(0),
  userPause(false),
  userStepInc(false),
  userQuit(false),
  guiEnabled(true),
  physicsEnabled(true),
  timeout(-1),
  selectedEntity(NULL)
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Simulator::~Simulator()
{
  this->Close();
}

////////////////////////////////////////////////////////////////////////////////
/// Closes the Simulator and frees everything
void Simulator::Close()
{
  if (!this->loaded)
    return;

  GZ_DELETE (this->gui)
  GZ_DELETE (this->xmlFile)
  GZ_DELETE (this->gazeboConfig)
  gazebo::World::Instance()->Close();
  gazebo::OgreAdaptor::Instance()->Close();

  //GZ_DELETE(this->renderEngine);
}

////////////////////////////////////////////////////////////////////////////////
/// Load the world configuration file
/// Any error that reach this level must make the simulator exit
void Simulator::Load(const std::string &worldFileName, unsigned int serverId )
{
  if (loaded)
  {
    this->Close();
    loaded=false;
  }

  // Load the world file
  this->xmlFile=new gazebo::XMLConfig();
  try
  {
    this->xmlFile->Load(worldFileName);
  }
  catch (GazeboError e)
  {
    gzthrow("The XML config file can not be loaded, please make sure is a correct file\n" << e); 
  }

  XMLConfigNode *rootNode(xmlFile->GetRootNode());

  // Load the messaging system
  gazebo::GazeboMessage::Instance()->Load(rootNode);

  // load the configuration options 
  this->gazeboConfig=new gazebo::GazeboConfig();
  try
  {
    this->gazeboConfig->Load();
  }
  catch (GazeboError e)
  {
    gzthrow("Error loading the Gazebo configuration file, check the .gazeborc file on your HOME directory \n" << e); 
  }

  // Load the Ogre rendering system
  OgreAdaptor::Instance()->Load(rootNode);

  // Create and initialize the Gui
  if (this->guiEnabled)
  {
    try
    {
      XMLConfigNode *childNode = rootNode->GetChild("gui");

      if (childNode)
      {
        int width = childNode->GetTupleInt("size",0,640);
        int height = childNode->GetTupleInt("size",1,480);
        int x = childNode->GetTupleInt("pos",0,0);
        int y = childNode->GetTupleInt("pos",1,0);

        //gzmsg(1) << "Creating GUI: Pos[" << x << " " << y << "] Size[" << width << " " << height << "]\n";

        // Create the GUI
        this->gui = new Gui(x, y, width, height, "Gazebo");
        Fl::check();
        Fl::wait(0.3);
        this->gui->Load(childNode);
      }
    }
    catch (GazeboError e)
    {
      gzthrow( "Error loading the GUI\n" << e);
    }
  }
  else
    this->gui = NULL;

  //Initialize RenderEngine
  try
  {
    OgreAdaptor::Instance()->Init(rootNode);
    this->renderEngine = OgreAdaptor::Instance();
  }
  catch (gazebo::GazeboError e)
  {
    gzthrow("Failed to Initialize the Rendering engine subsystem\n" << e );
  }

  // Initialize the GUI
  if (this->gui)
  {
    this->gui->Init();
  }

  //Create the world
  try
  {
    gazebo::World::Instance()->Load(rootNode, serverId);
  }
  catch (GazeboError e)
  {
    gzthrow("Failed to load the World\n"  << e);
  }

  this->loaded=true;
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize the simulation
int Simulator::Init()
{
  this->startTime = this->GetWallTime();

  //Initialize the world
  if (gazebo::World::Instance()->Init() != 0)
    return -1;

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Save the world configuration file
void Simulator::Save(const std::string& filename)
{
  std::fstream output;

  output.open(filename.c_str(), std::ios::out);

  // Write out the xml header
  output << "<?xml version=\"1.0\"?>\n";
  output << "<gazebo:world\n\
    xmlns:xi=\"http://www.w3.org/2001/XInclude\"\n\
    xmlns:gazebo=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#gz\"\n\
    xmlns:model=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#model\"\n\
    xmlns:sensor=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor\"\n\
    xmlns:window=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#window\"\n\
    xmlns:param=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#param\"\n\
    xmlns:body=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#body\"\n\
    xmlns:geom=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#geom\"\n\
    xmlns:joint=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#joint\"\n\
    xmlns:interface=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#interface\"\n\
    xmlns:ui=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#ui\"\n\
    xmlns:rendering=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#rendering\"\n\
    xmlns:controller=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#controller\"\n\
    xmlns:physics=\"http://playerstage.sourceforge.net/gazebo/xmlschema/#physics\">\n\n";

  std::string prefix = "  ";

  if (output.is_open())
  {
    GazeboMessage::Instance()->Save(prefix, output);
    output << "\n";

    World::Instance()->GetPhysicsEngine()->Save(prefix, output);
    output << "\n";

    this->GetRenderEngine()->Save(prefix, output);
    output << "\n";

    this->gui->Save(prefix, output);
    output << "\n";

    World::Instance()->Save(prefix, output);
    output << "\n";

    output << "</gazebo:world>\n";
    output.close();
  }
  else
  {
    gzerr(0) << "Unable to save XML file to file[" << filename << "]\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Finalize the simulation
void Simulator::Fini( )
{
  gazebo::World::Instance()->Fini();
}

////////////////////////////////////////////////////////////////////////////////
/// Main simulation loop, when this loop ends the simulation finish
void Simulator::MainLoop()
{
  double step = World::Instance()->GetPhysicsEngine()->GetStepTime();
  double physicsUpdateRate = World::Instance()->GetPhysicsEngine()->GetUpdateRate();
  double renderUpdateRate = OgreAdaptor::Instance()->GetUpdateRate();
  double physicsUpdatePeriod = 1.0 / physicsUpdateRate;
  double renderUpdatePeriod = 1.0 / renderUpdateRate;

  double currTime;
  double elapsedTime;

  this->prevPhysicsTime = this->GetRealTime();
  this->prevRenderTime = this->GetRealTime();
 
  while (!this->userQuit)
  {
    currTime = this->GetRealTime();

    if (physicsUpdateRate == 0 || 
        currTime - this->prevPhysicsTime >= physicsUpdatePeriod) 
    {

      // Update the physics engine
      if (!this->GetUserPause() ||
          (this->GetUserPause() && this->GetUserStepInc()))
      {
        this->simTime += step;
        this->iterations++;
        this->pause=false;
        this->SetUserStepInc(!this->GetUserStepInc());
      }
      else
      {
        this->pauseTime += step;
        this->pause=true;
      }

      this->prevPhysicsTime = this->GetRealTime();

      World::Instance()->Update();
    }

    // Update the rendering
    if (renderUpdateRate == 0 || 
        currTime - this->prevRenderTime >= renderUpdatePeriod)
    {
      //this->GetRenderEngine()->Render(); 
      //this->prevRenderTime = this->GetRealTime();
    }

    // Update the gui
    if (this->gui)
    {
      this->gui->Update();
    }

    elapsedTime = (this->GetRealTime() - currTime);

    // Wait if we're going too fast
    /*if ( elapsedTime < 1.0/MAX_FRAME_RATE )
    {
      usleep( (int)((1.0/MAX_FRAME_RATE - elapsedTime) * 1e6)  );
    }*/

    if (this->timeout > 0 && this->GetRealTime() > this->timeout)
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Gets our current GUI interface
Gui *Simulator::GetUI() const
{
  return this->gui;
}

////////////////////////////////////////////////////////////////////////////////
/// Gets local configuration for this computer
GazeboConfig *Simulator::GetGazeboConfig() const
{
  return this->gazeboConfig;
}

OgreAdaptor *Simulator::GetRenderEngine() const
{
  return this->renderEngine;
}

////////////////////////////////////////////////////////////////////////////////
// Return when this simulator is paused
bool Simulator::IsPaused() const
{
  return this->pause;
}


////////////////////////////////////////////////////////////////////////////////
/// Get the number of iterations of this simulation session
unsigned long Simulator::GetIterations() const
{
  return this->iterations;
}

////////////////////////////////////////////////////////////////////////////////
// Get the simulation time
double Simulator::GetSimTime() const
{
  return this->simTime;
}

////////////////////////////////////////////////////////////////////////////////
// Get the pause time
double Simulator::GetPauseTime() const
{
  return this->pauseTime;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the start time
double Simulator::GetStartTime() const
{
  return this->startTime;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the real time (elapsed time)
double Simulator::GetRealTime() const
{
  return this->GetWallTime() - this->startTime;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the wall clock time
double Simulator::GetWallTime() const
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + tv.tv_usec * 1e-6;
}


void Simulator::SetUserQuit()
{
  //  this->Save("test.xml");
  this->userQuit = true;
}

////////////////////////////////////////////////////////////////////////////////
bool Simulator::GetUserPause() const
{
  return this->userPause;
}

////////////////////////////////////////////////////////////////////////////////
void Simulator::SetUserPause(bool pause)
{
  this->userPause = pause;
}

////////////////////////////////////////////////////////////////////////////////
bool Simulator::GetUserStepInc() const
{
  return this->userStepInc;
}

////////////////////////////////////////////////////////////////////////////////
void Simulator::SetUserStepInc(bool step)
{
  this->userStepInc = step;
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
/// Set the length of time the simulation should run.
void Simulator::SetTimeout(double time)
{
  this->timeout = time;
}

////////////////////////////////////////////////////////////////////////////////
// Set the physics enabled/disabled
void Simulator::SetPhysicsEnabled( bool enabled )
{
  this->physicsEnabled = enabled;
}

////////////////////////////////////////////////////////////////////////////////
// Get the physics enabled/disabled
bool Simulator::GetPhysicsEnabled() const
{
  return this->physicsEnabled;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the selected entity
void Simulator::SetSelectedEntity( Entity *ent )
{
  if (this->selectedEntity)
  {
    this->selectedEntity->GetVisualNode()->ShowSelectionBox(false);
    this->selectedEntity->SetSelected(false);
  }

  if (this->selectedEntity != ent)
  {
    this->selectedEntity = ent;
    this->selectedEntity->GetVisualNode()->ShowSelectionBox(true);
    this->selectedEntity->SetSelected(true);
  }
  else
    this->selectedEntity = NULL;

}

////////////////////////////////////////////////////////////////////////////////
/// Get the selected entity
Entity *Simulator::GetSelectedEntity() const
{
  return this->selectedEntity;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the model that currently selected
Model *Simulator::GetSelectedModel() const
{
  Model *model = NULL;
  Body *body = NULL;
  Geom *geom = NULL;

  if (!this->selectedEntity)
    return NULL;

  if ( (model = dynamic_cast<Model*>(this->selectedEntity)) != NULL )
    return model;
  else
  {
    if ( (body = dynamic_cast<Body*>(this->selectedEntity)) != NULL )
      model = body->GetModel();
    else if ( (geom = dynamic_cast<Geom*>(this->selectedEntity)) != NULL )
      model = geom->GetModel();
    else
      gzerr(0) << "Unknown type\n";
  }

  return model;
}
