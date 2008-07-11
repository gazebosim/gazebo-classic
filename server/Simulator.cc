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

#define MAX_FRAME_RATE 35

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
  userStep(false),
  userStepInc(false),
  userQuit(false),
  guiEnabled(true)
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
    gzthrow("Failed to load the GUI\n"  << e);
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
  // Saving in the preferred order
  XMLConfigNode* root=xmlFile->GetRootNode();
  GazeboMessage::Instance()->Save(root);
  World::Instance()->GetPhysicsEngine()->Save(root);
  this->GetRenderEngine()->Save(root);
  this->SaveGui(root);
  World::Instance()->Save(root);

  if (xmlFile->Save(filename)<0)
  {
   gzthrow("The XML file could not be written back to " << filename );
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
      this->simTime += step;

      // Update the physics engine
      if (!this->GetUserPause() && !this->GetUserStep() ||
          (this->GetUserStep() && this->GetUserStepInc()))
      {
        this->iterations++;
        this->pause=false;
        this->SetUserStepInc(!this->GetUserStepInc());
      }
      else
      {
        this->pauseTime += step;
        this->pause=true;
      }

      World::Instance()->Update();

      this->prevPhysicsTime = this->GetRealTime();
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
    if ( elapsedTime < 1.0/MAX_FRAME_RATE )
    {
      usleep( (int)((1.0/MAX_FRAME_RATE - elapsedTime) * 1e6)  );
    }
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
  return userPause;
}

////////////////////////////////////////////////////////////////////////////////
void Simulator::SetUserPause(bool pause)
{
  userPause = pause;
}

////////////////////////////////////////////////////////////////////////////////
bool Simulator::GetUserStep() const 
{
  return userStep;
}

////////////////////////////////////////////////////////////////////////////////
void Simulator::SetUserStep( bool step )
{
  userStep = step;
}

////////////////////////////////////////////////////////////////////////////////
bool Simulator::GetUserStepInc() const
{
  return userStepInc;
}

////////////////////////////////////////////////////////////////////////////////
void Simulator::SetUserStepInc(bool step)
{
  userStepInc = step;
}


void Simulator::SaveGui(XMLConfigNode *node)
{
  Vector2<int> size;
  XMLConfigNode* childNode = node->GetChild("gui");

  if (childNode && this->gui)
  {
    size.x = this->gui->GetWidth();
    size.y = this->gui->GetHeight();
    childNode->SetValue("size", size);
    //TODO: node->SetValue("pos", Vector2<int>(x,y));
  }

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


