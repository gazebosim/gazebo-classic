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
//#include <boost/signals.hpp>
//#include <boost/bind.hpp>

#include "World.hh"
#include "GuiFactory.hh"
#include "Gui.hh"
#include "DummyGui.hh"
#include "XMLConfig.hh"
#include "GazeboConfig.hh"
#include "gazebo.h"
#include "PhysicsEngine.hh"
#include "OgreAdaptor.hh"
#include "OgreCreator.hh"
#include "GazeboMessage.hh"
#include "Global.hh"

#include "Simulator.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Simulator::Simulator()
{
  this->gui=NULL;

  this->loaded = false;
  this->pause = false;

  this->iterations = 0;
  this->pauseTime = 0.0;
  this->startTime = 0.0;
  this->simTime = 0.0;

  this->userQuit = false;
  this->userPause = false;
  this->userStep = false;
  this->userStepInc = false;

  this->xmlFile=NULL;
  this->gazeboConfig=NULL;
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

  try
  {
    //Create and initialize the Gui
    this->LoadGui(rootNode);
  }
  catch (GazeboError e)
  {
    gzthrow( "Error loading the GUI\n" << e);
  }

  //Initialize RenderingEngine
  try
  {
    gazebo::OgreAdaptor::Instance()->Init(rootNode);
  }
  catch (gazebo::GazeboError e)
  {
    gzthrow("Failed to Initialize the OGRE Rendering system\n" << e );
 }

  //Preload basic shapes that can be used anywhere
  OgreCreator::CreateBasicShapes();

  try
  {
    //Create the world
    gazebo::World::Instance()->Load(rootNode, serverId);
  }
  catch (GazeboError e)
  {
    gzthrow("Failed to load the GUI\n"  << e);
  }

  this->loaded=true;
}

////////////////////////////////////////////////////////////////////////////////
/// Save the world configuration file
void Simulator::Save(const std::string& filename)
{
  // Saving in the preferred order
  XMLConfigNode* root=xmlFile->GetRootNode();
  gazebo::GazeboMessage::Instance()->Save(root);
  World::Instance()->GetPhysicsEngine()->Save(root);
  this->SaveGui(root);
  gazebo::OgreAdaptor::Instance()->Save(root);
  World::Instance()->Save(root);

  if (xmlFile->Save(filename)<0)
  {
   gzthrow("The XML file could not be written back to " << filename );
   }
}


////////////////////////////////////////////////////////////////////////////////
/// Initialize the simulation
int Simulator::Init()
{
  this->startTime = this->GetWallTime();

  //Initialize the world
  if (gazebo::World::Instance()->Init() != 0)
    return -1;

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
  double step= World::Instance()->GetPhysicsEngine()->GetStepTime();
  double currTime;
  double elapsedTime;

  while (!this->userQuit)
  {
    currTime = this->GetRealTime();

    if ((currTime - this->prevPhysicsTime) >= step) 
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

      World::Instance()->Update(); //physics

      this->prevPhysicsTime = this->GetRealTime();
    }

    // Update the rendering
    if (currTime - this->prevRenderTime > 0.02)
    {
      gazebo::OgreAdaptor::Instance()->Render(); 
      this->prevRenderTime = this->GetRealTime();
    }

    // Update the gui
    this->gui->Update();

    elapsedTime = (this->GetRealTime()-currTime)*2.0;

    // Wait if we're going too fast
    if ( elapsedTime < 0.02 )
    {
      usleep( (0.02 - elapsedTime) * 1e6  );
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

/*
   These methods are needeD?
////////////////////////////////////////////////////////////////////////////////
void Global::SetIterations(unsigned long count)
{
iterations = count;
}

////////////////////////////////////////////////////////////////////////////////
void Global::IncIterations()
{
iterations++;
}
*/

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




//TODO: Move to Gui or create GuiEngine and move it there
void Simulator::LoadGui(XMLConfigNode *rootNode)
{
  gazebo::XMLConfigNode *childNode;

  childNode = rootNode->GetChild("gui");

  if (childNode)
  {
    int width = childNode->GetTupleInt("size",0,640);
    int height = childNode->GetTupleInt("size",1,480);
    int x = childNode->GetTupleInt("pos",0,0);
    int y = childNode->GetTupleInt("pos",1,0);
    std::string type = childNode->GetString("type","fltk",1);
 
    gzmsg(1) << "Creating GUI:\n\tType[" << type << "] Pos[" << x << " " << y << "] Size[" << width << " " << height << "]\n";
    if (type != "fltk")
    {
      gzthrow("The only GUI available is 'fltk', for no-GUI simulation, delete the 'gui' tag and its children");
    }

    // Create the GUI
    this->gui = GuiFactory::NewGui(type, x, y, width, height, type+"::Gazebo");

    // Initialize the GUI
    this->gui->Init();

  }
  else
  {
    // Create a dummy GUI
    gzmsg(1) <<"Creating a dummy GUI";
    this->gui = GuiFactory::NewGui(std::string("dummy"), 0, 0, 0, 0, std::string());
  }
}


void Simulator::SaveGui(XMLConfigNode *node)
{
  Vector2<int> size;
  XMLConfigNode* childNode = node->GetChild("gui");

  if (childNode)
  {
    size.x = this->gui->GetWidth();
    size.y = this->gui->GetHeight();
    childNode->SetValue("size", size);
    //TODO: node->SetValue("pos", Vector2<int>(x,y));
  }

}
