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
/* Desc: Camera for viewing the world
 * Author: Nate Koenig
 * Date: 19 Jun 2008
 * SVN: $Id$
 */

#include <Ogre.h>
#include <sstream>

#include "Scene.hh"
#include "Simulator.hh"
#include "RTShaderSystem.hh"
#include "Global.hh"
#include "RenderControl.hh"
#include "OgreCamera.hh"
#include "OgreAdaptor.hh"
#include "OgreCreator.hh"
#include "OgreVisual.hh"
#include "OgreDynamicLines.hh"
#include "World.hh"
#include "UserCamera.hh"   

using namespace gazebo;

int UserCamera::count = 0;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
UserCamera::UserCamera(RenderControl *parentWindow, unsigned int sceneIndex )
  : OgreCamera("UserCamera", sceneIndex)
{
  std::stringstream stream;

  int w, h;
  parentWindow->GetSize(&w, &h);

  this->window = OgreCreator::Instance()->CreateWindow(parentWindow, w, h);

  stream << "UserCamera_" << this->count++;
  this->name = stream.str(); 

  this->viewport = NULL;

  World::Instance()->ConnectShowCamerasSignal( boost::bind(&UserCamera::ShowVisual, this, _1) );
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
UserCamera::~UserCamera()
{
  if (this->visual)
  {
    delete this->visual;
    this->visual = NULL;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Load child
void UserCamera::Load(XMLConfigNode *node)
{
  OgreCamera::Load(node);

  this->SetFOV( DTOR(60) );
  this->SetClipDist(0.01, 50);
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize
void UserCamera::Init()
{
  this->SetSceneNode( this->scene->GetManager()->getRootSceneNode()->createChildSceneNode( this->GetName() + "_SceneNode") );

  OgreCamera::Init();

  this->visual = new OgreVisual(this->pitchNode);

  // The lines draw a visualization of the camera
  OgreDynamicLines *line = OgreCreator::Instance()->CreateDynamicLine(
      OgreDynamicRenderable::OT_LINE_LIST);

  float f = 0.2;

  // Create the front face
  line->AddPoint(Vector3(0, -f, -f)); 
  line->AddPoint(Vector3(0, -f, +f)); 

  line->AddPoint(Vector3(0, -f, +f)); 
  line->AddPoint(Vector3(0, +f, +f)); 

  line->AddPoint(Vector3(0, +f, +f)); 
  line->AddPoint(Vector3(0, +f, -f)); 

  line->AddPoint(Vector3(0, +f, -f)); 
  line->AddPoint(Vector3(0, -f, -f)); 


  // Create the connecting lines
  line->AddPoint(Vector3(-0.4, 0, 0)); 
  line->AddPoint(Vector3(+0.0, -f, -f)); 

  line->AddPoint(Vector3(-0.4, 0, 0)); 
  line->AddPoint(Vector3(+0.0, -f, +f)); 

  line->AddPoint(Vector3(-0.4, 0, 0)); 
  line->AddPoint(Vector3(+0.0, +f, +f)); 

  line->AddPoint(Vector3(-0.4, 0, 0)); 
  line->AddPoint(Vector3(+0.0, +f, -f)); 

  line->AddPoint(Vector3(-0.4, 0, 0)); 
  line->AddPoint(Vector3(+0.0, -f, -f)); 

  // Draw up arrow
  line->AddPoint(Vector3(0, 0, +f)); 
  line->AddPoint(Vector3(0, 0, +f+0.1)); 

  line->AddPoint(Vector3(0.0, -0.02, +f+0.1)); 
  line->AddPoint(Vector3(0.0, +0.02, +f+0.1)); 

  line->AddPoint(Vector3(0.0, +0.02, +f+0.1)); 
  line->AddPoint(Vector3(0.0, +0.00, +f+0.15)); 

  line->AddPoint(Vector3(0.0, +0.00, +f+0.15)); 
  line->AddPoint(Vector3(0.0, -0.02, +f+0.1)); 

  line->setMaterial("Gazebo/WhiteGlow");
  line->setVisibilityFlags(GZ_LASER_CAMERA);

  this->visual->AttachObject(line);
  this->visual->SetVisible(false);

  this->window->removeAllViewports();
  this->viewport = this->window->addViewport(this->GetOgreCamera());

  this->SetAspectRatio( Ogre::Real(this->viewport->getActualWidth()) / Ogre::Real(this->viewport->getActualHeight()) );

  this->lastUpdate = Simulator::Instance()->GetRealTime();

  double ratio = (double)this->viewport->getActualWidth() / (double)this->viewport->getActualHeight();
  double vfov = fabs(2.0 * atan(tan(this->GetHFOV().GetAsRadian() / 2.0) / ratio));
  this->GetOgreCamera()->setAspectRatio(ratio);
  this->GetOgreCamera()->setFOVy(Ogre::Radian(vfov));

  this->viewport->setClearEveryFrame(true);
  this->viewport->setBackgroundColour( this->scene->GetBackgroundColor().GetOgreColor() );
  this->viewport->setVisibilityMask(this->visibilityMask);

  RTShaderSystem::AttachViewport(this);
}

////////////////////////////////////////////////////////////////////////////////
/// Update
void UserCamera::Render()
{
  if (Simulator::Instance()->GetRealTime() - this->lastUpdate < 
      this->renderPeriod)
    return;

  {
    boost::recursive_mutex::scoped_lock md_lock(*Simulator::Instance()->GetMDMutex());
    OgreCamera::Update();
    this->lastUpdate = Simulator::Instance()->GetRealTime();
    this->newData = true;
    this->window->update(false);
  }
}

void UserCamera::PostRender()
{
  this->window->swapBuffers();

  if (this->newData && this->saveFramesP->GetValue())
  {
    char tmp[1024];
    if (!this->savePathnameP->GetValue().empty())
    {
      sprintf(tmp, "%s/%s-%04d.jpg", this->savePathnameP->GetValue().c_str(),
          this->name.c_str(), this->saveCount);
    }
    else
    {
      sprintf(tmp, "%s-%04d.jpg", this->name.c_str(), this->saveCount);
    }

    this->window->writeContentsToFile(tmp);

    this->saveCount++;
  }
  this->newData = false;
}


////////////////////////////////////////////////////////////////////////////////
// Finalize
void UserCamera::Fini()
{
  OgreCamera::Fini();
}

////////////////////////////////////////////////////////////////////////////////
/// Resize the camera
void UserCamera::Resize(unsigned int w, unsigned int h)
{
  this->window->resize(w, h);
  this->window->windowMovedOrResized();

  if (this->viewport)
    this->viewport->setDimensions(0,0,1,1);
}

////////////////////////////////////////////////////////////////////////////////
// Set the dimensions of the viewport
void UserCamera::SetViewportDimensions(float x, float y, float w, float h)
{
  this->viewport->setDimensions(x, y, w, h);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the average FPS
float UserCamera::GetAvgFPS()
{
  float lastFPS, avgFPS, bestFPS, worstFPS;
  this->window->getStatistics(lastFPS, avgFPS, bestFPS, worstFPS);

  return avgFPS;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the triangle count
unsigned int UserCamera::GetTriangleCount()
{
  return this->window->getTriangleCount();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the ogre window
Ogre::RenderWindow *UserCamera::GetWindow()
{
  return this->window;
}

////////////////////////////////////////////////////////////////////////////////
// Set whether to show the visual
void UserCamera::ShowVisual(bool s)
{
  this->visual->SetVisible(s);
}
