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
 * SVN: $Id:$
 */

#include <Ogre.h>
#include <sstream>

#include "Global.hh"
#include "GLWindow.hh"
#include "OgreCamera.hh"
#include "OgreAdaptor.hh"
#include "OgreCreator.hh"
#include "UserCamera.hh"   

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
UserCamera::UserCamera(GLWindow *parentWindow)
  : OgreCamera("User")
{
  this->window = OgreCreator::CreateWindow(parentWindow, 
                     parentWindow->w(), parentWindow->h());
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
UserCamera::~UserCamera()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load child
void UserCamera::Load(XMLConfigNode *node)
{
  OgreCamera::LoadCam(node);

  this->SetClipDist(0.1, 1000);
  this->SetFOV( DTOR(60) );
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize
void UserCamera::Init()
{
  this->SetCameraSceneNode( OgreAdaptor::Instance()->sceneMgr->getRootSceneNode()->createChildSceneNode( this->GetCameraName() + "_SceneNode") );

  OgreCamera::InitCam();

  this->viewport = this->window->addViewport(this->GetOgreCamera());
  this->viewport->setBackgroundColour(Ogre::ColourValue::Black);

  this->SetAspectRatio( Ogre::Real(this->viewport->getActualWidth()) / Ogre::Real(this->viewport->getActualHeight()) );

  //this->viewport->setVisibilityMask(this->visibilityMask);
}

////////////////////////////////////////////////////////////////////////////////
/// Update
void UserCamera::Update()
{
  OgreCamera::UpdateCam();

  OgreAdaptor::Instance()->UpdateWindow(this->window, this);
}


////////////////////////////////////////////////////////////////////////////////
// Finalize
void UserCamera::Fini()
{
  OgreCamera::FiniCam();
}

////////////////////////////////////////////////////////////////////////////////
/// Resize the camera
void UserCamera::Resize(unsigned int w, unsigned int h)
{
  this->window->resize(w, h);
  this->viewport->setDimensions(0,0,1,1);
  this->SetAspectRatio( Ogre::Real(this->viewport->getActualWidth()) / Ogre::Real(this->viewport->getActualHeight()) );
}

////////////////////////////////////////////////////////////////////////////////
// Set the dimensions of the viewport
void UserCamera::SetViewportDimensions(float x, float y, float w, float h)
{
  this->viewport->setDimensions(0, 0, 0.5, 0.5);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the average FPS
float UserCamera::GetAvgFPS()
{
  float lastFPS, avgFPS, bestFPS, worstFPS;
  this->window->getStatistics(lastFPS, avgFPS, bestFPS, worstFPS);

  return avgFPS;
}
