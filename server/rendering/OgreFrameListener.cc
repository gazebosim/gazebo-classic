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
/* Desc: OGRE frame listener
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * CVS: $Id$
 */

#include <OgreWindowEventUtilities.h>

#include "Global.hh"
#include "Pose3d.hh"
#include "OgreHUD.hh"
#include "CameraManager.hh"
#include "OgreAdaptor.hh"
#include "OgreFrameListener.hh"

using namespace gazebo;

OgreFrameListener::OgreFrameListener()
{
}

OgreFrameListener::~OgreFrameListener()
{
  CameraManager::Instance()->Clear();
}

bool OgreFrameListener::frameStarted( const Ogre::FrameEvent &evt)
{
  CameraManager::Instance()->FrameStarted(evt.timeSinceLastFrame);
  return true;
}

bool OgreFrameListener::frameEnded( const Ogre::FrameEvent &/*evt*/)
{
  return true;
}

bool OgreFrameListener::frameRenderingQueued( const Ogre::FrameEvent &evt)
{
  return true;
}
