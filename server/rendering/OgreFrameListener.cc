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
#include "CameraSensor.hh"
#include "CameraManager.hh"
#include "OgreAdaptor.hh"
#include "OgreFrameListener.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
OgreFrameListener::OgreFrameListener()
{
/*
  this->selectedObject = NULL;

  this->leftPressed = false;
  this->rightPressed = false;
  this->middlePressed = false;

  // Create ray scene query to handle mouse picking
  this->raySceneQuery = OgreAdaptor::Instance()->sceneMgr->createRayQuery(Ogre::Ray());
  */
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
OgreFrameListener::~OgreFrameListener()
{
}

////////////////////////////////////////////////////////////////////////////////
// Frame is started
bool OgreFrameListener::frameStarted( const Ogre::FrameEvent &evt)
{
  CameraSensor *camera;

  if ((camera = CameraManager::Instance()->GetActiveCamera()))
  {
    OgreHUD::Instance()->SetCamera(camera);
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Frame ended
bool OgreFrameListener::frameEnded( const Ogre::FrameEvent &/*evt*/) 
{
  return true;
}


/*
bool OgreFrameListener::mouseMoved(const OIS::MouseEvent &e)
{
  CameraSensor *camera;

  if ((camera = CameraManager::Instance()->GetActiveCamera()))
  {
    if (this->leftPressed)
    {
      camera->RotateYaw(-e.state.X.rel * this->rotateAmount);
      camera->RotatePitch(-e.state.Y.rel * this->rotateAmount);
    }
  }
  
  return true;
}

bool OgreFrameListener::mousePressed(const OIS::MouseEvent &e, 
                                     OIS::MouseButtonID id)
{

  switch (id)
  {
    case OIS::MB_Left:
      this->LeftMousePressed(e);
      break;
    case OIS::MB_Right:
      this->rightPressed = true;
      break;
    case OIS::MB_Middle:
      this->middlePressed = true;
      break;
    default:
      break;
  }

  return true;
}

void OgreFrameListener::LeftMousePressed(const OIS::MouseEvent &e)
{

  CameraSensor *camera;

  if ((camera = CameraManager::Instance()->GetActiveCamera()))
  {
    float xscale = (float)camera->GetImageWidth() / e.state.width;
    float yscale =  (float)camera->GetImageHeight() / e.state.height;

    //int xoffset = (camera->GetImageWidth() - e.state.width)/2 + 5;
    //int yoffset = (camera->GetImageHeight() - e.state.height)/2 + 5;

    int xoffset = 5;
    int yoffset = 5;

    xscale = yscale = 1;
    float x = (xscale * (e.state.X.abs+xoffset)) / (float)camera->GetImageWidth();
    float y = (yscale * (e.state.Y.abs+yoffset)) / (float)camera->GetImageHeight();

 //   printf("OFfset[%d %d]\n",xoffset, yoffset);

//    printf("Base[%d %d] XY[%4.2f %4.2f] Mouse[%d %d] Camera[%d %d] Scale[%4.2f %4.2f]\n",e.state.X.abs+xoffset, e.state.Y.abs+yoffset, x,y, e.state.width, e.state.height, camera->GetImageWidth(), camera->GetImageHeight(), xscale, yscale);
    Ogre::Ray mouseRay = camera->GetOgreCamera()->getCameraToViewportRay(x, y);
    this->raySceneQuery->setRay(mouseRay);
    this->raySceneQuery->setSortByDistance(true);

    // Execute query
    Ogre::RaySceneQueryResult &result = this->raySceneQuery->execute();
    Ogre::RaySceneQueryResult::iterator iter = result.begin();

    // Get results, create a node/entity on the position
    for ( iter = result.begin(); iter != result.end(); iter++ )
    {
      if (iter->movable && iter->movable->getName().substr(0,5) == "Geom_")
      {
        if (this->selectedObject)
          this->selectedObject->showBoundingBox(false);

        this->selectedObject = iter->movable->getParentSceneNode();
        this->selectedObject->showBoundingBox(true);

        break;
      }
    }
    
  }
 
  this->leftPressed = true;
}

bool OgreFrameListener::mouseReleased(const OIS::MouseEvent & e, OIS::MouseButtonID id)
{
  switch (id)
  {
    case OIS::MB_Left:
      this->leftPressed = false;
      break;
    case OIS::MB_Right:
      this->rightPressed = false;
      break;
    case OIS::MB_Middle:
      this->middlePressed = false;
      break;
    default:
      break;
  }
  
  return true;
}

void OgreFrameListener::SetWindowExtents(unsigned int w, unsigned int h)
{

  const OIS::MouseState &mouseState = this->mMouse->getMouseState();
  mouseState.width = w;
  mouseState.height = h;
}
*/
