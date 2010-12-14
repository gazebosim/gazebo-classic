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

#include "FPSViewController.hh"
#include "OrbitViewController.hh"

#include "RenderTypes.hh"
#include "GazeboError.hh"
#include "Model.hh"
#include "Body.hh"
#include "Events.hh"
#include "Scene.hh"
#include "RTShaderSystem.hh"
#include "Global.hh"
#include "RenderControl.hh"
#include "Camera.hh"
#include "OgreAdaptor.hh"
#include "Visual.hh"
#include "OgreDynamicLines.hh"
#include "World.hh"
#include "UserCamera.hh"   

using namespace gazebo;

int UserCamera::count = 0;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
UserCamera::UserCamera(const std::string &name, Scene *scene)
  : Camera(name, scene)
{
  std::stringstream stream;

  std::cout << "New User Camera\n";

  //int w, h;
  //parentWindow->GetSize(&w, &h);
  //this->window = OgreCreator::Instance()->CreateWindow(parentWindow, w, h);

  stream << "UserCamera_" << this->count++;
  this->name = stream.str(); 

  Events::ConnectShowCamerasSignal( boost::bind(&UserCamera::ToggleShowVisual, this) );
  Events::ConnectRenderSignal( boost::bind(&UserCamera::Render, this) );
  Events::ConnectPostRenderSignal( boost::bind(&UserCamera::PostRender, this) );

  this->animState = NULL;

  this->viewController = new FPSViewController(this);
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

  delete this->viewController;
}

////////////////////////////////////////////////////////////////////////////////
// Load child
void UserCamera::Load(XMLConfigNode *node)
{
  Camera::Load(node);

  this->SetFOV( DTOR(60) );
  this->SetClipDist(0.01, 50);
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize
void UserCamera::Init()
{
  this->SetSceneNode( this->scene->GetManager()->getRootSceneNode()->createChildSceneNode( this->GetName() + "_SceneNode") );

  Camera::Init();

  this->visual = new Visual(this->GetName() + "_OUTLINE", this->pitchNode);

  // The lines draw a visualization of the camera
  OgreDynamicLines *line = this->visual->AddDynamicLine( RENDERING_LINE_LIST );

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

  this->visual->SetVisible(false);

  //this->window->removeAllViewports();
  //this->viewport = this->window->addViewport(this->GetCamera());

  //this->SetAspectRatio( Ogre::Real(this->viewport->getActualWidth()) / Ogre::Real(this->viewport->getActualHeight()) );

 // double ratio = (double)this->viewport->getActualWidth() / (double)this->viewport->getActualHeight();
  //double vfov = fabs(2.0 * atan(tan(this->GetHFOV().GetAsRadian() / 2.0) / ratio));
  //this->GetCamera()->setAspectRatio(ratio);
  //this->GetCamera()->setFOVy(Ogre::Radian(vfov));

  //this->viewport->setClearEveryFrame(true);
  //this->viewport->setBackgroundColour( this->scene->GetBackgroundColor().GetOgreColor() );
  //this->viewport->setVisibilityMask(this->visibilityMask);

  //RTShaderSystem::AttachViewport(this);
}

////////////////////////////////////////////////////////////////////////////////
/// Update
void UserCamera::Render()
{
  this->viewController->Update();

  Camera::Update();

  if (this->animState)
  {
    this->animState->addTime(0.01);
    if (this->animState->hasEnded())
    {
      this->animState = NULL;

      this->scene->GetManager()->destroyAnimation("cameratrack");
      this->scene->GetManager()->destroyAnimationState("cameratrack");
    }
  }

  // NATY
  //this->window->update(false);
}

////////////////////////////////////////////////////////////////////////////////
// Post Render
void UserCamera::PostRender()
{
  // NATY
  //this->window->swapBuffers();

  if (**this->saveFramesP)
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

    // NATY
    //this->window->writeContentsToFile(tmp);

    this->saveCount++;
  }
}


////////////////////////////////////////////////////////////////////////////////
// Finalize
void UserCamera::Fini()
{
  Camera::Fini();
}

////////////////////////////////////////////////////////////////////////////////
/// Handle a mouse event
void UserCamera::HandleMouseEvent(const MouseEvent &evt)
{
  this->viewController->HandleMouseEvent(evt);
}

////////////////////////////////////////////////////////////////////////////////
// Set view controller
void UserCamera::SetViewController( const std::string type )
{
  delete this->viewController;
  this->viewController = NULL;

  if (type == OrbitViewController::GetTypeString())
    this->viewController = new OrbitViewController(this);
  else if (type == FPSViewController::GetTypeString())
    this->viewController = new FPSViewController(this);
  else
    gzthrow("Invalid view controller type: " + type );
}

////////////////////////////////////////////////////////////////////////////////
/// Resize the camera
/* NATY: probably can remove if window resize workd
void UserCamera::Resize(unsigned int w, unsigned int h)
{
  if (this->viewport)
    this->viewport->setDimensions(0,0,1,1);
}
*/

////////////////////////////////////////////////////////////////////////////////
// Set the dimensions of the viewport
void UserCamera::SetViewportDimensions(float x, float y, float w, float h)
{
  //this->viewport->setDimensions(x, y, w, h);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the average FPS
float UserCamera::GetAvgFPS()
{
  float lastFPS, avgFPS, bestFPS, worstFPS;
  // NATY: Put back in
  //this->window->getStatistics(lastFPS, avgFPS, bestFPS, worstFPS);

  return avgFPS;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the triangle count
unsigned int UserCamera::GetTriangleCount()
{
  //NATY: put back in
  //return this->window->getTriangleCount();
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the ogre window
/*Ogre::RenderWindow *UserCamera::GetWindow()
{
  return this->window;
}*/

////////////////////////////////////////////////////////////////////////////////
// Toggle whether to show the visual
void UserCamera::ToggleShowVisual()
{
  this->visual->ToggleVisible();
}

////////////////////////////////////////////////////////////////////////////////
// Set whether to show the visual
void UserCamera::ShowVisual(bool s)
{
  this->visual->SetVisible(s);
}

//////////////////////////////////////////////////////////////////////////////
// Move the camera to focus on an entity
void UserCamera::MoveToEntity(Entity *entity)
{
  if (!entity)
    return;

  if (this->scene->GetManager()->hasAnimation("cameratrack"))
  {
    this->scene->GetManager()->destroyAnimation("cameratrack");
    this->scene->GetManager()->destroyAnimationState("cameratrack");
  }

  Ogre::Animation *anim = this->scene->GetManager()->createAnimation("cameratrack",.5);
  anim->setInterpolationMode(Ogre::Animation::IM_SPLINE);

  Ogre::NodeAnimationTrack *strack = anim->createNodeTrack(0,this->sceneNode);
  Ogre::NodeAnimationTrack *ptrack = anim->createNodeTrack(1,this->pitchNode);

  Vector3 start = this->GetWorldPose().pos;
  start.Correct();
  Vector3 end = entity->GetWorldPose().pos;
  end.Correct();
  Vector3 dir = end - start;
  dir.Correct();

  double yawAngle = atan2(dir.y,dir.x);
  double pitchAngle = atan2(-dir.z, sqrt(dir.x*dir.x + dir.y*dir.y));
  Ogre::Quaternion yawFinal(Ogre::Radian(yawAngle), Ogre::Vector3(0,0,1));
  Ogre::Quaternion pitchFinal(Ogre::Radian(pitchAngle), Ogre::Vector3(0,1,0));

  Ogre::TransformKeyFrame *key;

  key = strack->createNodeKeyFrame(0);
  key->setTranslate(Ogre::Vector3(start.x, start.y, start.z));
  key->setRotation(this->sceneNode->getOrientation());

  key = ptrack->createNodeKeyFrame(0);
  key->setRotation(this->pitchNode->getOrientation());

  Vector3 min, max, size;
  Box box = entity->GetBoundingBox();
  size = box.max-box.min;

  double scale = std::max(std::max(size.x, size.y), size.z);
  scale += 0.5;

  dir.Normalize();
  double dist = start.Distance(end);

  Vector3 mid = start + dir*(dist*.5 - scale);
  key = strack->createNodeKeyFrame(.2);
  key->setTranslate( Ogre::Vector3(mid.x, mid.y, mid.z));
  key->setRotation(yawFinal);

  key = ptrack->createNodeKeyFrame(.2);
  key->setRotation(pitchFinal);

  end = start + dir*(dist - scale);
  key = strack->createNodeKeyFrame(.5);
  key->setTranslate( Ogre::Vector3(end.x, end.y, end.z));
  key->setRotation(yawFinal);

  key = ptrack->createNodeKeyFrame(.5);
  key->setRotation(pitchFinal);

  this->animState = this->scene->GetManager()->createAnimationState("cameratrack");
  this->animState->setEnabled(true);
  this->animState->setLoop(false);
}

//////////////////////////////////////////////////////////////////////////////
/// Set the camera to track an entity
void UserCamera::TrackModel( Model *model )
{
  /* NATY: Put back in
  this->sceneNode->getParent()->removeChild(this->sceneNode);

  if (model)
  {
    Body *b = model->GetCanonicalBody();
    b->GetVisualNode()->GetSceneNode()->addChild(this->sceneNode);
    this->camera->setAutoTracking(true, b->GetVisualNode()->GetSceneNode() );
  }
  else
  {
    this->origParentNode->addChild(this->sceneNode);
    this->camera->setAutoTracking(false, NULL);
    this->camera->setPosition(Ogre::Vector3(0,0,0));
    this->camera->setOrientation(Ogre::Quaternion(-.5,-.5,.5,.5));
  }
  */
}
