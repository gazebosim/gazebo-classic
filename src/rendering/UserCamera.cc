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
/* Desc: Camera for viewing the world
 * Author: Nate Koenig
 * Date: 19 Jun 2008
 */

#include "rendering/ogre.h"
#include <sstream>

#include "common/Global.hh"
#include "common/Console.hh"
#include "common/Exception.hh"
#include "common/Events.hh"


#include "rendering/WindowManager.hh"
#include "rendering/FPSViewController.hh"
#include "rendering/OrbitViewController.hh"
#include "rendering/RenderTypes.hh"
#include "rendering/Scene.hh"
#include "rendering/RTShaderSystem.hh"
#include "rendering/Camera.hh"
#include "rendering/Visual.hh"
#include "rendering/DynamicLines.hh"
#include "rendering/UserCamera.hh"   

using namespace gazebo;
using namespace rendering;


int UserCamera::count = 0;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
UserCamera::UserCamera(const std::string &name_, Scene *scene_)
  : Camera(name_, scene_)
{
  std::stringstream stream;

  stream << "UserCamera_" << this->count++;
  this->name = stream.str(); 

  this->connections.push_back( event::Events::ConnectShowCamerasSignal( boost::bind(&UserCamera::ToggleShowVisual, this) ) );
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

  this->connections.clear();
}

////////////////////////////////////////////////////////////////////////////////
// Load child
void UserCamera::Load( sdf::ElementPtr _sdf )
{
  Camera::Load(_sdf);
}

////////////////////////////////////////////////////////////////////////////////
// Load 
void UserCamera::Load( )
{
  Camera::Load();
}

////////////////////////////////////////////////////////////////////////////////
/// Initialize
void UserCamera::Init()
{
  Camera::Init();
  this->SetFOV( DTOR(60) );
  this->SetClipDist(0.001, 100);

  this->visual = new Visual(this->GetName() + "_OUTLINE", this->pitchNode);

  // The lines draw a visualization of the camera
  DynamicLines *line = this->visual->CreateDynamicLine( RENDERING_LINE_LIST );

  float f = 0.2;

  // Create the front face
  line->AddPoint(math::Vector3(0, -f, -f)); 
  line->AddPoint(math::Vector3(0, -f, +f)); 

  line->AddPoint(math::Vector3(0, -f, +f)); 
  line->AddPoint(math::Vector3(0, +f, +f)); 

  line->AddPoint(math::Vector3(0, +f, +f)); 
  line->AddPoint(math::Vector3(0, +f, -f)); 

  line->AddPoint(math::Vector3(0, +f, -f)); 
  line->AddPoint(math::Vector3(0, -f, -f)); 


  // Create the connecting lines
  line->AddPoint(math::Vector3(-0.4, 0, 0)); 
  line->AddPoint(math::Vector3(+0.0, -f, -f)); 

  line->AddPoint(math::Vector3(-0.4, 0, 0)); 
  line->AddPoint(math::Vector3(+0.0, -f, +f)); 

  line->AddPoint(math::Vector3(-0.4, 0, 0)); 
  line->AddPoint(math::Vector3(+0.0, +f, +f)); 

  line->AddPoint(math::Vector3(-0.4, 0, 0)); 
  line->AddPoint(math::Vector3(+0.0, +f, -f)); 

  line->AddPoint(math::Vector3(-0.4, 0, 0)); 
  line->AddPoint(math::Vector3(+0.0, -f, -f)); 

  // Draw up arrow
  line->AddPoint(math::Vector3(0, 0, +f)); 
  line->AddPoint(math::Vector3(0, 0, +f+0.1)); 

  line->AddPoint(math::Vector3(0.0, -0.02, +f+0.1)); 
  line->AddPoint(math::Vector3(0.0, +0.02, +f+0.1)); 

  line->AddPoint(math::Vector3(0.0, +0.02, +f+0.1)); 
  line->AddPoint(math::Vector3(0.0, +0.00, +f+0.15)); 

  line->AddPoint(math::Vector3(0.0, +0.00, +f+0.15)); 
  line->AddPoint(math::Vector3(0.0, -0.02, +f+0.1)); 

  line->setMaterial("Gazebo/WhiteGlow");
  line->setVisibilityFlags(GZ_LASER_CAMERA);

  this->visual->SetVisible(false);

  //this->SetAspectRatio( Ogre::Real(this->viewport->getActualWidth()) / Ogre::Real(this->viewport->getActualHeight()) );

 // double ratio = (double)this->viewport->getActualWidth() / (double)this->viewport->getActualHeight();
  //double vfov = fabs(2.0 * atan(tan(this->GetHFOV().GetAsRadian() / 2.0) / ratio));
  //this->GetCamera()->setAspectRatio(ratio);
  //this->GetCamera()->setFOVy(Ogre::Radian(vfov));

  //this->viewport->setClearEveryFrame(true);
  //this->viewport->setBackgroundColour( this->scene->GetBackgroundColor().GetOgreColor() );
  //this->viewport->setVisibilityMask(this->visibilityMask);

  //RTShaderSystem::AttachViewport(this->viewport, this->scene);
}

////////////////////////////////////////////////////////////////////////////////
/// Update
void UserCamera::Update()
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
}

////////////////////////////////////////////////////////////////////////////////
// Post Render
void UserCamera::PostRender()
{
  Camera::PostRender();
  sdf::ElementPtr elem = this->sdf->GetOrCreateElement("save");

  if (elem->GetValueBool("enabled"))
  {
    std::string path = elem->GetValueString("path");

    char tmp[1024];
    if (!path.empty())
    {
      sprintf(tmp, "%s/%s-%04d.jpg", path.c_str(),
          this->name.c_str(), this->saveCount);
    }
    else
    {
      sprintf(tmp, "%s-%04d.jpg", this->name.c_str(), this->saveCount);
    }

    // TODO: Use the window manager instead.
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
void UserCamera::HandleMouseEvent(const common::MouseEvent &evt)
{
  this->viewController->HandleMouseEvent(evt);
}

////////////////////////////////////////////////////////////////////////////////
// Set view controller
void UserCamera::SetViewController( const std::string &type )
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
void UserCamera::Resize(unsigned int w, unsigned int h)
{
  if (this->viewport)
  {
    this->viewport->setDimensions(0,0,1,1);
    double ratio = (double)this->viewport->getActualWidth() / 
                   (double)this->viewport->getActualHeight();

    double hfov = this->sdf->GetOrCreateElement("horizontal_fov")->GetValueDouble("angle");
    double vfov = 2.0 * atan(tan( hfov / 2.0) / ratio);
    this->camera->setAspectRatio(ratio);
    this->camera->setFOVy(Ogre::Radian(vfov));
  }

}

////////////////////////////////////////////////////////////////////////////////
// Set the dimensions of the viewport
void UserCamera::SetViewportDimensions(float /*x_*/, float /*y_*/, float /*w_*/, float /*h_*/)
{
  //this->viewport->setDimensions(x, y, w, h);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the average frames per second
float UserCamera::GetAvgFPS() const
{
  return WindowManager::Instance()->GetAvgFPS(this->windowId);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the triangle count 
float UserCamera::GetTriangleCount() const
{
  return WindowManager::Instance()->GetTriangleCount(this->windowId);
}

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
// Move the camera to focus on an scene node
void UserCamera::MoveToVisual( Visual *visual_)
{
  if (!visual_)
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

  math::Vector3 start = this->GetWorldPose().pos;
  start.Correct();
  math::Vector3 end = visual_->GetWorldPose().pos;
  end.Correct();
  math::Vector3 dir = end - start;
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

  math::Vector3 size = visual_->GetBoundingBox().GetSize();

  double scale = std::max(std::max(size.x, size.y), size.z);
  scale += 0.5;

  dir.Normalize();
  double dist = start.Distance(end);

  math::Vector3 mid = start + dir*(dist*.5 - scale);
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
/// Set the camera to track a scene node
void UserCamera::TrackVisual( Visual * /*visual_*/ )
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
