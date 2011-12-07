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

#include "common/Console.hh"
#include "common/Exception.hh"
#include "common/Events.hh"

#include "rendering/GUIOverlay.hh"
#include "rendering/Conversions.hh"
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

  this->connections.push_back(
      event::Events::ConnectPreRender(
        boost::bind(&UserCamera::Update, this)));
  
  this->connections.push_back(
      event::Events::ConnectShowCameras(
        boost::bind(&UserCamera::ToggleShowVisual, this) ) );

  this->connections.push_back(
      event::Events::ConnectRender( boost::bind(&Camera::Render, this)));
  this->connections.push_back( event::Events::ConnectPostRender(
        boost::bind(&Camera::PostRender, this)));

  this->animState = NULL;

  this->gui = new GUIOverlay();

  this->orbitViewController = new OrbitViewController(this);
  this->fpsViewController = new FPSViewController(this);
  this->viewController = this->fpsViewController;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
UserCamera::~UserCamera()
{
  delete this->orbitViewController;
  delete this->fpsViewController;

  delete this->gui;
  this->gui = NULL;

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
  this->SetHFOV(DTOR(60));
  this->SetClipDist(0.001, 100);

  /*this->visual = new Visual(this->GetName() + "_OUTLINE", this->pitchNode);

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
  line->setVisibilityFlags(GZ_VISIBILITY_GUI);

  this->visual->SetVisible(false);
  */
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

  if (this->gui)
    this->gui->Update();
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
void UserCamera::HandleMouseEvent(const common::MouseEvent &_evt)
{
  if (!this->gui || !this->gui->HandleMouseEvent(_evt))
    this->viewController->HandleMouseEvent(_evt);
}

bool UserCamera::AttachToVisualImpl( VisualPtr _visual,bool _inheritOrientation,
                                     double _minDist, double _maxDist )
{
  Camera::AttachToVisualImpl(_visual, _inheritOrientation);
  if (_visual)
  {
    math::Pose origPose = this->GetWorldPose();
    double yaw = atan2(origPose.pos.x - _visual->GetWorldPose().pos.x,
                       origPose.pos.y - _visual->GetWorldPose().pos.y);
    yaw = _visual->GetWorldPose().rot.GetAsEuler().z;

    double zDiff = origPose.pos.z - _visual->GetWorldPose().pos.z;
    double pitch = 0;

    if (fabs(zDiff) > 1e-3) 
    {
      double dist = _visual->GetWorldPose().pos.Distance( 
          this->GetWorldPose().pos);
      pitch = acos(zDiff/dist );
    }

    this->RotateYaw(yaw);
    this->RotatePitch(pitch);

    math::Box bb = _visual->GetBoundingBox();
    math::Vector3 pos = bb.GetCenter();
    pos.z = bb.max.z; 

    this->SetViewController(OrbitViewController::GetTypeString(), pos);
    ((OrbitViewController*)this->viewController)->SetDistanceRange( _minDist, _maxDist );
  }
  else
    this->SetViewController(FPSViewController::GetTypeString());

  return true;
}

bool UserCamera::TrackVisualImpl( VisualPtr _visual )
{
  Camera::TrackVisualImpl( _visual );
  if (_visual)
    this->SetViewController(OrbitViewController::GetTypeString());
  else
    this->SetViewController(FPSViewController::GetTypeString());

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Set view controller
void UserCamera::SetViewController( const std::string &type )
{
  if (this->viewController->GetTypeString() == type)
    return;

  if (type == OrbitViewController::GetTypeString())
    this->viewController = this->orbitViewController;
  else if (type == FPSViewController::GetTypeString())
    this->viewController = this->fpsViewController;
  else
    gzthrow("Invalid view controller type: " + type );

  this->viewController->Init();
}

////////////////////////////////////////////////////////////////////////////////
// Set view controller
void UserCamera::SetViewController( const std::string &type, 
                                    const math::Vector3 &_pos )
{
  if (this->viewController->GetTypeString() == type)
    return;

  if (type == OrbitViewController::GetTypeString())
    this->viewController = this->orbitViewController;
  else if (type == FPSViewController::GetTypeString())
    this->viewController = this->fpsViewController;
  else
    gzthrow("Invalid view controller type: " + type );

  this->viewController->Init(_pos);
}

////////////////////////////////////////////////////////////////////////////////
/// Resize the camera
void UserCamera::Resize(unsigned int /*_w*/, unsigned int /*_h*/)
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

    if (this->gui)
    {
      this->gui->Resize(this->viewport->getActualWidth(),
                        this->viewport->getActualHeight());
    }
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
  //this->visual->ToggleVisible();
}

////////////////////////////////////////////////////////////////////////////////
// Set whether to show the visual
void UserCamera::ShowVisual(bool /*_s*/)
{
  //this->visual->SetVisible(_s);
}

//////////////////////////////////////////////////////////////////////////////
void UserCamera::MoveToVisual( const std::string &_name )
{
  VisualPtr visual = this->scene->GetVisual(_name);
  if (visual)
    this->MoveToVisual(visual);
  else
    gzerr << "MoveTo Unknown visual[" << _name << "]\n";
}

//////////////////////////////////////////////////////////////////////////////
// Move the camera to focus on an scene node
void UserCamera::MoveToVisual( VisualPtr _visual )
{
    if (!_visual)
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

  math::Box box = _visual->GetBoundingBox();
  math::Vector3 size = box.GetSize();
  double maxSize = std::max(std::max(size.x, size.y), size.z);


  math::Vector3 start = this->GetWorldPose().pos;
  start.Correct();
  math::Vector3 end = box.GetCenter();
  end.Correct();
  math::Vector3 dir = end - start;
  dir.Correct();
  dir.Normalize();

  double dist = start.Distance(end) - maxSize;

  math::Vector3 mid = start + dir*(dist*.5);
  mid.z = box.GetCenter().z + box.GetSize().z + 2.0;

  dir = end - mid;
  dir.Correct();

  dist = mid.Distance(end) - maxSize;

  double yawAngle = atan2(dir.y,dir.x);
  double pitchAngle = atan2(-dir.z, sqrt(dir.x*dir.x + dir.y*dir.y));
  Ogre::Quaternion yawFinal(Ogre::Radian(yawAngle), Ogre::Vector3(0,0,1));
  Ogre::Quaternion pitchFinal(Ogre::Radian(pitchAngle), Ogre::Vector3(0,1,0));

  dir.Normalize();

  double scale = maxSize / tan( (this->GetHFOV()/2.0).GetAsRadian() );

  end = mid + dir*(dist - scale);

  Ogre::TransformKeyFrame *key;

  key = strack->createNodeKeyFrame(0);
  key->setTranslate(Ogre::Vector3(start.x, start.y, start.z));
  key->setRotation(this->sceneNode->getOrientation());

  key = ptrack->createNodeKeyFrame(0);
  key->setRotation(this->pitchNode->getOrientation());

  key = strack->createNodeKeyFrame(.2);
  key->setTranslate( Ogre::Vector3(mid.x, mid.y, mid.z));
  key->setRotation(yawFinal);

  key = ptrack->createNodeKeyFrame(.2);
  key->setRotation(pitchFinal);

  key = strack->createNodeKeyFrame(.5);
  key->setTranslate( Ogre::Vector3(end.x, end.y, end.z));
  key->setRotation(yawFinal);

  key = ptrack->createNodeKeyFrame(.5);
  key->setRotation(pitchFinal);

  this->animState = this->scene->GetManager()->createAnimationState("cameratrack");

  this->animState->setTimePosition(0);
  this->animState->setEnabled(true);
  this->animState->setLoop(false);
}

void UserCamera::MoveToPosition( const math::Vector3 &_end, 
                                 double _pitch, double _yaw, double _time)
{
  Ogre::TransformKeyFrame *key;
  math::Vector3 start = this->GetWorldPose().pos;

  Ogre::Quaternion yawFinal(Ogre::Radian(_yaw), Ogre::Vector3(0,0,1));
  Ogre::Quaternion pitchFinal(Ogre::Radian(_pitch), Ogre::Vector3(0,1,0));

  Ogre::Animation *anim = this->scene->GetManager()->createAnimation("cameratrack",_time);
  anim->setInterpolationMode(Ogre::Animation::IM_SPLINE);

  Ogre::NodeAnimationTrack *strack = anim->createNodeTrack(0,this->sceneNode);
  Ogre::NodeAnimationTrack *ptrack = anim->createNodeTrack(1,this->pitchNode);

  key = strack->createNodeKeyFrame(0);
  key->setTranslate(Ogre::Vector3(start.x, start.y, start.z));
  key->setRotation(this->sceneNode->getOrientation());

  key = ptrack->createNodeKeyFrame(0);
  key->setRotation(this->pitchNode->getOrientation());

  key = strack->createNodeKeyFrame(_time);
  key->setTranslate( Ogre::Vector3(_end.x, _end.y, _end.z));
  key->setRotation(yawFinal);

  key = ptrack->createNodeKeyFrame(_time);
  key->setRotation(pitchFinal);

  this->animState = this->scene->GetManager()->createAnimationState("cameratrack");

  this->animState->setTimePosition(0);
  this->animState->setEnabled(true);
  this->animState->setLoop(false);
}

//////////////////////////////////////////////////////////////////////////////
void UserCamera::SetRenderTarget(Ogre::RenderTarget *_target)
{
  Camera::SetRenderTarget(_target);

  this->viewport->setVisibilityMask(GZ_VISIBILITY_ALL);
  this->gui->Init(this->renderTarget);
  this->initialized = true;
}

//////////////////////////////////////////////////////////////////////////////
GUIOverlay *UserCamera::GetGUIOverlay()
{
  return this->gui;
}
