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
/* Desc: GL Window
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id$
 */

#include "Model.hh"
#include <X11/keysym.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <FL/Fl_Menu_Item.H>
#include <FL/Fl_Menu_Bar.H>

#include <GL/glx.h>

#include <Ogre.h>
#include "OgreDynamicLines.hh"
#include "OgreVisual.hh"
#include "Param.hh"
#include "Entity.hh"
#include "Body.hh"
#include "OgreCamera.hh"
#include "OgreCreator.hh"
#include "Simulator.hh"
#include "Global.hh"
#include "GazeboMessage.hh"
#include "MainMenu.hh"
#include "CameraManager.hh"
#include "UserCamera.hh"
#include "World.hh"
#include "Gui.hh"

#include "OgreHUD.hh"
#include "OgreAdaptor.hh"

#include "GLFrame.hh"
#include "GLWindow.hh"

#include <boost/thread.hpp>
#include "Events.hh"

using namespace gazebo;

GLWindow *GLWindow::activeWin = NULL;

////////////////////////////////////////////////////////////////////////////////
// Constructor
GLWindow::GLWindow( int x, int y, int w, int h, const std::string &label)
    : Fl_Gl_Window( x, y, w, h, "" )
{
  this->end();

  this->userCamera = NULL;
  this->moveAmount = 0.1;
  this->moveScale = 1;
  this->rotateAmount = 0.1;

  this->directionVec.x = 0;
  this->directionVec.y = 0;
  this->directionVec.z = 0;
  this->leftMousePressed = false;
  this->rightMousePressed = false;
  this->middleMousePressed = false;

  this->keys.clear();

  if (activeWin == NULL)
    activeWin = this;

  Events::ConnectCreateEntitySignal( boost::bind(&GLWindow::CreateEntity, this, _1) );
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GLWindow::~GLWindow()
{
  Events::DisconnectCreateEntitySignal( boost::bind(&GLWindow::CreateEntity, this, _1) );

  if (this->userCamera)
    delete this->userCamera;
}

////////////////////////////////////////////////////////////////////////////////
/// Create user cameras
void GLWindow::CreateCameras()
{
  this->show();
  Fl::check();

  this->make_current();

  // Create the default camera.
  this->userCamera = new UserCamera( this );
  this->userCamera->Load(NULL);
}


////////////////////////////////////////////////////////////////////////////////
// Init
void GLWindow::Init()
{
  this->show();
  Fl::check();
  this->mouseDrag = false;

  if (this->userCamera == NULL)
  {
    // Create the default camera.
    this->userCamera = new UserCamera( this );
    this->userCamera->Load(NULL);
  }

  this->userCamera->Init();
  this->userCamera->RotatePitch( DTOR(30) );
  this->userCamera->Translate( Vector3(-5, 0, 1) );

  this->activeCamera = this->userCamera;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the width of the gui's rendering window
unsigned int GLWindow::GetWidth() const
{
  return this->w();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the height of the gui's rendering window
unsigned int GLWindow::GetHeight() const
{
  return this->h();
}


////////////////////////////////////////////////////////////////////////////////
void GLWindow::flush()
{
  return;
}

////////////////////////////////////////////////////////////////////////////////
// Update function
void GLWindow::Update()
{
  if (this->activeCamera && this->activeCamera->GetUserMovable())
  {
    Time diff = Simulator::Instance()->GetRealTime() - this->lastUpdateTime;
    this->activeCamera->Translate( this->directionVec * diff.Double()  );
    this->directionVec.Set(0,0,0);
  }

  this->lastUpdateTime = Simulator::Instance()->GetRealTime();

  // continuously apply force to selected body
  Entity *entity = World::Instance()->GetSelectedEntity(); 

  if (entity && entity->GetType() == Entity::BODY &&
      (this->keys[FL_Control_L] || this->keys[FL_Control_R]) )
  {
    Body *body = (Body*)(entity);
    if (this->rightMousePressed && body)
    {
      body->SetForce(this->forceVec);
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Get a pointer to the camera
UserCamera *GLWindow::GetCamera() const
{
  return this->userCamera;
}

////////////////////////////////////////////////////////////////////////////////
// Get the average FPS for this window
float GLWindow::GetAvgFPS() const
{
  return this->activeCamera->GetAvgFPS();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the number of triangles being rendered
unsigned int GLWindow::GetTriangleCount() const
{
  return this->activeCamera->GetTriangleCount();
}

////////////////////////////////////////////////////////////////////////////////
/// Handle a mouse button push
void GLWindow::HandleMousePush()
{
  // reset applied forces to 0
  this->forceVec = 0;
  this->torqueVec = 0;

  this->mousePushPos = this->mousePos;

  this->boxMaker.MousePushCB(this->mousePos);
  this->sphereMaker.MousePushCB(this->mousePos);
  this->cylinderMaker.MousePushCB(this->mousePos);
  this->hingeJointMaker.MousePushCB(this->mousePos);

  if (this->boxMaker.IsActive() || this->sphereMaker.IsActive() ||
      this->cylinderMaker.IsActive() || this->hingeJointMaker.IsActive())
    return;

  if (World::Instance()->GetSelectedEntity())
  {
    OgreAdaptor::Instance()->GetEntityAt(this->activeCamera, 
                                         this->mousePos, this->mouseModifier);
  }

  // Get the mouse button that was pressed (if one was pressed)
  switch (Fl::event_button())
  {
    case FL_LEFT_MOUSE:
      this->leftMousePressed = true;
      //this->torqueVec = Vector3(0,0,0); // not necessary
      break;

    case FL_RIGHT_MOUSE:
      this->rightMousePressed = true;
      //this->forceVec = Vector3(0,0,0); // not necessary
      break;

    case FL_MIDDLE_MOUSE:
      this->middleMousePressed = true;
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Handle a mouse button release
void GLWindow::HandleMouseRelease()
{
  OgreCreator::SetVisible("guiline", false);

  this->boxMaker.MouseReleaseCB(this->mousePos);
  this->sphereMaker.MouseReleaseCB(this->mousePos);
  this->cylinderMaker.MouseReleaseCB(this->mousePos);
  this->hingeJointMaker.MouseReleaseCB(this->mousePos);

  if (this->boxMaker.IsActive() || this->sphereMaker.IsActive() ||
      this->cylinderMaker.IsActive() || this->hingeJointMaker.IsActive())
    return;

  // Get the mouse button that was pressed (if one was pressed)
  switch (Fl::event_button())
  {
    case FL_LEFT_MOUSE:
      this->leftMousePressed = false;
      break;

    case FL_RIGHT_MOUSE:
      this->rightMousePressed = false;
      break;

    case FL_MIDDLE_MOUSE:
      this->middleMousePressed = false;
      break;
  }

  if (!this->mouseDrag && 
      (this->keys[FL_Control_L] || this->keys[FL_Control_R]))
  {
    Entity *entity = OgreAdaptor::Instance()->GetEntityAt(this->activeCamera, 
                                         this->mousePos, this->mouseModifier);

    Model *model = Simulator::Instance()->GetParentModel(entity);
    Body *body = Simulator::Instance()->GetParentBody(entity);

    // reset applied forces to 0
    this->forceVec = 0;
    this->torqueVec = 0;

    switch (Fl::event_button())
    {
      case FL_LEFT_MOUSE:
        if (model) 
          World::Instance()->SetSelectedEntity( model );
        this->mouseOriginPos = Vector2<int>( Fl::event_x(), Fl::event_y() );
        break;

      case FL_RIGHT_MOUSE:
        if (body) 
          World::Instance()->SetSelectedEntity( body );
        this->mouseOriginPos = Vector2<int>( Fl::event_x(), Fl::event_y() );
        break;

      case FL_MIDDLE_MOUSE:
        break;
    }
  }

  this->mouseDrag = false;
}

////////////////////////////////////////////////////////////////////////////////
/// Handle a mouse drag
void GLWindow::HandleMouseDrag()
{
  // stop simulation when this is happening
  boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMRMutex());
  this->mouseDrag = true;

  this->boxMaker.MouseDragCB(this->mousePos);
  this->sphereMaker.MouseDragCB(this->mousePos);
  this->cylinderMaker.MouseDragCB(this->mousePos);
  this->hingeJointMaker.MouseDragCB(this->mousePos);
  if (this->boxMaker.IsActive() || this->sphereMaker.IsActive() ||
      this->cylinderMaker.IsActive() || this->hingeJointMaker.IsActive())
    return;

  if (this->activeCamera && this->activeCamera->GetUserMovable())
  {
    Vector2<int> drag = this->mousePos - this->prevMousePos;

    double vpw        = this->activeCamera->GetViewportWidth();
    double vph        = this->activeCamera->GetViewportHeight();
    Vector3 camUp     = this->activeCamera->GetUp();
    Vector3 camRight  = this->activeCamera->GetRight();

    Entity *entity = World::Instance()->GetSelectedEntity();

    if (this->leftMousePressed)
    {

      if ( entity && (entity->GetType() == Entity::MODEL || 
           entity->GetType() == Entity::BODY) && 
           (this->keys[FL_Control_L] || this->keys[FL_Control_R])  )
      {
        if ((entity->GetType() == Entity::MODEL || 
             entity->GetType() == Entity::BODY) && 
            this->mouseModifier.substr(0,3) == "rot")
          this->EntityRotate(entity);
        else if (this->mouseModifier.substr(0,5) == "trans")
          this->EntityTranslate(entity);
      }
      else
      {
        //
        // interactively rotate view
        //
        this->activeCamera->RotateYaw(DTOR(drag.x * this->rotateAmount));
        this->activeCamera->RotatePitch(DTOR(-drag.y * this->rotateAmount));
      }
    }
    else if (this->rightMousePressed)
    {
      if ( entity && (entity->GetType() == Entity::MODEL || entity->GetType() == Entity::BODY) && 
           (this->keys[FL_Control_L] || this->keys[FL_Control_R]))
      {
        if (entity->GetType() == Entity::MODEL)
        {
          Model *model = (Model*)(entity);
          double distance, scaleX, scaleY;
          Pose3d modelPose;
          Vector3 moveVector;

          //
          // interactively set pose to selected model
          //
          modelPose = model->GetAbsPose();
          distance = (modelPose.pos - 
              this->activeCamera->GetCameraPosition()).GetLength();
          scaleX = distance * 
            tan (this->activeCamera->GetHFOV().GetAsRadian() / 2.0f ) * 2.0f;
          scaleY = distance * 
            tan (this->activeCamera->GetVFOV().GetAsRadian() / 2.0f ) * 2.0f;
          moveVector = (camRight*drag.x/vpw*scaleX - camUp*drag.y/vph*scaleY);
          // std::cout << " vpw " << vpw
          //           << " vph " << vph
          //           << " distance " << distance
          //           << " scaleX " << scaleX
          //           << " scaleY " << scaleY
          //           << std::endl;

          modelPose.pos += moveVector;
          model->SetAbsPose(modelPose);
          //std::cout << "set pose (" << modelPose << ") to model (" 
          //          << model->GetName() << ")" << std::endl;
        }

        if (entity->GetType() == Entity::BODY)
        {
          double distance, scaleX, scaleY, forceScale;
          Pose3d bodyPose;
          Vector3 moveVector;
          Body *body = (Body*)(entity);

          //
          // interactively set force to selected body
          //
          bodyPose = body->GetAbsPose();
          distance = (bodyPose.pos - 
              this->activeCamera->GetCameraPosition()).GetLength();
          scaleX = distance * 
            tan (this->activeCamera->GetHFOV().GetAsRadian() / 2.0f ) * 2.0f;
          scaleY = distance * 
            tan (this->activeCamera->GetVFOV().GetAsRadian() / 2.0f ) * 2.0f;
          moveVector = (camRight*drag.x/vpw*scaleX - camUp*drag.y/vph*scaleY);
          // std::cout << " vpw " << vpw
          //           << " vph " << vph
          //           << " distance " << distance
          //           << " scaleX " << scaleX
          //           << " scaleY " << scaleY
          //           << std::endl;

          forceScale = 10000.0;
          this->forceVec = moveVector*forceScale;
          //std::cout << "set body force to" << this->forceVec 
          //          << " to body " << body->GetName() << std::endl;
        }
      }
      else
      {
        //
        // interactively pan view
        //
        this->directionVec.x = 0;
        this->directionVec.y =  drag.x * this->moveAmount;
        this->directionVec.z =  drag.y * this->moveAmount;
      }
    }
    else if (this->middleMousePressed)
    {
      if (entity && entity->GetType() == Entity::BODY && 
          (this->keys[FL_Control_L] || this->keys[FL_Control_R]))
      {
        double distance, scaleX, scaleY;
        Vector3 moveVector;
        Pose3d bodyPose;

        Body *body = (Body*)(entity);

        //
        // interactively set pose to selected body
        //
        bodyPose = body->GetAbsPose();

        //Vector2<double> ddrag((double)drag.x,(double)drag.y);
        //if (drag.x*drag.x + drag.y*drag.y > 0)
        //  ddrag.Normalize();
        //Vector3 dragVector     = (camRight*ddrag.x - camUp*ddrag.y);
        //bodyPose.pos += dragVector*0.05;

        distance = (bodyPose.pos - 
            this->activeCamera->GetCameraPosition()).GetLength();
        scaleX = distance * 
          tan (this->activeCamera->GetHFOV().GetAsRadian() / 2.0f ) * 2.0f;
        scaleY = distance * 
          tan (this->activeCamera->GetVFOV().GetAsRadian() / 2.0f ) * 2.0f;
        moveVector = (camRight*drag.x/vpw*scaleX - camUp*drag.y/vph*scaleY);

        bodyPose.pos += moveVector;
        body->SetAbsPose(bodyPose);
        //std::cout << "set pose (" << bodyPose << ") to Body (" 
        //          << body->GetName() << ")" << std::endl;
      }
      else
      {
        this->directionVec.x =  drag.y * this->moveAmount;
        this->directionVec.y =  0;
        this->directionVec.z =  0;
      }
    }
  }

}

////////////////////////////////////////////////////////////////////////////////
// Handle mouse wheel movement
void GLWindow::HandleMouseWheel(int dx, int dy)
{
  // stop simulation when this is happening
  boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMRMutex());

  Entity *entity = World::Instance()->GetSelectedEntity();

  if ( entity && (entity->GetType() == Entity::MODEL || entity->GetType() == Entity::BODY) && 
      (this->keys[FL_Control_L] || this->keys[FL_Control_R]) )
  {
    // FIXME: old
    if (entity->GetType() == Entity::MODEL)
    {
      Model *model = (Model*)(entity);

      Pose3d pose = model->GetAbsPose();
      pose.pos.z += dy * 0.05;
      model->SetAbsPose(pose);
      //std::cout << "set pose z(" << pose.pos.z << ") to model (" 
      //          << model->GetName() << ")" << std::endl;
    }
    // FIXME: old
    if (entity->GetType() == Entity::BODY)
    {
      Body *body = (Body*)(entity);
      Pose3d pose = body->GetAbsPose();
      pose.pos.z += dy * 0.05;
      body->SetAbsPose(pose);
      //std::cout << "set pose z(" << pose.pos.z << ") to body (" 
      //          << body->GetName() << ")" << std::endl;
    }
  }
  else if (this->activeCamera && this->activeCamera->GetUserMovable())
  {
    this->directionVec.x -=  50.0 * dy * this->moveAmount;
    this->directionVec.y =  0;
    this->directionVec.z =  0;
  }

}

////////////////////////////////////////////////////////////////////////////////
/// Handle a key press
void GLWindow::HandleKeyPress(int keyNum)
{
  if (this->boxMaker.IsActive() || this->sphereMaker.IsActive() ||
      this->cylinderMaker.IsActive() || this->hingeJointMaker.IsActive())
    return;

  std::map<int,int>::iterator iter;
  this->keys[keyNum] = 1;

  // loop through the keys to find the modifiers -- swh
  float moveAmount = this->moveAmount;
  for (iter = this->keys.begin(); iter!= this->keys.end(); iter++)
  {
    if (iter->second == 1)
    {
      switch (iter->first)
      {
        case FL_Control_L:
        case FL_Control_R:
          moveAmount = this->moveAmount * 10;
          break;
        case FL_CTRL+'q':
          Simulator::Instance()->SetUserQuit();
          break;
      }
    }
  }

  for (iter = this->keys.begin(); iter!= this->keys.end(); iter++)
  {
    if (iter->second == 1)
    {
      switch (iter->first)
      {
        case ' ':
          Simulator::Instance()->SetPaused(!Simulator::Instance()->IsPaused() );
          break;

        case '=':
        case '+':
          this->moveAmount *= 2;
          break;

        case '-':
        case '_':
          this->moveAmount *= 0.5;
          break;

        case XK_j:
          this->forceVec.z -= 100*this->moveAmount;
          break;
        case XK_k:
          this->forceVec.z += 100*this->moveAmount;
          break;
        case XK_h:
          this->forceVec.y -= 100*this->moveAmount;
          break;
        case XK_l:
          this->forceVec.y += 100*this->moveAmount;
          break;
        case XK_x:
          this->forceVec.x += 100*this->moveAmount;
          break;
        case XK_z:
          this->forceVec.x -= 100*this->moveAmount;
          break;

        case XK_Up:
        case XK_w:
          this->directionVec.x += this->moveAmount;
          break;

        case XK_Down:
        case XK_s:
          this->directionVec.x -= this->moveAmount;
          break;

        case XK_Left:
        case XK_a:
          this->directionVec.y += this->moveAmount;
          break;

        case XK_Right:
        case XK_d:
          this->directionVec.y -= this->moveAmount;
          break;

        case XK_Page_Down:
        case XK_e:
          this->directionVec.z += this->moveAmount;
          break;

        case XK_Page_Up:
        case XK_q:
          this->directionVec.z -= this->moveAmount;
          break;
        default:
          break;
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Handle a key release
void GLWindow::HandleKeyRelease(int keyNum)
{
  this->keys[keyNum] = 0;

  // Handle all toggle keys
  switch (keyNum)
  {
    case FL_Escape:
      Simulator::Instance()->SetUserQuit();
      break;

    case '[':
      CameraManager::Instance()->IncActiveCamera();
      break;

    case ']':
      CameraManager::Instance()->DecActiveCamera();
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Handle events
int GLWindow::handle(int event)
{
  bool handled = false;

  // Get the mouse position
  this->mousePos = Vector2<int>( Fl::event_x(), Fl::event_y() );

  // Handle Events
  switch(event)
  {
    case FL_UNFOCUS:
    case FL_LEAVE:
      handled = true;
      break;

    case FL_FOCUS:
    case FL_ENTER: 
      activeWin = this;
      handled = true;
      break;

    case FL_CLOSE:
      Simulator::Instance()->SetUserQuit();
      handled = true;
      break;

    case FL_PUSH:
      this->HandleMousePush();
      handled = true;
      break;

    case FL_RELEASE:
      this->HandleMouseRelease();
      handled = true;
      break;

    case FL_DRAG:
      this->HandleMouseDrag();
      handled = true;
      break;

    case FL_SHORTCUT:
    case FL_KEYDOWN:
      if (Fl::event_key() == FL_Escape)
        World::Instance()->SetSelectedEntity(NULL);
      else if ((Fl::event_state() | FL_CTRL) && Fl::event_key() == 113)
        // Capture CTRL-q
        Simulator::Instance()->SetUserQuit();
      else if (activeWin != this)
        activeWin->HandleKeyPress(Fl::event_key());
      else
        this->HandleKeyPress(Fl::event_key());
      handled = true;
      break;

    case FL_KEYUP:
      if (Fl::event_key() == FL_Escape)
        World::Instance()->SetSelectedEntity(NULL);
      else if (activeWin != this)
        activeWin->HandleKeyRelease(Fl::event_key());
      else
        this->HandleKeyRelease(Fl::event_key());
      handled = true;
      break;

    case FL_MOUSEWHEEL:
      this->HandleMouseWheel(Fl::event_dx(), Fl::event_dy());
      handled = true;
      break;
  }

  this->prevMousePos = this->mousePos;

  if (Fl::event_key() == FL_Escape)
  {
    return 1;
  }


  if (!handled)
    return Fl_Gl_Window::handle(event);
  else
    return 1;
}

////////////////////////////////////////////////////////////////////////////////
// Handle resizing the window
void GLWindow::resize(int x, int y, int w, int h)
{
  Fl_Gl_Window::resize(x,y,w,h);
  this->userCamera->Resize(w,h);

  this->redraw();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the visual info
XVisualInfo *GLWindow::GetVisualInfo() const
{
  return this->visual;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the display
Display *GLWindow::GetDisplay() const
{
  return this->display;
}


////////////////////////////////////////////////////////////////////////////////
/// Set the active camera
void GLWindow::SetActiveCamera( OgreCamera *camera )
{
   this->activeCamera = camera;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the style of the view = "front, left, top, user"
void GLWindow::SetViewStyle(std::string view)
{
  UserCamera *cam =  this->GetCamera();
  Pose3d pose = cam->GetWorldPose();

  if (view == "Top")
  {
    pose.rot.SetFromEuler( Vector3(0, DTOR(90), 0) );
  }
  else if (view == "Left")
  {
    pose.rot.SetFromEuler( Vector3(0, 0, DTOR(90) ) );
  }
  else if (view == "Front")
  {
    pose.rot.SetFromEuler( Vector3(0, 0,0) );
  }
  else
  {
    OgreCamera *cam = CameraManager::Instance()->GetCamera( view );
    ((UserCamera*)this->activeCamera)->SetCamera(cam);
  }

  cam->SetWorldPose( pose );
}

////////////////////////////////////////////////////////////////////////////////
// Get point on a plane
Vector3 GLWindow::GetWorldPointOnPlane(int x, int y, Vector3 planeNorm, double d)
{
  Vector3 origin, dir;
  double dist;

  // Cast two rays from the camera into the world
  CameraManager::Instance()->GetActiveCamera()->GetCameraToViewportRay(x, y, origin, dir);

  dist = origin.GetDistToPlane(dir, planeNorm, d);

  // Compute two points on the plane. The first point is the current
  // mouse position, the second is the previous mouse position
  return origin + dir * dist; 
}

////////////////////////////////////////////////////////////////////////////////
// Rotate and entity, or apply torque
void GLWindow::EntityRotate(Entity *entity)
{
  Vector3 planeNorm, planeNorm2;
  Vector3 p1, p2;
  Vector3 a,b;
  Vector3 ray(0,0,0);

  Pose3d modelPose = entity->GetAbsPose();

  // Figure out which axis to rotate around
  if (this->mouseModifier == "rotx")
    ray.x = 1.0;
  else if (this->mouseModifier == "roty")
    ray.y = 1.0;
  else if (this->mouseModifier == "rotz")
    ray.z = 1.0;

  // Compute the normal to the plane on which to rotate
  planeNorm = modelPose.rot.RotateVector(ray);
  double d = -modelPose.pos.GetDotProd(planeNorm);

  p1 = this->GetWorldPointOnPlane( this->mousePos.x, this->mousePos.y,
      planeNorm, d);

  p2 = this->GetWorldPointOnPlane( this->prevMousePos.x, 
      this->prevMousePos.y, planeNorm, d);

  OgreCreator::DrawLine(entity->GetAbsPose().pos,p1, "guiline");

  // Get point vectors relative to the entity's pose
  a = p1 - entity->GetAbsPose().pos;
  b = p2 - entity->GetAbsPose().pos;

  a.Normalize();
  b.Normalize();

  // Get the angle between the two vectors. This is the amount to
  // rotate the entity 
  float angle = acos(a.GetDotProd(b));
  if (isnan(angle))
    angle = 0;

  // Compute the normal to the plane which is defined by the
  // direction of rotation
  planeNorm2 = a.GetCrossProd(b);
  planeNorm2.Normalize();

  // Switch rotation direction if the two normals don't line up
  if ( planeNorm.GetDotProd(planeNorm2) > 0)
    angle *= -1;

  if (entity->GetType() == Entity::MODEL) 
  {
    Quatern delta; 
    delta.SetFromAxis( ray.x, ray.y, ray.z,angle);
    
    modelPose.rot = modelPose.rot * delta;
    entity->SetAbsPose(modelPose);
  }
  else
  {
    ((Body*)entity)->SetTorque(planeNorm * angle * Gui::forceMultiplier);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Translate an entity, or apply force
void GLWindow::EntityTranslate(Entity *entity)
{
  Pose3d pose = entity->GetAbsPose();

  Vector3 origin1, dir1, p1;
  Vector3 origin2, dir2, p2;

  // Cast two rays from the camera into the world
  this->activeCamera->GetCameraToViewportRay(this->mousePos.x, 
      this->mousePos.y, origin1, dir1);
  this->activeCamera->GetCameraToViewportRay(this->prevMousePos.x, 
      this->prevMousePos.y, origin2, dir2);

  Vector3 moveVector(0,0,0);
  Vector3 planeNorm(0,0,1);
  if (this->mouseModifier == "transx")
    moveVector.x = 1;
  else if (this->mouseModifier == "transy")
    moveVector.y = 1;
  else if (this->mouseModifier == "transz")
  {
    moveVector.z = 1;
    planeNorm.Set(1,0,0);
  }

  // Compute the distance from the camera to plane of translation
  double d = -pose.pos.GetDotProd(planeNorm);
  double dist1 = origin1.GetDistToPlane(dir1, planeNorm, d);
  double dist2 = origin2.GetDistToPlane(dir2, planeNorm, d);

  // Compute two points on the plane. The first point is the current
  // mouse position, the second is the previous mouse position
  p1 = origin1 + dir1 * dist1;
  p2 = origin2 + dir2 * dist2;

  OgreCreator::DrawLine(entity->GetAbsPose().pos,p2, "guiline");

  moveVector *= p1 - p2;

  if (entity->GetType() == Entity::MODEL)
  {
    pose.pos += moveVector;
    entity->SetRelativePose(pose);
  }
  else if (entity->GetType() == Entity::BODY)
  {
    Body *body = (Body*)(entity);
    moveVector *= Gui::forceMultiplier;
    body->SetForce(moveVector);
  }
}

void GLWindow::CreateEntity(std::string name)
{
  World::Instance()->SetSelectedEntity(NULL);
  this->boxMaker.Stop();
  this->sphereMaker.Stop();
  this->cylinderMaker.Stop();
  this->hingeJointMaker.Stop();

  this->cursor(FL_CURSOR_CROSS);

  if (name == "box")
    this->boxMaker.Start();
  else if (name == "cylinder")
    this->cylinderMaker.Start();
  else if (name == "sphere")
    this->sphereMaker.Start();
  else if (name == "hingejoint")
    this->hingeJointMaker.Start();
  else
    this->cursor(FL_CURSOR_DEFAULT);

}
