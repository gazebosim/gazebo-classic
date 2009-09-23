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

#include "OgreHUD.hh"
#include "OgreAdaptor.hh"

#include "GLFrame.hh"
#include "GLWindow.hh"

#include <boost/thread.hpp>

using namespace gazebo;

GLWindow *GLWindow::activeWin = NULL;

////////////////////////////////////////////////////////////////////////////////
// Constructor
GLWindow::GLWindow( int x, int y, int w, int h, const std::string &label)
    : Fl_Gl_Window( x, y, w, h, "" )
{
  this->end();

  this->moveAmount = 0.1;
  this->moveScale = 1;
  this->rotateAmount = 0.1;

  this->directionVec.x = 0;
  this->directionVec.y = 0;
  this->directionVec.z = 0;
  this->leftMousePressed = false;
  this->rightMousePressed = false;
  this->middleMousePressed = false;
  this->controlKeyPressed = false;
  this->altKeyPressed = false;


  this->keys.clear();

  if (activeWin == NULL)
    activeWin = this;
}


////////////////////////////////////////////////////////////////////////////////
// Destructor
GLWindow::~GLWindow()
{
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
    this->activeCamera->Translate( 
        this->directionVec * (Simulator::Instance()->GetRealTime() - this->lastUpdateTime) );
    this->directionVec.Set(0,0,0);
  }

  this->lastUpdateTime = Simulator::Instance()->GetRealTime();

  // continuously apply force to selected body
  Entity *entity = Simulator::Instance()->GetSelectedEntity(); 

  if (entity->IsBody())
  {
    Body *body = (Body*)(entity);
    if (this->rightMousePressed && body)
    {
      body->SetForce(this->forceVec);
    }
    if (this->leftMousePressed && body)
    {
      body->SetTorque(this->torqueVec);
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
/// Handle a mouse button push
void GLWindow::HandleMousePush()
{
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

  if (!this->mouseDrag && this->controlKeyPressed)
  {
    Entity *entity = OgreAdaptor::Instance()->GetEntityAt(this->activeCamera, 
                                                          this->mousePos);

    Model *currModel = Simulator::Instance()->GetParentModel(
        Simulator::Instance()->GetSelectedEntity());

    Model *model = Simulator::Instance()->GetParentModel(entity);
    Body *body = Simulator::Instance()->GetParentBody(entity);

    if (currModel == model)
    {
      Simulator::Instance()->SetSelectedEntity(NULL);
      return;
    }


    switch (Fl::event_button())
    {
      case FL_LEFT_MOUSE:
        if (model) 
          Simulator::Instance()->SetSelectedEntity( model );
        this->mouseOriginPos = Vector2<int>( Fl::event_x(), Fl::event_y() );
        break;

      case FL_RIGHT_MOUSE:
        if (body) 
          Simulator::Instance()->SetSelectedEntity( body );
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

  if (this->activeCamera && this->activeCamera->GetUserMovable())
  {
    Vector2<int> drag = this->mousePos - this->prevMousePos;

    double vpw        = this->activeCamera->GetViewportWidth();
    double vph        = this->activeCamera->GetViewportHeight();
    Vector3 camUp     = this->activeCamera->GetUp();
    Vector3 camRight  = this->activeCamera->GetRight();

    Entity *entity = Simulator::Instance()->GetSelectedEntity();

    if (this->leftMousePressed)
    {
      if (entity->IsModel() || entity->IsBody())
      {
        if (entity->IsModel())
        {
          Model *model = (Model*)(entity);

          //
          // interactively change rotation pose to selected item, 
          // rotate about axis perpendicular to view port plane
          //
          Pose3d modelPose = model->GetPose();

          double distance = (modelPose.pos - this->activeCamera->GetCameraPosition()).GetLength();
          double scaleX = distance * tan (this->activeCamera->GetHFOV().GetAsRadian() / 2.0f ) * 2.0f;
          //double scaleY = distance * 
          //  tan (this->activeCamera->GetVFOV().GetAsRadian() / 2.0f ) * 2.0f;

          // axis perpendicular to view port plane
          Vector3 camRay = camRight.GetCrossProd(camUp);
          Quatern yawDragQ; 
          yawDragQ.SetFromAxis( camRay.x,camRay.y,camRay.z,drag.x/vpw*scaleX);

          modelPose.rot = yawDragQ * modelPose.rot;
          Vector3 eul = modelPose.rot.GetAsEuler();
          /*std::cout << "Set euler angles r(" << eul.x << ") p(" 
                    << eul.y << ") to model (" << model->GetName() 
                    << ")" << std::endl;
                    */

          model->SetPose(modelPose);
        }

        if (entity->IsBody())
        {
          Body *body = (Body*)(entity);
          double distance, scaleX, scaleY, torqueScale;
          Quatern qUp, qRight, upRightDragQ;

          //
          // interactively set torque selected item
          //
          Pose3d bodyPose = body->GetPose();

          distance = (bodyPose.pos - 
              this->activeCamera->GetCameraPosition()).GetLength();
          scaleX = distance * tan 
            (this->activeCamera->GetHFOV().GetAsRadian() / 2.0f ) * 2.0f;
          scaleY = distance * tan 
            (this->activeCamera->GetVFOV().GetAsRadian() / 2.0f ) * 2.0f;

          qUp.SetFromAxis(camUp.x,camUp.y,camUp.z,(double)drag.x/vpw*scaleX);

          qRight.SetFromAxis(camRight.x,camRight.y,camRight.z,
                             (double)drag.y/vpw*scaleY);
          upRightDragQ = qUp * qRight;

          torqueScale = 10000.0;
          this->torqueVec = upRightDragQ.GetAsEuler()*torqueScale;
          /*std::cout << "set euler torques (" << this->torqueVec 
                    << ") to body (" << body->GetName() << ")" << std::endl;
                    */
          body->SetTorque(this->torqueVec);
        }
      }
      else
      {
        //
        // interactively rotate view
        //
        this->activeCamera->RotateYaw(DTOR(-drag.x * this->rotateAmount));
        this->activeCamera->RotatePitch(DTOR(drag.y * this->rotateAmount));
      }
    }
    else if (this->rightMousePressed)
    {
      if (entity->IsModel() || entity->IsBody())
      {
        if (entity->IsModel())
        {
          Model *model = (Model*)(entity);
          double distance, scaleX, scaleY;
          Pose3d modelPose;
          Vector3 moveVector;

          //
          // interactively set pose to selected model
          //
          modelPose = model->GetPose();
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
          model->SetPose(modelPose);
          //std::cout << "set pose (" << modelPose << ") to model (" 
          //          << model->GetName() << ")" << std::endl;
        }

        if (entity->IsBody())
        {
          double distance, scaleX, scaleY, forceScale;
          Pose3d bodyPose;
          Vector3 moveVector;
          Body *body = (Body*)(entity);

          //
          // interactively set force to selected body
          //
          bodyPose = body->GetPose();
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
      if (entity->IsBody())
      {
        double distance, scaleX, scaleY;
        Vector3 moveVector;
        Pose3d bodyPose;

        Body *body = (Body*)(entity);

        //
        // interactively set pose to selected body
        //
        bodyPose = body->GetPose();

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
        body->SetPose(bodyPose);
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

  this->mouseDrag = true;
}

////////////////////////////////////////////////////////////////////////////////
// Handle mouse wheel movement
void GLWindow::HandleMouseWheel(int dx, int dy)
{
  // stop simulation when this is happening
  boost::recursive_mutex::scoped_lock lock(*Simulator::Instance()->GetMRMutex());

  Entity *entity = Simulator::Instance()->GetSelectedEntity();

  if (entity->IsModel() || entity->IsBody())
  {
    // FIXME: old
    if (entity->IsModel())
    {
      Model *model = (Model*)(entity);

      Pose3d pose = model->GetPose();
      pose.pos.z += dy * 0.05;
      model->SetPose(pose);
      //std::cout << "set pose z(" << pose.pos.z << ") to model (" 
      //          << model->GetName() << ")" << std::endl;
    }
    // FIXME: old
    if (entity->IsBody())
    {
      Body *body = (Body*)(entity);
      Pose3d pose = body->GetPose();
      pose.pos.z += dy * 0.05;
      body->SetPose(pose);
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
          this->controlKeyPressed = true;
          //std::cout << "press control" << std::endl;
          break;

        case FL_Alt_L:
        case FL_Alt_R:
          this->altKeyPressed = true;
          //std::cout << "press alt" << std::endl;
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

        case XK_Control_L:
        case XK_Control_R:
          this->controlKeyPressed = true;
          break;

        case XK_Alt_L:
        case XK_Alt_R:
          this->altKeyPressed = true;
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

    case FL_Control_L:
    case FL_Control_R:
      this->controlKeyPressed = false;
      //std::cout << "releasing control" << std::endl;
      break;

    case FL_Alt_L:
    case FL_Alt_R:
      this->altKeyPressed = false;
      //std::cout << "releasing alt" << std::endl;
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
      if (activeWin != this)
        activeWin->HandleKeyPress(Fl::event_key());
      else
        this->HandleKeyPress(Fl::event_key());
      handled = true;
      break;

    case FL_KEYUP:
      if (activeWin != this)
        activeWin->HandleKeyRelease(Fl::event_key());
      else
        this->HandleKeyRelease(Fl::event_key());
      handled = true;
      break;

    case FL_MOUSEWHEEL:
      this->HandleMouseWheel(Fl::event_dx(), Fl::event_dy());
      handled = true;
      break;

    default:
      break;
  }

  this->prevMousePos = this->mousePos;

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

  cam->SetWorldPose( pose );
}
