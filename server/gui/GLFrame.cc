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
/* Desc: FLTK GL Frame
 * Author: Nate Koenig
 * Date: 18 Jun 2008
 * SVN: $Id:$
 */

#include <boost/bind.hpp>

#include "CameraManager.hh"
#include "Global.hh"
#include "Pose3d.hh"
#include "GLWindow.hh"
#include "GLFrameManager.hh"
#include "UserCamera.hh"
#include "GLFrame.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
GLFrame::GLFrame(int x, int y, int w, int h, const std::string &name)
  : Fl_Group(x,y,w,h, "")
{

  this->box(FL_DOWN_BOX);

  this->headerBar = new Fl_Group(x,y,w,30);
  this->headerBar->box(FL_UP_BOX);

  this->viewChoice = new Fl_Choice(x+2, y+2, 150,26);
  this->viewChoice->add("View");
  this->viewChoice->mode(0, FL_MENU_DIVIDER);
  this->viewChoice->add("Top", "", &gazebo::GLFrame::ViewCB, this);
  this->viewChoice->add("Front", "", &gazebo::GLFrame::ViewCB, this);
  this->viewChoice->add("Left", "",  &gazebo::GLFrame::ViewCB, this);
  this->viewChoice->add("User", "", &gazebo::GLFrame::ViewCB, this);
  this->viewChoice->value(0);

  this->splitChoice = new Fl_Choice(this->viewChoice->x()+this->viewChoice->w()+2, this->viewChoice->y(), 150, 26);

  this->splitChoice->add("Split Window");
  this->splitChoice->mode(0,FL_MENU_DIVIDER);
  this->splitChoice->add("Horizontal","", &gazebo::GLFrame::SplitCB, this);
  this->splitChoice->add("Vertical","",  &gazebo::GLFrame::SplitCB, this);
  this->splitChoice->value(0);

  this->headerBar->end();
  this->headerBar->resizable(NULL);

  this->glWindow = new GLWindow(x+1,y+30, w-2, h-60);

  this->footerBar = new Fl_Group(x,y+h-30,w,30);
  this->footerBar->box(FL_UP_BOX);

  int x1 = this->footerBar->x() + 35;
  int y1 = this->footerBar->y() + 5;
  this->outputXYZ = new Fl_Output(x1, y1, 150, 20,"XYZ");
  this->outputXYZ->box(FL_NO_BOX);
  this->outputXYZ->value("[0.0 0.0 0.0]");

  x1 += this->outputXYZ->w() + 30;
  this->outputRPY = new Fl_Output(x1, y1, 150, 20,"RPY");
  this->outputRPY->box(FL_NO_BOX);
  this->outputRPY->value("[0.0 0.0 0.0]");

  this->footerBar->end();
  this->footerBar->resizable(NULL);

  this->end();

  this->resizable(NULL);
  this->resizable(this->glWindow);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GLFrame::~GLFrame()
{
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the frame
void GLFrame::Init()
{
  this->glWindow->Init();

  CameraManager::Instance()->ConnectAddCameraSignal( boost::bind(&GLFrame::CameraAddedSlot, this, _1) );
}

////////////////////////////////////////////////////////////////////////////////
// Update the frame
void GLFrame::Update()
{  
  char buff[256];
  this->glWindow->Update();

  Pose3d pose = this->glWindow->GetCamera()->GetWorldPose();

  sprintf(buff, "[%6.2f, %6.2f, %6.2f]", pose.pos.x, pose.pos.y, pose.pos.z);
  this->outputXYZ->value(buff);

  sprintf( buff,"[%6.2f, %6.2f, %6.2f]", RTOD(pose.rot.GetRoll()),
     RTOD(pose.rot.GetPitch()), RTOD(pose.rot.GetYaw()) );
  this->outputRPY->value(buff);

  this->footerBar->redraw();
}

////////////////////////////////////////////////////////////////////////////////
/// Boost slot, called when a new camera is added.
void GLFrame::CameraAddedSlot(OgreCamera *newCamera)
{
  if (newCamera->GetCameraName().substr(0, 4) != "User")
  {
    this->viewChoice->add(newCamera->GetCameraName().c_str(), "", &gazebo::GLFrame::ViewCB, this);
    this->viewChoice->redraw();
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Split the current window
void GLFrame::SplitCB(Fl_Widget *widget, void *data)
{
  GLFrame *frame = reinterpret_cast<GLFrame *>(data);
  Fl_Choice *choice = dynamic_cast<Fl_Choice *>(widget);

  GLFrameManager *manager = dynamic_cast<GLFrameManager*>(frame->parent());

  switch (choice->value())
  {
    case 1:
      manager->Split(frame, "horz");
      break;
    case 2:
      manager->Split(frame, "vert");
      break;
  }

  choice->value(0);
}

////////////////////////////////////////////////////////////////////////////////
// Switch view callback
void GLFrame::ViewCB(Fl_Widget *widget, void *data)
{
  /*GLFrame *frame = reinterpret_cast<GLFrame *>(data);
  Fl_Choice *choice = dynamic_cast<Fl_Choice *>(widget);
  CameraManager *manager = CameraManager::Instance();
  OgreCamera *cam = manager->GetCamera(choice->text());

  frame->glWindow->SetActiveCamera( cam );
  */
}

////////////////////////////////////////////////////////////////////////////////
/// Get the pose of the camera attached to this frame
Pose3d GLFrame::GetCameraPose() const
{
  return this->glWindow->GetCamera()->GetWorldPose();  
}

////////////////////////////////////////////////////////////////////////////////
/// Set the pose of the camera attached to this frame
void GLFrame::SetCameraPose( const Pose3d &pose )
{
  this->glWindow->GetCamera()->SetWorldPose( pose );  
}

////////////////////////////////////////////////////////////////////////////////
/// Get a pointer to the render window
GLWindow *GLFrame::GetWindow() const
{
  return this->glWindow;
}
