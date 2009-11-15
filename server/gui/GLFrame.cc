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
 * SVN: $Id$
 */

#include <boost/bind.hpp>

#include "XMLConfig.hh"
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
  this->outputXYZ->labelsize(10);
  this->outputXYZ->textsize(10);
  this->outputXYZ->value("[0.0 0.0 0.0]");

  x1 += this->outputXYZ->w() + 20;
  this->outputRPY = new Fl_Output(x1, y1, 150, 20,"RPY");
  this->outputRPY->box(FL_NO_BOX);
  this->outputRPY->labelsize(10);
  this->outputRPY->textsize(10);
  this->outputRPY->value("[0.0 0.0 0.0]");

  x1 += this->outputRPY->w();

  Fl_Box *fillerBox = new Fl_Box(x1,y1,this->w() - (x1-x + 175) ,20);

  x1 += fillerBox->w();

  Fl_Group *statsGroup = new Fl_Group(x1, y1,100,20);
  this->fps = new Fl_Value_Output(x1,y1,25,20,"FPS");
  this->fps->labelsize(10);
  this->fps->textsize(10);
  this->fps->align(FL_ALIGN_RIGHT);
  this->fps->precision(0);

  x1 += this->fps->w() + 30;
  this->triangleCount = new Fl_Value_Output(x1,y1,50,20,"Triangles");
  this->triangleCount->labelsize(10);
  this->triangleCount->textsize(10);
  this->triangleCount->align(FL_ALIGN_RIGHT);
  this->triangleCount->precision(0);
  statsGroup->resizable(NULL);
  statsGroup->end();

  this->footerBar->end();
  this->footerBar->resizable(NULL);
  this->footerBar->resizable(fillerBox);

  this->end();

  this->resizable(NULL);
  this->resizable(this->glWindow);

  // Set default starting pose of the camera
  this->startPose.pos.Set(-2, 0, 2);
  this->startPose.rot.SetFromEuler( Vector3(0, DTOR(30), 0) );

  this->saveFrames = false;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GLFrame::~GLFrame()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the frame
void GLFrame::Load( XMLConfigNode *node )
{

  this->saveFrames = false;

  if (node)
  {
    this->startPose.pos = node->GetVector3("xyz", Vector3(0,0,0));
    this->startPose.rot = node->GetRotation("rpy", Quatern());
    this->saveFrames = node->GetBool("saveFrames",false,0);
    this->savePathname = node->GetString("saveFramePath","",0);
  }

}

////////////////////////////////////////////////////////////////////////////////
/// Create user cameras
void GLFrame::CreateCameras()
{
  this->glWindow->CreateCameras();
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the frame
void GLFrame::Init()
{
  this->glWindow->Init();
  this->glWindow->GetCamera()->SetWorldPose(this->startPose);
  this->glWindow->GetCamera()->EnableSaveFrame(this->saveFrames);
  this->glWindow->GetCamera()->SetSaveFramePathname(this->savePathname);

  CameraManager::Instance()->ConnectAddCameraSignal( boost::bind(&GLFrame::CameraAddedSlot, this, _1) );

  // Add all the current cameras
  for (unsigned int i=0; i < CameraManager::Instance()->GetNumCameras(); i++)
  {
    this->CameraAddedSlot( CameraManager::Instance()->GetCamera(i) );
  }

  this->show();
  this->redraw();
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

  this->fps->value(this->glWindow->GetAvgFPS());
  this->triangleCount->value(this->glWindow->GetTriangleCount());

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
  printf("VIEW CB\n");
  GLFrame *frame = reinterpret_cast<GLFrame *>(data);
  Fl_Choice *choice = dynamic_cast<Fl_Choice *>(widget);

  frame->glWindow->SetViewStyle(choice->text());
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
