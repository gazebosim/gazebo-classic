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
/* Desc: Toolbar
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id:$
 */

#include <stdio.h>
#include <FL/Fl_Value_Output.H>

#include "Global.hh"
#include "CameraManager.hh"
#include "CameraSensor.hh"
#include "Toolbar.hh"

using namespace gazebo;

Toolbar::Toolbar(int x, int y, int w, int h, const char *l)
  : Fl_Group(x,y,w,h,l)
{
  char *buffer = new char[256];

  this->box(FL_UP_BOX);

  CameraSensor *camera = CameraManager::Instance()->GetActiveCamera();

  if (camera)
  {
    sprintf(buffer,"%s [%d x %d]", camera->GetName().c_str(), camera->GetImageWidth(), camera->GetImageHeight());

  }
  else
  {
    sprintf(buffer,"Camera");
  }

  this->cameraInfoGrp = new Fl_Group(x+10,y+20,w-20,25*3, buffer);

  // Camera Info Group
  this->cameraInfoGrp->box(FL_BORDER_BOX);

  // Camera X output
  x = this->cameraInfoGrp->x()+20;
  y = this->cameraInfoGrp->y()+2;
  this->outputX = new Fl_Value_Output(x,y,60,20,"X");
  this->outputX->precision(2);

  // Camera Y output
  x = this->outputX->x();
  y = this->outputX->y() + this->outputX->h()+5;
  this->outputY = new Fl_Value_Output(x,y,60,20,"Y");
  this->outputY->precision(2);

  // Camera Z output
  x = this->outputY->x();
  y = this->outputY->y() + this->outputX->h()+5;
  this->outputZ = new Fl_Value_Output(x,y,60,20,"Z");
  this->outputZ->precision(2);

  // Camera ROLL output
  x = this->outputX->x() + this->outputX->w()+20;
  y = this->outputX->y();
  this->outputRoll = new Fl_Value_Output(x,y,60,20,"R");
  this->outputRoll->precision(2);

  // Camera Pitch output
  x = this->outputRoll->x();
  y = this->outputRoll->y() + this->outputRoll->h() + 5;
  this->outputPitch = new Fl_Value_Output(x,y,60,20,"P");
  this->outputPitch->precision(2);

  // Camera Yaw output
  x = this->outputPitch->x();
  y = this->outputPitch->y() + this->outputPitch->h() + 5;
  this->outputYaw = new Fl_Value_Output(x,y,60,20,"Y");
  this->outputYaw->precision(2);

  this->cameraInfoGrp->end();

  this->end();
  
  delete buffer;
}

Toolbar::~Toolbar()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Update the toolbar data
void Toolbar::Update()
{
  char *buffer = new char[256];
  CameraSensor *camera = CameraManager::Instance()->GetActiveCamera();

  if (camera != NULL)
  {
    sprintf(buffer,"%s [%d x %d]", camera->GetName().c_str(), camera->GetImageWidth(), camera->GetImageHeight());

    if (strcmp(buffer,this->cameraInfoGrp->label()) != 0)
    {
      this->cameraInfoGrp->label(buffer);
    }

    Pose3d pose = camera->GetWorldPose();
    pose -= Global::poseOffset;

    this->outputX->value(pose.pos.x);
    this->outputY->value(pose.pos.y);
    this->outputZ->value(pose.pos.z);
    this->outputRoll->value(RTOD(pose.rot.GetRoll()));
    this->outputPitch->value(RTOD(pose.rot.GetPitch()));
    this->outputYaw->value(RTOD(pose.rot.GetYaw()));
  }
}

