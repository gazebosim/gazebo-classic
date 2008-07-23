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
#include <FL/Fl_Output.H>
#include <FL/Fl_Button.H>

#include "Entity.hh"
#include "Model.hh"
#include "Simulator.hh"
#include "CameraManager.hh"
#include "OgreCamera.hh"
#include "Toolbar.hh"
#include "Global.hh"

using namespace gazebo;

Toolbar::Toolbar(int x, int y, int w, int h, const char *l)
    : Fl_Group(x,y,w,h,l)
{
  char *buffer = new char[256];

  this->box(FL_UP_BOX);

  this->entityInfoGrp = new Fl_Group(x+10,y+20,w-20,25*3, "Entity");

  // Camera Info Group
  this->entityInfoGrp->box(FL_BORDER_BOX);

  // Entity name output
  x = this->entityInfoGrp->x()+50;
  y = this->entityInfoGrp->y()+2;
  this->entityName = new Fl_Output(x,y, this->entityInfoGrp->w()-55,20, "Name: ");

  y = this->entityName->y() + this->entityName->h() + 5;
  this->entityPos = new Fl_Output(x,y, this->entityInfoGrp->w()-55,20, "XYZ: ");

  y = this->entityPos->y() + this->entityPos->h() + 5;
  this->entityRot = new Fl_Output(x,y, this->entityInfoGrp->w()-55,20, "RPY: ");

  this->entityInfoGrp->end();

  this->end();

  this->resizable(NULL);

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
  Entity *entity = Simulator::Instance()->GetSelectedEntity();

  if (entity)
  {
    Model *model = dynamic_cast<Model *>(entity);

    sprintf(buffer,"%s", entity->GetName().c_str());
    if (strcmp(buffer, this->entityName->value()) != 0)
      this->entityName->value(buffer);

    if (model)
    {
      Pose3d pose = model->GetPose();
      Vector3 rpy = pose.rot.GetAsEuler();

      sprintf(buffer,"%4.2f,  %4.2f,  %4.2f",pose.pos.x, pose.pos.y, pose.pos.z);
      if (strcmp( buffer, this->entityPos->value()) != 0)
        this->entityPos->value(buffer);

      sprintf(buffer,"%4.2f,  %4.2f,  %4.2f",rpy.x, rpy.y, rpy.z);
      if (strcmp( buffer, this->entityRot->value()) != 0)
        this->entityRot->value(buffer);

    }
  }
  else
  {
    this->entityName->value("");
    this->entityPos->value("");
    this->entityRot->value("");
  }
}
