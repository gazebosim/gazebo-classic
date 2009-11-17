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
/* Desc: StatusBar
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id$
 */

#include <stdio.h>
#include <string.h>
#include <FL/Fl_Value_Output.H>
#include <FL/Fl_Output.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Box.H>
#include <string.h>

#include "Gui.hh"
#include "Simulator.hh"
#include "OgreAdaptor.hh"
#include "StatusBar.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
StatusBar::StatusBar(int x, int y, int w, int h, const char *l)
    : Fl_Group(x,y,w,h,l)
{
  x += 5;
  y += 5;

  this->box(FL_NO_BOX);
  this->color(BG_COLOR);

  this->percentOutput = new Fl_Value_Output(x,y,40,20,"x Real Time");
  this->percentOutput->labelsize(11);
  this->percentOutput->align(FL_ALIGN_RIGHT);
  this->percentOutput->textsize(11);
  this->percentOutput->precision(2);
  this->percentOutput->box(FL_BORDER_BOX);
  this->percentOutput->color(FL_WHITE);

  x = this->percentOutput->x() + this->percentOutput->w() + 85;
  this->realTime = new Fl_Value_Output(x,y,45,20,"(sec) Real Time");
  this->realTime->labelsize(11);
  this->realTime->textsize(11);
  this->realTime->align(FL_ALIGN_RIGHT);
  this->realTime->precision(2);
  this->realTime->box(FL_BORDER_BOX);
  this->realTime->color(FL_WHITE);

  x = this->realTime->x() + this->realTime->w() + 105;
  this->simTime = new Fl_Value_Output(x,y,45,20,"(sec) Sim Time");
  this->simTime->labelsize(11);
  this->simTime->textsize(11);
  this->simTime->align(FL_ALIGN_RIGHT);
  this->simTime->precision(2);
  this->simTime->box(FL_BORDER_BOX);
  this->simTime->color(FL_WHITE);

  x = this->simTime->x() + this->simTime->w() + 100;
  this->pauseTime = new Fl_Value_Output(x,y,45,20,"Pause Time");
  this->pauseTime->labelsize(11);
  this->pauseTime->textsize(11);
  this->pauseTime->align(FL_ALIGN_RIGHT);
  this->pauseTime->precision(2);
  this->pauseTime->box(FL_BORDER_BOX);
  this->pauseTime->color(FL_WHITE);

  this->resizable(NULL);

  this->end();
  this->show();

  this->lastUpdateTime = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
StatusBar::~StatusBar()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Update the toolbar data
void StatusBar::Update()
{
  //float percent = 0;
  float sim = 0;
  float real = 0;

  if (Simulator::Instance()->GetRealTime() - this->lastUpdateTime > this->statusUpdatePeriod)
  {
    if (Simulator::Instance()->GetRealTime() < this->statusUpdatePeriod )
    {
      this->percent = (Simulator::Instance()->GetSimTime() / Simulator::Instance()->GetRealTime());
      this->percentLastRealTime =0;
      this->percentLastSimTime = 0;
    }
    else
    {
      this->percent = ((Simulator::Instance()->GetSimTime()-this->percentLastSimTime)
               / (Simulator::Instance()->GetRealTime()-this->percentLastRealTime)  );
      this->percentLastRealTime =Simulator::Instance()->GetRealTime();
      this->percentLastSimTime = Simulator::Instance()->GetSimTime();
    }

    sim = Simulator::Instance()->GetSimTime();
    if (sim > 31536000)
    {
      sim /= (31536000);
      this->simTime->label("(dys) Sim Time");
    }
    else if (sim > 86400)
    {
      sim /= (86400);
      this->simTime->label("(dys) Sim Time");
    }
    else if (sim > 3600)
    {
      sim /= 3600;
      this->simTime->label("(hrs) Sim Time");
    }
    else if (sim > 999)
    {
      sim /= 60;
      this->simTime->label("(min) Sim Time");
    }

    real = Simulator::Instance()->GetRealTime();
    if (sim > 31536000)
    {
      real /= (31536000);
      this->realTime->label("(dys) Real Time");
    }
    else if (sim > 86400)
    {
      real /= (86400);
      this->realTime->label("(dys) Real Time");
    }
    else if (real > 3600)
    {
      real /= 3600;
      this->realTime->label("(hrs) Real Time");
    }
    else if (real > 999)
    {
      real /= 60;
      this->realTime->label("(min) Real Time");
    }

    this->percentOutput->value(this->percent);

    this->realTime->value(real);
    this->simTime->value(sim);
    this->pauseTime->value(Simulator::Instance()->GetPauseTime());

    this->lastUpdateTime = Simulator::Instance()->GetRealTime();
  }
}
