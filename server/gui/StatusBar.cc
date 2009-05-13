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
#include <string.h>

#include "Gui.hh"
#include "Simulator.hh"
#include "OgreAdaptor.hh"
#include "StatusBar.hh"

using namespace gazebo;

StatusBar::StatusBar(int x, int y, int w, int h, const char *l)
    : Fl_Group(x,y,w,h,l)
{
  x += 5;
  y += 5;

  this->box(FL_UP_BOX);
  this->fps = new Fl_Value_Output(x,y,25,20,"FPS");
  this->fps->labelsize(11);
  this->fps->textsize(11);
  this->fps->align(FL_ALIGN_RIGHT);
  this->fps->precision(0);

  x = this->fps->x() + this->fps->w() + 35;
  this->percentOutput = new Fl_Value_Output(x,y,40,20,"x Real Time");
  this->percentOutput->labelsize(11);
  this->percentOutput->align(FL_ALIGN_RIGHT);
  this->percentOutput->textsize(11);
  this->percentOutput->precision(2);

  x = this->percentOutput->x() + this->percentOutput->w() + 85;
  this->realTime = new Fl_Value_Output(x,y,45,20,"(sec) Real Time");
  this->realTime->labelsize(11);
  this->realTime->textsize(11);
  this->realTime->align(FL_ALIGN_RIGHT);
  this->realTime->precision(2);

  x = this->realTime->x() + this->realTime->w() + 105;
  this->simTime = new Fl_Value_Output(x,y,45,20,"(sec) Sim Time");
  this->simTime->labelsize(11);
  this->simTime->textsize(11);
  this->simTime->align(FL_ALIGN_RIGHT);
  this->simTime->precision(2);

  x = this->simTime->x() + this->simTime->w() + 100;
  this->pauseTime = new Fl_Value_Output(x,y,45,20,"Pause Time");
  this->pauseTime->labelsize(11);
  this->pauseTime->textsize(11);
  this->pauseTime->align(FL_ALIGN_RIGHT);
  this->pauseTime->precision(2);

  x = this->w() - 80;
  //x = this->pauseTime->x() + this->pauseTime->w() + 80;
  this->playButton = new Fl_Button(x, y, 30, 20, "@||");
  this->playButton->callback( &gazebo::StatusBar::PlayPauseButtonCB, this );

  x = this->playButton->x() + this->playButton->w() + 15;
  this->stepButton = new Fl_Button(x, y, 30, 20, "@>|");
  this->stepButton->callback( &gazebo::StatusBar::StepButtonCB, this );
  this->stepButton->deactivate();

  this->resizable(NULL);
  this->end();
  this->show();

  this->lastUpdateTime = 0;
}

StatusBar::~StatusBar()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Update the toolbar data
void StatusBar::Update()
{
  float avgFPS = 0;
  float percent = 0;
  float sim = 0;
  float real = 0;

  if (Simulator::Instance()->GetRealTime() - this->lastUpdateTime > 0.05)
  {
    avgFPS = this->gui->GetAvgFPS();
    percent = (Simulator::Instance()->GetSimTime() / Simulator::Instance()->GetRealTime());

    sim = Simulator::Instance()->GetSimTime();
    if (sim > 99999)
    {
      sim /= (120*24);
      this->simTime->label("(dys) Sim Time");
    }
    else if (sim > 9999)
    {
      sim /= 120;
      this->simTime->label("(hrs) Sim Time");
    }
    else if (sim > 999)
    {
      sim /= 60;
      this->simTime->label("(min) Sim Time");
    }

    real = Simulator::Instance()->GetRealTime();
    if (sim > 99999)
    {
      real /= (120*24);
      this->realTime->label("(dys) Real Time");
    }
    else if (real > 9999)
    {
      real /= 120;
      this->realTime->label("(hrs) Real Time");
    }
    else if (real > 999)
    {
      real /= 60;
      this->realTime->label("(min) Real Time");
    }

    //this->iterations->value(Simulator::Instance()->GetIterations());
    this->fps->value(avgFPS);
    this->percentOutput->value(percent);

    //if (Simulator::Instance()->GetRealTime() - this->realTime->value() > 0.1)

    this->realTime->value(real);
    this->simTime->value(sim);
    this->pauseTime->value(Simulator::Instance()->GetPauseTime());

    this->lastUpdateTime = Simulator::Instance()->GetRealTime();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Play pause button callback
void StatusBar::PlayPauseButtonCB( Fl_Widget *w, void *data )
{
  StatusBar *sb = (StatusBar*)(data);

  if (strcmp(w->label(), "@||") == 0)
  {
    Simulator::Instance()->SetUserPause(true);

    sb->stepButton->activate();
    w->label("@>");
  }
  else
  {
    Simulator::Instance()->SetUserPause(false);
    sb->stepButton->deactivate();
    w->label("@||");
  }

  w->clear_visible_focus();
}

////////////////////////////////////////////////////////////////////////////////
/// Set button callback
void StatusBar::StepButtonCB( Fl_Widget * /*w*/, void * /*data*/ )
{
  Simulator::Instance()->SetUserStepInc( true );
}
