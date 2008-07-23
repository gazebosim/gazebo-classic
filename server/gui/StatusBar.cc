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
 * SVN: $Id:$
 */

#include <stdio.h>
#include <FL/Fl_Value_Output.H>
#include <FL/Fl_Output.H>
#include <FL/Fl_Button.H>

#include "Gui.hh"
#include "Simulator.hh"
#include "OgreAdaptor.hh"
#include "StatusBar.hh"

using namespace gazebo;

StatusBar::StatusBar(int x, int y, int w, int h, const char *l)
    : Fl_Group(x,y,w,h,l)
{
  x += 30;
  y += 5;

  this->box(FL_UP_BOX);
  this->fps = new Fl_Value_Output(x,y,40,20,"FPS");
  this->fps->precision(0);

  x = this->fps->x() + this->fps->w() + 80;
  this->realTime = new Fl_Value_Output(x,y,55,20,"Real Time");
  this->realTime->precision(2);

  x = this->realTime->x() + this->realTime->w() + 75;
  this->simTime = new Fl_Value_Output(x,y,55,20,"Sim Time");
  this->simTime->precision(2);

  x = this->simTime->x() + this->simTime->w() + 90;
  this->pauseTime = new Fl_Value_Output(x,y,55,20,"Pause Time");
  this->pauseTime->precision(2);

  x = this->pauseTime->x() + this->pauseTime->w() + 30;
  this->playButton = new Fl_Button(x, y, 30, 20, "@||");
  this->playButton->callback( &gazebo::StatusBar::PlayPauseButtonCB, this );

  x = this->playButton->x() + this->playButton->w() + 15;
  this->stepButton = new Fl_Button(x, y, 30, 20, "@>|");
  this->stepButton->callback( &gazebo::StatusBar::StepButtonCB, this );
  this->stepButton->deactivate();

  this->resizable(NULL);
  this->end();
  this->show();

}

StatusBar::~StatusBar()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Update the toolbar data
void StatusBar::Update()
{
  float avgFPS = 0;

  avgFPS = this->gui->GetAvgFPS();

  //this->iterations->value(Simulator::Instance()->GetIterations());
  this->fps->value(avgFPS);

  if (Simulator::Instance()->GetRealTime() - this->realTime->value() > 0.1)
  {
    this->realTime->value(Simulator::Instance()->GetRealTime());
  }

  this->simTime->value(Simulator::Instance()->GetSimTime());
  this->pauseTime->value(Simulator::Instance()->GetPauseTime());
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
