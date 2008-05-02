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
#include <Ogre.h>
#include <FL/Fl_Value_Output.H>
#include <FL/Fl_Output.H>

#include "Simulator.hh"
#include "OgreAdaptor.hh"
#include "StatusBar.hh"

using namespace gazebo;

StatusBar::StatusBar(int x, int y, int w, int h, const char *l)
    : Fl_Group(x,y,w,h,l)
{
  x += 30;
  y += 5;
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

  x = this->pauseTime->x() + this->pauseTime->w() + 80;
  this->iterations = new Fl_Value_Output(x,y,65,20,"Iterations");
  this->iterations->precision(0);

  x = this->w() - 80;
  this->statusString = new Fl_Output(x,y,80,20,"");
  this->statusString->value("Running");
  this->statusString->color(FL_GREEN);

  this->end();
}

StatusBar::~StatusBar()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Update the toolbar data
void StatusBar::Update()
{
  float lastFPS, avgFPS, bestFPS, worstFPS;
  OgreAdaptor::Instance()->window->getStatistics(lastFPS, avgFPS, bestFPS, worstFPS);

  this->iterations->value(Simulator::Instance()->GetIterations());
  this->fps->value(avgFPS);

  if (Simulator::Instance()->GetRealTime() - this->realTime->value() > 0.1)
  {
    this->realTime->value(Simulator::Instance()->GetRealTime());
  }
  this->simTime->value(Simulator::Instance()->GetSimTime());
  this->pauseTime->value(Simulator::Instance()->GetPauseTime());

  if (Simulator::Instance()->GetUserPause())
  {
    this->statusString->value("PAUSED");
    this->statusString->color(FL_RED);
  }
  else if (Simulator::Instance()->GetUserStep())
  {
    this->statusString->value("STEP");
    this->statusString->color(FL_RED);
  }
  else
  {
    this->statusString->value("RUNNING");
    this->statusString->color(FL_GREEN);
  }

}

int StatusBar::handle( int event )
{
  return 0;
}
