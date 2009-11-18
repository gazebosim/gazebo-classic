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
 * SVN: $Id$
 */

#include <stdio.h>

#include <FL/Fl_Button.H>
#include <FL/Fl_Box.H>
#include <FL/Fl_Bitmap.H>
#include <FL/Fl_RGB_Image.H>

#include "Image.hh"
#include "Gui.hh"
#include "Global.hh"
#include "Simulator.hh"
#include "Toolbar.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Toolbar::Toolbar(int x, int y, int w, int h, const char *l)
    : Fl_Group(x,y,w,h,l)
{
  this->box(FL_THIN_DOWN_BOX);

  this->color(BG_COLOR);

  unsigned char *data = NULL;
  unsigned int dataCount;

  Image image;
  image.Load("blue_play_button.png");
  image.Rescale(20,20);
  image.GetData(&data, dataCount);

  this->playImage[0] = new Fl_RGB_Image(data, image.GetWidth(), 
                                        image.GetHeight(), 4);

  data = NULL;
  image.Load("grey_play_button.png");
  image.Rescale(20,20);
  image.GetData(&data, dataCount);
  this->playImage[1] = new Fl_RGB_Image(data, image.GetWidth(), 
                                        image.GetHeight(), 4);

  data = NULL;
  image.Load("blue_pause_button.png");
  image.Rescale(20,20);
  image.GetData(&data, dataCount);
  this->pauseImage[0] = new Fl_RGB_Image(data, image.GetWidth(), 
                                         image.GetHeight(), 4);

  data = NULL;
  image.Load("grey_pause_button.png");
  image.Rescale(20,20);
  image.GetData(&data, dataCount);
  this->pauseImage[1] = new Fl_RGB_Image(data, image.GetWidth(), 
                                         image.GetHeight(), 4);

  data = NULL;
  image.Load("blue_step_button.png");
  image.Rescale(20,20);
  image.GetData(&data, dataCount);
  this->stepImage[0] = new Fl_RGB_Image(data, image.GetWidth(), 
                                        image.GetHeight(), 4);

  data = NULL;
  image.Load("grey_step_button.png");
  image.Rescale(20,20);
  image.GetData(&data, dataCount);
  this->stepImage[1] = new Fl_RGB_Image(data, image.GetWidth(), 
                                        image.GetHeight(), 4);


  x += 5;
  y += 5;
  this->playButton = new Fl_Button(x, y, 20, 20);
  this->playButton->callback( &gazebo::Toolbar::PlayButtonCB, this );
  this->playButton->color(BG_COLOR, BG_COLOR);
  this->playButton->image(this->playImage[1]);
  this->playButton->box(FL_NO_BOX);
  this->playButton->deactivate();

  x = this->playButton->x() + this->playButton->w() + 10;
  this->pauseButton = new Fl_Button(x, y, 20, 20);
  this->pauseButton->callback( &gazebo::Toolbar::PauseButtonCB, this );
  this->pauseButton->color(BG_COLOR, BG_COLOR);
  this->pauseButton->image(this->pauseImage[0]);
  this->pauseButton->box(FL_NO_BOX);

  x = this->pauseButton->x() + this->pauseButton->w() + 10;
  this->stepButton = new Fl_Button(x, y, 20, 20);
  this->stepButton->callback( &gazebo::Toolbar::StepButtonCB, this );
  this->stepButton->color(BG_COLOR, BG_COLOR);
  this->stepButton->image(this->stepImage[1]);
  this->stepButton->box(FL_NO_BOX);
  this->stepButton->deactivate();

  //x = this->stepButton->x() + this->stepButton->w() + 10;
  //this->moveButton = new Fl_Button(x,y,20,20);

  this->end();
  this->resizable(NULL);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Toolbar::~Toolbar()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Update the toolbar data
void Toolbar::Update()
{
  if (Simulator::Instance()->IsPaused())
  {
    //this->playButton->label("@>");
    //this->stepButton->activate();
  }
  else
  {
    //this->playButton->label("@||");
    //this->stepButton->deactivate();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Play pause button callback
void Toolbar::PlayButtonCB( Fl_Widget *w, void *data )
{
  Toolbar *tb = (Toolbar*)(data);
/*
  if (strcmp(w->label(), "@||") == 0)
  {
    Simulator::Instance()->SetPaused(true);

    tb->stepButton->activate();
    w->label("@>");
  }
  else
  {
    w->label("@||");
  }*/

  if (Simulator::Instance()->IsPaused())
  {
    Simulator::Instance()->SetPaused(false);

    tb->stepButton->deactivate();
    tb->stepButton->image(tb->stepImage[1]);

    tb->pauseButton->activate();
    tb->pauseButton->image(tb->pauseImage[0]);

    tb->playButton->image( tb->playImage[1] );
    tb->playButton->deactivate();
  }

  w->clear_visible_focus();
}

////////////////////////////////////////////////////////////////////////////////
// Play pause button callback
void Toolbar::PauseButtonCB( Fl_Widget *w, void *data )
{
  Toolbar *tb = (Toolbar*)(data);

  if (!Simulator::Instance()->IsPaused())
  {
    Simulator::Instance()->SetPaused(true);

    tb->stepButton->image( tb->stepImage[0] );
    tb->stepButton->activate();

    tb->playButton->image( tb->playImage[0] );
    tb->playButton->activate();

    tb->pauseButton->image( tb->pauseImage[1] );
    tb->pauseButton->deactivate();
  }

  w->clear_visible_focus();
}

////////////////////////////////////////////////////////////////////////////////
/// Set button callback
void Toolbar::StepButtonCB( Fl_Widget *w, void * /*data*/ )
{
  Simulator::Instance()->SetStepInc( true );
  w->clear_visible_focus();
}
