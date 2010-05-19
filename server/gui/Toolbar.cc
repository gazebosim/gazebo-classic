/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2203
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
 *  Foundation, Inc., 59 Temple Place, Suite 322, Boston, MA  02111-1227  USA
 *
 */
/* Desc: Toolbar
 * Author: Nate Koenig
 * Date: 13 Feb 2206
 * SVN: $Id$
 */

#include <stdio.h>

#include <FL/Fl_Box.H>
#include <FL/Fl_Bitmap.H>
#include <FL/Fl_Image.H>
#include <FL/fl_draw.H>

#include "Events.hh"
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
  this->box(FL_NO_BOX);

  this->color(BG_COLOR);

  unsigned char *data = NULL;
  unsigned int dataCount;

  Image image;
  image.Load("control_play_blue.png");
  image.Rescale(22,22);
  image.GetData(&data, dataCount);

  this->playImage[0] = new Fl_RGB_Image(data, image.GetWidth(), 
                                        image.GetHeight(), 4);

  data = NULL;
  image.Load("control_play.png");
  image.Rescale(22,22);
  image.GetData(&data, dataCount);
  this->playImage[1] = new Fl_RGB_Image(data, image.GetWidth(), 
                                        image.GetHeight(), 4);

  data = NULL;
  image.Load("control_pause_blue.png");
  image.Rescale(22,22);
  image.GetData(&data, dataCount);
  this->pauseImage[0] = new Fl_RGB_Image(data, image.GetWidth(), 
                                         image.GetHeight(), 4);

  data = NULL;
  image.Load("control_pause.png");
  image.Rescale(22,22);
  image.GetData(&data, dataCount);
  this->pauseImage[1] = new Fl_RGB_Image(data, image.GetWidth(), 
                                         image.GetHeight(), 4);

  data = NULL;
  image.Load("control_end_blue.png");
  image.Rescale(22,22);
  image.GetData(&data, dataCount);
  this->stepImage[0] = new Fl_RGB_Image(data, image.GetWidth(), 
                                        image.GetHeight(), 4);

  data = NULL;
  image.Load("control_end.png");
  image.Rescale(22,22);
  image.GetData(&data, dataCount);
  this->stepImage[1] = new Fl_RGB_Image(data, image.GetWidth(), 
                                        image.GetHeight(), 4);

  data = NULL;
  image.Load("box_create_blue.png");
  image.Rescale(22,22);
  image.GetData(&data, dataCount);
  this->boxImage[0] = new Fl_RGB_Image(data, image.GetWidth(), 
                                        image.GetHeight(), 4);

  data = NULL;
  image.Load("sphere_create_blue.png");
  image.Rescale(22,22);
  image.GetData(&data, dataCount);
  this->sphereImage[0] = new Fl_RGB_Image(data, image.GetWidth(), 
                                        image.GetHeight(), 4);

  data = NULL;
  image.Load("cylinder_create_blue.png");
  image.Rescale(22,22);
  image.GetData(&data, dataCount);
  this->cylinderImage[0] = new Fl_RGB_Image(data, image.GetWidth(), 
                                        image.GetHeight(), 4);


  data = NULL;
  image.Load("cursor.png");
  image.Rescale(22,22);
  image.GetData(&data, dataCount);
  this->cursorImage = new Fl_RGB_Image(data, image.GetWidth(), 
                                        image.GetHeight(), 4);

  data = NULL;
  image.Load("hand_cursor.png");
  image.Rescale(22,22);
  image.GetData(&data, dataCount);
  this->handCursorImage = new Fl_RGB_Image(data, image.GetWidth(), 
                                        image.GetHeight(), 4);

  data = NULL;
  image.Load("pointlight.png");
  image.Rescale(22,22);
  image.GetData(&data, dataCount);
  Fl_RGB_Image *pointlightImage = new Fl_RGB_Image(data, image.GetWidth(), 
                                                   image.GetHeight(), 4);
  data = NULL;
  image.Load("spotlight.png");
  image.Rescale(22,22);
  image.GetData(&data, dataCount);
  Fl_RGB_Image *spotlightImage = new Fl_RGB_Image(data, image.GetWidth(), 
                                                   image.GetHeight(), 4);

  data = NULL;
  image.Load("directionallight.png");
  image.Rescale(22,22);
  image.GetData(&data, dataCount);
  Fl_RGB_Image *directionallightImage = new Fl_RGB_Image(data, image.GetWidth(),
                                                         image.GetHeight(), 4);


  y += 4;
  x += 5;
  y += 0;
  this->playButton = new Fl_Button(x, y, 22, 22);
  this->playButton->callback( &gazebo::Toolbar::PlayButtonCB, this );
  this->playButton->color(BG_COLOR, BG_COLOR);
  this->playButton->image(this->playImage[1]);
  this->playButton->box(FL_NO_BOX);
  this->playButton->deactivate();
  this->playButton->clear_visible_focus();

  x = this->playButton->x() + this->playButton->w() + 10;
  this->pauseButton = new Fl_Button(x, y, 22, 22);
  this->pauseButton->callback( &gazebo::Toolbar::PauseButtonCB, this );
  this->pauseButton->color(BG_COLOR, BG_COLOR);
  this->pauseButton->image(this->pauseImage[0]);
  this->pauseButton->box(FL_NO_BOX);
  this->pauseButton->clear_visible_focus();

  x = this->pauseButton->x() + this->pauseButton->w() + 10;
  this->stepButton = new Fl_Button(x, y, 22, 22);
  this->stepButton->callback( &gazebo::Toolbar::StepButtonCB, this );
  this->stepButton->color(BG_COLOR, BG_COLOR);
  this->stepButton->image(this->stepImage[1]);
  this->stepButton->box(FL_NO_BOX);
  this->stepButton->deactivate();
  this->stepButton->clear_visible_focus();

  y = this->y();
  x = this->stepButton->x() + this->stepButton->w() + 10;
  Fl_Box *box = new Fl_Box(x,y+4,3,22);
  box->box(FL_DOWN_BOX);

  x += 13;
  this->cursorButton = new ToolbarButton(x, y, 30, 30);
  this->cursorButton->callback( &gazebo::Toolbar::CursorButtonCB, this );
  this->cursorButton->image(this->cursorImage);
  this->cursorButton->set();
  this->cursorButton->clear_visible_focus();


  y = this->y();
  x = this->cursorButton->x() + this->cursorButton->w() + 10;
  this->handCursorButton = new ToolbarButton(x, y, 30, 30);
  this->handCursorButton->callback(&gazebo::Toolbar::HandCursorButtonCB, this);
  this->handCursorButton->image(this->handCursorImage);
  this->handCursorButton->clear_visible_focus();

  x = this->handCursorButton->x() + this->handCursorButton->w() + 10;
  this->boxButton = new ToolbarButton(x, y, 30, 30);
  this->boxButton->callback( &gazebo::Toolbar::BoxButtonCB, this );
  this->boxButton->image(this->boxImage[0]);
  this->boxButton->clear_visible_focus();

  x = this->boxButton->x() + this->boxButton->w() + 10;
  this->sphereButton = new ToolbarButton(x, y, 30, 30);
  this->sphereButton->callback( &gazebo::Toolbar::SphereButtonCB, this );
  this->sphereButton->image(this->sphereImage[0]);
  this->sphereButton->clear_visible_focus();

  x = this->sphereButton->x() + this->sphereButton->w() + 10;
  this->cylinderButton = new ToolbarButton(x, y, 30, 30);
  this->cylinderButton->callback( &gazebo::Toolbar::CylinderButtonCB, this );
  this->cylinderButton->image(this->cylinderImage[0]);
  this->cylinderButton->clear_visible_focus();

  x = this->cylinderButton->x() + this->cylinderButton->w() + 10;
  ToolbarButton *pointLightButton = new ToolbarButton(x,y,30,30);
  pointLightButton->callback( &gazebo::Toolbar::PointLightCB, this );
  pointLightButton->image(pointlightImage);
  pointLightButton->clear_visible_focus();

  x = pointLightButton->x() + pointLightButton->w() + 10;
  ToolbarButton *spotLightButton = new ToolbarButton(x,y,30,30);
  spotLightButton->callback( &gazebo::Toolbar::SpotLightCB, this );
  spotLightButton->image(spotlightImage);
  spotLightButton->clear_visible_focus();

  x = spotLightButton->x() + spotLightButton->w() + 10;
  ToolbarButton *directionalLightButton = new ToolbarButton(x,y,30,30);
  directionalLightButton->callback( &gazebo::Toolbar::DirectionalLightCB, this);
  directionalLightButton->image(directionallightImage);
  directionalLightButton->clear_visible_focus();


  this->end();
  this->resizable(NULL);

  Events::ConnectMoveModeSignal( boost::bind(&Toolbar::MoveModeCB, this, _1) );
  Events::ConnectManipModeSignal( boost::bind(&Toolbar::ManipModeCB, this, _1) );
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
    this->stepButton->activate();
    this->stepButton->image(this->stepImage[0]);

    this->pauseButton->deactivate();
    this->pauseButton->image(this->pauseImage[1]);

    this->playButton->activate();
    this->playButton->image( this->playImage[0] );
  }
  else
  {
    this->stepButton->deactivate();
    this->stepButton->image(this->stepImage[1]);

    this->pauseButton->activate();
    this->pauseButton->image(this->pauseImage[0]);

    this->playButton->deactivate();
    this->playButton->image( this->playImage[1] );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Play pause button callback
void Toolbar::PlayButtonCB( Fl_Widget *w, void *data )
{
  Toolbar *tb = (Toolbar*)(data);

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

////////////////////////////////////////////////////////////////////////////////
/// Box button callback
void Toolbar::BoxButtonCB( Fl_Widget *w, void * /*data*/ )
{
  Events::createEntitySignal("box");
}

////////////////////////////////////////////////////////////////////////////////
/// Sphere button callback
void Toolbar::SphereButtonCB( Fl_Widget *w, void * /*data*/ )
{
  Events::createEntitySignal("sphere");
}

////////////////////////////////////////////////////////////////////////////////
/// Box button callback
void Toolbar::CylinderButtonCB( Fl_Widget *w, void * /*data*/ )
{
  Events::createEntitySignal("cylinder");
}

////////////////////////////////////////////////////////////////////////////////
/// Cursor button callback
void Toolbar::CursorButtonCB( Fl_Widget *w, void * /*data*/ )
{
  Events::createEntitySignal("");
  Events::moveModeSignal(true);
}

////////////////////////////////////////////////////////////////////////////////
/// Cursor button callback
void Toolbar::HandCursorButtonCB( Fl_Widget *w, void * /*data*/ )
{
  Events::createEntitySignal("");
  Events::manipModeSignal(true);
}

void Toolbar::MoveModeCB(bool mode)
{
  if (this->handCursorButton->value() == 1)
  {
    this->handCursorButton->value(0);
    this->cursorButton->set();
  }
}

////////////////////////////////////////////////////////////////////////////////
void Toolbar::ManipModeCB(bool mode)
{
}

////////////////////////////////////////////////////////////////////////////////
void Toolbar::PointLightCB(Fl_Widget *w, void * /*data*/)
{
  Events::createEntitySignal("pointlight");
}

////////////////////////////////////////////////////////////////////////////////
void Toolbar::SpotLightCB(Fl_Widget *w, void * /*data*/)
{
  Events::createEntitySignal("spotlight");
}

////////////////////////////////////////////////////////////////////////////////
void Toolbar::DirectionalLightCB(Fl_Widget *w, void * /*data*/)
{
  Events::createEntitySignal("directionallight");
}
