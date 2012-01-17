/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <FL/Fl.H>
#include <FL/x.H>
#include <stdio.h>

#include "GLWindow.hh"

extern void CreateWindow(Display *display, int screen,
                         long winId, unsigned int width,
                         unsigned int height);
extern void ResizeWindow(int w, int h);

GLWindow::GLWindow(int x, int y, int w, int h, const char *t)
    : Fl_Gl_Window(x, y, w, h, t)
{
  this->end();
  this->resizable(this);
}

GLWindow::~GLWindow()
{}
void GLWindow::Init()
{
  this->show();

  this->make_current();
  CreateWindow(fl_display, fl_visual->screen,
               (long)(Fl_X::i(this)->xid), this->w(), this->h());
}

void GLWindow::Update()
{
  Fl::check();
}


void GLWindow::resize(int x, int y, int w, int h)
{
  printf("Resizing window to[%d %d]\n", w, h);
  this->make_current();
  Fl_Gl_Window::resize(x, y, w, h);
  ResizeWindow(w, h);

  this->redraw();
}


