#include <FL/Fl.H>
#include <FL/x.H>
#include <stdio.h>

#include "GLWindow.hh"

extern void CreateWindow(Display *display, int screen, 
                         long winId, unsigned int width, 
                         unsigned int height);
extern void ResizeWindow(int w, int h);

GLWindow::GLWindow( int x, int y, int w, int h, const char *t)
    : Fl_Gl_Window( x, y, w, h, t )
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
  printf("Resizing window to[%d %d]\n",w,h);
  this->make_current();
  Fl_Gl_Window::resize(x,y,w,h);
  ResizeWindow(w,h);

  this->redraw();
}
