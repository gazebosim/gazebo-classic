#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/glx.h>

#include "OgreAdaptor.hh"
#include "FLTKWindow.hh"

using namespace gazebo;

FLTKWindow::FLTKWindow(int x, int y, int w, int h, const char* label) 
  : Fl_Window( x, y, w, h, label )
{
}


FLTKWindow::~FLTKWindow()
{
}


void FLTKWindow::show()
{
  this->OpenDisplay();
  Fl_Window::show();
}

void FLTKWindow::OpenDisplay()
{
  XSetWindowAttributes swa;
  int attrCount;
  int attrList[32];
  const char *optDisplay = NULL;

  optDisplay =  getenv("DISPLAY");
  this->display = XOpenDisplay(optDisplay);

  attrCount = 0;
  attrList[attrCount++] = GLX_RGBA;
  attrList[attrCount++] = GLX_DEPTH_SIZE;
  attrList[attrCount++] = 1;
  attrList[attrCount++] = GLX_STENCIL_SIZE;
  attrList[attrCount++] = 1;
  attrList[attrCount++] = GLX_DOUBLEBUFFER;
  attrList[attrCount] = 0;

  this->visual = glXChooseVisual(this->display, DefaultScreen(this->display), attrList);
  if (!this->visual)
    printf("Error choosing visual\n");

  this->colormap = XCreateColormap(this->display, RootWindow(this->display, this->visual->screen),
      this->visual->visual, AllocNone);

  /* create a window */
  swa.colormap = this->colormap;
  swa.border_pixel = 0;
  swa.event_mask = StructureNotifyMask;

  this->windowId = XCreateWindow(this->display, 
      RootWindow(this->display, this->visual->screen),
      0, 0, this->w(), this->h(),
      0, this->visual->depth, InputOutput, this->visual->visual,
      CWBorderPixel|CWColormap|CWEventMask, &swa);

  fl_open_display(this->display);
  Fl_X::set_xid((Fl_Window*)(this), this->windowId);
}

void FLTKWindow::draw()
{
  OgreAdaptor::Instance()->Render();
}

void FLTKWindow::flush()
{
  this->draw();
}

/*int FLTKWindow::handle(int event)
{
  switch (event)
  {
    case FL_PUSH:
      return this->HandlePush();

    case FL_DRAG:
      return this->HandleDrag();

    case FL_KEYBOARD:
      return this->HandleKeyboard();

    default:
      return 1;
  }
}

int FLTKWindow::HandlePush()
{
  return 1;
}

int FLTKWindow::HandleDrag()
{
  return 1;
}

int FLTKWindow::HandleKeyboard()
{
  int key = Fl::event_key();
  return 1;
}
*/
