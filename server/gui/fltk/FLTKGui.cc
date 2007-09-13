#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/glx.h>

#include "GuiFactory.hh"
#include "OgreAdaptor.hh"
#include "GazeboMessage.hh"
#include "FLTKGui.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_GUI("fltk", FLTKGui);

////////////////////////////////////////////////////////////////////////////////
// Constructor
FLTKGui::FLTKGui(int x, int y, int w, int h, const std::string &label) 
  : Gui(), Fl_Window( x, y, w, h, label.c_str() )
{
}


////////////////////////////////////////////////////////////////////////////////
// Destructor
FLTKGui::~FLTKGui()
{
}

////////////////////////////////////////////////////////////////////////////////
// Init
void FLTKGui::Init()
{
  this->OpenDisplay();

  Fl_Window::show();
}

////////////////////////////////////////////////////////////////////////////////
// Update
void FLTKGui::Update()
{
  /// \todo Fix the wait: not sure this is the most efficient method
  this->draw();

  Fl::wait(0.05);
  
  //printf("WH[%d %d]\n",this->w(), this->h());
}

////////////////////////////////////////////////////////////////////////////////
/// Get the width of the gui's rendering window
unsigned int FLTKGui::GetWidth() const
{
  return this->w();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the height of the gui's rendering window
unsigned int FLTKGui::GetHeight() const
{
  return this->h();
}


////////////////////////////////////////////////////////////////////////////////
// Open the display
void FLTKGui::OpenDisplay()
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
    gzmsg(0) << "Error choosing visual\n";

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

////////////////////////////////////////////////////////////////////////////////
// Render the scene
void FLTKGui::draw()
{
  OgreAdaptor::Instance()->Render();
}

////////////////////////////////////////////////////////////////////////////////
// Flush by drawing
void FLTKGui::flush()
{
  this->draw();
}

////////////////////////////////////////////////////////////////////////////////
// resize the window
void FLTKGui::resize(int x, int y, int w, int h)
{
  OgreAdaptor::Instance()->ResizeWindow(w, h);
}

////////////////////////////////////////////////////////////////////////////////
// Handle events
int FLTKGui::handle(int event)
{
  switch (event)
  {
    case FL_CLOSE:
      printf("CLOSE\n");
      return 0;

    case FL_PUSH:
      printf("PUSH\n");
      return this->HandlePush();

    case FL_DRAG:
      printf("Drag\n");
      return this->HandleDrag();

    case FL_RELEASE:
      printf("Release\n");
      return 0;

    case FL_KEYBOARD:
      return this->HandleKeyboard();

    default:
      return 1;
  }
}

int FLTKGui::HandlePush()
{
  return 1;
}

int FLTKGui::HandleDrag()
{
  printf("dragging\n");
  //OgreAdaptor::Instance()->ResizeWindow(this->w(), this->h());
  return 1;
}

int FLTKGui::HandleKeyboard()
{
  int key = Fl::event_key();
  return 1;
}
