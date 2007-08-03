#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/glx.h>

#include "GuiFactory.hh"
#include "OgreAdaptor.hh"
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
  this->draw();
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

/*int FLTKGui::handle(int event)
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

int FLTKGui::HandlePush()
{
  return 1;
}

int FLTKGui::HandleDrag()
{
  return 1;
}

int FLTKGui::HandleKeyboard()
{
  int key = Fl::event_key();
  return 1;
}
*/
