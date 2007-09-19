#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/glx.h>

#include "Global.hh"
#include "InputHandler.hh"
#include "GuiFactory.hh"
#include "OgreAdaptor.hh"
#include "GazeboMessage.hh"
#include "FLTKGui.hh"

using namespace gazebo;

extern void MainCallback();

GZ_REGISTER_STATIC_GUI("fltk", FLTKGui);

////////////////////////////////////////////////////////////////////////////////
// Constructor
FLTKGui::FLTKGui(int x, int y, int w, int h, const std::string &label) 
  : Gui(), Fl_Gl_Window( x, y, w, h, label.c_str() )
{
this->end();
this->show();

this->display = fl_display;
this->visual = fl_visual;
this->colormap = fl_colormap;
this->windowId = Fl_X::i(this)->xid;

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
  Fl_Window::show();
}

void UpdateCallback(void *data)
{
  FLTKGui *gui = (FLTKGui*)(data);

  gui->draw();
  MainCallback();

  Fl::repeat_timeout(0.03, UpdateCallback, gui);
}



////////////////////////////////////////////////////////////////////////////////
// Update
void FLTKGui::Run()
{
  Fl::add_timeout(0.03, UpdateCallback, this);
  Fl::run();
  Global::userQuit = true;
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
  InputEvent gzevent;

  // Get the mouse position
  gzevent.SetMousePos( Vector2<int>( Fl::event_x(), Fl::event_y() ) );

  // Get the key that was pressed (if one was pressed) 
  gzevent.SetKey(Fl::event_key());

  // Get the mouse button that was pressed (if one was pressed)
  switch (Fl::event_button())
  {
    case FL_LEFT_MOUSE:
      gzevent.SetMouseButton(InputEvent::LEFT_MOUSE);
      break;

    case FL_RIGHT_MOUSE:
      gzevent.SetMouseButton(InputEvent::RIGHT_MOUSE);
      break;

    case FL_MIDDLE_MOUSE:
      gzevent.SetMouseButton(InputEvent::MIDDLE_MOUSE);
      break;
    default:
      gzevent.SetMouseButton(InputEvent::NONE);
  }

  // Set the type of the event
  switch (event)
  {
    case FL_HIDE:
    case FL_DEACTIVATE:
    case FL_CLOSE:
      Global::userQuit = true;
      break;

    case FL_PUSH:
      gzevent.SetType(InputEvent::MOUSE_PRESS);
      break;

    case FL_RELEASE:
      gzevent.SetType(InputEvent::MOUSE_RELEASE);
      break;

    case FL_DRAG:
      gzevent.SetType(InputEvent::MOUSE_DRAG);
      break;

    case FL_SHORTCUT:
    case FL_KEYDOWN:
      gzevent.SetType(InputEvent::KEY_PRESS);
      break;

    case FL_KEYUP:
      gzevent.SetType(InputEvent::KEY_RELEASE);
      break;

    default:
      return 1;
  }

  InputHandler::Instance()->HandleEvent(&gzevent);

  return 0;
}

