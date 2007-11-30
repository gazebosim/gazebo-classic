#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <FL/Fl_Menu_Item.H>
#include <FL/Fl_Menu_Bar.H>

#include <GL/glx.h>
#include <boost/thread/thread.hpp>

#include "FLTKMainWindow.hh"
#include "Global.hh"
#include "InputHandler.hh"
#include "GuiFactory.hh"
#include "OgreAdaptor.hh"
#include "GazeboMessage.hh"
#include "GuiFactory.hh"
#include "MainMenu.hh"
#include "FLTKGui.hh"

using namespace gazebo;

//GZ_REGISTER_STATIC_GUI("fltk", FLTKGui);

////////////////////////////////////////////////////////////////////////////////
// Constructor
FLTKGui::FLTKGui(int x, int y, int w, int h, const std::string &label) 
  : Fl_Gl_Window( x, y, w, h, label.c_str() )
{

  this->end();
  this->inputHandler = InputHandler::Instance();
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
  this->show();

  // Must have the next two lines right here!!!!
  this->make_current();
  this->valid(1);

  this->display = fl_display;
  this->visual = fl_visual;
  this->colormap = fl_colormap;
  this->windowId = Fl_X::i(this)->xid;

  Fl_Window::show();

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
// Update function
void FLTKGui::Update()
{
}

void FLTKGui::flush()
{
  return;
}

////////////////////////////////////////////////////////////////////////////////
/// Handle events
int FLTKGui::handle(int event)
{
  bool handled = false;
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
      handled = true;
      break;

    case FL_RIGHT_MOUSE:
      gzevent.SetMouseButton(InputEvent::RIGHT_MOUSE);
      handled = true;
      break;

    case FL_MIDDLE_MOUSE:
      gzevent.SetMouseButton(InputEvent::MIDDLE_MOUSE);
      handled = true;
      break;

    default:
      gzevent.SetMouseButton(InputEvent::NONE);
      break;
  }


  // Set the type of the event
  switch (event)
  {
    case FL_ENTER:
    case FL_LEAVE:
    case FL_DEACTIVATE:
    case FL_HIDE:
      this->inputHandler->ClearEvents();
      return 0;

    case FL_CLOSE:
      Global::SetUserQuit(true);
      return 0;

    case FL_PUSH:
      gzevent.SetType(InputEvent::MOUSE_PRESS);
      handled = true;
      break;

    case FL_RELEASE:
      gzevent.SetType(InputEvent::MOUSE_RELEASE);
      handled = true;
      break;

    case FL_DRAG:
      gzevent.SetType(InputEvent::MOUSE_DRAG);
      handled = true;
      break;

    case FL_SHORTCUT:
    case FL_KEYDOWN:
      gzevent.SetType(InputEvent::KEY_PRESS);
      handled = true;
      break;

    case FL_KEYUP:
      gzevent.SetType(InputEvent::KEY_RELEASE);
      handled = true;
      break;
  }

  this->inputHandler->HandleEvent(&gzevent);

  if (!handled)
    return Fl_Gl_Window::handle(event);
  else
    return 1;
}
