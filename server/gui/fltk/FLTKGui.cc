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
#include "FLTKGui.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_GUI("fltk", FLTKGui);

void quit_cb(Fl_Widget*, void*) {Global::SetUserQuit(true);}

Fl_Menu_Item menuitems[] = {
  { "File", 0, 0, 0, FL_SUBMENU },
  { "Quit", 0, quit_cb, 0, FL_MENU_DIVIDER },
  { 0 },
  
  { 0 }
};


////////////////////////////////////////////////////////////////////////////////
// Constructor
FLTKGui::FLTKGui(int x, int y, int w, int h, const std::string &label) 
  : Gui(), Fl_Gl_Window( x, y, w, h, label.c_str() )
{
//  this->begin();

  this->end();
  this->show();

  this->display = fl_display;
  this->visual = fl_visual;
  this->colormap = fl_colormap;
  this->windowId = Fl_X::i(this)->xid;

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
  Fl_Window::show();
  printf("GUI Init\n");
  /*this->show();
  this->display = fl_display;
  this->visual = fl_visual;
  this->colormap = fl_colormap;
  this->windowId = Fl_X::i(this)->xid;
  */
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
/// Handle events
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
      printf("CLOSE\n");
      Global::SetUserQuit(true);
      return 0;

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
      return 0;
  }

  //boost::mutex::scoped_lock lock(Global::mutex);

  this->inputHandler->HandleEvent(&gzevent);

  return 0;
}
