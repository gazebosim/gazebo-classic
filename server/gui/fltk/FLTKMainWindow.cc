#include <string>

#include <FL/Fl_Menu_Item.H>
#include <FL/Fl_Menu_Bar.H>
#include <FL/Fl_Choice.H>

#include "FLTKGui.hh"
#include "Global.hh"
#include "GuiFactory.hh"
#include "FLTKMainWindow.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_GUI("fltk", FLTKMainWindow);


void quit_cb(Fl_Widget*, void*) {Global::userQuit = true;}

Fl_Menu_Item menuitems[] = {
  { "File", 0, 0, 0, FL_SUBMENU },
  { "Quit", 0, quit_cb, 0, FL_MENU_DIVIDER },
  { 0 },
  
  { 0 }
};


////////////////////////////////////////////////////////////////////////////////
/// Constructor
FLTKMainWindow::FLTKMainWindow (int x, int y, int w, int h, const std::string &t)
  : Gui(), Fl_Window(x, y, w, h, t.c_str())
{
  this->begin();

  /*Fl_Menu_Bar *m = new Fl_Menu_Bar(0, 0, w, 30, "Menu Bar");
  m->copy(menuitems);
  */

  this->glWindow = new FLTKGui(0, 0, w, h, "");

//  printf("GLWindow Size[%d %d]\n", this->glWindow->w(), this->glWindow->h());

  this->end();
  this->show();

  this->glWindow->Init();

  this->display = this->glWindow->display;
  this->visual = this->glWindow->visual;
  this->colormap = this->glWindow->colormap;
  this->windowId = this->glWindow->windowId;
  
//  this->resizable(this->glWindow);
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
FLTKMainWindow::~FLTKMainWindow()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Initalize the gui
void FLTKMainWindow::Init()
{
}

////////////////////////////////////////////////////////////////////////////////
/// Get the width of the gui's rendering window
unsigned int FLTKMainWindow::GetWidth() const
{
  return this->glWindow->w();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the height of the gui's rendering window
unsigned int FLTKMainWindow::GetHeight() const
{
  return this->glWindow->h();
}
