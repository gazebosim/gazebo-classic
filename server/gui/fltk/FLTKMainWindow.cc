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

Fl_Menu_Item menuitems[] = {
  { "&File", FL_ALT+'f', 0, 0, FL_SUBMENU },
  { "&Open", FL_ALT+'o', 0, 0, FL_MENU_INACTIVE },
  { 0 },
  
  { 0 }
};

////////////////////////////////////////////////////////////////////////////////
/// Constructor
FLTKMainWindow::FLTKMainWindow (int x, int y, int w, int h, const std::string &t)
  : Gui(), Fl_Window(x, y, w, h+30, t.c_str())
{

  this->begin();

  Fl_Menu_Bar *m = new Fl_Menu_Bar(0, 0, w, 30, "Menu Bar");
  m->copy(menuitems);

  this->glWindow = new FLTKGui(0, 30, w, h, "");

  printf("GLWindow Size[%d %d]\n", this->glWindow->w(), this->glWindow->h());

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
//  Fl_Window::draw();

  /*Fl_Window::show();
  this->glWindow->Init();
  */
}

////////////////////////////////////////////////////////////////////////////////
/// Update the gui
void FLTKMainWindow::Update()
{
  this->glWindow->Update();
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

////////////////////////////////////////////////////////////////////////////////
/*void FLTKMainWindow::draw()
{
  printf("draw\n");
  Fl_Window::draw();
  this->glWindow->draw();
}*/

////////////////////////////////////////////////////////////////////////////////
/*void FLTKMainWindow::flush()
{
 // this->glWindow->draw();
}*/

////////////////////////////////////////////////////////////////////////////////
/*void FLTKMainWindow::resize(int x, int y, int w, int h)
{
  printf("resize\n");
//  this->glWindow->resize(x, y, w, h);
}*/

////////////////////////////////////////////////////////////////////////////////
/*int FLTKMainWindow::handle(int event)
{
  printf("Handle\n");
 // this->glWindow->handle(event);
}*/
