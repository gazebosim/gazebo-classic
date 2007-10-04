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
    { "&File", 0, 0, 0, 0 },
    { 0 },
    
    { 0 }
  };


////////////////////////////////////////////////////////////////////////////////
/// Constructor
FLTKMainWindow::FLTKMainWindow (int x, int y, int w, int h, const std::string &t)
  : Gui(), Fl_Window(x, y, w, h+30, t.c_str())
{

  this->begin();
//  Fl_Choice *c = new Fl_Choice(0,0,20,30,"Label");

  Fl_Menu_Bar *m = new Fl_Menu_Bar(0, 0, w, 30, "Menu Bar");
  m->copy(menuitems);

  this->glWindow = new FLTKGui(0, 40, w, h, "");

  printf("GLWindow Size[%d %d]\n", this->glWindow->w(), this->glWindow->h());

  this->end();
  this->show();

  this->glWindow->Init();

  this->display = this->glWindow->display;
  this->visual = this->glWindow->visual;
  this->colormap = this->glWindow->colormap;
  this->windowId = this->glWindow->windowId;
  
  this->resizable(this->glWindow);
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
  /*Fl_Window::show();
  this->glWindow->Init();
  */
}

////////////////////////////////////////////////////////////////////////////////
/// Update the gui
void FLTKMainWindow::Update()
{
  Fl::wait();
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
//  this->redraw();
  this->glWindow->draw();
}

////////////////////////////////////////////////////////////////////////////////
void FLTKMainWindow::flush()
{
 // this->glWindow->draw();
}*/

////////////////////////////////////////////////////////////////////////////////
void FLTKMainWindow::resize(int x, int y, int w, int h)
{
  this->glWindow->resize(x, y, w, h);
}

////////////////////////////////////////////////////////////////////////////////
int FLTKMainWindow::handle(int event)
{
  this->glWindow->handle(event);
}
