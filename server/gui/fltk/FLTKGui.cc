#include "CameraSensor.hh"
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <FL/Fl_Menu_Item.H>
#include <FL/Fl_Menu_Bar.H>

#include <GL/glx.h>

#include "World.hh"
#include "FLTKMainWindow.hh"
#include "Global.hh"
#include "GuiFactory.hh"
#include "GazeboMessage.hh"
#include "GuiFactory.hh"
#include "MainMenu.hh"
#include "CameraManager.hh"

#include "OgreHUD.hh"

#include "FLTKGui.hh"

using namespace gazebo;

//GZ_REGISTER_STATIC_GUI("fltk", FLTKGui);

////////////////////////////////////////////////////////////////////////////////
// Constructor
FLTKGui::FLTKGui(int x, int y, int w, int h, const std::string &label) 
  : Fl_Gl_Window( x, y, w, h, label.c_str() )
{

  this->end();

  this->moveAmount = 1.0;
  this->moveScale = 1;
  this->rotateAmount = 0.5;

  this->directionVec.x = 0;
  this->directionVec.y = 0;
  this->directionVec.z = 0;

  this->keys.clear();

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
  CameraSensor *camera = CameraManager::Instance()->GetActiveCamera();
  std::map<int,int>::iterator iter;

  for (iter = this->keys.begin(); iter!= this->keys.end(); iter++)
  {
    if (iter->second == 1)
    {
      switch (iter->first)
      {
        case XK_Up:
        case XK_w:
          this->directionVec.z -= this->moveAmount;
          break;

        case XK_Down:
        case XK_s:
          this->directionVec.z += this->moveAmount;
          break;

        case XK_Left:
        case XK_a:
          this->directionVec.x -= this->moveAmount;
          break;

        case XK_Right:
        case XK_d:
          this->directionVec.x += this->moveAmount;
          break;

        case XK_Page_Down:
        case XK_e:
          this->directionVec.y -= this->moveAmount;
          break;

        case XK_Page_Up:
        case XK_q:
          this->directionVec.y += this->moveAmount;
          break;

        default:
          break;
      }
    }
  }

  camera->Translate( this->directionVec * (World::Instance()->GetRealTime() - this->lastUpdateTime) );
  this->directionVec.Set(0,0,0);

  this->lastUpdateTime = World::Instance()->GetRealTime();
}

void FLTKGui::flush()
{
  return;
}

////////////////////////////////////////////////////////////////////////////////
/// Handle a mouse button push
void FLTKGui::HandleMousePush()
{
  // Get the mouse button that was pressed (if one was pressed)
  switch (Fl::event_button())
  {
    case FL_LEFT_MOUSE:
      this->leftMousePressed = true;
      break;

    case FL_RIGHT_MOUSE:
      this->rightMousePressed = true;
      break;

    case FL_MIDDLE_MOUSE:
      this->middleMousePressed = true;
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Handle a mouse button release
void FLTKGui::HandleMouseRelease()
{
  // Get the mouse button that was pressed (if one was pressed)
  switch (Fl::event_button())
  {
    case FL_LEFT_MOUSE:
      this->leftMousePressed = false;
      break;

    case FL_RIGHT_MOUSE:
      this->rightMousePressed = false;
      break;

    case FL_MIDDLE_MOUSE:
      this->middleMousePressed = false;
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Handle a mouse drag
void FLTKGui::HandleMouseDrag()
{
  CameraSensor *camera = CameraManager::Instance()->GetActiveCamera();

  if (camera)
  {
    if (this->leftMousePressed)
    {
      Vector2<int> d = this->mousePos - this->prevMousePos;
      camera->RotateYaw(-d.x * this->rotateAmount);
      camera->RotatePitch(-d.y * this->rotateAmount);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Handle a key press
void FLTKGui::HandleKeyPress()
{
  this->keys[Fl::event_key()] = 1;
}

////////////////////////////////////////////////////////////////////////////////
/// Handle a key release
void FLTKGui::HandleKeyRelease()
{
  this->keys[Fl::event_key()] = 0;

  // Handle all toggle keys
  switch (Fl::event_key())
  {
    case 't':
      if (Global::GetUserPause())
        Global::SetUserPause(false);
      Global::SetUserStep( !Global::GetUserStep() );
      Global::SetUserStepInc( false );
      break;

    case 'h':
      //OgreHUD::Instance()->ToggleHelp();
      break;

    case ' ':
      if (Global::GetUserStep())
      {
        Global::SetUserStepInc( true );
      }
      else
        Global::SetUserPause( !Global::GetUserPause() );
      break;

    case FL_Escape:
      Global::SetUserQuit( true );
      break;

    case '[':
      CameraManager::Instance()->IncActiveCamera();
      break;

    case ']':
      CameraManager::Instance()->DecActiveCamera();
      break;

    case FL_Tab:
      //OgreHUD::Instance()->ToggleVisible();
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Handle events
int FLTKGui::handle(int event)
{
  bool handled = false;

  // Get the mouse position
  this->mousePos = Vector2<int>( Fl::event_x(), Fl::event_y() );

  // Set the type of the event
  switch (event)
  {
    case FL_ENTER:
    case FL_LEAVE:
    case FL_DEACTIVATE:
    case FL_HIDE:
      //this->inputHandler->ClearEvents();
      return 0;

    case FL_CLOSE:
      Global::SetUserQuit(true);
      return 0;

    case FL_PUSH:
      this->HandleMousePush();
      handled = true;
      break;

    case FL_RELEASE:
      this->HandleMouseRelease();
      handled = true;
      break;

    case FL_DRAG:
      this->HandleMouseDrag();
      handled = true;
      break;

    case FL_SHORTCUT:
    case FL_KEYDOWN:
      this->HandleKeyPress();
      handled = true;
      break;

    case FL_KEYUP:
      this->HandleKeyRelease();
      handled = true;
      break;
  }

  this->prevMousePos = this->mousePos;

  if (!handled)
    return Fl_Gl_Window::handle(event);
  else
    return 1;
}
