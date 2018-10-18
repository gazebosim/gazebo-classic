#include <iostream>
#include <sstream>

#ifdef __WXGTK__
#include <gdk/gdk.h>
#include <gtk/gtk.h>
#include <gdk/gdkx.h>
#include <wx/gtk/win_gtk.h>
#include <GL/glx.h>
#endif

#include "Body.hh"
#include "OgreAdaptor.hh"
#include "Scene.hh"
#include "OgreCreator.hh"
#include "GazeboError.hh"
#include "World.hh"
#include "Events.hh"
#include "UserCamera.hh"
#include "OrbitViewController.hh"
#include "RenderControl.hh"

using namespace gazebo;

IMPLEMENT_CLASS(RenderControl, wxControl)
BEGIN_EVENT_TABLE(RenderControl, wxControl)
EVT_SIZE (RenderControl::OnSize)
END_EVENT_TABLE()

////////////////////////////////////////////////////////////////////////////////
// Constructor
RenderControl::RenderControl(wxWindow *parent)
  : wxControl(parent, wxID_ANY, wxDefaultPosition, wxSize(320,240), wxSUNKEN_BORDER, wxDefaultValidator)
{
  this->userCamera = NULL;
  this->cursorState = "default";
  this->mouseModifier = "rotx";
  this->currMaker = NULL;

  SetFocus();

  Connect( wxEVT_LEFT_DOWN, wxMouseEventHandler( RenderControl::OnMouseEvent ), NULL, this );
  Connect( wxEVT_MIDDLE_DOWN, wxMouseEventHandler( RenderControl::OnMouseEvent ), NULL, this );
  Connect( wxEVT_RIGHT_DOWN, wxMouseEventHandler( RenderControl::OnMouseEvent ), NULL, this );
  Connect( wxEVT_MOTION, wxMouseEventHandler( RenderControl::OnMouseEvent ), NULL, this );
  Connect( wxEVT_LEFT_UP, wxMouseEventHandler( RenderControl::OnMouseEvent ), NULL, this );
  Connect( wxEVT_MIDDLE_UP, wxMouseEventHandler( RenderControl::OnMouseEvent ), NULL, this );
  Connect( wxEVT_RIGHT_UP, wxMouseEventHandler( RenderControl::OnMouseEvent ), NULL, this );
  Connect( wxEVT_MOUSEWHEEL, wxMouseEventHandler( RenderControl::OnMouseEvent ), NULL, this );
  Connect( wxEVT_LEFT_DCLICK, wxMouseEventHandler( RenderControl::OnMouseEvent ), NULL, this );

  Connect( wxEVT_KEY_UP, wxKeyEventHandler(RenderControl::OnKeyUp), NULL, this);
  Connect( wxEVT_KEY_DOWN, wxKeyEventHandler(RenderControl::OnKeyDown), NULL, this);

  Events::ConnectCreateEntitySignal( boost::bind(&RenderControl::CreateEntity, this, _1) );
  Events::ConnectMoveModeSignal( boost::bind(&RenderControl::MoveModeCB, this, _1) );
  Events::ConnectManipModeSignal( boost::bind(&RenderControl::ManipModeCB, this, _1) );
  Events::ConnectSetSelectedEntitySignal(
     boost::bind(&RenderControl::SetSelectedEntityCB, this, _1) );
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
RenderControl::~RenderControl()
{
  if (this->userCamera)
    delete this->userCamera;
}

////////////////////////////////////////////////////////////////////////////////
/// Get an ogre window handle
std::string RenderControl::GetOgreHandle() const
{
  std::string handle;

#ifdef __WXMSW__
  // Handle for Windows systems
  handle = Ogre::StringConverter::toString((size_t)((HWND)GetHandle()));
#elif defined(__WXGTK__)
  // Handle for GTK-based systems
  std::stringstream str;
  GtkWidget* widget = m_wxwindow;
  gtk_widget_set_double_buffered (widget, FALSE);
  if (!GTK_WIDGET_REALIZED(widget))
  {
    gtk_widget_realize( widget );
  }

  // Grab the window object
  GtkPizza* pizza = GTK_PIZZA(widget);
  GdkWindow* gdkWin = pizza->bin_window;
  Window wid = GDK_WINDOW_XWINDOW(gdkWin);

  XSync(GDK_WINDOW_XDISPLAY(widget->window), False);
  XSync(GDK_WINDOW_XDISPLAY(pizza->bin_window), False);

  str << wid;// << ':';
  handle = str.str();

#elif defined(__WXMAC__)
  handle = Ogre::StringConverter::toString((size_t)(GetHandle()));
#else
  // Any other unsupported system
#error("Not supported on this platform.")
#endif

  return handle;
}

////////////////////////////////////////////////////////////////////////////////
// On key up event
void RenderControl::OnKeyUp( wxKeyEvent &event)
{
  if(event.GetKeyCode() == WXK_CONTROL)
  {
    Events::moveModeSignal(true);
  }
}

////////////////////////////////////////////////////////////////////////////////
// On key down event
void RenderControl::OnKeyDown( wxKeyEvent &event)
{
  if(event.GetKeyCode() == WXK_CONTROL)
  {
    Events::manipModeSignal(true);
  }
}

////////////////////////////////////////////////////////////////////////////////
// On mouse event callback
void RenderControl::OnMouseEvent( wxMouseEvent &event)
{
  SetFocus();
  this->mouseEvent.pos.Set( event.GetX(), event.GetY() );

  if (event.LeftDown() || event.MiddleDown() || event.RightDown())
    this->mouseEvent.pressPos = this->mouseEvent.pos;

  if (event.LeftUp() || event.MiddleUp() || event.RightUp())
    this->userCamera->GetScene()->SetVisible("guiline", false);

  this->mouseEvent.left = event.LeftIsDown() ? MouseEvent::DOWN : MouseEvent::UP;
  this->mouseEvent.right = event.RightIsDown() ? MouseEvent::DOWN : MouseEvent::UP;
  this->mouseEvent.middle = event.MiddleIsDown() ? MouseEvent::DOWN : MouseEvent::UP;
 
  if (event.GetWheelRotation() < 0) 
  {
    this->mouseEvent.scroll.y = 1;
    this->mouseEvent.middle = MouseEvent::SCROLL;
  }
  else if (event.GetWheelRotation() > 0)
  {
    this->mouseEvent.scroll.y = -1;
    this->mouseEvent.middle = MouseEvent::SCROLL;
  }

  this->mouseEvent.dragging = event.Dragging();


  if (this->currMaker)
  {
    if (event.LeftDown())
      this->currMaker->MousePushCB(this->mouseEvent);
    else if (event.LeftUp())
    {
      this->currMaker->MouseReleaseCB(this->mouseEvent);
    }
    else if (event.Dragging())
      this->currMaker->MouseDragCB(this->mouseEvent);

    if (!this->currMaker || this->currMaker->IsActive())
      return;
  }
  else if (this->GetCursorState() == "manip")
  {
    Entity *entity = World::Instance()->GetSelectedEntity();

    if (entity)
    {
      if (event.LeftDown())
      {
        this->userCamera->GetScene()->GetEntityAt(this->userCamera, 
            this->mouseEvent.pos, 
            this->mouseModifier);
      }
      else if (event.Dragging())
      {
        if (this->mouseModifier.substr(0,3) == "rot")
          this->EntityRotate(entity);
        else if (this->mouseModifier.substr(0,5) == "trans")
          this->EntityTranslate(entity);
      }
    }
  }
  else
    this->userCamera->HandleMouseEvent(this->mouseEvent);

  this->mouseEvent.prevPos = this->mouseEvent.pos;
}

////////////////////////////////////////////////////////////////////////////////
// Reparent the window
bool RenderControl::Reparent(wxWindowBase* new_parent)
{
  bool ret = true;

  std::cout << "RenderControl::Reparent\n";
  // Dear god I hate GTK.  Because gtk_widget_reparent() does not work properly (https://netfiles.uiuc.edu/rvmorale/www/gtk-faq-es/x639.html),
  // When a window is reparented in wx the drawable client area has to be destroyed and re-created, causing its XID to change.  This causes both:
  // 1) Any window that was its child (like the Ogre render window's) is destroyed
  // 2) The XID changes
  // The following seems to be the only thing that actually works when trying to work around this:
  // 1) Reparent the render window's window to the root window
  // 2) Allow wx to reparent the window
  // 3) Reparent the render window's window to the new parent window
/*#if defined(__WXGTK__)
  Window win;
  Display* disp;

  if (this->renderWindow)
  {
    this->renderWindow->getCustomAttribute("WINDOW", &win);
    this->renderWindow->getCustomAttribute("XDISPLAY", &disp);

    wxWindow* top = wxTheApp->GetTopWindow();
    GtkPizza* pizza = GTK_PIZZA(top->m_wxwindow);
    Window parent = GDK_WINDOW_XWINDOW(pizza->bin_window);

    XSync(disp, False);
    XReparentWindow(disp, win, parent, 0, 0);
    XSync(disp, False);
  }
#endif

  bool ret = wxControl::Reparent(new_parent);

#if defined(__WXGTK__)
  if (this->renderWindow)
  {
    XSync(disp, False);

    gtk_widget_set_double_buffered(m_wxwindow, FALSE);
    if (!GTK_WIDGET_REALIZED(m_wxwindow))
    {
      gtk_widget_realize(m_wxwindow);
    }

    GtkPizza* pizza = GTK_PIZZA(m_wxwindow);

    XSync(GDK_WINDOW_XDISPLAY(m_wxwindow->window), False);
    XSync(GDK_WINDOW_XDISPLAY(pizza->bin_window), False);

    GdkWindow* gdkWin = pizza->bin_window;
    Window parent = GDK_WINDOW_XWINDOW(gdkWin);
    XReparentWindow(disp, win, parent, 0, 0);

//    XSync(disp, False);

    XMapWindow(disp, win);

    XSync(disp, False);
  }
#endif
*/

  return ret;
}

////////////////////////////////////////////////////////////////////////////////
wxSize RenderControl::DoGetBestSize () const
{
  return wxSize (320, 240);
}

////////////////////////////////////////////////////////////////////////////////
/// Create the camera
void RenderControl::CreateCamera(unsigned int scene)
{
  this->userCamera = new UserCamera( this, scene);
  this->userCamera->Load(NULL);
}

////////////////////////////////////////////////////////////////////////////////
// Get the user camera
UserCamera *RenderControl::GetCamera()
{
  return this->userCamera;
}

////////////////////////////////////////////////////////////////////////////////
void RenderControl::Init()
{
  if (this->userCamera == NULL)
    gzthrow("Invalid user camera\n");

  Pose3d pose;

  pose.pos.Set(-4, 0.0, 2);
  pose.rot.SetFromEuler( Vector3(DTOR(0.0),DTOR(15.0), DTOR(0.0)) ); 
  this->userCamera->Init();
  this->userCamera->SetWorldPose(pose);
}

////////////////////////////////////////////////////////////////////////////////
void RenderControl::CreateEntity(std::string name)
{
  if (this->currMaker)
    this->currMaker->Stop();

  this->SetCursorState("create");

  if (name.size() > 0)
    Events::setSelectedEntitySignal("");

  if (name == "box")
    this->currMaker = &this->boxMaker;
  else if (name == "cylinder")
    this->currMaker = &this->cylinderMaker;
  else if (name == "sphere")
    this->currMaker = &this->sphereMaker;
  else if (name == "pointlight")
    this->currMaker = &this->pointLightMaker;
  else if (name == "spotlight")
    this->currMaker = &this->spotLightMaker;
  else if (name == "directionallight")
    this->currMaker = &this->directionalLightMaker;
  else
  {
    this->currMaker = NULL;
    this->SetCursorState("default");
  }

  if (this->currMaker)
    this->currMaker->Start();
}

void RenderControl::OnSize( wxSizeEvent &evt )
{
  int width;
  int height;
  wxSize size = evt.GetSize ();
  width = size.GetWidth ();
  height = size.GetHeight ();

  if (this->userCamera)
    this->userCamera->Resize(width,height);

  evt.Skip();
}

////////////////////////////////////////////////////////////////////////////////
// Get the cursor state
std::string RenderControl::GetCursorState() const
{
  return this->cursorState;
}

////////////////////////////////////////////////////////////////////////////////
// Set the cursor state
void RenderControl::SetCursorState(const std::string &state)
{
  this->cursorState = state;

  /*if (state == "default")
    this->cursor(FL_CURSOR_DEFAULT);
  else if (state == "manip")
    this->cursor(FL_CURSOR_HAND);
  else if (state == "create")
    this->cursor(FL_CURSOR_CROSS);
    */
}

void RenderControl::MoveModeCB(bool mode)
{
  if (mode)
  {
    this->SetCursorState("default");
    this->currMaker = NULL;
  }
}

void RenderControl::ManipModeCB(bool mode)
{
  if (mode)
    this->SetCursorState("manip");
  else
    this->SetCursorState("move");
}

////////////////////////////////////////////////////////////////////////////////
// Rotate and entity, or apply torque
void RenderControl::EntityRotate(Entity *entity)
{
  Vector3 planeNorm, planeNorm2;
  Vector3 p1, p2;
  Vector3 a,b;
  Vector3 ray(0,0,0);

  Pose3d modelPose = entity->GetWorldPose();

  // Figure out which axis to rotate around
  if (this->mouseModifier == "rotx")
    ray.x = 1.0;
  else if (this->mouseModifier == "roty")
    ray.y = 1.0;
  else if (this->mouseModifier == "rotz")
    ray.z = 1.0;

  // Compute the normal to the plane on which to rotate
  planeNorm = modelPose.rot.RotateVector(ray);
  double d = -modelPose.pos.GetDotProd(planeNorm);

  p1 = this->GetWorldPointOnPlane( this->mouseEvent.pos.x, 
      this->mouseEvent.pos.y, planeNorm, d);

  p2 = this->GetWorldPointOnPlane( this->mouseEvent.prevPos.x, 
      this->mouseEvent.prevPos.y, planeNorm, d);

  this->userCamera->GetScene()->DrawLine(entity->GetWorldPose().pos,p1, "guiline");

  // Get point vectors relative to the entity's pose
  a = p1 - entity->GetWorldPose().pos;
  b = p2 - entity->GetWorldPose().pos;

  a.Normalize();
  b.Normalize();

  // Get the angle between the two vectors. This is the amount to
  // rotate the entity 
  float angle = acos(a.GetDotProd(b));
  if (isnan(angle))
    angle = 0;

  // Compute the normal to the plane which is defined by the
  // direction of rotation
  planeNorm2 = a.GetCrossProd(b);
  planeNorm2.Normalize();

  // Switch rotation direction if the two normals don't line up
  if ( planeNorm.GetDotProd(planeNorm2) > 0)
    angle *= -1;

  if (entity->HasType(MODEL)) 
  {
    Quatern delta; 
    delta.SetFromAxis( ray.x, ray.y, ray.z,angle);
    
    modelPose.rot = modelPose.rot * delta;
    entity->SetWorldPose(modelPose);
  }
  else
  {
    // TODO: make the forceMultiplier  user controllable 
    double forceMultiplier = 1.0;

    ((Body*)entity)->SetTorque(planeNorm * angle * forceMultiplier);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Translate an entity, or apply force
void RenderControl::EntityTranslate(Entity *entity)
{
  Pose3d pose = entity->GetWorldPose();

  Vector3 origin1, dir1, p1;
  Vector3 origin2, dir2, p2;

  // Cast two rays from the camera into the world
  this->userCamera->GetCameraToViewportRay(this->mouseEvent.pos.x, 
      this->mouseEvent.pos.y, origin1, dir1);
  this->userCamera->GetCameraToViewportRay(this->mouseEvent.prevPos.x, 
      this->mouseEvent.prevPos.y, origin2, dir2);

  Vector3 moveVector(0,0,0);
  Vector3 planeNorm(0,0,1);
  if (this->mouseModifier == "transx")
    moveVector.x = 1;
  else if (this->mouseModifier == "transy")
    moveVector.y = 1;
  else if (this->mouseModifier == "transz")
  {
    moveVector.z = 1;
    planeNorm.Set(1,0,0);
  }

  // Compute the distance from the camera to plane of translation
  double d = -pose.pos.GetDotProd(planeNorm);
  double dist1 = origin1.GetDistToPlane(dir1, planeNorm, d);
  double dist2 = origin2.GetDistToPlane(dir2, planeNorm, d);

  // Compute two points on the plane. The first point is the current
  // mouse position, the second is the previous mouse position
  p1 = origin1 + dir1 * dist1;
  p2 = origin2 + dir2 * dist2;

  this->userCamera->GetScene()->DrawLine(entity->GetWorldPose().pos,p2, "guiline");

  moveVector *= p1 - p2;

  if (entity->HasType(MODEL))
  {
    pose.pos += moveVector;
    entity->SetRelativePose(pose);
  }
  else if (entity->HasType(BODY))
  {
    // TODO: make the forceMultiplier  user controllable 
    double forceMultiplier = 1.0;

    Body *body = (Body*)(entity);
    moveVector *= forceMultiplier;
    body->SetForce(moveVector);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Get point on a plane
Vector3 RenderControl::GetWorldPointOnPlane(int x, int y, Vector3 planeNorm, double d)
{
  Vector3 origin, dir;
  double dist;

  // Cast two rays from the camera into the world
  //CameraManager::Instance()->GetActiveCamera()->GetCameraToViewportRay(x, y, origin, dir);
  this->userCamera->GetCameraToViewportRay(x, y, origin, dir);

  dist = origin.GetDistToPlane(dir, planeNorm, d);

  // Compute two points on the plane. The first point is the current
  // mouse position, the second is the previous mouse position
  return origin + dir * dist; 
}

void RenderControl::SetSelectedEntityCB(const std::string &name)
{
  //this->SetCursorState("manip");
  if (this->currMaker)
  {
    this->currMaker->Stop();
    this->currMaker = NULL;
  }
}
