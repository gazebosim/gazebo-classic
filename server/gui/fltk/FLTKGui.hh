#ifndef FLTKGUI_HH
#define FLTKGUI_HH

#include <string>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <map>

#include <FL/Fl.H>
#include <FL/x.H>
#include <FL/Enumerations.H>
#include <FL/Fl_Gl_Window.H>

#include "Gui.hh"
#include "Vector3.hh"
#include "Vector2.hh"

namespace gazebo
{
  class FLTKGui : public Fl_Gl_Window
  {
    public: FLTKGui( int x, int y, int w, int h, const std::string &label );
    public: virtual ~FLTKGui();

    /// \brief Initalize the gui
    public: virtual void Init();

    public: void Update();

    /// \brief Get the width of the gui's rendering window
    public: virtual unsigned int GetWidth() const;

    /// \brief Get the height of the gui's rendering window
    public: virtual unsigned int GetHeight() const;

    public: void flush();

    /// \brief Handle event
    public: int handle(int event);
           
    /// \brief Handle a mouse button push
    private: void HandleMousePush();

    /// \brief Handle a mouse button release
    private: void HandleMouseRelease();

    /// \brief Handle a mouse drag
    private: void HandleMouseDrag();

    /// \brief Handle a key press
    private: void HandleKeyPress();

    /// \brief Handle a key release
    private: void HandleKeyRelease();

    /// ID of the window
    public: Window windowId;

    /// Pointer to the Xvisual
    public: XVisualInfo *visual;

    /// colormap
    public: Colormap colormap;

    /// pointer to the display
    public: Display *display;

    private: float moveAmount;
    private: float moveScale;
    private: float rotateAmount;

    private: Vector3 directionVec;

    private: bool leftMousePressed;
    private: bool rightMousePressed;
    private: bool middleMousePressed;
    private: Vector2<int> prevMousePos;
    private: Vector2<int> mousePos;
    private: std::map<int,int> keys;

    private: double lastUpdateTime;
  };

}

#endif
