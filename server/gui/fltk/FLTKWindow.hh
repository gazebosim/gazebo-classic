#ifndef FLTKWINDOW_HH
#define FLTKWINDOW_HH

#include <string>

#include <X11/Xlib.h>
#include <X11/Xutil.h>

#include <FL/Fl.h>
#include <FL/x.H>
#include <FL/Enumerations.h>
#include <FL/Fl_Window.H>

#include "Vector3.hh"

namespace gazebo
{
  class FLTKWindow : public Fl_Window
  {
    public: FLTKWindow( int x, int y, int w, int h, const char* label = NULL );
    public: virtual ~FLTKWindow();

    public: virtual void show();

    public: virtual void draw();
    public: virtual void flush();

/*  public: int handle(int event);

    private: int HandlePush();
    private: int HandleDrag();
    private: int HandleKeyboard();
                */

    private: void OpenDisplay();

    public: Window windowId;
    public: XVisualInfo *visual;
    public: Colormap colormap;
    public: Display *display;

    private: Vector3 translateVec;
    private: float translateScale;

  };

}
#endif
