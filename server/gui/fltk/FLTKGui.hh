#ifndef FLTKGUI_HH
#define FLTKGUI_HH

#include <string>

#include <FL/Fl.H>
#include <FL/x.H>
#include <FL/Enumerations.H>
#include <FL/Fl_Gl_Window.H>

#include "Gui.hh"
#include "InputEvent.hh"
#include "Vector3.hh"

namespace gazebo
{
  class InputHandler;

  class FLTKGui : public Fl_Gl_Window
  {
    public: FLTKGui( int x, int y, int w, int h, const std::string &label );
    public: virtual ~FLTKGui();

    /// \brief Initalize the gui
    public: virtual void Init();

    /// \brief Get the width of the gui's rendering window
    public: virtual unsigned int GetWidth() const;

    /// \brief Get the height of the gui's rendering window
    public: virtual unsigned int GetHeight() const;

    public: void draw();
    public: void flush();

    public: int handle(int event);

    private: Vector3 translateVec;
    private: float translateScale;

    public: Window windowId;
    public: XVisualInfo *visual;
    public: Colormap colormap;
    public: Display *display;

    private: InputHandler *inputHandler;
  };

}
#endif
