#ifndef FLTKGUI_HH
#define FLTKGUI_HH

#include <string>

#include <FL/Fl.h>
#include <FL/x.H>
#include <FL/Enumerations.h>
#include <FL/Fl_Window.H>

#include "Gui.hh"
#include "Vector3.hh"

namespace gazebo
{
  class FLTKGui : public Gui, public Fl_Window
  {
    public: FLTKGui( int x, int y, int w, int h, const std::string &label );
    public: virtual ~FLTKGui();

    /// \brief Initalize the gui
    public: virtual void Init();

    /// \brief Update the gui
    public: virtual void Update();

    /// \brief Get the width of the gui's rendering window
    public: virtual unsigned int GetWidth() const;

    /// \brief Get the height of the gui's rendering window
    public: virtual unsigned int GetHeight() const;

    public: virtual void draw();
    public: virtual void flush();

/*  public: int handle(int event);

    private: int HandlePush();
    private: int HandleDrag();
    private: int HandleKeyboard();
                */

    private: void OpenDisplay();

    private: Vector3 translateVec;
    private: float translateScale;

  };

}
#endif
