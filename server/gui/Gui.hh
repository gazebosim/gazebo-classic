#ifndef GUI_HH
#define GUI_HH

#include <X11/Xlib.h>
#include <X11/Xutil.h>


namespace gazebo
{

  /// \brief The base class for all GUIs
  class Gui
  {
    /// \brief Constructor
    public: Gui();

    /// \brief Destructor
    public: virtual ~Gui();

    /// \brief Initalize the gui
    public: virtual void Init() = 0;

    /// \brief Update the gui
    public: virtual void Update() = 0;

    /// \brief Get the width of the gui's rendering window
    public: virtual unsigned int GetWidth() const = 0;

    /// \brief Get the height of the gui's rendering window
    public: virtual unsigned int GetHeight() const = 0;

    /// \brief Get the id of the window
    public: Window GetWindowId() const;
            
    /// \brief Get the visual info
    public: XVisualInfo *GetVisualInfo() const;

    /// \brief Get the display
    public: Display *GetDisplay() const;

    protected: Window windowId;
    protected: XVisualInfo *visual;
    protected: Colormap colormap;
    protected: Display *display;
  };
}
#endif
