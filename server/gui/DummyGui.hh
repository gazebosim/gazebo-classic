/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: FLTK Mainwindow
 * Author: Renaud Barate
 * Date: 16 Feb 2008
 */
#ifndef DUMMYGUI_HH
#define DUMMYGUI_HH


#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <GL/glx.h>

#include "Gui.hh"

namespace gazebo
{


 /// \brief Dummy window used to run gazebo without display
  class DummyGui : public Gui
  {
    /// \brief Constructor
    public: DummyGui(int x, int y, int w, int h, const std::string &t);

    /// \brief Destructor
    public: virtual ~DummyGui();

    public: virtual void Init() {}
    public: virtual void Update();

    /// \brief Get the width of the gui's rendering window
    public: unsigned int GetWidth() const { return 1; }

    /// \brief Get the height of the gui's rendering window
    public: unsigned int GetHeight() const { return 1; }

    /// \brief Handle an event
    public: int handle(int event) { return 0; }

    /// \brief Get the id of the window
    public: Window GetWindowId() const;

    /// \brief Get the visual info
    public: XVisualInfo *GetVisualInfo() const;

    /// \brief Get the display
    public: Display *GetDisplay() const;

    /// ID of the window
    protected: Window windowId;

    /// Pointer to the Xvisual
    protected: XVisualInfo *visual;

    /// pointer to the display
    protected: Display *display;

    /// GLX context used to render the scenes
    protected: GLXContext context;
  };

}

#endif


