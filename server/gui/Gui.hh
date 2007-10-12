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
/* Desc: GUI base class
 * Author: Nate Koenig
 * Date: 17 Sep 2007
 * SVN: $Id$
 */

#ifndef GUI_HH
#define GUI_HH

#include <X11/Xlib.h>
#include <X11/Xutil.h>

namespace gazebo
{

  class OgreAdaptor;

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
    public: virtual void Update() {};

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

    /// \brief Handle a keyboard event
    protected: void HandleKeyboard(int key);

    protected: Window windowId;
    protected: XVisualInfo *visual;
    protected: Colormap colormap;
    protected: Display *display;

    private: OgreAdaptor *ogre;
  };
}
#endif
