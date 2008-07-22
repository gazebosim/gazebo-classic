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
/* Desc: GL Window
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id:$
 */

#ifndef FLTKGUI_HH
#define FLTKGUI_HH

#include <string>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <map>

#include <FL/Fl.H>
#include <FL/Fl_Choice.H>
#include <FL/Fl_Group.H>
#include <FL/x.H>
#include <FL/Enumerations.H>
#include <FL/Fl_Gl_Window.H>

#include "Gui.hh"
#include "Vector3.hh"
#include "Vector2.hh"


namespace gazebo
{
  class UserCamera;
  class OgreCamera;
  class GLFrame;
  class WindowManager;

  /// \brief OpenGL window to display camera data
  class GLWindow : public Fl_Gl_Window
  {
    /// \brief Constructor
    public: GLWindow(int x, int y, int w, int h, const std::string &label="" );

    /// \brief Destructor
    public: virtual ~GLWindow();

    /// \brief Initalize the gui
    public: virtual void Init();

    /// \brief Update the window
    public: void Update();

    /// \brief Get the width of the gui's rendering window
    public: virtual unsigned int GetWidth() const;

    /// \brief Get the height of the gui's rendering window
    public: virtual unsigned int GetHeight() const;

    public: void flush();

    /// \brief Handle event
    public: int handle(int event);
   
    /// \brief Handle resizing
    public: void resize(int x, int y, int w, int h);

    /// \brief Get the visual info
    public: XVisualInfo *GetVisualInfo() const;

    /// \brief Get the display
    public: Display *GetDisplay() const;

    /// \brief Get a pointer to the camera
    public: UserCamera *GetCamera() const;

    // \brief Get the average FPS for this window
    public: float GetAvgFPS() const;

    /// \brief Set the active camera
    public: void SetActiveCamera( OgreCamera *camera );

    /// \brief Set the style of the view = "front, left, top, user"
    public: void SetViewStyle(std::string view);

    /// \brief Handle a mouse button push
    private: void HandleMousePush();

    /// \brief Handle a mouse button release
    private: void HandleMouseRelease();

    /// \brief Handle a mouse drag
    private: void HandleMouseDrag();

    /// \brief Handle a key press
    private: void HandleKeyPress(int keyNum);

    /// \brief Handle a key release
    private: void HandleKeyRelease(int keyNum);

    /// \brief Handle mouse wheel movement
    private: void HandleMouseWheel(int dx, int dy);

    /// ID of the window
    private: Window windowId;

    /// Pointer to the Xvisual
    private: XVisualInfo *visual;

    /// colormap
    private: Colormap colormap;

    /// pointer to the display
    private: Display *display;

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

    private: bool mouseDrag;

    /// Pointer to the camera
    private: UserCamera *userCamera;
    private: OgreCamera *activeCamera;

    private: static GLWindow *activeWin;
  };

}

#endif
