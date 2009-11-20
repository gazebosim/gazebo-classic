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
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id$
 */

#ifndef FLTKMAINWINDOW_HH
#define FLTKMAINWINDOW_HH

#include <X11/Xlib.h>
#include <X11/Xutil.h>

#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Slider.H>
#include <string>
#include <iostream>

#include "Param.hh"
#include "Vector2.hh"

//#define BG_COLOR fl_rgb_color(235,225,208 )
#define BG_COLOR FL_BACKGROUND_COLOR

namespace gazebo
{
  class GLWindow;
  class Sidebar;
  class Toolbar;
  class StatusBar;
  class GLFrameManager;
  class XMLConfigNode;

  /// \brief FLTK Main Window
  class Gui : public Fl_Double_Window
  {
    /// \brief Constructor
    public: Gui (int x, int y, int w, int h, const std::string &t);
 
    /// \brief Destructor
    public: virtual ~Gui();

    /// \brief Load the gui
    public: virtual void Load(XMLConfigNode *node);

    /// \brief Save the gui params in xml format
    public: virtual void Save(std::string &prefix, std::ostream &stream);

    /// \brief Create the user camera's 
    public: void CreateCameras();

    /// \brief Initalize the gui
    public: virtual void Init();

    public: virtual void Update();

    /// \brief Get the width of the gui's rendering window
    public: unsigned int GetWidth() const;

    /// \brief Get the height of the gui's rendering window
    public: unsigned int GetHeight() const;

    /// \brief Handle an event
    public: int handle(int event);

    /// \brief Get the average FPS
    public: float GetAvgFPS() const;

    /// \brief Time slider cb
    private: static void TimeSliderCB( Fl_Widget * w, void *data);

    private: Toolbar *toolbar;
    private: Sidebar *sidebar;
    private: StatusBar *statusbar;
    private: GLFrameManager *frameMgr;

    private: bool hasFocus;

    private: ParamT<Vector2<int> > *sizeP;
    private: ParamT<Vector2<int> > *posP;
    private: std::vector<Param*> parameters;

    private: Fl_Slider *timeSlider;
  };
}

#endif
