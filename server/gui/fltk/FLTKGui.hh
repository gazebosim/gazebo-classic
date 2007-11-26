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
/* Desc: FLTK Gui
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id:$
 */

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

  /// \brief FLTK Gui
  class FLTKGui : public Fl_Gl_Window, public Gui
  {
    /// \brief Constructor
    public: FLTKGui( int x, int y, int w, int h, const std::string &label );

    /// \brief Destructor
    public: virtual ~FLTKGui();

    /// \brief Initalize the gui
    public: virtual void Init();

    /// \brief Get the width of the gui's rendering window
    public: virtual unsigned int GetWidth() const;

    /// \brief Get the height of the gui's rendering window
    public: virtual unsigned int GetHeight() const;

    /// \brief Handle event
    public: int handle(int event);

    private: Vector3 translateVec;
    private: float translateScale;

    private: InputHandler *inputHandler;
  };

}
#endif
