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
/* Desc: StatusBar
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id$
 */
#ifndef STATUSBAR_HH
#define STATUSBAR_HH

#include <FL/Fl_Group.H>

class Fl_Value_Output;
class Fl_Output;
class Fl_Button;

namespace gazebo
{

  /// \brief Toolbar
  class StatusBar : public Fl_Group
  {
    /// \brief Constructor
    public: StatusBar (int x, int y, int w, int h, const char *l=0);
   
    /// \brief Destructor
    public: virtual ~StatusBar();
    
    /// \brief Update the toolbar data
    public: void Update();
  
    /// \brief Play Pause button callback
    public: static void PlayPauseButtonCB( Fl_Widget *w, void *data );

    /// \brief Set button callback
    public: static void StepButtonCB( Fl_Widget *w, void *data );

    private: Fl_Value_Output *iterations;
    private: Fl_Value_Output *percentOutput;
    private: Fl_Value_Output *fps;
    private: Fl_Value_Output *realTime;
    private: Fl_Value_Output *pauseTime;
    private: Fl_Value_Output *simTime;

    private: Fl_Button *playButton;
    private: Fl_Button *stepButton;

    public: Gui *gui;

    private: double lastUpdateTime;

    // calculated percent speedup in last percentWindowDuration seconds real time.
    private: double percent, percentLastRealTime, percentLastSimTime;
    private: static const double statusUpdatePeriod = 0.05;
  };
  
}

#endif
