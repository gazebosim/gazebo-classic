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
/* Desc: FLTK main menu
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id$
 */

#ifndef MAINMENU_HH
#define MAINMENU_HH

#include <FL/Fl_Menu_Bar.H>

namespace gazebo
{
  class MainMenu : public Fl_Menu_Bar
  {
    public: MainMenu(int x, int y, int w, int h, char *name=0);

    public: static void OpenCB(Fl_Widget * /*w*/, void * /*data*/);

    public: static void SaveFramesCB(Fl_Widget *w, void *data);

    public: static void QuitCB(Fl_Widget *w, void *data);

    public: static void SaveWorldCB(Fl_Widget *w, void *data);

    public: static void ShowBoundingBoxesCB(Fl_Widget *w, void *data);

    public: static void ShowJointsCB(Fl_Widget * /*w*/, void * /*data*/);

    public: static void ShowPhysicsCB(Fl_Widget * /*w*/, void * /*data*/);

    public: static void ResetCB(Fl_Widget * /*w*/, void * /*data*/);

    public: static void WireframeCB(Fl_Widget * /*w*/, void * /*data*/);

    public: static void SplitCB(Fl_Widget * /*w*/, void * /*data*/);

    public: static void ShowContactsCB(Fl_Widget * /*w*/, void * /*data*/);

    public: static void ShowLightsCB(Fl_Widget * /*w*/, void * /*data*/);

    public: static void ShowCamerasCB(Fl_Widget * /*w*/, void * /*data*/);

    public: static void PerPixelLightingCB(Fl_Widget * /*w*/, void * /*data*/);

  };
}

#endif
