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
/* Desc: Toolbar
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id: Toolbar.hh 8381 2009-11-10 17:10:09Z natepak $
 */

#ifndef TOOLBAR_HH
#define TOOLBAR_HH

#include <FL/Fl_Group.H>
#include <FL/Fl_Button.H>

class Fl_Box;

namespace gazebo
{
  class Common;
  class Gui;

  /// \brief Toolbar
  class Toolbar : public Fl_Group
  {
    /// \brief Constructor
    public: Toolbar (int x, int y, int w, int h, const char *l=0);
  
    /// \brief Destructor
    public: virtual ~Toolbar();

    /// \brief Update the toolbar data
    public: void Update();

    /// \brief Play pause button callback
    public: static void PlayButtonCB( Fl_Widget *w, void *data );

    /// \brief Pause pause button callback
    public: static void PauseButtonCB( Fl_Widget *w, void *data );

    /// \brief Set button callback
    public: static void StepButtonCB( Fl_Widget * /*w*/, void * /*data*/ );

    /// \brief Box button callback
    public: static void BoxButtonCB( Fl_Widget *w, void * /*data*/ );

    /// \brief Sphere button callback
    public: static void SphereButtonCB( Fl_Widget *w, void * /*data*/ );

    /// \brief Cylinder button callback
    public: static void CylinderButtonCB( Fl_Widget *w, void * /*data*/ );

    /// \brief Cursor button callback
    public: static void CursorButtonCB( Fl_Widget *w, void * /*data*/ );

    /// \brief Cursor button callback
    public: static void HandCursorButtonCB( Fl_Widget *w, void * /*data*/ );

    private: void ManipModeCB(bool mode);
    private: void MoveModeCB(bool mode);



    private: class ToolbarButton : public Fl_Button
          {
            public: ToolbarButton(int x, int y, int w, int h, const char *l=0)
                    : Fl_Button(x,y,w,h,l)
            {
              this->box(FL_FLAT_BOX);
              this->down_box(FL_DOWN_BOX);
              this->type(FL_RADIO_BUTTON);
            }
            public: virtual ~ToolbarButton() {}
            public: int handle(int event)
                    {
                      int ret = Fl_Button::handle(event);
                      switch(event)
                      {
                        case FL_ENTER:
                          this->box(FL_UP_BOX);
                          this->redraw();
                          break;
                        case FL_LEAVE:
                          this->box(FL_FLAT_BOX);
                          this->redraw();
                          break;
                      }

                      return ret;
                    }
          };


    private: Fl_Button *playButton;
    private: Fl_Button *pauseButton;
    private: Fl_Button *stepButton;
    private: Fl_Button *moveButton;
    private: ToolbarButton *boxButton;
    private: ToolbarButton *sphereButton;
    private: ToolbarButton *cylinderButton;
    private: ToolbarButton *cursorButton;
    private: ToolbarButton *handCursorButton;

    private: Fl_RGB_Image *playImage[2];
    private: Fl_RGB_Image *pauseImage[2];
    private: Fl_RGB_Image *stepImage[2];
    private: Fl_RGB_Image *boxImage[2];
    private: Fl_RGB_Image *sphereImage[2];
    private: Fl_RGB_Image *cylinderImage[2];
    private: Fl_RGB_Image *cursorImage, *handCursorImage;

    public: Gui *gui;
  };
  
}

#endif
