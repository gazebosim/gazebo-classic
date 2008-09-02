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
 * SVN: $Id:$
 */

#ifndef TOOLBAR_HH
#define TOOLBAR_HH

#include <FL/Fl_Group.H>
#include <FL/Fl_Hold_Browser.H>

class Fl_Value_Output;
class Fl_Output;
class Fl_Input;
class Fl_Button;

namespace gazebo
{
  class Entity;

  /// \brief Toolbar
  class Toolbar : public Fl_Group
  {
    /// \brief Constructor
    public: Toolbar (int x, int y, int w, int h, const char *l=0);
  
    /// \brief Destructor
    public: virtual ~Toolbar();

    /// \brief Update the toolbar data
    public: void Update();

    public: static void AttributeBrowserCB( Fl_Widget * w, void *data);

    private: void AddEntityToAttributeBrowser(Entity *ent, std::string prefix);
    private: void AddToBrowser(const std::string &line);

    private: Fl_Hold_Browser *entityBrowser;

    private: int columnWidths[3];
    private: int attrCount;
  };
  
}


#endif

