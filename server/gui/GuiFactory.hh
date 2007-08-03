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
/*
 * Desc: Factory for creating a gui
 * Author: Andrew Howard
 * Date: 18 May 2003
 * SVN info: $Id:$
 */

#ifndef GUIFACTORY_HH
#define GUIFACTORY_HH

#include <string>
#include <map>

namespace gazebo
{

  // Forward declarations
  class Gui;

  // Prototype for gui factory functions
  typedef Gui* (*GuiFactoryFn) (int x, int y, int w, int h, const std::string &label);

  /// \brief The gui factory
  class GuiFactory
  {
    /// \brief Register all known guis.
    public: static void RegisterAll();

    /// \brief Register a gui class (called by gui registration function).
    public: static void RegisterGui(std::string type, std::string  classname,
                GuiFactoryFn factoryfn);

    /// \brief Create a new instance of a gui.  Used by the world when
    /// reading the world file.
    public: static Gui *NewGui(const std::string &classname, int x, int y, int w, int h, const std::string &label);

    /// \brief A list of registered sensor classes
    private: static std::map<std::string, GuiFactoryFn> guis;
  };


  /// \brief Static gui registration macro
  ///
  /// Use this macro to register guis with the server.
  /// \param name Gui type name, as it appears in the world file.
  /// \param classname C++ class name for the gui.
#define GZ_REGISTER_STATIC_GUI(name, classname) \
  Gui *New##classname(int x, int y, int w, int h, const std::string &label) \
  { \
    return new classname(x, y, w, h, label); \
  } \
  void Register##classname() \
  {\
    GuiFactory::RegisterGui("static", name, New##classname);\
  }

}

#endif
