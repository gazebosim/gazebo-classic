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
 * Desc: Factory for creating guis
 * Author: Nate Koenig
 * Date: 03 Aug 2007
 * SVN info: $Id$
 */
#include <FL/Fl_Menu_Item.H>
#include <FL/Fl_Menu_Bar.H>

#include "GazeboError.hh"
#include "Gui.hh"
#include "GuiFactory.hh"

using namespace gazebo;

void RegisterFLTKMainWindow();

std::map<std::string, GuiFactoryFn> GuiFactory::guis;


////////////////////////////////////////////////////////////////////////////////
// Register a gui class.
void GuiFactory::RegisterGui(std::string type, std::string classname,
                             GuiFactoryFn factoryfn)
{
  guis[classname] = factoryfn;
}


////////////////////////////////////////////////////////////////////////////////
// Create a new instance of a gui.
Gui *GuiFactory::NewGui(const std::string &classname, int x, int y, int w, int h, const std::string &label)
{
  if (guis[classname])
  {
    return (guis[classname]) (x, y, w, h, label);
  }
  else
  {
    std::ostringstream stream;
    stream << "Unable to make a GUI of type " << classname;
    gzthrow(stream.str());
  }

  return NULL;
}

