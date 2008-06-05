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

#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "Gui.hh"
#include "GuiFactory.hh"
#include "XMLConfig.hh"

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
Gui *GuiFactory::CreateGui(const std::string &classname, int x, int y, int w, int h, const std::string &label)
{
  if (guis[classname])
  {
    return (guis[classname]) (x, y, w, h, label);
  }
  else
  {
      gzthrow("Unable to make a GUI of type " << classname);
  }

  return NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Parse the XML config file and load the correct GUI with correct parameters
Gui* GuiFactory::NewGui(XMLConfigNode *rootNode)
{
  XMLConfigNode *childNode;
  Gui *gui = NULL;
  
  childNode = rootNode->GetChild("gui");
  
  if (childNode)
  {
    int width = childNode->GetTupleInt("size",0,640);
    int height = childNode->GetTupleInt("size",1,480);
    int x = childNode->GetTupleInt("pos",0,0);
    int y = childNode->GetTupleInt("pos",1,0);
    std::string type = childNode->GetString("type","fltk",1);
 
    gzmsg(1) << "Creating GUI:\n\tType[" << type << "] Pos[" << x << " " << y << "] Size[" << width << " " << height << "]\n";
    if (type != "fltk")
    {
      gzthrow("The only GUI available is 'fltk', for no-GUI simulation, delete the 'gui' tag and its children");
    }

    // Create the GUI
    gui = GuiFactory::CreateGui(type, x, y, width, height, type+"::Gazebo");
  }
  else
  {
    // Create a dummy GUI
    gzmsg(1) <<"Creating a dummy GUI";
    gui = GuiFactory::CreateGui(std::string("dummy"), 0, 0, 0, 0, std::string());
  }
  return gui;
 
}

