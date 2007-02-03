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
 * Desc: Controller base class.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN info: $Id$
 */

#include <string>

class XMLConfigNode;

class Controller
{
  /// Constructor
  public: Controller();

  /// Destructor
  public: virtual ~Controller();

  /// Load the controller. Called once on startup
  public: int Load(XMLConfigNode *node);

  /// Initialize the controller. Called once on startup.
  public: int Init();

  /// Update the controller. Called every cycle.
  public: int Update();

  /// Finialize the controller. Called once on completion.
  public: int Fini();

  /// Load function for the child class
  public: virtual int LoadChild(XMLConfigNode *node) {return 0;}

  /// Init function for the child class
  public: virtual int InitChild() {return 0;}

  /// Update function for the child class
  public: virtual int UpdateChild() {return 0;}

  /// Fini function for the child class
  public: virtual int FiniChild() {return 0;}

  /// Return the name of this controller
  public: std::string GetName() const;

  protected: std::string name;
};
