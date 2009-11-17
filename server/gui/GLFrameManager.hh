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
/* Desc: FLTK GL Frame manager
 * Author: Nate Koenig
 * Date: 18 Jun 2008
 * SVN: $Id$
 */

#ifndef GLFRAMEMANAGER_HH
#define GLFRAMEMANAGER_HH

#include <FL/Fl_Tile.H>
#include <vector>

namespace gazebo
{
  class GLFrame;
  class GLWindow;
  class XMLConfigNode;

  /// \brief Class to manage all the GL frames
  class GLFrameManager : public Fl_Tile
  {
    /// \brief Constructor
    public: GLFrameManager(int x, int y, int w, int h, const std::string &name);

    /// \brief Destructor
    public: virtual ~GLFrameManager();

    /// \brief Load the frame manager
    public: void Load( XMLConfigNode *node );

    /// \brief Save the gui params in xml format
    public: virtual void Save(std::string &prefix, std::ostream &stream);

    /// \brief Create user cameras
    public: void CreateCameras();

    /// \brief Initalize the window manager
    public: void Init();

    /// \brief Update the window manager
    public: void Update();

    /// \brief Split a frame
    public: void Split(GLFrame *parent, const std::string &type);

    /// \brief Get the avg FPS
    public: float GetFPS() const;

    /// Vector of all the frames
    private: std::vector<GLFrame *> frames;
    private: XMLConfigNode *configNode;
  };
}
#endif
