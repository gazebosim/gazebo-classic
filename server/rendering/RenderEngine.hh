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
/* Desc: Middleman between actual rendering engine and Gazebo
 * Author: Nate Koenig, Jordi Polo
 * Date: 02 Jun 2008
 */

#ifndef RENDERENGINE 
#define RENDERENGINE

#include <string>

namespace gazebo
{
/// \addtogroup gazebo_rendering
/// \{


class XMLConfigNode;

/// \brief Rendering engine for the simulation
class RenderEngine 
{

  /// \brief Constructor
  public: RenderEngine();

  /// \brief Destructor
  public: virtual ~RenderEngine();

  /// \brief Closes the present rendering engine, frees the resources 
  public: virtual void Close() = 0;

  /// \brief Default initialization. 
  ///  Let the rendering engine  create the window and rendering context
  public: virtual void Init(XMLConfigNode *rootNode) = 0;

  /// \brief Save rendering engine settings 
  public: virtual void Save(XMLConfigNode *node) = 0;

  /// \brief Render a single frame
  public: virtual void Render() = 0;
 
  /// \brief Resize the rendering window
  public: virtual void ResizeWindow(unsigned int w, unsigned int h) = 0;

  /// \brief Gets the average frame per second rate achieved
  public: virtual float GetAverageFPS() const = 0;

  /// \brief Gets the minimum time between renders, set by in the file to limit Framerate
  public: double GetUpdateRate() const;

  /// \brief Gets the type (name) of this Rendering Engine
  public: std::string GetType() const;

  protected: double updateRate;
  protected: std::string type;
  

};

}
#endif
