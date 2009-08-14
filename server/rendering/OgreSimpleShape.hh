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
/* Desc: Create various 3d shapes
 * Author: Nathan Koenig
 * Date: 3 Jan 2008
 */

#ifndef OGRESIMPLESHAPE_HH
#define OGRESIMPLESHAPE_HH

#include <Ogre.h>

#include "Vector3.hh"
#include "Vector2.hh"

namespace gazebo
{
/// \addtogroup gazebo_rendering
/// \{

/// \brief Class used to render simple shapes
class OgreSimpleShape
{
  /// \brief Constructor
  private: OgreSimpleShape();

  /// \brief Destructor
  private: ~OgreSimpleShape();

  /// \brief Create a sphere mesh
  public: static void CreateSphere(const std::string &name, float radius, int rings, int segments);

  /// \brief Create a Box mesh
  public: static void CreateBox(const std::string &name, const Vector3 &sides,
                                const Vector2<double> &uvCoords);

  /// \brief Create a cylinder mesh
  public: static void CreateCylinder(const std::string &name, float radius, 
                                     float height, int rings, int segments);

  /// \brief Create a cone mesh
  public: static void CreateCone(const std::string &name, float radius, 
                                 float height, int rings, int segments);

  /// \brief Create a tube mesh
  public: static void CreateTube(const std::string &name, float innerRadius, 
                                 float outterRadius, float height, int rings, 
                                 int segments);

};

/// \}
}

#endif
