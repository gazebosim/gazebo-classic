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
/* Desc: ODE Heightmap shape
 * Author: Nate Keonig
 * Date: 12 Nov 2009
 * SVN: $Id$
 */

#ifndef ODEHEIGHTMAPSHAPE_HH
#define ODEHEIGHTMAPSHAPE_HH

#include "HeightmapShape.hh"
#include "Vector2.hh"
#include "ODEPhysics.hh"
#include "Geom.hh"

namespace gazebo
{
  /// \brief ODE Height map geom
  class ODEHeightmapShape : public HeightmapShape
  {
    /// \brief Constructor
    public: ODEHeightmapShape(Geom *parent);

    /// \brief Destructor
    public: virtual ~ODEHeightmapShape();

    /// \brief Update function 
    public: void Update();

    /// \brief Load the heightmap
    protected: virtual void Load(XMLConfigNode *node);

    /// Create a lookup table of the terrain's height
    private: void FillHeightMap();

    /// \brief Called by ODE to get the height at a vertex
    private: static dReal GetHeightCallback(void *data, int x, int y);

    private: dHeightfieldDataID odeData;

    private: unsigned int odeVertSize;
    private: Vector3 odeScale;

    private: std::vector<double> heights;
  };

  /// \}
}

#endif
