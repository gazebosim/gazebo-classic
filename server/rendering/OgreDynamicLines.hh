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
/* Desc: Dynamic line generator
 * Author: Nate Koenig
 * Date: 28 June 2007
 * CVS: $Id$
 */

#ifndef OGREDYNAMICLINES_HH
#define OGREDYNAMICLINES_HH

#include "Vector3.hh"
#include "OgreDynamicRenderable.hh"

#include <vector>

namespace gazebo
{

/// \addtogroup gazebo_rendering
/// \{

/// \brief Class for drawing lines
class OgreDynamicLines : public OgreDynamicRenderable
{
  /// Constructor
  public: OgreDynamicLines(OperationType opType=RENDERING_LINE_STRIP);

  /// Destructor
  public: virtual ~OgreDynamicLines();

  /// \brief Returns "gazebo::ogredynamicslines"
  public: virtual const Ogre::String &getMovableType() const;

  /// Add a point to the point list
  /// \param pt Vector3 point
  public: void AddPoint(const Vector3 &pt);

  /// Change the location of an existing point in the point list
  /// \param index Index of the point to set
  /// \param value Vector3 value to set the point to
  public: void SetPoint(unsigned int index, const Vector3 &value);

  /// Return the location of an existing point in the point list
  /// \param index Number of the point to return
  /// \return Vector3 value of the point
  public: const Vector3& GetPoint(unsigned int index) const;

  /// Return the total number of points in the point list
  /// \return Number of points
  public: unsigned int GetNumPoints() const;

  /// Remove all points from the point list
  public: void Clear();

  /// Call this to update the hardware buffer after making changes.  
  public: void Update();

  /// \brief Implementation DynamicRenderable, creates a simple vertex-only decl
  protected: virtual void  CreateVertexDeclaration();

  /// \brief Implementation DynamicRenderable, pushes point list out to hardware memory
  protected: virtual void FillHardwareBuffers();

  private: std::vector<Vector3> points;
  private: bool dirty;
};

/// \}
}
#endif
