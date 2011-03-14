/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/* Desc: Dynamic line generator
 * Author: Nate Koenig
 * Date: 28 June 2007
 * CVS: $Id$
 */

#ifndef OGREDYNAMICLINES_HH
#define OGREDYNAMICLINES_HH

#include "common/Vector3.hh"
#include "OgreDynamicRenderable.hh"

#include <vector>

namespace gazebo
{
	namespace rendering
{

/// \addtogroup gazebo_rendering
/// \{

/// \brief Class for drawing lines
class OgreDynamicLines : public OgreDynamicRenderable
{
  /// Constructor
  public: OgreDynamicLines(RenderOpType opType=RENDERING_LINE_STRIP);

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
}
#endif
