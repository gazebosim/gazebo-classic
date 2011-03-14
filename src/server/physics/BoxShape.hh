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
/* Desc: Box geometry
 * Author: Nate Keonig, Andrew Howard
 * Date: 8 May 2003
 * SVN: $Id: BoxGeom.hh 7039 2008-09-24 18:06:29Z natepak $
 */

#ifndef BOXSHAPE_HH
#define BOXSHAPE_HH

#include "Param.hh"
#include "Shape.hh"

namespace gazebo
{

  /// \addtogroup gazebo_physics_geom
  /// \{
  /** \defgroup gazebo_box_geom Box Geom
    \brief Box geom

    \par Attributes
    The following attributes are supported.

    \htmlinclude default_geom_attr_include.html

    - size (float tuple, meters)
      - Size of the box
      - Default: 0 0 0

    \par Example
    \verbatim
    <geom:box name="geom_name">
      <xyz>1 2 3</xyz>
      <rpy>0 0 30</rpy>
      <size>0.1 0.2 0.3</size>
      <mass>0.5</mass>
      <laserFiducialId>1</laserFiducialId>
      <laserRetro>0.5</laserRetro>

      <visual>
        <size>0.1 0.2 0.3</size>
        <mesh>default</mesh>
        <material>Gazebo/Red</material>
      </visual>
    </geom:box>
    \endverbatim
    */
  /// \}
  /// \addtogroup gazebo_box_geom 
  /// \{

  /// \brief Box geom
  class BoxShape : public Shape
  {
    /// \brief Constructor
    public: BoxShape(Geom *parent);

    /// \brief Destructor
    public: virtual ~BoxShape();

    /// \brief Load the box
    public: virtual void Load(XMLConfigNode *node);

    /// \brief Save child parameters
    public: virtual void Save(std::string &prefix, std::ostream &stream);

    /// \brief Set the size of the box
    public: virtual void SetSize( const Vector3 &size );

    private: ParamT<Vector3> *sizeP;
  };

  /// \}

}
#endif
