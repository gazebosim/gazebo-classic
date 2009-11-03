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
/* Desc: Geom class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id: Geom.hh 7640 2009-05-13 02:06:08Z natepak $
 */

#ifndef BulletGEOM_HH
#define BulletGEOM_HH

#include "Param.hh"
#include "Entity.hh"
#include "Pose3d.hh"
#include "Vector3.hh"
#include "Geom.hh"

class btCollisionShape;

namespace gazebo
{
  class Body;
  class XMLConfigNode;
  class BulletPhysics;

  /// \addtogroup gazebo_physics_ode
  /// \brief Base class for all Bullet geoms
  /// \{

  /// \brief Base class for all geoms
  class BulletGeom : public Geom
  {
  
    /// \brief Constructor
    //public: Geom(Body *body, const std::string &name);
    public: BulletGeom(Body *body);
  
    /// \brief Destructor
    public: virtual ~BulletGeom();

    /// \brief Load the geom
    public: virtual void Load(XMLConfigNode *node);

    /// \brief Load the geom
    public: virtual void Save(std::string &prefix, std::ostream &stream);

    /// \brief Update function for geoms
    public: virtual void Update();

    /// \brief On pose change
    public: virtual void OnPoseChange();

    /// \brief Set the category bits, used during collision detection
    /// \param bits The bits
    public: virtual void SetCategoryBits(unsigned int bits);
  
    /// \brief Set the collide bits, used during collision detection
    /// \param bits The bits
    public: virtual void SetCollideBits(unsigned int bits);
  
    /// \brief Get the mass of the geom
    public: Mass GetBodyMassMatrix();
  
    /// \brief Get the bounding box, defined by the physics engine
    public: virtual void GetBoundingBox( Vector3 &min, Vector3 &max ) const;

    /// \brief Set the collision shape
    public: void SetCollisionShape( btCollisionShape *shape );

    /// \brief Get the bullet collision shape
    public: btCollisionShape *GetCollisionShape() const;

    /// \brief Set the index of the compound shape
    public: void SetCompoundShapeIndex( int index );

    protected: BulletPhysics *bulletPhysics;
    protected: btCollisionShape *collisionShape;

    protected: int compoundShapeIndex;
  };

  /// \}

}
#endif
