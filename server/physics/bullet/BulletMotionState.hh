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
/* Desc: Bullet motion state class.
 * Author: Nate Koenig 
 * Date: 25 May 2009
 */

#ifndef BULLETMOTIONSTATE
#define BULLETMOTIONSTATE

#include <btBulletDynamicsCommon.h>

#include "Pose3d.hh"

namespace gazebo
{
  class OgreVisual;
  class Body;

  class BulletMotionState : public btMotionState
  {
    /// \brief Constructor
    public: BulletMotionState(Body *body);

    /// \brief Constructor
    //public: BulletMotionState(const Pose3d &initPose);

    /// \brief Destructor
    public: virtual ~BulletMotionState();

    /// \brief Set the visual
    public: void SetVisual(OgreVisual *vis);

    /// \brief Get the pose
    public: Pose3d GetWorldPose() const;

    /// \brief Set the position of the body
    /// \param pos Vector position
    public: virtual void SetWorldPosition(const Vector3 &pos);

    /// \brief Set the rotation of the body
    /// \param rot Quaternion rotation
    public: virtual void SetWorldRotation(const Quatern &rot);

    /// \brief Set the pose
    public: void SetWorldPose(const Pose3d &pose);

    /// \brief Set the center of mass offset
    public: void SetCoMOffset( const Pose3d &com );

    /// \brief Get the world transform
    public: virtual void getWorldTransform(btTransform &worldTrans) const;

    /// \brief Set the world transform
    public: virtual void setWorldTransform(const btTransform &worldTrans);

    private: OgreVisual *visual;
    private: Pose3d worldPose;
    private: Pose3d comOffset;
    private: Body *body;
  };
}
#endif
