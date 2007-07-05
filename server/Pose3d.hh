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
/* Desc: External interfaces for Gazebo
 * Author: Nate Koenig
 * Date: 03 Apr 2007
 * SVN: $Id$
 */

#ifndef POSE3D_HH
#define POSE3D_HH

#include <iostream>

#include "Vector3.hh"
#include "Quatern.hh"


namespace gazebo
{

/// @addtogroup gazebocore
/// @{


/// Pose3d encapsulates a position and rotation in three space
class Pose3d
{
  /// Default constructors
  public: Pose3d();

  /// Constructor
  /// \param pos A position
  /// \param rot A rotation
  public: Pose3d( const Vector3 &pos, const Quatern &rot);

  /// Copy constructor
  /// \param pose Pose3d to copy
  public: Pose3d( const Pose3d &pose );

  /// Destructor
  public: virtual ~Pose3d();

  /// See if a pose is finite (e.g., not nan)
  public: bool IsFinite() const;

  /// Addition operator
  /// \param pose Pose to add to this pose
  /// \return The resulting pose
  public: Pose3d operator+(const Pose3d &pose) const;

  /// Add-Equals operator
  /// \param pose Pose to add to this pose
  /// \return The resulting pose
  public: const Pose3d &operator+=(const Pose3d &pose);

  /// Subtraction operator
  /// \param pose Pose to subtract from this one
  /// \return The resulting pose
  public: Pose3d operator-(const Pose3d &pose) const;

  /// Subtraction operator
  /// \param pose Pose to subtract from this one
  /// \return The resulting pose
  public: const Pose3d &operator-=(const Pose3d &pose);
          
  /// Add one point to a vector: result = this + pos
  /// \param pos Position to add to this pose
  /// \return The resulting position
  public: Vector3 CoordPositionAdd(const Vector3 &pos) const;

  /// Add one point to another: result = this + pose
  /// \param pose The Pose to add
  /// \return The resulting position
  public: Vector3 CoordPositionAdd(const Pose3d &pose) const;

  /// Subtract one position from another: result = this - pose
  /// \param pose Pose to subtract
  /// \return The resulting position
  public: Vector3 CoordPositionSub(const Pose3d &pose) const;

  /// Add one rotation to another: result =  this->rot + rot
  /// \param rot Rotation to add
  /// \return The resulting rotation
  public: Quatern CoordRotationAdd(const Quatern &rot) const;

  /// Subtract one rotation from another: result = this->rot - rot
  /// \param rot The rotation to subtract
  /// \return The resulting rotation
  public: Quatern CoordRotationSub(const Quatern &rot) const;

  /// \brief Find the inverse of a pose; i.e., if b = this + a, given b and 
  ///        this, find a
  public: Pose3d CoordPoseSolve(const Pose3d &b) const;

  /// Reset the pose
  public: void Reset();

  /// The position
  public: Vector3 pos;

  /// The rotation
  public: Quatern rot;

  /// Ostream operator
  /// \param out Ostream
  /// \param pose Pose to output
  /// \return the Ostream
  public: friend std::ostream &operator<<(std::ostream &out, const gazebo::Pose3d &pose)
          {
            out << "Pos[" << pose.pos << "] Rot[" << pose.rot << "]";
            return out;

          }
};

/// @}

}

#endif

