/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_PHYSICS_INERTIAL_HH_
#define GAZEBO_PHYSICS_INERTIAL_HH_

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <string>

#include <sdf/sdf.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Matrix3.hh>

#include "gazebo/math/Pose.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/math/Quaternion.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Matrix3.hh"

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    // Forward declare private data
    class InertialPrivate;

    /// \addtogroup gazebo_physics
    /// \{

    /// \class Inertial Inertial.hh physics/physics.hh
    /// \brief A class for inertial information about a link
    class GZ_PHYSICS_VISIBLE Inertial
    {
      /// \brief Default Constructor
      public: Inertial();

      /// \brief Constructor.
      /// \param[in] _mass Mass value in kg if using metric.
      public: explicit Inertial(const double _mass);

      /// \brief Copy constructor.
      /// \param[in] _inertial Inertial element to copy
      public: Inertial(const Inertial &_inertial);

      /// \brief Destructor.
      public: virtual ~Inertial();

      /// \brief Load from SDF values.
      /// \param[in] _sdf SDF value to load from.
      public: void Load(sdf::ElementPtr _sdf);

      /// \brief update the parameters using new sdf values.
      /// \param[in] _sdf Update values from.
      public: void UpdateParameters(sdf::ElementPtr _sdf);

      /// \brief Reset all the mass properties.
      public: void Reset();

      /// \brief Set the mass.
      public: void SetMass(const double _m);

      /// \brief Get the mass
      /// \deprecated See Mass() const
      public: double GetMass() const GAZEBO_DEPRECATED(8.0);

      /// \brief Get the mass
      /// \deprecated See Mass() const
      public: double Mass() const;

      /// \brief Set the mass matrix.
      /// \param[in] _ixx X second moment of inertia (MOI) about x axis.
      /// \param[in] _iyy Y second moment of inertia about y axis.
      /// \param[in] _izz Z second moment of inertia about z axis.
      /// \param[in] _ixy XY inertia.
      /// \param[in] _ixz XZ inertia.
      /// \param[in] _iyz YZ inertia.
      public: void SetInertiaMatrix(
                  const double _ixx, const double _iyy, const double _izz,
                  const double _ixy, const double _ixz, const double iyz);

      /// \brief Set the center of gravity.
      /// \param[in] _cx X position.
      /// \param[in] _cy Y position.
      /// \param[in] _cz Z position.
      public: void SetCoG(const double _cx, const double _cy, const double _cz);

      /// \brief Set the center of gravity.
      /// \param[in] _center Center of the gravity.
      /// \deprecated See version that accepts ignition::math parameters.
      public: void SetCoG(const math::Vector3 &_center) GAZEBO_DEPRECATED(8.0);

      /// \brief Set the center of gravity.
      /// \param[in] _center Center of the gravity.
      public: void SetCoG(const ignition::math::Vector3d &_center);

      /// \brief Set the center of gravity and rotation offset of inertial
      ///        coordinate frame relative to Link frame.
      /// \param[in] _cx Center offset in x-direction in Link frame
      /// \param[in] _cy Center offset in y-direction in Link frame
      /// \param[in] _cz Center offset in z-direction in Link frame
      /// \param[in] _rx Roll angle offset of inertial coordinate frame.
      /// \param[in] _ry Pitch angle offset of inertial coordinate frame.
      /// \param[in] _rz Yaw angle offset of inertial coordinate frame.
      public: void SetCoG(const double _cx, const double _cy, const double _cz,
                          const double _rx, const double _ry, const double _rz);

      /// \brief Set the center of gravity.
      /// \param[in] _c Transform to center of gravity.
      /// \deprecated See version that accepts ignition::math parameters.
      public: void SetCoG(const math::Pose &_c) GAZEBO_DEPRECATED(8.0);

      /// \brief Set the center of gravity.
      /// \param[in] _c Transform to center of gravity.
      public: void SetCoG(const ignition::math::Pose3d &_c);

      /// \brief Get the center of gravity.
      /// \return The center of gravity.
      /// \deprecated See CoG() const
      public: math::Vector3 GetCoG() const GAZEBO_DEPRECATED(8.0);

      /// \brief Get the center of gravity.
      /// \return The center of gravity.
      public: const ignition::math::Vector3d &CoG() const;

      /// \brief Get the pose about which the mass and inertia matrix is
      /// specified in the Link frame.
      /// \return The inertial pose.
      /// \deprecated See Pose() const
      public: const math::Pose GetPose() const GAZEBO_DEPRECATED(8.0);

      /// \brief Get the pose about which the mass and inertia matrix is
      /// specified in the Link frame.
      /// \return The inertial pose.
      public: const ignition::math::Pose3d Pose() const;

      /// \brief Get the principal moments of inertia (Ixx, Iyy, Izz).
      /// \return The principal moments.
      /// \deprecated See PrincipalMoments() const
      public: math::Vector3 GetPrincipalMoments() const GAZEBO_DEPRECATED(8.0);

      /// \brief Get the principal moments of inertia (Ixx, Iyy, Izz).
      /// \return The principal moments.
      public: ignition::math::Vector3d PrincipalMoments() const;

      /// \brief Get the products of inertia (Ixy, Ixz, Iyz).
      /// \return The products of inertia.
      /// \deprecated See ProductsOfInertia() const
      public: math::Vector3 GetProductsofInertia() const GAZEBO_DEPRECATED(8.0);

      /// \brief Get the products of inertia (Ixy, Ixz, Iyz).
      /// \return The products of inertia.
      public: ignition::math::Vector3d ProductsOfInertia() const;

      /// \brief Get IXX
      /// \return IXX value
      /// \deprecated See IXX() const
      public: double GetIXX() const GAZEBO_DEPRECATED(8.0);

      /// \brief Get IXX
      /// \return IXX value
      public: double IXX() const;

      /// \brief Get IYY
      /// \return IYY value
      /// \deprecated See IYY() const
      public: double GetIYY() const GAZEBO_DEPRECATED(8.0);

      /// \brief Get IYY
      /// \return IYY value
      public: double IYY() const;

      /// \brief Get IZZ
      /// \return IZZ value
      /// \deprecated See IZZ() const
      public: double GetIZZ() const GAZEBO_DEPRECATED(8.0);

      /// \brief Get IZZ
      /// \return IZZ value
      public: double IZZ() const;

      /// \brief Get IXY
      /// \return IXY value
      /// \deprecated See IXY() const
      public: double GetIXY() const GAZEBO_DEPRECATED(8.0);

      /// \brief Get IXY
      /// \return IXY value
      public: double IXY() const;

      /// \brief Get IXZ
      /// \return IXZ value
      /// \deprecated See IXZ() const
      public: double GetIXZ() const GAZEBO_DEPRECATED(8.0);

      /// \brief Get IXZ
      /// \return IXZ value
      public: double IXZ() const;

      /// \brief Get IXZ
      /// \return IYZ value
      /// \deprecated See IYZ() const
      public: double GetIYZ() const GAZEBO_DEPRECATED(8.0);

      /// \brief Get IXZ
      /// \return IYZ value
      public: double IYZ() const;

      /// \brief Set IXX
      /// \param[in] _v IXX value
      public: void SetIXX(const double _v);

      /// \brief Set IYY
      /// \param[in] _v IYY value
      public: void SetIYY(const double _v);

      /// \brief Set IZZ
      /// \param[in] _v IZZ value
      public: void SetIZZ(const double _v);

      /// \brief Set IXY
      /// \param[in] _v IXY value
      public: void SetIXY(const double _v);

      /// \brief Set IXZ
      /// \param[in] _v IXZ value
      public: void SetIXZ(const double _v);

      /// \brief Set IYZ
      /// \param[in] _v IXX value
      public: void SetIYZ(const double _v);

      /// \brief Rotate this mass.
      /// \param[in] _rot Rotation amount.
      /// \deprecated See version that accepts ignition::math parameters.
      public: void Rotate(const math::Quaternion &_rot) GAZEBO_DEPRECATED(8.0);

      /// \brief Rotate this mass.
      /// \param[in] _rot Rotation amount.
      public: void Rotate(const ignition::math::Quaterniond &_rot);

      /// \brief Equal operator.
      /// \param[in] _inertial Inertial to copy.
      /// \return Reference to this object.
      public: Inertial &operator=(const Inertial &_inertial);

      /// \brief Addition operator.
      /// Assuming both CG and Moment of Inertia (MOI) are defined
      /// in the same reference Link frame.
      /// New CG is computed from masses and perspective offsets,
      /// and both MOI contributions relocated to the new cog.
      /// \param[in] _inertial Inertial to add.
      /// \return The result of the addition.
      public: Inertial operator+(const Inertial &_inertial) const;

      /// \brief Addition equal operator.
      /// \param[in] _inertial Inertial to add.
      /// \return Reference to this object.
      public: const Inertial &operator+=(const Inertial &_inertial);

      /// \brief Update parameters from a message
      /// \param[in] _msg Message to read
      public: void ProcessMsg(const msgs::Inertial &_msg);

      /// \brief Get the equivalent inertia from a point in local Link frame
      /// If you specify GetMOI(this->GetPose()), you should get
      /// back the Moment of Inertia (MOI) exactly as specified in the SDF.
      /// If _pose is different from pose of the Inertial block, then
      /// the MOI is rotated accordingly, and contributions from changes
      /// in MOI location location due to point mass is added to the final MOI.
      /// \param[in] _pose location in Link local frame
      /// \return equivalent inertia at _pose
      /// \deprecated See MOI(const ignition::math::Pose3d &) const
      public: math::Matrix3 GetMOI(const math::Pose &_pose)
        const GAZEBO_DEPRECATED(8.0);

      /// \brief Get the equivalent inertia from a point in local Link frame
      /// If you specify GetMOI(this->GetPose()), you should get
      /// back the Moment of Inertia (MOI) exactly as specified in the SDF.
      /// If _pose is different from pose of the Inertial block, then
      /// the MOI is rotated accordingly, and contributions from changes
      /// in MOI location location due to point mass is added to the final MOI.
      /// \param[in] _pose location in Link local frame
      /// \return equivalent inertia at _pose
      public: ignition::math::Matrix3d MOI(
                  const ignition::math::Pose3d &_pose) const;

      /// \brief Get the equivalent inertia from a point in local Link frame
      /// If you specify GetMOI(this->GetPose()), you should get
      /// back the Moment of Inertia (MOI) exactly as specified in the SDF.
      /// If _pose is different from pose of the Inertial block, then
      /// the MOI is rotated accordingly, and contributions from changes
      /// in MOI location location due to point mass is added to the final MOI.
      /// \param[in] _pos location in Link local frame
      /// \return equivalent inertia at _pose
      public: ignition::math::Matrix3d MOI(
                  const ignition::math::Vector3d &_pos) const;

      /// \brief Get equivalent Inertia values with the Link frame offset,
      /// while holding the Pose of CoG constant in the world frame.
      /// \param[in] _frameOffset amount to offset the Link frame by, this
      /// is a transform defined in the Link frame.
      /// \return Inertial parameters with the shifted frame.
      /// \deprecated See operator()(const ignition::math::Pose3d &) const
      public: Inertial GetInertial(const math::Pose &_frameOffset) const
              GAZEBO_DEPRECATED(8.0);

      /// \brief Get equivalent Inertia values with the Link frame offset,
      /// while holding the Pose of CoG constant in the world frame.
      /// \param[in] _frameOffset amount to offset the Link frame by, this
      /// is a transform defined in the Link frame.
      /// \return Inertial parameters with the shifted frame.
      /// \deprecated See operator()(const ignition::math::Pose3d &) const
      public: Inertial operator()(
                  const ignition::math::Pose3d &_frameOffset) const;

      /// \brief Get equivalent Inertia values with the Link frame offset,
      /// while holding the Pose of CoG constant in the world frame.
      /// \param[in] _frameOffset amount to offset the Link frame by, this
      /// is a transform defined in the Link frame.
      /// \return Inertial parameters with the shifted frame.
      /// \deprecated See operator()(const ignition::math::Pose3d &) const
      public: Inertial operator()(
                  const ignition::math::Vector3d &_frameOffset) const;

      /// \brief Output operator.
      /// \param[in] _out Output stream.
      /// \param[in] _inertial Inertial object to output.
      public: friend std::ostream &operator<<(std::ostream &_out,
                  const gazebo::physics::Inertial &_inertial)
              {
                _out << "Mass[" << _inertial.Mass() << "] CoG["
                    << _inertial.CoG() << "]\n";
                _out << "IXX[" << _inertial.PrincipalMoments().X() << "] "
                     << "IYY[" << _inertial.PrincipalMoments().Y() << "] "
                     << "IZZ[" << _inertial.PrincipalMoments().Z() << "]\n";
                _out << "IXY[" << _inertial.ProductsOfInertia().X() << "] "
                     << "IXZ[" << _inertial.ProductsOfInertia().Y() << "] "
                     << "IYZ[" << _inertial.ProductsOfInertia().Z() << "]\n";
                return _out;
              }

      /// \brief returns Moments of Inertia as a Matrix3
      /// \return Moments of Inertia as a Matrix3
      /// \deprecated See MOI() const
      public: math::Matrix3 GetMOI() const GAZEBO_DEPRECATED(8.0);

      /// \brief returns Moments of Inertia as a Matrix3
      /// \return Moments of Inertia as a Matrix3
      public: ignition::math::Matrix3d MOI() const;

      /// \brief Sets Moments of Inertia (MOI) from a Matrix3
      /// \param[in] Moments of Inertia as a Matrix3
      /// \deprecated See versoin that accepts ignition math parameters
      public: void SetMOI(const math::Matrix3 &_moi) GAZEBO_DEPRECATED(8.0);

      /// \brief Sets Moments of Inertia (MOI) from a Matrix3
      /// \param[in] Moments of Inertia as a Matrix3
      public: void SetMOI(const ignition::math::Matrix3d &_moi);

      /// \internal
      /// \brief Private data pointer.
      private: std::unique_ptr<InertialPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
