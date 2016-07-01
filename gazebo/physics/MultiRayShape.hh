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
#ifndef _MULTIRAYSHAPE_HH_
#define _MULTIRAYSHAPE_HH_

#include <vector>
#include <string>

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Angle.hh"

#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/Shape.hh"
#include "gazebo/physics/RayShape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class MultiRayShape MultiRayShape.hh physics/physics.hh
    /// \brief Laser collision contains a set of ray-collisions,
    /// structured to simulate a laser range scanner.
    class GZ_PHYSICS_VISIBLE MultiRayShape : public Shape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent collision shape.
      public: explicit MultiRayShape(CollisionPtr _parent);

      /// \brief Constructor for a stand alone multiray shape. Stand alone
      /// means the multiray shape is not attached to a Collision object.
      ///
      /// Example:
      ///
      ///  gazebo::physics::MultiRayShapePtr rays =
      ///      boost::dynamic_pointer_cast<gazebo::physics::MultiRayShape>(
      ///        world->GetPhysicsEngine()->CreateShape("multiray",
      ///          gazebo::physics::CollisionPtr()));
      ///
      /// \param[in] _physicsEngine Pointer to the physics engine.
      public: explicit MultiRayShape(PhysicsEnginePtr _physicsEngine);

      /// \brief Destructor.
      public: virtual ~MultiRayShape();

      /// \brief Init the shape.
      public: virtual void Init();

      /// \brief Set the scale of the multi ray shape.
      /// \return _scale Scale to set the multi ray shape to.
      public: virtual void SetScale(const math::Vector3 &_scale);

      /// \brief Get detected range for a ray.
      /// \param[in] _index Index of the ray.
      /// \returns Returns DBL_MAX for no detection.
      public: double GetRange(unsigned int _index);

      /// \brief Get detected retro (intensity) value for a ray.
      /// \param[in] _index Index of the ray.
      /// \return Retro value for the ray.
      public: double GetRetro(unsigned int _index);

      /// \brief Get detected fiducial value for a ray.
      /// \param[in] _index Index of the ray.
      /// \return Fiducial value for the ray.
      public: int GetFiducial(unsigned int _index);

      /// \brief Get the minimum range.
      /// \return Minimum range of all the rays.
      public: double GetMinRange() const;

      /// \brief Get the maximum range.
      /// \return Maximum range of all the rays.
      public: double GetMaxRange() const;

      /// \brief Get the range resolution.
      /// \return Range resolution of all the rays.
      public: double GetResRange() const;

      /// \brief Get the horizontal sample count.
      /// \return Horizontal sample count.
      public: int GetSampleCount() const;

      /// \brief Get the horizontal resolution.
      /// \return Horizontal resolution.
      public: double GetScanResolution() const;

      /// \brief Get the minimum angle.
      /// \return Minimum angle of ray scan.
      public: math::Angle GetMinAngle() const;

      /// \brief Get the maximum angle.
      /// \return Maximum angle of ray scan.
      public: math::Angle GetMaxAngle() const;

      /// \brief Get the vertical sample count.
      /// \return Verical sample count.
      public: int GetVerticalSampleCount() const;

      /// \brief Get the vertical range resolution.
      /// \return Vertical range resolution.
      public: double GetVerticalScanResolution() const;

      /// \brief Get the vertical min angle.
      /// \return Vertical min angle.
      public: math::Angle GetVerticalMinAngle() const;

      /// \brief Get the vertical max angle.
      /// \return Vertical max angle.
      public: math::Angle GetVerticalMaxAngle() const;

      /// \brief Update the ray collisions.
      public: void Update();

      /// \TODO This function is not implemented.
      /// \brief Fill a message with this shape's values.
      /// \param[out] _msg Message that contains the shape's values.
      public: void FillMsg(msgs::Geometry &_msg);

      /// \TODO This function is not implemented.
      /// \brief Update the ray based on a message.
      /// \param[in] _msg Message to update from.
      public: virtual void ProcessMsg(const msgs::Geometry &_msg);

      /// Documentation inherited
      public: virtual double ComputeVolume() const;

      /// \brief Connect a to the new laser scan signal.
      /// \param[in] _subscriber Callback function.
      /// \return The connection, which must be kept in scope.
      public: template<typename T>
              event::ConnectionPtr ConnectNewLaserScans(T _subscriber)
              {return this->newLaserScans.Connect(_subscriber);}

      /// \brief Disconnect from the new laser scans signal.
      /// \param[in] _conn Connection to remove.
      /// \deprecated Use event::~Connection to disconnect
      public: void DisconnectNewLaserScans(event::ConnectionPtr &_conn)
              GAZEBO_DEPRECATED(8.0)
              {this->newLaserScans.Disconnect(_conn->Id());}

      /// \brief Method for updating the rays. This function is normally
      /// called automatically, such as when a laser sensor is updated.
      /// Only call this function on a standalone multiray shape.
      /// \sa explicit MultiRayShape(PhysicsEnginePtr _physicsEngine)
      public: virtual void UpdateRays() = 0;

      /// \brief Add a ray to the collision.
      /// \param[in] _start Start of the ray.
      /// \param[in] _end End of the ray.
      public: virtual void AddRay(const math::Vector3 &_start,
                                  const math::Vector3 &_end);

      /// \brief Set the points of a ray.
      /// \param[in] _rayIndex Index of the ray to set.
      /// \param[in] _start Start of the ray.
      /// \param[in] _end End of the ray.
      /// \return True if the ray was set. False can be returned if the
      /// _rayIndex is invalid.
      public: bool SetRay(const unsigned int _rayIndex,
                  const ignition::math::Vector3d &_start,
                  const ignition::math::Vector3d &_end);

      /// \brief Get the number of rays.
      /// \return Number of rays in this shape.
      public: unsigned int RayCount() const;

      /// \brief Get a pointer to a ray
      /// \param[in] _rayIndex index to the ray
      /// \return Pointer to the ray, or NULL on error
      /// \sa RayCount()
      public: RayShapePtr Ray(const unsigned int _rayIndex) const;

      /// \brief Ray data
      protected: std::vector<RayShapePtr> rays;

      /// \brief Pose offset of all the rays.
      protected: math::Pose offset;

      /// \brief Ray SDF element pointer.
      protected: sdf::ElementPtr rayElem;

      /// \brief Scan SDF element pointer.
      protected: sdf::ElementPtr scanElem;

      /// \brief Horizontal SDF element pointer.
      protected: sdf::ElementPtr horzElem;

      /// \brief Vertical SDF element pointer.
      protected: sdf::ElementPtr vertElem;

      /// \brief Range SDF element pointer.
      protected: sdf::ElementPtr rangeElem;

      /// \brief New laser scans event.
      protected: event::EventT<void()> newLaserScans;

      /// \brief Min range of a ray
      private: double minRange = 0;

      /// \brief Max range of a ray
      private: double maxRange = 1000;
    };
    /// \}
  }
}
#endif
