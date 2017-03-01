/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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

#ifndef GAZEBO_SIMPLETRACKEDVEHICLEPLUGIN_HH
#define GAZEBO_SIMPLETRACKEDVEHICLEPLUGIN_HH

#include <string>

#include <boost/algorithm/string.hpp>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"

#include <gazebo/physics/ode/ode_inc.h>
#include <gazebo/physics/ode/ODELink.hh>
#include <gazebo/physics/ode/ODECollision.hh>
#include <gazebo/ode/contact.h>

#include "plugins/TrackedVehiclePlugin.hh"


namespace gazebo {

  /// \class SimpleTrackedVehiclePlugin SimpleTrackedVehiclePlugin.hh
  /// \brief A very fast, but also very accurate model of non-deformable tracks
  ///        without grousers.
  /// \since 8.1
  ///
  /// The plugin processes the following parameters, plus the common parameters
  /// defined in TrackedVehiclePlugin.
  ///
  /// <body>  Body of the vehicle to which the two tracks are connected.
  /// <left_track>  The left track link's name.
  /// <right_track>  The right track link's name.
  /// <collide_without_contact_bitmask> Collision bitmask that would be set to
  ///     the whole vehicle.

  class SimpleTrackedVehiclePlugin : public TrackedVehiclePlugin {

    public: SimpleTrackedVehiclePlugin();

    public: virtual ~SimpleTrackedVehiclePlugin();

    /// \brief Called when the plugin is loaded
    /// \param[in] model Pointer to the model for which the plugin is loaded
    /// \param[in] _sdf Pointer to the SDF for _model
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Initialize the plugin.
    public: virtual void Init();

    /// \brief Reset the plugin.
    public: virtual void Reset();

    /// \brief Set new target velocity for the tracks.
    ///
    /// Descendant classes need to implement this function.
    ///
    /// \param[in] left Velocity of left track.
    /// \param[in] right Velocity of right track.
    protected: virtual void SetTrackVelocity(double left, double right);

    /// \brief Body of the robot.
    protected: physics::LinkPtr body;

    /// \brief The tracks controlled by this plugin.
    protected: physics::LinkPtr leftTrack, rightTrack;

    /// \brief Desired velocities of the tracks.
    protected: double leftTrackVelocity = 0, rightTrackVelocity = 0;

    /// \brief Compute and apply the forces that make the tracks move.
    protected: void driveTracks(const common::UpdateInfo&);

    /// \brief Set collide categories and bits of all geometries to the
    ///        required values.
    ///
    /// This is a workaround for https://bitbucket.org/osrf/gazebo/issues/1855 .
    protected: void setGeomCategories();

    /// \brief Compute the direction of friction force in given contact point.
    /// \param[in] linearSpeed Linear speed of the vehicle.
    /// \param[in] angularSpeed Angular speed of the vehicle.
    /// \param[in] drivingStraight Whether the vehicle should steer.
    /// \param[in] bodyPose Pose of the vehicle body.
    /// \param[in] bodyYAxisGlobal Direction of the y-axis of the body in world.
    /// \param[in] centerOfRotation Center of the circle the vehicle
    ///             circumferences (Inf/-Inf if driving straight).
    /// \param[in] odeContact The ode contact information (to be changed).
    /// \param[in] beltDirection World-frame forward direction of the belt.
    /// \return Direction of the friction force in world frame.
    protected: ignition::math::Vector3d computeFrictionDirection(
        const double linearSpeed, const double angularSpeed,
        const bool drivingStraight, const ignition::math::Pose3d &bodyPose,
        const ignition::math::Vector3d &bodyYAxisGlobal,
        const ignition::math::Vector3d &centerOfRotation,
        const dContact *odeContact,
        const ignition::math::Vector3d &beltDirection) const;

    /// \brief Compute the velocity of the surface motion in all contact points.
    /// \param[in] beltSpeed The desired belt speed.
    /// \param[in] beltDirection Forward direction of the belt.
    /// \param[in] frictionDirection First friction direction.
    protected: double computeSurfaceMotion(const double beltSpeed,
        const ignition::math::Vector3d &beltDirection,
        const ignition::math::Vector3d &frictionDirection) const;

    private: transport::NodePtr node;
    private: event::ConnectionPtr beforePhysicsUpdateConnection;
    private: transport::SubscriberPtr contactsSubscriber;

    /// \brief This bitmask will be set to the whole vehicle body.
    protected: uint collideWithoutContactBitmask;
    /// \brief Category for the non-track parts of the robot.
    protected: static const uint ROBOT_CATEGORY = 0x10000000;
    /// \brief Category for tracks.
    protected: static const uint BELT_CATEGORY = 0x20000000;
    /// \brief Category for all items on the left side.
    protected: static const uint LEFT_CATEGORY = 0x40000000;

    private: physics::ContactManager* contactManager;

    /// \brief An empty callback which we need to register to the contacts
    ///        publisher so that contacts are computed.
    private: void ignoreContacts(ConstContactsPtr&)
    {
    }

    /// \brief The signum function.
    /// \param[in] The value.
    /// \return The signum of the value.
    private: template <typename T> int sgn(T val) const
    {
      return (T(0) < val) - (val < T(0));
    }


    /// \class dContact_iterator
    /// \brief An iterator over all contacts between two geometries.
    class dContact_iterator : std::iterator<std::input_iterator_tag, dContact*>
    {
      /// \brief The contact to return as the next element.
      private: pointer currentContact;
      /// \brief Index of the last examined joint.
      private: size_t jointIndex;
      /// \brief The body the contact should belong to.
      private: dBodyID body;
      /// \brief The geometries to search contacts for.
      private: dGeomID geom1, geom2;
      /// \brief True if at least one value has been returned.
      private: bool initialized;

      public: typedef dContact_iterator self_type;

      // Constructors.
      public: dContact_iterator();
      public: dContact_iterator(bool _initialized);
      public: dContact_iterator(const self_type& rhs);
      public: dContact_iterator(dBodyID _body, dGeomID _geom1, dGeomID _geom2);

      /// Use to "instantiate" the iterator from user code
      public: static self_type begin(dBodyID _body, dGeomID _geom1,
                                     dGeomID _geom2);
      public: static self_type end();

      /// Finding the next element; this is the main magic.
      public: self_type operator++();

      // Operators. It is required to implement them in iterators.
      public: bool operator==(const self_type& rhs);
      public: void operator=(const self_type& rhs);
      public: self_type operator++(int);
      public: reference operator*();
      public: pointer operator->();
      public: bool operator!=(const self_type& rhs);
    };

  };

}


#endif //GAZEBO_SIMPLETRACKEDVEHICLEPLUGIN_HH
