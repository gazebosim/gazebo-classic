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
#include <unordered_map>

#include <boost/algorithm/string.hpp>

#include <gazebo/physics/ode/ode_inc.h>
#include <gazebo/physics/ode/ODELink.hh>
#include <gazebo/physics/ode/ODECollision.hh>
#include <gazebo/ode/contact.h>

#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"

#include "plugins/TrackedVehiclePlugin.hh"


namespace gazebo {

  /// \class SimpleTrackedVehiclePlugin SimpleTrackedVehiclePlugin.hh
  /// \brief A very fast, but also very accurate model of non-deformable tracks
  ///        without grousers.
  /// \since 8.1
  ///
  /// The motion model is based on adjusting motion1 in ODE contact properties
  /// and on computing Instantaneous Center of Rotation for a tracked vehicle.
  /// A detailed description of the model is given in
  /// https://arxiv.org/abs/1703.04316 .
  ///
  /// The plugin processes the following parameters, plus the common parameters
  /// defined in TrackedVehiclePlugin.
  ///
  /// <body>  Body of the vehicle to which the two tracks are connected.
  /// <left_track>  The left track link's name.
  /// <right_track>  The right track link's name.
  /// <left_flipper>  The name of a left flipper link.
  ///     Can appear multiple times.
  /// <right_flipper>  The name of a right flipper link.
  ///     Can appear multiple times.
  /// <collide_without_contact_bitmask> Collision bitmask that will be set to
  ///     the whole vehicle (default is 1u).

  class SimpleTrackedVehiclePlugin :
    public TrackedVehiclePlugin
  {
    public: SimpleTrackedVehiclePlugin() = default;

    public: virtual ~SimpleTrackedVehiclePlugin();

    /// \brief Called when the plugin is loaded
    /// \param[in] model Pointer to the model for which the plugin is loaded
    /// \param[in] _sdf Pointer to the SDF for _model
    public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

    /// \brief Initialize the plugin.
    public: void Init() override;

    /// \brief Reset the plugin.
    public: void Reset() override;

    /// \brief Set new target velocity for the tracks.
    ///
    /// \param[in] _left Velocity of left track.
    /// \param[in] _right Velocity of right track.
    protected: void SetTrackVelocityImpl(double _left, double _right) override;

    /// \brief Update surface parameters of the tracks to correspond to the
    ///        values set in this plugin.
    protected: void UpdateTrackSurface() override;

    /// \brief Body of the robot.
    protected: physics::LinkPtr body;

    /// \brief The tracks controlled by this plugin.
    protected: std::unordered_map<Tracks, physics::LinkPtr> tracks;

    /// \brief Desired velocities of the tracks.
    protected: std::unordered_map<Tracks, double> trackVelocity;

    /// \brief Compute and apply the forces that make the tracks move.
    protected: void DriveTracks(const common::UpdateInfo &/*_unused*/);

    /// \brief Return the number of tracks on the given side. Should always be
    /// at least 1 for the main track. If flippers are present, the number is
    /// higher.
    public: size_t GetNumTracks(Tracks side) const;

    /// \brief Set collide categories and bits of all geometries to the
    ///        required values.
    ///
    /// This is a workaround for https://bitbucket.org/osrf/gazebo/issues/1855 .
    protected: void SetGeomCategories();

    /// \brief Compute the direction of friction force in given contact point.
    /// \param[in] _linearSpeed Linear speed of the vehicle.
    /// \param[in] _angularSpeed Angular speed of the vehicle.
    /// \param[in] _drivingStraight Whether the vehicle should steer.
    /// \param[in] _bodyPose Pose of the vehicle body.
    /// \param[in] _bodyYAxisGlobal Direction of the y-axis of the body in
    ///            world frame.
    /// \param[in] _centerOfRotation Center of the circle the vehicle
    ///            follows (Inf/-Inf if driving straight).
    /// \param[in] _contactWorldPosition World position of the contact point.
    /// \param[in] _contactNormal Corrected contact normal (pointing inside
    ///            the track).
    /// \param[in] _beltDirection World-frame forward direction of the belt.
    /// \return Direction of the friction force in world frame.
    protected: ignition::math::Vector3d ComputeFrictionDirection(
      double _linearSpeed, double _angularSpeed,
      bool _drivingStraight, const ignition::math::Pose3d &_bodyPose,
      const ignition::math::Vector3d &_bodyYAxisGlobal,
      const ignition::math::Vector3d &_centerOfRotation,
      const ignition::math::Vector3d &_contactWorldPosition,
      const ignition::math::Vector3d &_contactNormal,
      const ignition::math::Vector3d &_beltDirection) const;

    /// \brief Compute the velocity of the surface motion in all contact points.
    /// \param[in] _beltSpeed The desired belt speed.
    /// \param[in] _beltDirection Forward direction of the belt.
    /// \param[in] _frictionDirection First friction direction.
    protected: double ComputeSurfaceMotion(double _beltSpeed,
      const ignition::math::Vector3d &_beltDirection,
      const ignition::math::Vector3d &_frictionDirection) const;

    private: transport::NodePtr node;

    private: event::ConnectionPtr beforePhysicsUpdateConnection;

    /// \brief This bitmask will be set to the whole vehicle body.
    protected: unsigned int collideWithoutContactBitmask;
    /// \brief Category for the non-track parts of the robot.
    protected: static const unsigned int ROBOT_CATEGORY = 0x10000000;
    /// \brief Category for tracks.
    protected: static const unsigned int BELT_CATEGORY = 0x20000000;
    /// \brief Category for all items on the left side.
    protected: static const unsigned int LEFT_CATEGORY = 0x40000000;

    private: physics::ContactManager *contactManager;

    /// \class ContactIterator
    /// \brief An iterator over all contacts between two geometries.
    class ContactIterator : std::iterator<std::input_iterator_tag, dContact>
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

      // Constructors.
      public: ContactIterator();
      public: explicit ContactIterator(bool _initialized);
      public: ContactIterator(const ContactIterator &_rhs);
      public: ContactIterator(dBodyID _body, dGeomID _geom1, dGeomID _geom2);

      /// \brief Use to "instantiate" the iterator from user code
      public: static ContactIterator begin(dBodyID _body, dGeomID _geom1,
                                     dGeomID _geom2);
      public: static ContactIterator end();

      /// \brief Finding the next element; this is the main logic.
      public: ContactIterator operator++();

      // Operators. It is required to implement them in iterators.
      public: bool operator==(const ContactIterator &_rhs);
      public: ContactIterator &operator=(const ContactIterator &_rhs);
      public: ContactIterator operator++(int _unused);
      public: reference operator*();
      public: pointer operator->();
      public: pointer getPointer();
      public: bool operator!=(const ContactIterator &_rhs);
    };
  };
}


#endif
