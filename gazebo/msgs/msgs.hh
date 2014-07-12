/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef MESSAGES_UTILITY_H
#define MESSAGES_UTILITY_H

#include <string>

#include <sdf/sdf.hh>
#include <ignition/math.hh>

#include "gazebo/msgs/MessageTypes.hh"

#include "gazebo/common/SphericalCoordinates.hh"
#include "gazebo/common/Color.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Image.hh"

namespace gazebo
{
  /// \ingroup gazebo_msgs Messages
  /// \brief Messages namespace
  namespace msgs
  {
    /// \addtogroup gazebo_msgs Messages
    /// \brief All messages and helper functions
    /// \{

    /// \brief Create a request message
    /// \param[in] _request Request string
    /// \param[in] _data Optional data string
    /// \return A Request message
    GAZEBO_VISIBLE
    msgs::Request *CreateRequest(const std::string &_request,
                                 const std::string &_data = "");

    /// \brief Initialize a message
    /// \param[in] _message Message to initialize
    /// \param[in] _id Optional string id
    GAZEBO_VISIBLE
    void Init(google::protobuf::Message &_message, const std::string &_id ="");

    /// \brief Time stamp a header
    /// \param[in] _header Header to stamp
    GAZEBO_VISIBLE
    void Stamp(msgs::Header *_header);

    /// \brief Set the time in a time message
    /// \param[in] _time A Time message
    GAZEBO_VISIBLE
    void Stamp(msgs::Time *_time);

    /// \cond
    GAZEBO_VISIBLE
    std::string Package(const std::string &type,
        const google::protobuf::Message &message);
    /// \endcond

    /// \brief Convert a ignition::math::Vector3d to a msgs::Vector3d
    /// \param[in] _v The vector to convert
    /// \return A msgs::Vector3d object
    GAZEBO_VISIBLE
    msgs::Vector3d      Convert(const ignition::math::Vector3d &_v);

    /// \brief Convert a ignition::math::Quaterniond to a msgs::Quaternion
    /// \param[in] _q The quaternion to convert
    /// \return A msgs::Quaternion object
    GAZEBO_VISIBLE
    msgs::Quaternion Convert(const ignition::math::Quaterniond &_q);

    /// \brief Convert an ignition::math::Pose3d to a msgs::Pose
    /// \param[in] _p The pose to convert
    /// \return A msgs::Pose object
    GAZEBO_VISIBLE
    msgs::Pose Convert(const ignition::math::Pose3d &_p);

    /// \brief Convert a common::Color to a msgs::Color
    /// \param[in] _c The color to convert
    /// \return A msgs::Color object
    GAZEBO_VISIBLE
    msgs::Color Convert(const common::Color &_c);

    /// \brief Convert a common::Time to a msgs::Time
    /// \param[in] _t The time to convert
    /// \return A msgs::Time object
    GAZEBO_VISIBLE
    msgs::Time  Convert(const common::Time &_t);

    /// \brief Convert a math::Plane to a msgs::PlaneGeom
    /// \param[in] _p The plane to convert
    /// \return A msgs::PlaneGeom object
    GAZEBO_VISIBLE
    msgs::PlaneGeom Convert(const ignition::math::Planed &_p);

    /// \brief Convert a msgs::Vector3d to a math::Vector
    /// \param[in] _v The plane to convert
    /// \return A ignition::math::Vector3d object
    GAZEBO_VISIBLE
    ignition::math::Vector3d Convert(const msgs::Vector3d &_v);

    /// \brief Convert a msgs::Vector2d to a math::Vector2d
    /// \param[in] _v The vector2 to convert
    /// \return A math::Vector2d object
    GAZEBO_VISIBLE
    ignition::math::Vector2d Convert(const msgs::Vector2d &_v);

    /// \brief Convert a msgs::Quaternion to a ignition::math::Quaterniond
    /// \param[in] _q The quaternion to convert
    /// \return A ignition::math::Quaterniond object
    GAZEBO_VISIBLE
    ignition::math::Quaterniond Convert(const msgs::Quaternion &_q);

    /// \brief Convert a msgs::Pose to an ignition::math::Pose3d
    /// \param[in] _q The pose to convert
    /// \return An ignition::math::Pose3d object
    GAZEBO_VISIBLE
    ignition::math::Pose3d Convert(const msgs::Pose &_p);

    /// \brief Convert a msgs::Image to a common::Image
    /// \param[out] _img The common::Image container
    /// \param[in] _msg The Image message to convert
    GAZEBO_VISIBLE
    void Set(common::Image &_img, const msgs::Image &_msg);

    /// \brief Convert a msgs::Color to a common::Color
    /// \param[in] _c The color to convert
    /// \return A common::Color object
    GAZEBO_VISIBLE
    common::Color Convert(const msgs::Color &_c);

    /// \brief Convert a msgs::Time to a common::Time
    /// \param[in] _t The time to convert
    /// \return A common::Time object
    GAZEBO_VISIBLE
    common::Time Convert(const msgs::Time &_t);

    /// \brief Convert a msgs::PlaneGeom to a common::Plane
    /// \param[in] _p The plane to convert
    /// \return A common::Plane object
    GAZEBO_VISIBLE
    ignition::math::Planed Convert(const msgs::PlaneGeom &_p);

    /// \brief Set a msgs::Image from a common::Image
    /// \param[out] _msg A msgs::Image pointer
    /// \param[in] _i A common::Image reference
    GAZEBO_VISIBLE
    void Set(msgs::Image *_msg, const common::Image &_i);

    /// \brief Set a msgs::Vector3d from a ignition::math::Vector3d
    /// \param[out] _pt A msgs::Vector3d pointer
    /// \param[in] _v A ignition::math::Vector3d reference
    GAZEBO_VISIBLE
    void Set(msgs::Vector3d *_pt, const ignition::math::Vector3d &_v);

    /// \brief Set a msgs::Vector2d from a ignition::math::Vector3d
    /// \param[out] _pt A msgs::Vector2d pointer
    /// \param[in] _v An ignition::math::Vector2d reference
    GAZEBO_VISIBLE
    void Set(msgs::Vector2d *_pt, const ignition::math::Vector2d &_v);

    /// \brief Set a msgs::Quaternion from a ignition::math::Quaterniond
    /// \param[out] _q A msgs::Quaternion pointer
    /// \param[in] _v A ignition::math::Quaterniond reference
    GAZEBO_VISIBLE
    void Set(msgs::Quaternion *_q, const ignition::math::Quaterniond &_v);

    /// \brief Set a msgs::Pose from a ignition::math::Pose3d
    /// \param[out] _p A msgs::Pose pointer
    /// \param[in] _v A ignition::math::Pose3d reference
    GAZEBO_VISIBLE
    void Set(msgs::Pose *_p, const ignition::math::Pose3d &_v);

    /// \brief Set a msgs::Color from a common::Color
    /// \param[out] _p A msgs::Color pointer
    /// \param[in] _v A common::Color reference
    GAZEBO_VISIBLE
    void Set(msgs::Color *_c, const common::Color &_v);

    /// \brief Set a msgs::Time from a common::Time
    /// \param[out] _p A msgs::Time pointer
    /// \param[in] _v A common::Time reference
    GAZEBO_VISIBLE
    void Set(msgs::Time *_t, const common::Time &_v);

    /// \brief Set a msgs::SphericalCoordinates from
    /// a common::SphericalCoordinates object.
    /// \param[out] _p A msgs::SphericalCoordinates pointer.
    /// \param[in] _v A common::SphericalCoordinates reference
    void Set(msgs::SphericalCoordinates *_s,
             const common::SphericalCoordinates &_v);

    /// \brief Set a msgs::Plane from an ignition::math::Planed
    /// \param[out] _p A msgs::Plane pointer
    /// \param[in] _v An ignition::math::Planed reference
    GAZEBO_VISIBLE
    void Set(msgs::PlaneGeom *_p, const ignition::math::Planed &_v);

    /// \brief Create a msgs::TrackVisual from a track visual SDF element
    /// \param[in] _sdf The sdf element
    /// \return The new msgs::TrackVisual object
    GAZEBO_VISIBLE
    msgs::TrackVisual TrackVisualFromSDF(sdf::ElementPtr _sdf);

    /// \brief Create a msgs::GUI from a GUI SDF element
    /// \param[in] _sdf The sdf element
    /// \return The new msgs::GUI object
    GAZEBO_VISIBLE
    msgs::GUI GUIFromSDF(sdf::ElementPtr _sdf);

    /// \brief Create a msgs::Light from a light SDF element
    /// \param[in] _sdf The sdf element
    /// \return The new msgs::Light object
    GAZEBO_VISIBLE
    msgs::Light LightFromSDF(sdf::ElementPtr _sdf);

    /// \brief Create a msgs::MeshGeom from a mesh SDF element
    /// \param[in] _sdf The sdf element
    /// \return The new msgs::MeshGeom object
    GAZEBO_VISIBLE
    msgs::MeshGeom MeshFromSDF(sdf::ElementPtr _sdf);

    /// \brief Create a msgs::Geometry from a geometry SDF element
    /// \param[in] _sdf The sdf element
    /// \return The new msgs::Geometry object
    GAZEBO_VISIBLE
    msgs::Geometry GeometryFromSDF(sdf::ElementPtr _sdf);

    /// \brief Create a msgs::Visual from a visual SDF element
    /// \param[in] _sdf The sdf element
    /// \return The new msgs::Visual object
    GAZEBO_VISIBLE
    msgs::Visual VisualFromSDF(sdf::ElementPtr _sdf);

    /// \brief Create a msgs::Fog from a fog SDF element
    /// \param[in] _sdf The sdf element
    /// \return The new msgs::Fog object
    GAZEBO_VISIBLE
    msgs::Fog FogFromSDF(sdf::ElementPtr _sdf);

    /// \brief Create a msgs::Scene from a scene SDF element
    /// \param[in] _sdf The sdf element
    /// \return The new msgs::Scene object
    GAZEBO_VISIBLE
    msgs::Scene SceneFromSDF(sdf::ElementPtr _sdf);

    /// \brief Create an SDF element from a msgs::Scene
    /// \param[in] _msg Light messsage
    /// \param[in] _sdf if supplied, performs an update from _msg intead of
    /// creating a new sdf element.
    /// \return The new SDF element
    GAZEBO_VISIBLE
    sdf::ElementPtr LightToSDF(const msgs::Light &_msg,
        sdf::ElementPtr _sdf = sdf::ElementPtr());

    /// \cond
    GAZEBO_VISIBLE
    const google::protobuf::FieldDescriptor *GetFD(
        google::protobuf::Message &message, const std::string &name);
    /// \endcond

    /// \brief Get the header from a protobuf message
    /// \param[in] _message A google protobuf message
    /// \return A pointer to the message's header
    GAZEBO_VISIBLE
    msgs::Header *GetHeader(google::protobuf::Message &_message);

    /// \}
  }
}

#endif
