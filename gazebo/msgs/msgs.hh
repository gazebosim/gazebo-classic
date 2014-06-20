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

#include "gazebo/msgs/MessageTypes.hh"

#include "gazebo/math/MathTypes.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/math/Plane.hh"
#include "gazebo/math/Box.hh"

#include "gazebo/common/Color.hh"
#include "gazebo/common/Time.hh"

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
    msgs::Request *CreateRequest(const std::string &_request,
                                 const std::string &_data = "");

    /// \brief Initialize a message
    /// \param[in] _message Message to initialize
    /// \param[in] _id Optional string id
    void Init(google::protobuf::Message &_message, const std::string &_id ="");

    /// \brief Time stamp a header
    /// \param[in] _header Header to stamp
    void Stamp(msgs::Header *_header);

    /// \brief Set the time in a time message
    /// \param[in] _time A Time message
    void Stamp(msgs::Time *_time);

    /// \cond
    std::string Package(const std::string &type,
        const google::protobuf::Message &message);
    /// \endcond

    /// \brief Convert a math::Vector3 to a msgs::Vector3d
    /// \param[in] _v The vector to convert
    /// \return A msgs::Vector3d object
    msgs::Vector3d      Convert(const math::Vector3 &_v);

    /// \brief Convert a math::Quaternion to a msgs::Quaternion
    /// \param[in] _q The quaternion to convert
    /// \return A msgs::Quaternion object
    msgs::Quaternion Convert(const math::Quaternion &_q);

    /// \brief Convert a math::Pose to a msgs::Pose
    /// \param[in] _p The pose to convert
    /// \return A msgs::Pose object
    msgs::Pose       Convert(const math::Pose &_p);

    /// \brief Convert a common::Color to a msgs::Color
    /// \param[in] _c The color to convert
    /// \return A msgs::Color object
    msgs::Color      Convert(const common::Color &_c);

    /// \brief Convert a common::Time to a msgs::Time
    /// \param[in] _t The time to convert
    /// \return A msgs::Time object
    msgs::Time       Convert(const common::Time &_t);

    /// \brief Convert a math::Plane to a msgs::PlaneGeom
    /// \param[in] _p The plane to convert
    /// \return A msgs::PlaneGeom object
    msgs::PlaneGeom Convert(const math::Plane &_p);

    /// \brief Convert a msgs::Vector3d to a math::Vector
    /// \param[in] _v The plane to convert
    /// \return A math::Vector3 object
    math::Vector3    Convert(const msgs::Vector3d &_v);

    /// \brief Convert a msgs::Quaternion to a math::Quaternion
    /// \param[in] _q The quaternion to convert
    /// \return A math::Quaternion object
    math::Quaternion Convert(const msgs::Quaternion &_q);

    /// \brief Convert a msgs::Pose to a math::Pose
    /// \param[in] _q The pose to convert
    /// \return A math::Pose object
    math::Pose       Convert(const msgs::Pose &_p);

    /// \brief Convert a msgs::Image to a common::Image
    /// \param[out] _img The common::Image container
    /// \param[in] _msg The Image message to convert
    void Set(common::Image &_img, const msgs::Image &_msg);

    /// \brief Convert a msgs::Color to a common::Color
    /// \param[in] _c The color to convert
    /// \return A common::Color object
    common::Color    Convert(const msgs::Color &_c);

    /// \brief Convert a msgs::Time to a common::Time
    /// \param[in] _t The time to convert
    /// \return A common::Time object
    common::Time     Convert(const msgs::Time &_t);

    /// \brief Convert a msgs::PlaneGeom to a common::Plane
    /// \param[in] _p The plane to convert
    /// \return A common::Plane object
    math::Plane      Convert(const msgs::PlaneGeom &_p);

    /// \brief Set a msgs::Image from a common::Image
    /// \param[out] _msg A msgs::Image pointer
    /// \param[in] _i A common::Image reference
    void Set(msgs::Image *_msg, const common::Image &_i);

    /// \brief Set a msgs::Vector3d from a math::Vector3
    /// \param[out] _pt A msgs::Vector3d pointer
    /// \param[in] _v A math::Vector3 reference
    void Set(msgs::Vector3d *_pt, const math::Vector3 &_v);

    /// \brief Set a msgs::Vector2d from a math::Vector3
    /// \param[out] _pt A msgs::Vector2d pointer
    /// \param[in] _v A math::Vector2d reference
    void Set(msgs::Vector2d *_pt, const math::Vector2d &_v);

    /// \brief Set a msgs::Quaternion from a math::Quaternion
    /// \param[out] _q A msgs::Quaternion pointer
    /// \param[in] _v A math::Quaternion reference
    void Set(msgs::Quaternion *_q, const math::Quaternion &_v);

    /// \brief Set a msgs::Pose from a math::Pose
    /// \param[out] _p A msgs::Pose pointer
    /// \param[in] _v A math::Pose reference
    void Set(msgs::Pose *_p, const math::Pose &_v);

    /// \brief Set a msgs::Color from a common::Color
    /// \param[out] _p A msgs::Color pointer
    /// \param[in] _v A common::Color reference
    void Set(msgs::Color *_c, const common::Color &_v);

    /// \brief Set a msgs::Time from a common::Time
    /// \param[out] _p A msgs::Time pointer
    /// \param[in] _v A common::Time reference
    void Set(msgs::Time *_t, const common::Time &_v);

    /// \brief Set a msgs::Plane from a math::Plane
    /// \param[out] _p A msgs::Plane pointer
    /// \param[in] _v A math::Plane reference
    void Set(msgs::PlaneGeom *_p, const math::Plane &_v);

    /// \brief Create a msgs::TrackVisual from a track visual SDF element
    /// \param[in] _sdf The sdf element
    /// \return The new msgs::TrackVisual object
    msgs::TrackVisual TrackVisualFromSDF(sdf::ElementPtr _sdf);

    /// \brief Create a msgs::GUI from a GUI SDF element
    /// \param[in] _sdf The sdf element
    /// \return The new msgs::GUI object
    msgs::GUI GUIFromSDF(sdf::ElementPtr _sdf);

    /// \brief Create a msgs::Light from a light SDF element
    /// \param[in] _sdf The sdf element
    /// \return The new msgs::Light object
    msgs::Light LightFromSDF(sdf::ElementPtr _sdf);

    /// \brief Create a msgs::MeshGeom from a mesh SDF element
    /// \param[in] _sdf The sdf element
    /// \return The new msgs::MeshGeom object
    msgs::MeshGeom MeshFromSDF(sdf::ElementPtr _sdf);

    /// \brief Create a msgs::Geometry from a geometry SDF element
    /// \param[in] _sdf The sdf element
    /// \return The new msgs::Geometry object
    msgs::Geometry GeometryFromSDF(sdf::ElementPtr _sdf);

    /// \brief Create a msgs::Visual from a visual SDF element
    /// \param[in] _sdf The sdf element
    /// \return The new msgs::Visual object
    msgs::Visual VisualFromSDF(sdf::ElementPtr _sdf);

    /// \brief Create a msgs::Fog from a fog SDF element
    /// \param[in] _sdf The sdf element
    /// \return The new msgs::Fog object
    msgs::Fog FogFromSDF(sdf::ElementPtr _sdf);

    /// \brief Create a msgs::Scene from a scene SDF element
    /// \param[in] _sdf The sdf element
    /// \return The new msgs::Scene object
    msgs::Scene SceneFromSDF(sdf::ElementPtr _sdf);

    /// \cond
    const google::protobuf::FieldDescriptor *GetFD(
        google::protobuf::Message &message, const std::string &name);
    /// \endcond

    /// \brief Get the header from a protobuf message
    /// \param[in] _message A google protobuf message
    /// \return A pointer to the message's header
    msgs::Header *GetHeader(google::protobuf::Message &_message);

    /// \}
  }
}

#endif

