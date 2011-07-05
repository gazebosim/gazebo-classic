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
#ifndef MESSAGES_UTILITY_HH
#define MESSAGES_UTILITY_HH

#include "msgs/MessageTypes.hh"
#include "sdf/sdf.h"

#include "math/MathTypes.hh"
#include "math/Vector3.hh"
#include "math/Pose.hh"
#include "math/Plane.hh"
#include "math/Box.hh"

#include "common/Color.hh"
#include "common/Time.hh"

namespace gazebo
{
	namespace msgs
  {
    void Init(google::protobuf::Message &message, const std::string &id="");


    void Stamp(msgs::Header *);
    void Stamp(msgs::Time *);

    std::string Package(const std::string &type, 
        const google::protobuf::Message &message);

    msgs::Packet Package2(const std::string &type, 
        const google::protobuf::Message &message);

    msgs::Point      Convert(const math::Vector3 &v);
    msgs::Quaternion Convert(const math::Quaternion &q);
    msgs::Pose       Convert(const math::Pose &p);
    msgs::Color      Convert(const common::Color &c);
    msgs::Time       Convert(const common::Time &t);
    msgs::Plane      Convert(const math::Plane &p);

    math::Vector3    Convert(const msgs::Point &v);
    math::Quaternion Convert(const msgs::Quaternion &q);
    math::Pose       Convert(const msgs::Pose &p);
    common::Color    Convert(const msgs::Color &c);
    common::Time     Convert(const msgs::Time &t);
    math::Plane      Convert(const msgs::Plane &p);

    void Set(msgs::Point *pt, const math::Vector3 &v);
    void Set(msgs::Quaternion *q, const math::Quaternion &v);
    void Set(msgs::Pose *p, const math::Pose &v);
    void Set(msgs::Color *c, const common::Color &v);
    void Set(msgs::Time *t, const common::Time &v);
    void Set(msgs::Plane *p, const math::Plane &v);

    msgs::Light      LightFromSDF(sdf::ElementPtr _sdf);
    msgs::Visual     VisualFromSDF(sdf::ElementPtr _sdf);
    msgs::Shadows    ShadowsFromSDF(sdf::ElementPtr _sdf);
    msgs::Fog        FogFromSDF(sdf::ElementPtr _sdf);
    msgs::Scene      SceneFromSDF(sdf::ElementPtr _sdf);


    const google::protobuf::FieldDescriptor *GetFD(google::protobuf::Message &message, const std::string &name);

    msgs::Header *GetHeader(google::protobuf::Message &message);
  }
}

#endif
