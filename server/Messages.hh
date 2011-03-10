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
#ifndef MESSAGES_HH
#define MESSAGES_HH

#include "msgs/visual.pb.h"
#include "msgs/insert_model.pb.h"
#include "msgs/selection.pb.h"
#include "msgs/color.pb.h"
#include "msgs/pose.pb.h"
#include "msgs/point.pb.h"
#include "msgs/header.pb.h"
#include "msgs/light.pb.h"
#include "msgs/plane.pb.h"
#include "msgs/quaternion.pb.h"
#include "msgs/time.pb.h"

#include "XMLConfig.hh"
#include "Color.hh"
#include "Vector3.hh"
#include "Time.hh"
#include "Pose3d.hh"
#include "Plane.hh"
#include "Box.hh"

namespace gazebo
{
  class Message
  {
    public:
      static void Init(google::protobuf::Message &message, 
                       const std::string &id);

      static void CreationStamp(google::protobuf::Message &message);
      static void CreationStamp(msgs::Header &header);

      static void SendStamp(google::protobuf::Message &message);
      static void SendStamp(msgs::Header &header);

      static msgs::Point      Convert(const Vector3 &v);
      static msgs::Quaternion Convert(const Quatern &q);
      static msgs::Pose       Convert(const Pose3d &p);
      static msgs::Color      Convert(const Color &c);
      static msgs::Time       Convert(const Time &t);
      static msgs::Plane      Convert(const Plane &p);

      static Vector3          Convert(const msgs::Point &v);
      static Quatern          Convert(const msgs::Quaternion &q);
      static Pose3d           Convert(const msgs::Pose &p);
      static Color            Convert(const msgs::Color &c);
      static Time             Convert(const msgs::Time &t);
      static Plane            Convert(const msgs::Plane &p);

      static void Set(msgs::Point *pt, const Vector3 &v);
      static void Set(msgs::Quaternion *q, const Quatern &v);
      static void Set(msgs::Pose *p, const Pose3d &v);
      static void Set(msgs::Color *c, const Color &v);
      static void Set(msgs::Time *t, const Time &v);
      static void Set(msgs::Plane *p, const Plane &v);

      static msgs::Light      LightFromXML(XMLConfigNode *node);
      static msgs::Visual     VisualFromXML(XMLConfigNode *node);

    private:
      static void Stamp(msgs::Header &, const std::string &type);

      static const google::protobuf::FieldDescriptor *GetFD(google::protobuf::Message &message, const std::string &name);

      static msgs::Header *GetHeader(google::protobuf::Message &message);
  };
}

#endif
