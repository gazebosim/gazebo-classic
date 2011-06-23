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

#include "msgs/MessageTypes.hh"
#include "math/MathTypes.hh"

#include "common/Color.hh"
#include "math/Vector3.hh"
#include "common/Time.hh"
#include "math/Pose.hh"
#include "math/Plane.hh"
#include "math/Box.hh"

namespace gazebo
{
	namespace common
  {
    class XMLConfigNode;

    class Message
    {
      public:
        static void Init(google::protobuf::Message &message, 
                         const std::string &id);

        static void Stamp(msgs::Header *);
        static void Stamp(msgs::Time *);

        static std::string Package(const std::string &type, 
                                   const google::protobuf::Message &message);

        static msgs::Packet Package2(const std::string &type, 
                                     const google::protobuf::Message &message);

        static msgs::Point      Convert(const math::Vector3 &v);
        static msgs::Quaternion Convert(const math::Quatern &q);
        static msgs::Pose       Convert(const math::Pose &p);
        static msgs::Color      Convert(const Color &c);
        static msgs::Time       Convert(const Time &t);
        static msgs::Plane      Convert(const math::Plane &p);
  
        static math::Vector3          Convert(const msgs::Point &v);
        static math::Quatern          Convert(const msgs::Quaternion &q);
        static math::Pose           Convert(const msgs::Pose &p);
        static Color            Convert(const msgs::Color &c);
        static Time             Convert(const msgs::Time &t);
        static math::Plane            Convert(const msgs::Plane &p);
  
        static void Set(msgs::Point *pt, const math::Vector3 &v);
        static void Set(msgs::Quaternion *q, const math::Quatern &v);
        static void Set(msgs::Pose *p, const math::Pose &v);
        static void Set(msgs::Color *c, const Color &v);
        static void Set(msgs::Time *t, const Time &v);
        static void Set(msgs::Plane *p, const math::Plane &v);
  
        static msgs::Light      LightFromXML(XMLConfigNode *node);
        static msgs::Visual     VisualFromXML(XMLConfigNode *node);
        static msgs::Shadows    ShadowsFromXML(XMLConfigNode *node);
        static msgs::Fog     FogFromXML(XMLConfigNode *node);
        static msgs::Scene     SceneFromXML(XMLConfigNode *node);
  
      private:
  
        static const google::protobuf::FieldDescriptor *GetFD(google::protobuf::Message &message, const std::string &name);
  
        static msgs::Header *GetHeader(google::protobuf::Message &message);
    };
  }

}
#endif
