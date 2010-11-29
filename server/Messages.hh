#ifndef MESSAGES_HH
#define MESSAGES_HH

#include "Time.hh"
#include "Pose3d.hh"

namespace gazebo
{
  enum MessageType{ INSERT_MODEL };

  class Message
  {
    public: Message(MessageType t) : type(t), stamp(Time::GetWallTime()) {}
    public: Message(const Message &m) : type(m.type), stamp(m.stamp) {}

    public: virtual Message *Clone() const { return new Message(*this); }

    public: MessageType type;
    public: Time stamp;
  };

  class InsertModelMsg : public Message
  {
    public: InsertModelMsg() : Message(INSERT_MODEL) {}
    public: InsertModelMsg(const InsertModelMsg &m) : Message(m), 
            xmlStr(m.xmlStr) {}
    public: virtual Message *Clone() const 
            { InsertModelMsg *msg = new InsertModelMsg(*this); return msg; }
    public: std::string xmlStr;
  };

  class InsertVisualMsg : public Message
  {
    public: InsertVisualMsg() : Message(INSERT_VISUAL) {}
    public: InsertVisualMsg(const InsertVisualMsg &m) : Message(m), 
            xmlStr(m.xmlStr) {}
    public: virtual Message *Clone() const 
            { InsertVisualMsg *msg = new InsertVisualMsg(*this); return msg; }

    public: std::string parentId;
    public: std::string id;
    public: std::string mesh;
    public: std::string material;
    public: bool castShadows;
    public: bool attachAxes;
    public: bool visible;
  };

  class UpdatePoseMsg : public Message
  {
    public: UpdatePoseMsg() : Message(UPDATE_POSE), id(0) {}
    public: UpdatePoseMsg(const UpdatePoseMsg &m) : Message(m), 
            id(m.id), pose(m.pose) {}
    public: virtual Message *Clone() const 
            { UpdatePoseMsg *msg = new UpdatePoseMsg(*this); return msg; }

    public: std::string id;
    public: Pose3d pose;
  };
}

#endif
