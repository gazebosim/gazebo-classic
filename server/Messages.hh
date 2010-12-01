#ifndef MESSAGES_HH
#define MESSAGES_HH

#include <vector>

#include "Vector3.hh"
#include "Time.hh"
#include "Pose3d.hh"


namespace gazebo
{
  enum MessageType{ INSERT_MODEL_MSG, VISUAL_MSG, POSE_MSG };

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
    public: InsertModelMsg() : Message(INSERT_MODEL_MSG) {}
    public: InsertModelMsg(const InsertModelMsg &m) : Message(m), 
            xmlStr(m.xmlStr) {}
    public: virtual Message *Clone() const 
            { InsertModelMsg *msg = new InsertModelMsg(*this); return msg; }
    public: std::string xmlStr;
  };

  class VisualMsg : public Message
  {
    public: enum ActionType {UPDATE, DELETE};
    public: enum RenderType {MESH_RESOURCE, POINTS, LINE_LIST};

    public: VisualMsg() : Message(VISUAL_MSG) {}
    public: VisualMsg(const VisualMsg &m) : Message(m) {}
    public: virtual Message *Clone() const 
            { VisualMsg *msg = new VisualMsg(*this); return msg; }

    public: std::string parentId;
    public: std::string id;
    public: ActionType action;
    public: RenderType render;
    public: std::string mesh;
    public: std::string material;
    public: bool castShadows;
    public: bool attachAxes;
    public: bool visible;
    public: Vector3 boundingbox_min;
    public: Vector3 boundingbox_max;
    public: std::vector<Vector3> points;
  };

  class UpdatePoseMsg : public Message
  {
    public: UpdatePoseMsg() : Message(POSE_MSG), id(0) {}
    public: UpdatePoseMsg(const UpdatePoseMsg &m) : Message(m), 
            id(m.id), pose(m.pose) {}
    public: virtual Message *Clone() const 
            { UpdatePoseMsg *msg = new UpdatePoseMsg(*this); return msg; }

    public: std::string id;
    public: Pose3d pose;
  };
}

#endif
