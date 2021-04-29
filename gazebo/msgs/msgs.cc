/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <google/protobuf/descriptor.h>
#include <algorithm>
#include <ignition/math/MassMatrix3.hh>
#include <ignition/math/Rand.hh>

#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/Image.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/msgs/msgs.hh"

namespace gazebo
{
  namespace msgs
  {
    /// \internal
    /// \brief Internal function to create an SDF element from msgs::Axis.
    /// It is only intended to be used by JointToSDF.
    /// \param[in] _msg The msgs::Axis object.
    /// \param[in] _sdf sdf::ElementPtr to fill with data.
    void AxisToSDF(const msgs::Axis &_msg, sdf::ElementPtr _sdf);

    /////////////////////////////////////////////
    /// Create a request message
    msgs::Request *CreateRequest(const std::string &_request,
        const std::string &_data)
    {
      msgs::Request *request = new msgs::Request;

      request->set_request(_request);
      request->set_data(_data);
      request->set_id(ignition::math::Rand::IntUniform(1, 10000));

      return request;
    }

    /////////////////////////////////////////////
    const google::protobuf::FieldDescriptor *GetFD(
        google::protobuf::Message &message, const std::string &name)
    {
      return message.GetDescriptor()->FindFieldByName(name);
    }

    /////////////////////////////////////////////
    msgs::Header *GetHeader(google::protobuf::Message &message)
    {
      google::protobuf::Message *msg = NULL;

      if (GetFD(message, "str_id"))
        msg = &message;
      else
      {
        const google::protobuf::FieldDescriptor *fd;
        fd = GetFD(message, "header");

        if (fd)
          msg = message.GetReflection()->MutableMessage(&message, fd);
      }

      return (msgs::Header*)msg;
    }

    /////////////////////////////////////////////
    void Init(google::protobuf::Message &_message, const std::string &_id)
    {
      msgs::Header *header = GetHeader(_message);

      if (header)
      {
        if (!_id.empty())
          header->set_str_id(_id);
        Stamp(header->mutable_stamp());
      }
    }

    /////////////////////////////////////////////
    void Stamp(msgs::Header *_hdr)
    {
      Stamp(_hdr->mutable_stamp());
    }

    /////////////////////////////////////////////
    void Stamp(msgs::Time *_time)
    {
      common::Time tm = common::Time::GetWallTime();

      _time->set_sec(tm.sec);
      _time->set_nsec(tm.nsec);
    }

    /////////////////////////////////////////////
    std::string Package(const std::string &type,
        const google::protobuf::Message &message)
    {
      std::string data;
      msgs::Packet pkg;

      Stamp(pkg.mutable_stamp());
      pkg.set_type(type);

      std::string *serialized_data = pkg.mutable_serialized_data();
      if (!message.IsInitialized())
        gzthrow("Can't serialize message of type[" + message.GetTypeName() +
            "] because it is missing required fields");

      if (!message.SerializeToString(serialized_data))
        gzthrow("Failed to serialized message");

      if (!pkg.SerializeToString(&data))
        gzthrow("Failed to serialized message");

      return data;
    }

    /////////////////////////////////////////////
    void Set(msgs::Vector3d *_pt, const ignition::math::Vector3d &_v)
    {
      _pt->set_x(_v.X());
      _pt->set_y(_v.Y());
      _pt->set_z(_v.Z());
    }

    /////////////////////////////////////////////
    void Set(msgs::Vector2d *_pt, const ignition::math::Vector2d &_v)
    {
      _pt->set_x(_v.X());
      _pt->set_y(_v.Y());
    }

    /////////////////////////////////////////////
    void Set(msgs::Quaternion *_q, const ignition::math::Quaterniond &_v)
    {
      _q->set_x(_v.X());
      _q->set_y(_v.Y());
      _q->set_z(_v.Z());
      _q->set_w(_v.W());
    }

    /////////////////////////////////////////////
    void Set(msgs::Pose *_p, const ignition::math::Pose3d &_v)
    {
      Set(_p->mutable_position(), _v.Pos());
      Set(_p->mutable_orientation(), _v.Rot());
    }

    /////////////////////////////////////////////
    void Set(msgs::Color *_c, const ignition::math::Color &_v)
    {
      _c->set_r(_v.R());
      _c->set_g(_v.G());
      _c->set_b(_v.B());
      _c->set_a(_v.A());
    }

    /////////////////////////////////////////////
    void Set(msgs::Time *_t, const common::Time &_v)
    {
      _t->set_sec(_v.sec);
      _t->set_nsec(_v.nsec);
    }

    /////////////////////////////////////////////////
    void Set(msgs::SphericalCoordinates *_s,
             const common::SphericalCoordinates &_v)
    {
      switch (_v.GetSurfaceType())
      {
        case common::SphericalCoordinates::EARTH_WGS84:
          _s->set_surface_model(msgs::SphericalCoordinates::EARTH_WGS84);
          break;
        default:
          gzerr << "Unable to map surface type[" <<  _v.GetSurfaceType()
            << "] to a SphericalCoordinates message.\n";
          _s->set_surface_model(msgs::SphericalCoordinates::EARTH_WGS84);
          break;
      };

      _s->set_latitude_deg(_v.LatitudeReference().Degree());
      _s->set_longitude_deg(_v.LongitudeReference().Degree());
      _s->set_heading_deg(_v.HeadingOffset().Degree());
      _s->set_elevation(_v.GetElevationReference());
    }

    /////////////////////////////////////////////////
    void Set(msgs::Inertial *_i, const ignition::math::MassMatrix3d &_m)
    {
      _i->set_mass(_m.Mass());
      _i->set_ixx(_m.Ixx());
      _i->set_iyy(_m.Iyy());
      _i->set_izz(_m.Izz());
      _i->set_ixy(_m.Ixy());
      _i->set_ixz(_m.Ixz());
      _i->set_iyz(_m.Iyz());
    }

    /////////////////////////////////////////////////
    void Set(msgs::Inertial *_i, const ignition::math::Inertiald &_m)
    {
      msgs::Set(_i, _m.MassMatrix());
      msgs::Set(_i->mutable_pose(), _m.Pose());
    }

    /////////////////////////////////////////////////
    void Set(msgs::PlaneGeom *_p, const ignition::math::Planed &_v)
    {
      Set(_p->mutable_normal(), _v.Normal());
      _p->mutable_size()->set_x(_v.Size().X());
      _p->mutable_size()->set_y(_v.Size().Y());
      _p->set_d(_v.Offset());
    }

    /////////////////////////////////////////////////
    void Set(common::Image &_img, const msgs::Image &_msg)
    {
      _img.SetFromData(
          (const unsigned char*)_msg.data().data(),
          _msg.width(),
          _msg.height(),
          (common::Image::PixelFormat)(_msg.pixel_format()));
    }

    /////////////////////////////////////////////////
    void Set(msgs::Image *_msg, const common::Image &_i)
    {
      _msg->set_width(_i.GetWidth());
      _msg->set_height(_i.GetHeight());
      _msg->set_pixel_format(_i.GetPixelFormat());
      _msg->set_step(_i.GetPitch());

      unsigned char *data = NULL;
      unsigned int size;
      _i.GetData(&data, size);
      _msg->set_data(data, size);
      if (data)
      {
        delete[] data;
      }
    }

    /////////////////////////////////////////////////
    msgs::Any ConvertAny(const double _d)
    {
      msgs::Any result;
      result.set_type(msgs::Any::DOUBLE);
      result.set_double_value(_d);
      return result;
    }

    /////////////////////////////////////////////////
    msgs::Any ConvertAny(const int _i)
    {
      msgs::Any result;
      result.set_type(msgs::Any::INT32);
      result.set_int_value(_i);
      return result;
    }

    /////////////////////////////////////////////////
    msgs::Any ConvertAny(const std::string &_s)
    {
      msgs::Any result;
      result.set_type(msgs::Any::STRING);
      result.set_string_value(_s);
      return result;
    }

    /////////////////////////////////////////////////
    msgs::Any ConvertAny(const char *_s)
    {
      msgs::Any result;
      result.set_type(msgs::Any::STRING);
      result.set_string_value(std::string(_s));
      return result;
    }

    /////////////////////////////////////////////////
    msgs::Any ConvertAny(const bool _b)
    {
      msgs::Any result;
      result.set_type(msgs::Any::BOOLEAN);
      result.set_bool_value(_b);
      return result;
    }

    /////////////////////////////////////////////////
    msgs::Any ConvertAny(const ignition::math::Vector3d &_v)
    {
      msgs::Any result;
      result.set_type(msgs::Any::VECTOR3D);
      result.mutable_vector3d_value()->CopyFrom(Convert(_v));
      return result;
    }

    /////////////////////////////////////////////////
    msgs::Any ConvertAny(const ignition::math::Color &_c)
    {
      msgs::Any result;
      result.set_type(msgs::Any::COLOR);
      result.mutable_color_value()->CopyFrom(Convert(_c));
      return result;
    }

    /////////////////////////////////////////////////
    msgs::Any ConvertAny(const ignition::math::Pose3d &_p)
    {
      msgs::Any result;
      result.set_type(msgs::Any::POSE3D);
      result.mutable_pose3d_value()->CopyFrom(Convert(_p));
      return result;
    }

    /////////////////////////////////////////////////
    msgs::Any ConvertAny(const ignition::math::Quaterniond &_q)
    {
      msgs::Any result;
      result.set_type(msgs::Any::QUATERNIOND);
      result.mutable_quaternion_value()->CopyFrom(Convert(_q));
      return result;
    }

    /////////////////////////////////////////////////
    msgs::Any ConvertAny(const common::Time &_t)
    {
      msgs::Any result;
      result.set_type(msgs::Any::TIME);
      result.mutable_time_value()->CopyFrom(Convert(_t));
      return result;
    }

    /////////////////////////////////////////////////
    msgs::Vector3d Convert(const ignition::math::Vector3d &_v)
    {
      msgs::Vector3d result;
      result.set_x(_v.X());
      result.set_y(_v.Y());
      result.set_z(_v.Z());
      return result;
    }

    /////////////////////////////////////////////////
    msgs::Vector2d Convert(const ignition::math::Vector2d &_v)
    {
      msgs::Vector2d result;
      result.set_x(_v.X());
      result.set_y(_v.Y());
      return result;
    }

    /////////////////////////////////////////////
    msgs::Quaternion Convert(const ignition::math::Quaterniond &_q)
    {
      msgs::Quaternion result;
      result.set_x(_q.X());
      result.set_y(_q.Y());
      result.set_z(_q.Z());
      result.set_w(_q.W());
      return result;
    }

    /////////////////////////////////////////////
    msgs::Pose Convert(const ignition::math::Pose3d &_p)
    {
      msgs::Pose result;
      result.mutable_position()->CopyFrom(Convert(_p.Pos()));
      result.mutable_orientation()->CopyFrom(Convert(_p.Rot()));
      return result;
    }

    /////////////////////////////////////////////
    msgs::Color Convert(const ignition::math::Color &_c)
    {
      msgs::Color result;
      result.set_r(_c.R());
      result.set_g(_c.G());
      result.set_b(_c.B());
      result.set_a(_c.A());
      return result;
    }

    /////////////////////////////////////////////
    msgs::Time Convert(const common::Time &_t)
    {
      msgs::Time result;
      result.set_sec(_t.sec);
      result.set_nsec(_t.nsec);
      return result;
    }

    /////////////////////////////////////////////
    msgs::Inertial Convert(const ignition::math::Inertiald &_i)
    {
      msgs::Inertial result;
      msgs::Set(&result, _i);
      return result;
    }

    /////////////////////////////////////////////
    msgs::Inertial Convert(const ignition::math::MassMatrix3d &_m)
    {
      msgs::Inertial result;
      msgs::Set(&result, _m);
      return result;
    }

    /////////////////////////////////////////////
    msgs::PlaneGeom Convert(const ignition::math::Planed &_p)
    {
      msgs::PlaneGeom result;
      result.mutable_normal()->CopyFrom(Convert(_p.Normal()));
      result.mutable_size()->set_x(_p.Size().X());
      result.mutable_size()->set_y(_p.Size().Y());
      result.set_d(_p.Offset());
      return result;
    }

    /////////////////////////////////////////////
    msgs::Joint::Type ConvertJointType(const std::string &_str)
    {
      msgs::Joint::Type result = msgs::Joint::REVOLUTE;
      if (_str == "revolute")
      {
        result = msgs::Joint::REVOLUTE;
      }
      else if (_str == "revolute2")
      {
        result = msgs::Joint::REVOLUTE2;
      }
      else if (_str == "prismatic")
      {
        result = msgs::Joint::PRISMATIC;
      }
      else if (_str == "universal")
      {
        result = msgs::Joint::UNIVERSAL;
      }
      else if (_str == "ball")
      {
        result = msgs::Joint::BALL;
      }
      else if (_str == "screw")
      {
        result = msgs::Joint::SCREW;
      }
      else if (_str == "gearbox")
      {
        result = msgs::Joint::GEARBOX;
      }
      else if (_str == "fixed")
      {
        result = msgs::Joint::FIXED;
      }
      else
      {
        gzerr << "Unrecognized JointType ["
              << _str
              << "], returning REVOLUTE"
              << std::endl;
      }
      return result;
    }

    /////////////////////////////////////////////
    std::string ConvertJointType(const msgs::Joint::Type &_type)
    {
      std::string result;
      switch (_type)
      {
        case msgs::Joint::REVOLUTE:
        {
          result = "revolute";
          break;
        }
        case msgs::Joint::REVOLUTE2:
        {
          result = "revolute2";
          break;
        }
        case msgs::Joint::PRISMATIC:
        {
          result = "prismatic";
          break;
        }
        case msgs::Joint::UNIVERSAL:
        {
          result = "universal";
          break;
        }
        case msgs::Joint::BALL:
        {
          result = "ball";
          break;
        }
        case msgs::Joint::SCREW:
        {
          result = "screw";
          break;
        }
        case msgs::Joint::GEARBOX:
        {
          result = "gearbox";
          break;
        }
        case msgs::Joint::FIXED:
        {
          result = "fixed";
          break;
        }
        default:
        {
          result = "unknown";
          gzerr << "Unrecognized JointType [" << _type << "]"
                << std::endl;
          break;
        }
      }
      return result;
    }

    /////////////////////////////////////////////////
    msgs::Geometry::Type ConvertGeometryType(const std::string &_str)
    {
      msgs::Geometry::Type result = msgs::Geometry::BOX;
      if (_str == "box")
      {
        result = msgs::Geometry::BOX;
      }
      else if (_str == "cylinder")
      {
        result = msgs::Geometry::CYLINDER;
      }
      else if (_str == "sphere")
      {
        result = msgs::Geometry::SPHERE;
      }
      else if (_str == "plane")
      {
        result = msgs::Geometry::PLANE;
      }
      else if (_str == "image")
      {
        result = msgs::Geometry::IMAGE;
      }
      else if (_str == "heightmap")
      {
        result = msgs::Geometry::HEIGHTMAP;
      }
      else if (_str == "mesh")
      {
        result = msgs::Geometry::MESH;
      }
      else if (_str == "polyline")
      {
        result = msgs::Geometry::POLYLINE;
      }
      else
      {
        gzwarn << "Geometry: '" << _str << "' is not recognized, "
            << " returning type as msgs::Geometry::BOX." << std::endl;
      }

      return result;
    }

    /////////////////////////////////////////////////
    std::string ConvertGeometryType(const msgs::Geometry::Type _type)
    {
      std::string result;
      switch (_type)
      {
        case msgs::Geometry::BOX:
        {
          result = "box";
          break;
        }
        case msgs::Geometry::CYLINDER:
        {
          result = "cylinder";
          break;
        }
        case msgs::Geometry::SPHERE:
        {
          result = "sphere";
          break;
        }
        case msgs::Geometry::PLANE:
        {
          result = "plane";
          break;
        }
        case msgs::Geometry::IMAGE:
        {
          result = "image";
          break;
        }
        case msgs::Geometry::HEIGHTMAP:
        {
          result = "heightmap";
          break;
        }
        case msgs::Geometry::MESH:
        {
          result = "mesh";
          break;
        }
        case msgs::Geometry::POLYLINE:
        {
          result = "polyline";
          break;
        }
        default:
        {
          result = "unknown";
          break;
        }
      }
      return result;
    }

    /////////////////////////////////////////////
    ignition::math::Vector3d ConvertIgn(const msgs::Vector3d &_v)
    {
      return ignition::math::Vector3d(_v.x(), _v.y(), _v.z());
    }

    /////////////////////////////////////////////
    ignition::math::Vector2d ConvertIgn(const msgs::Vector2d &_v)
    {
      return ignition::math::Vector2d(_v.x(), _v.y());
    }

    /////////////////////////////////////////////
    ignition::math::Quaterniond ConvertIgn(const msgs::Quaternion &_q)
    {
      return ignition::math::Quaterniond(_q.w(), _q.x(), _q.y(), _q.z());
    }

    /////////////////////////////////////////////
    ignition::math::Pose3d ConvertIgn(const msgs::Pose &_p)
    {
      return ignition::math::Pose3d(ConvertIgn(_p.position()),
                                    ConvertIgn(_p.orientation()));
    }

    /////////////////////////////////////////////
    ignition::math::Inertiald Convert(const msgs::Inertial &_i)
    {
      auto pose = msgs::ConvertIgn(_i.pose());
      return ignition::math::Inertiald(
        ignition::math::MassMatrix3d(
          _i.mass(),
          ignition::math::Vector3d(_i.ixx(), _i.iyy(), _i.izz()),
          ignition::math::Vector3d(_i.ixy(), _i.ixz(), _i.iyz())),
        pose);
    }

    /////////////////////////////////////////////
    ignition::math::Color Convert(const msgs::Color &_c)
    {
      return ignition::math::Color(_c.r(), _c.g(), _c.b(), _c.a());
    }

    /////////////////////////////////////////////
    common::Time Convert(const msgs::Time &_t)
    {
      return common::Time(_t.sec(), _t.nsec());
    }

    /////////////////////////////////////////////
    ignition::math::Planed ConvertIgn(const msgs::PlaneGeom &_p)
    {
      return ignition::math::Planed(ConvertIgn(_p.normal()),
          ignition::math::Vector2d(_p.size().x(), _p.size().y()),
          _p.d());
    }

    /////////////////////////////////////////////
    msgs::GUI GUIFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::GUI result;

      result.set_fullscreen(_sdf->Get<bool>("fullscreen"));

      // Set gui plugins
      if (_sdf->HasElement("plugin"))
      {
        sdf::ElementPtr pluginElem = _sdf->GetElement("plugin");
        while (pluginElem)
        {
          msgs::Plugin *pluginMsg = result.add_plugin();
          pluginMsg->CopyFrom(PluginFromSDF(pluginElem));
          pluginElem = pluginElem->GetNextElement("plugin");
        }
      }

      if (_sdf->HasElement("camera"))
      {
        sdf::ElementPtr camSDF = _sdf->GetElement("camera");
        msgs::GUICamera *guiCam = result.mutable_camera();

        guiCam->set_name(camSDF->Get<std::string>("name"));

        if (camSDF->HasElement("pose"))
        {
          msgs::Set(guiCam->mutable_pose(),
              camSDF->Get<ignition::math::Pose3d>("pose"));
        }

        if (camSDF->HasElement("view_controller"))
        {
          guiCam->set_view_controller(
              camSDF->Get<std::string>("view_controller"));
        }

        if (camSDF->HasElement("projection_type"))
        {
          guiCam->set_projection_type(
              camSDF->Get<std::string>("projection_type"));
        }

        if (camSDF->HasElement("track_visual"))
        {
          guiCam->mutable_track()->CopyFrom(
              TrackVisualFromSDF(camSDF->GetElement("track_visual")));
        }
      }

      return result;
    }

    /////////////////////////////////////////////
    msgs::Plugin PluginFromSDF(const sdf::ElementPtr _sdf)
    {
      msgs::Plugin result;

      result.set_name(_sdf->Get<std::string>("name"));
      result.set_filename(_sdf->Get<std::string>("filename"));

      std::stringstream ss;
      for (sdf::ElementPtr innerElem = _sdf->GetFirstElement();
          innerElem; innerElem = innerElem->GetNextElement(""))
      {
        ss << innerElem->ToString("");
      }
      result.set_innerxml(ss.str());

      return result;
    }

    /////////////////////////////////////////////////
    msgs::TrackVisual TrackVisualFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::TrackVisual result;

      result.set_name(_sdf->Get<std::string>("name"));

      if (_sdf->HasElement("min_dist"))
        result.set_min_dist(_sdf->GetElement("min_dist")->Get<double>());

      if (_sdf->HasElement("max_dist"))
        result.set_max_dist(_sdf->GetElement("max_dist")->Get<double>());

      if (_sdf->HasElement("static"))
        result.set_static_(_sdf->Get<bool>("static"));

      if (_sdf->HasElement("use_model_frame"))
        result.set_use_model_frame(_sdf->Get<bool>("use_model_frame"));

      if (_sdf->HasElement("inherit_yaw"))
        result.set_inherit_yaw(_sdf->Get<bool>("inherit_yaw"));

      if (_sdf->HasElement("xyz"))
      {
        msgs::Set(result.mutable_xyz(),
            _sdf->Get<ignition::math::Vector3d>("xyz"));
      }

      return result;
    }


    /////////////////////////////////////////////////
    msgs::Light LightFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::Light result;

      std::string type = _sdf->Get<std::string>("type");
      std::transform(type.begin(), type.end(), type.begin(), ::tolower);

      result.set_name(_sdf->Get<std::string>("name"));

      result.set_cast_shadows(_sdf->Get<bool>("cast_shadows"));

      if (type == "point")
        result.set_type(msgs::Light::POINT);
      else if (type == "spot")
        result.set_type(msgs::Light::SPOT);
      else if (type == "directional")
        result.set_type(msgs::Light::DIRECTIONAL);

      if (_sdf->HasElement("pose"))
      {
        result.mutable_pose()->CopyFrom(
            Convert(_sdf->Get<ignition::math::Pose3d>("pose")));
      }

      if (_sdf->HasElement("diffuse"))
      {
        result.mutable_diffuse()->CopyFrom(
            Convert(_sdf->Get<ignition::math::Color>("diffuse")));
      }

      if (_sdf->HasElement("specular"))
      {
        result.mutable_specular()->CopyFrom(
            Convert(_sdf->Get<ignition::math::Color>("specular")));
      }

      if (_sdf->HasElement("attenuation"))
      {
        sdf::ElementPtr elem = _sdf->GetElement("attenuation");
        result.set_attenuation_constant(elem->Get<double>("constant"));
        result.set_attenuation_linear(elem->Get<double>("linear"));
        result.set_attenuation_quadratic(elem->Get<double>("quadratic"));
        result.set_range(elem->Get<double>("range"));
      }

      if (_sdf->HasElement("direction"))
      {
        result.mutable_direction()->CopyFrom(
            Convert(_sdf->Get<ignition::math::Vector3d>("direction")));
      }

      if (_sdf->HasElement("spot"))
      {
        sdf::ElementPtr elem = _sdf->GetElement("spot");
        result.set_spot_inner_angle(elem->Get<double>("inner_angle"));
        result.set_spot_outer_angle(elem->Get<double>("outer_angle"));
        result.set_spot_falloff(elem->Get<double>("falloff"));
      }

      return result;
    }

    /////////////////////////////////////////////////
    msgs::MeshGeom MeshFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::MeshGeom result;

      if (_sdf->GetName() != "mesh")
      {
        gzerr << "Cannot create a mesh message from an "
          << _sdf->GetName() << " SDF element.\n";
        return result;
      }

        msgs::Set(result.mutable_scale(),
            _sdf->Get<ignition::math::Vector3d>("scale"));

        result.set_filename(common::asFullPath(_sdf->Get<std::string>("uri"),
            _sdf->FilePath()));

        if (_sdf->HasElement("submesh"))
        {
          sdf::ElementPtr submeshElem = _sdf->GetElement("submesh");
          if (submeshElem->HasElement("name") &&
              submeshElem->Get<std::string>("name") != "__default__")
          {
            result.set_submesh(submeshElem->Get<std::string>("name"));

            if (submeshElem->HasElement("center"))
              result.set_center_submesh(submeshElem->Get<bool>("center"));
          }
        }

      return result;
    }

    /////////////////////////////////////////////////
    msgs::Geometry GeometryFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::Geometry result;

      if (_sdf->GetName() != "geometry")
      {
        gzerr << "Cannot create a geometry message from an "
          << _sdf->GetName() << " SDF element.\n";
        return result;
      }

      // Load the geometry
      sdf::ElementPtr geomElem = _sdf->GetFirstElement();
      if (!geomElem)
        gzthrow("Invalid geometry element");

      if (geomElem->GetName() == "box")
      {
        result.set_type(msgs::Geometry::BOX);
        msgs::Set(result.mutable_box()->mutable_size(),
            geomElem->Get<ignition::math::Vector3d>("size"));
      }
      else if (geomElem->GetName() == "cylinder")
      {
        result.set_type(msgs::Geometry::CYLINDER);
        result.mutable_cylinder()->set_radius(
            geomElem->Get<double>("radius"));
        result.mutable_cylinder()->set_length(
            geomElem->Get<double>("length"));
      }
      else if (geomElem->GetName() == "sphere")
      {
        result.set_type(msgs::Geometry::SPHERE);
        result.mutable_sphere()->set_radius(
            geomElem->Get<double>("radius"));
      }
      else if (geomElem->GetName() == "plane")
      {
        result.set_type(msgs::Geometry::PLANE);
        msgs::Set(result.mutable_plane()->mutable_normal(),
            geomElem->Get<ignition::math::Vector3d>("normal"));
        msgs::Set(result.mutable_plane()->mutable_size(),
            geomElem->Get<ignition::math::Vector2d>("size"));
      }
      else if (geomElem->GetName() == "polyline")
      {
        sdf::ElementPtr polylineElem = geomElem;
        result.set_type(msgs::Geometry::POLYLINE);
        while (polylineElem)
        {
          msgs::Polyline *polylineMsg = result.add_polyline();
          polylineMsg->set_height(polylineElem->Get<double>("height"));
          sdf::ElementPtr pointElem = polylineElem->GetElement("point");
          while (pointElem)
          {
             ignition::math::Vector2d point =
               pointElem->Get<ignition::math::Vector2d>();
             pointElem = pointElem->GetNextElement("point");
             msgs::Vector2d *ptMsg = polylineMsg->add_point();
             msgs::Set(ptMsg, point);
          }
          polylineElem = polylineElem->GetNextElement("polyline");
        }
      }
      else if (geomElem->GetName() == "image")
      {
        result.set_type(msgs::Geometry::IMAGE);
        result.mutable_image()->set_scale(
            geomElem->Get<double>("scale"));
        result.mutable_image()->set_height(
            geomElem->Get<double>("height"));
        result.mutable_image()->set_uri(
            geomElem->Get<std::string>("uri"));
      }
      else if (geomElem->GetName() == "heightmap")
      {
        result.set_type(msgs::Geometry::HEIGHTMAP);

        // We do not want to set the size field to be the default values of
        // [1, 1, 1] if not specified (size is optional for DEMs). So mark it as
        // zero for now.
        // TODO remove the required rule in heightmapgeom.proto's size field
        ignition::math::Vector3d size;
        if (geomElem->HasElement("size"))
          size =  geomElem->Get<ignition::math::Vector3d>("size");
        msgs::Set(result.mutable_heightmap()->mutable_size(), size);

        msgs::Set(result.mutable_heightmap()->mutable_origin(),
            geomElem->Get<ignition::math::Vector3d>("pos"));

        if (geomElem->HasElement("sampling"))
        {
          result.mutable_heightmap()->set_sampling(
              geomElem->Get<unsigned int>("sampling"));
        }

        sdf::ElementPtr textureElem = geomElem->GetElement("texture");
        while (textureElem)
        {
          msgs::HeightmapGeom::Texture *tex =
            result.mutable_heightmap()->add_texture();
          tex->set_diffuse(textureElem->Get<std::string>("diffuse"));
          tex->set_normal(textureElem->Get<std::string>("normal"));
          tex->set_size(textureElem->Get<double>("size"));
          textureElem = textureElem->GetNextElement("texture");
        }

        sdf::ElementPtr blendElem = geomElem->GetElement("blend");
        while (blendElem)
        {
          msgs::HeightmapGeom::Blend *blend =
            result.mutable_heightmap()->add_blend();

          blend->set_min_height(blendElem->Get<double>("min_height"));
          blend->set_fade_dist(blendElem->Get<double>("fade_dist"));
          blendElem = blendElem->GetNextElement("blend");
        }

        // Set if the rendering engine uses terrain paging
        bool useTerrainPaging =
            geomElem->Get<bool>("use_terrain_paging");
        result.mutable_heightmap()->set_use_terrain_paging(useTerrainPaging);
        result.mutable_heightmap()->set_filename(
            geomElem->Get<std::string>("uri"));
      }
      else if (geomElem->GetName() == "mesh")
      {
        result.set_type(msgs::Geometry::MESH);
        result.mutable_mesh()->CopyFrom(MeshFromSDF(geomElem));
      }
      else if (geomElem->GetName() == "empty")
      {
        result.set_type(msgs::Geometry::EMPTY);
      }
      else
        gzthrow("Unknown geometry type\n");

      return result;
    }

    /////////////////////////////////////////////////
    msgs::Visual VisualFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::Visual result;

      result.set_name(_sdf->Get<std::string>("name"));

      if (_sdf->HasElement("cast_shadows"))
        result.set_cast_shadows(_sdf->Get<bool>("cast_shadows"));

      if (_sdf->HasElement("transparency"))
        result.set_transparency(_sdf->Get<double>("transparency"));

      if (_sdf->HasElement("laser_retro"))
        result.set_laser_retro(_sdf->Get<double>("laser_retro"));

      // Set the meta information
      if (_sdf->HasElement("meta"))
      {
        auto metaElem = _sdf->GetElement("meta");
        auto meta = result.mutable_meta();
        if (metaElem->HasElement("layer"))
          meta->set_layer(metaElem->Get<int32_t>("layer"));
      }

      // Load the geometry
      if (_sdf->HasElement("geometry"))
      {
        auto geomMsg = result.mutable_geometry();
        geomMsg->CopyFrom(GeometryFromSDF(_sdf->GetElement("geometry")));
      }

      /// Load the material
      if (_sdf->HasElement("material"))
      {
        sdf::ElementPtr elem = _sdf->GetElement("material");
        auto matMsg = result.mutable_material();

        if (elem->HasElement("script"))
        {
          sdf::ElementPtr scriptElem = elem->GetElement("script");
          matMsg->mutable_script()->set_name(
              scriptElem->Get<std::string>("name"));

          sdf::ElementPtr uriElem = scriptElem->GetElement("uri");
          while (uriElem)
          {
            matMsg->mutable_script()->add_uri(uriElem->Get<std::string>());
            uriElem = uriElem->GetNextElement("uri");
          }
        }

        if (elem->HasElement("lighting"))
        {
          matMsg->set_lighting(elem->Get<bool>("lighting"));
        }

        if (elem->HasElement("shader"))
        {
          sdf::ElementPtr shaderElem = elem->GetElement("shader");

          if (shaderElem->Get<std::string>("type") == "pixel")
            matMsg->set_shader_type(msgs::Material::PIXEL);
          else if (shaderElem->Get<std::string>("type") == "vertex")
            matMsg->set_shader_type(msgs::Material::VERTEX);
          else if (shaderElem->Get<std::string>("type") ==
              "normal_map_object_space")
            matMsg->set_shader_type(msgs::Material::NORMAL_MAP_OBJECT_SPACE);
          else if (shaderElem->Get<std::string>("type") ==
              "normal_map_tangent_space")
            matMsg->set_shader_type(msgs::Material::NORMAL_MAP_TANGENT_SPACE);
          else
            gzthrow(std::string("Unknown shader type[") +
                shaderElem->Get<std::string>("type") + "]");

          if (shaderElem->HasElement("normal_map"))
            matMsg->set_normal_map(
                shaderElem->GetElement("normal_map")->Get<std::string>());
        }

        if (elem->HasElement("ambient"))
          msgs::Set(matMsg->mutable_ambient(),
              elem->Get<ignition::math::Color>("ambient"));
        if (elem->HasElement("diffuse"))
          msgs::Set(matMsg->mutable_diffuse(),
              elem->Get<ignition::math::Color>("diffuse"));
        if (elem->HasElement("specular"))
          msgs::Set(matMsg->mutable_specular(),
              elem->Get<ignition::math::Color>("specular"));
        if (elem->HasElement("emissive"))
          msgs::Set(matMsg->mutable_emissive(),
              elem->Get<ignition::math::Color>("emissive"));
      }

      // Set the origin of the visual
      if (_sdf->HasElement("pose"))
      {
        msgs::Set(result.mutable_pose(),
            _sdf->Get<ignition::math::Pose3d>("pose"));
      }

      // Set plugins of the visual
      if (_sdf->HasElement("plugin"))
      {
        sdf::ElementPtr pluginElem = _sdf->GetElement("plugin");
        while (pluginElem)
        {
          msgs::Plugin *pluginMsg = result.add_plugin();
          pluginMsg->CopyFrom(PluginFromSDF(pluginElem));

          pluginElem = pluginElem->GetNextElement("plugin");
        }
      }

      return result;
    }

    /////////////////////////////////////////////////
    msgs::Collision CollisionFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::Collision result;

      if (_sdf->GetName() != "collision")
      {
        gzerr << "Cannot create a collision message from a ["
          << _sdf->GetName() << "] SDF element." << std::endl;
        return result;
      }

      result.set_name(_sdf->Get<std::string>("name"));

      if (_sdf->HasElement("laser_retro"))
        result.set_laser_retro(_sdf->Get<double>("laser_retro"));

      if (_sdf->HasElement("max_contacts"))
        result.set_max_contacts(_sdf->Get<double>("max_contacts"));

      if (_sdf->HasElement("pose"))
      {
        msgs::Set(result.mutable_pose(),
            _sdf->Get<ignition::math::Pose3d>("pose"));
      }

      // Load the geometry
      if (_sdf->HasElement("geometry"))
      {
        auto geomMsg = result.mutable_geometry();
        geomMsg->CopyFrom(GeometryFromSDF(_sdf->GetElement("geometry")));
      }

      // Load the surface
      if (_sdf->HasElement("surface"))
      {
        auto surfaceMsg = result.mutable_surface();
        surfaceMsg->CopyFrom(SurfaceFromSDF(_sdf->GetElement("surface")));
      }

      return result;
    }

    /////////////////////////////////////////////////
    msgs::Surface SurfaceFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::Surface result;

      if (_sdf->GetName() != "surface")
      {
        gzerr << "Cannot create a surface message from a ["
          << _sdf->GetName() << "] SDF element." << std::endl;
        return result;
      }

      // Load the friction
      if (_sdf->HasElement("friction"))
      {
        auto frictionMsg = result.mutable_friction();
        frictionMsg->CopyFrom(FrictionFromSDF(_sdf->GetElement("friction")));
      }

      // Load bounce elements
      if (_sdf->HasElement("bounce"))
      {
        sdf::ElementPtr bounceElem = _sdf->GetElement("bounce");
        if (bounceElem->HasElement("restitution_coefficient"))
        {
          result.set_restitution_coefficient(
              bounceElem->Get<double>("restitution_coefficient"));
        }
        if (bounceElem->HasElement("threshold"))
          result.set_bounce_threshold(bounceElem->Get<double>("threshold"));
      }

      // Load contact elements. Note the hierarchy differs in SDF and msg
      if (_sdf->HasElement("contact"))
      {
        sdf::ElementPtr contactElem = _sdf->GetElement("contact");

        /// \todo Getting only ODE elements, find a way to get others too
        if (contactElem->HasElement("ode"))
        {
          sdf::ElementPtr odeElem = contactElem->GetElement("ode");

          if (odeElem->HasElement("soft_cfm"))
            result.set_soft_cfm(odeElem->Get<double>("soft_cfm"));

          if (odeElem->HasElement("soft_erp"))
            result.set_soft_erp(odeElem->Get<double>("soft_erp"));

          if (odeElem->HasElement("kp"))
            result.set_kp(odeElem->Get<double>("kp"));

          if (odeElem->HasElement("kd"))
            result.set_kd(odeElem->Get<double>("kd"));

          if (odeElem->HasElement("max_vel"))
            result.set_max_vel(odeElem->Get<double>("max_vel"));

          if (odeElem->HasElement("min_depth"))
            result.set_min_depth(odeElem->Get<double>("min_depth"));
        }

        if (contactElem->HasElement("collide_without_contact"))
        {
          result.set_collide_without_contact(
              contactElem->Get<bool>("collide_without_contact"));
        }

        if (contactElem->HasElement("collide_without_contact_bitmask"))
        {
          result.set_collide_without_contact_bitmask(
              contactElem->Get<int>("collide_without_contact_bitmask"));
        }

        if (contactElem->HasElement("collide_bitmask"))
        {
          result.set_collide_bitmask(contactElem->Get<int>("collide_bitmask"));
        }
      }

      return result;
    }

    /////////////////////////////////////////////////
    msgs::Friction FrictionFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::Friction result;

      if (_sdf->GetName() != "friction")
      {
        gzerr << "Cannot create a friction message from a ["
          << _sdf->GetName() << "] SDF element." << std::endl;
        return result;
      }

      /// \todo Getting only ODE elements, find a way to get others too
      if (_sdf->HasElement("ode"))
      {
        sdf::ElementPtr odeElem = _sdf->GetElement("ode");

        if (odeElem->HasElement("mu"))
          result.set_mu(odeElem->Get<double>("mu"));

        if (odeElem->HasElement("mu2"))
          result.set_mu2(odeElem->Get<double>("mu2"));

        if (odeElem->HasElement("fdir1"))
        {
          msgs::Set(result.mutable_fdir1(),
              odeElem->Get<ignition::math::Vector3d>("fdir1"));
        }

        if (odeElem->HasElement("slip1"))
          result.set_slip1(odeElem->Get<double>("slip1"));

        if (odeElem->HasElement("slip2"))
          result.set_slip2(odeElem->Get<double>("slip2"));
      }

      // Load torsional friction
      if (_sdf->HasElement("torsional"))
      {
        sdf::ElementPtr torsionalElem = _sdf->GetElement("torsional");

        msgs::Friction::Torsional torsionalMsg;

        if (torsionalElem->HasElement("coefficient"))
        {
          torsionalMsg.set_coefficient(
              torsionalElem->Get<double>("coefficient"));
        }

        if (torsionalElem->HasElement("use_patch_radius"))
        {
          torsionalMsg.set_use_patch_radius(
              torsionalElem->Get<bool>("use_patch_radius"));
        }

        if (torsionalElem->HasElement("patch_radius"))
        {
          torsionalMsg.set_patch_radius(
              torsionalElem->Get<double>("patch_radius"));
        }

        if (torsionalElem->HasElement("surface_radius"))
        {
          torsionalMsg.set_surface_radius(
              torsionalElem->Get<double>("surface_radius"));
        }

        if (torsionalElem->HasElement("ode"))
        {
          sdf::ElementPtr odeElem = torsionalElem->GetElement("ode");

          if (odeElem->HasElement("slip"))
          {
            msgs::Friction::Torsional::ODE torsionalOdeMsg;
            torsionalOdeMsg.set_slip(odeElem->Get<double>("slip"));

            auto torsionalODE = torsionalMsg.mutable_ode();
            torsionalODE->CopyFrom(torsionalOdeMsg);
          }
        }

        auto torsional = result.mutable_torsional();
        torsional->CopyFrom(torsionalMsg);
      }

      return result;
    }

    /////////////////////////////////////////////////
    msgs::Axis AxisFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::Axis result;

      sdf::ElementPtr limitElem = _sdf->GetElement("limit");
      result.set_limit_lower(limitElem->Get<double>("lower"));
      result.set_limit_upper(limitElem->Get<double>("upper"));
      result.set_limit_effort(limitElem->Get<double>("effort"));
      result.set_limit_velocity(limitElem->Get<double>("velocity"));

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
      // use_parent_model_frame is deprecated, so always set it to false.
      result.set_use_parent_model_frame(false);
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
      result.set_xyz_expressed_in(
          _sdf->GetElement("xyz")->Get<std::string>("expressed_in"));

      sdf::ElementPtr dynamicsElem = _sdf->GetElement("dynamics");
      result.set_damping(dynamicsElem->Get<double>("damping"));
      result.set_friction(dynamicsElem->Get<double>("friction"));

      msgs::Set(result.mutable_xyz(),
          _sdf->Get<ignition::math::Vector3d>("xyz"));

      return result;
    }

    /////////////////////////////////////////////////
    msgs::Joint JointFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::Joint result;

      // Name
      result.set_name(_sdf->Get<std::string>("name"));

      // parent
      if (_sdf->HasElement("parent"))
         result.set_parent(_sdf->Get<std::string>("parent"));

      // child
      if (_sdf->HasElement("child"))
         result.set_child(_sdf->Get<std::string>("child"));

      // Pose
      ignition::math::Pose3d jointPose;
      if (_sdf->HasElement("pose"))
      {
        msgs::Set(result.mutable_pose(),
            _sdf->Get<ignition::math::Pose3d>("pose"));
      }

      // Type
      std::string type = _sdf->Get<std::string>("type");
      result.set_type(msgs::ConvertJointType(type));

      // axis1
      if (_sdf->HasElement("axis"))
      {
        msgs::Axis *axis = result.mutable_axis1();
        axis->CopyFrom(AxisFromSDF(_sdf->GetElement("axis")));
        result.add_angle(0);
      }

      // axis2
      if (_sdf->HasElement("axis2"))
      {
        msgs::Axis *axis = result.mutable_axis2();
        axis->CopyFrom(AxisFromSDF(_sdf->GetElement("axis2")));
        result.add_angle(0);
      }

      // physics
      if (_sdf->HasElement("physics"))
      {
        sdf::ElementPtr physicsElem = _sdf->GetElement("physics");
        if (physicsElem->HasElement("ode"))
        {
          sdf::ElementPtr odeElem = physicsElem->GetElement("ode");
          if (odeElem->HasElement("cfm"))
            result.set_cfm(odeElem->Get<double>("cfm"));
          if (odeElem->HasElement("bounce"))
            result.set_bounce(odeElem->Get<double>("bounce"));
          if (odeElem->HasElement("velocity"))
            result.set_velocity(odeElem->Get<double>("velocity"));
          if (odeElem->HasElement("fudge_factor"))
            result.set_fudge_factor(odeElem->Get<double>("fudge_factor"));

          if (odeElem->HasElement("limit"))
          {
            sdf::ElementPtr odeLimitElem = odeElem->GetElement("limit");
            if (odeLimitElem->HasElement("cfm"))
              result.set_limit_cfm(odeLimitElem->Get<double>("cfm"));
            if (odeLimitElem->HasElement("erp"))
              result.set_limit_erp(odeLimitElem->Get<double>("erp"));
          }
          if (odeElem->HasElement("suspension"))
          {
            sdf::ElementPtr odeSuspensionElem =
                odeElem->GetElement("suspension");
            if (odeSuspensionElem->HasElement("cfm"))
            {
              result.set_suspension_cfm(
                  odeSuspensionElem->Get<double>("cfm"));
            }
            if (odeSuspensionElem->HasElement("erp"))
            {
              result.set_suspension_erp(
                  odeSuspensionElem->Get<double>("erp"));
            }
          }
        }
      }

      // gearbox
      if (_sdf->HasElement("gearbox_reference_body"))
      {
        msgs::Joint::Gearbox *gearboxMsg = result.mutable_gearbox();
        gearboxMsg->set_gearbox_reference_body(
            _sdf->Get<std::string>("gearbox_reference_body"));
      }
      if (_sdf->HasElement("gearbox_ratio"))
      {
        msgs::Joint::Gearbox *gearboxMsg = result.mutable_gearbox();
        gearboxMsg->set_gearbox_ratio(_sdf->Get<double>("gearbox_ratio"));
      }

      // screw
      if (_sdf->HasElement("thread_pitch"))
      {
        msgs::Joint::Screw *screwMsg = result.mutable_screw();
        screwMsg->set_thread_pitch(_sdf->Get<double>("thread_pitch"));
      }

      return result;
    }

    /////////////////////////////////////////////////
    sdf::ElementPtr VisualToSDF(const msgs::Visual &_msg,
        sdf::ElementPtr _sdf)
    {
      sdf::ElementPtr visualSDF;

      if (_sdf)
      {
        visualSDF = _sdf;
      }
      else
      {
        visualSDF.reset(new sdf::Element);
        sdf::initFile("visual.sdf", visualSDF);
      }

      // Set the meta information
      if (_msg.has_meta())
      {
        if (_msg.meta().has_layer())
        {
          visualSDF->GetElement("meta")->GetElement("layer")->Set(
              _msg.meta().layer());
        }
      }

      if (_msg.has_name())
        visualSDF->GetAttribute("name")->Set(_msg.name());

      if (_msg.has_cast_shadows())
        visualSDF->GetElement("cast_shadows")->Set(_msg.cast_shadows());

      if (_msg.has_transparency())
        visualSDF->GetElement("transparency")->Set(_msg.transparency());

      if (_msg.has_laser_retro())
        visualSDF->GetElement("laser_retro")->Set(_msg.laser_retro());

      if (_msg.has_pose())
        visualSDF->GetElement("pose")->Set(ConvertIgn(_msg.pose()));

      // Load the geometry
      if (_msg.has_geometry())
      {
        sdf::ElementPtr geomElem = visualSDF->GetElement("geometry");
        geomElem = GeometryToSDF(_msg.geometry(), geomElem);
      }

      /// Load the material
      if (_msg.has_material())
      {
        sdf::ElementPtr materialElem = visualSDF->GetElement("material");
        materialElem = MaterialToSDF(_msg.material(), materialElem);
      }

      // Set plugins of the visual
      for (int i = 0; i < _msg.plugin_size(); ++i)
      {
        sdf::ElementPtr pluginElem = visualSDF->AddElement("plugin");
        pluginElem = PluginToSDF(_msg.plugin(i), pluginElem);
      }

      return visualSDF;
    }

    /////////////////////////////////////////////////
    sdf::ElementPtr MaterialToSDF(const msgs::Material &_msg,
        sdf::ElementPtr _sdf)
    {
      sdf::ElementPtr materialSDF;

      if (_sdf)
      {
        materialSDF = _sdf;
      }
      else
      {
        materialSDF.reset(new sdf::Element);
        sdf::initFile("material.sdf", materialSDF);
      }

      if (_msg.has_script())
      {
        sdf::ElementPtr scriptElem = materialSDF->GetElement("script");
        msgs::Material::Script script = _msg.script();

        if (script.has_name())
          scriptElem->GetElement("name")->Set(script.name());

        if (script.uri_size() > 0)
          while (scriptElem->HasElement("uri"))
            scriptElem->GetElement("uri")->RemoveFromParent();
        for (int i = 0; i < script.uri_size(); ++i)
        {
          sdf::ElementPtr uriElem = scriptElem->AddElement("uri");
          uriElem->Set(script.uri(i));
        }
      }

      if (_msg.has_shader_type())
      {
        sdf::ElementPtr shaderElem = materialSDF->GetElement("shader");
        shaderElem->GetAttribute("type")->Set(
          ConvertShaderType(_msg.shader_type()));
      }

      if (_msg.has_normal_map())
      {
        sdf::ElementPtr shaderElem = materialSDF->GetElement("shader");
        shaderElem->GetElement("normal_map")->Set(_msg.normal_map());
      }

      if (_msg.has_lighting())
        materialSDF->GetElement("lighting")->Set(_msg.lighting());

      if (_msg.has_ambient())
        materialSDF->GetElement("ambient")->Set(Convert(_msg.ambient()));
      if (_msg.has_diffuse())
        materialSDF->GetElement("diffuse")->Set(Convert(_msg.diffuse()));
      if (_msg.has_emissive())
        materialSDF->GetElement("emissive")->Set(Convert(_msg.emissive()));
      if (_msg.has_specular())
        materialSDF->GetElement("specular")->Set(Convert(_msg.specular()));

      return materialSDF;
    }

    /////////////////////////////////////////////////
    msgs::Material::ShaderType ConvertShaderType(const std::string &_str)
    {
      auto result = msgs::Material::VERTEX;
      if (_str == "vertex")
      {
        result = msgs::Material::VERTEX;
      }
      else if (_str == "pixel")
      {
        result = msgs::Material::PIXEL;
      }
      else if (_str == "normal_map_object_space")
      {
        result = msgs::Material::NORMAL_MAP_OBJECT_SPACE;
      }
      else if (_str == "normal_map_tangent_space")
      {
        result = msgs::Material::NORMAL_MAP_TANGENT_SPACE;
      }
      else
      {
        gzerr << "Unrecognized ShaderType ["
              << _str
              << "], returning VERTEX"
              << std::endl;
      }
      return result;
    }

    /////////////////////////////////////////////////
    std::string ConvertShaderType(const msgs::Material::ShaderType &_type)
    {
      std::string result;
      switch (_type)
      {
        case msgs::Material::VERTEX:
        {
          result = "vertex";
          break;
        }
        case msgs::Material::PIXEL:
        {
          result = "pixel";
          break;
        }
        case msgs::Material::NORMAL_MAP_OBJECT_SPACE:
        {
          result = "normal_map_object_space";
          break;
        }
        case msgs::Material::NORMAL_MAP_TANGENT_SPACE:
        {
          result = "normal_map_tangent_space";
          break;
        }
        default:
        {
          result = "unknown";
          gzerr << "Unrecognized ShaderType [" << _type << "]"
                << std::endl;
          break;
        }
      }
      return result;
    }

    /////////////////////////////////////////////////
    msgs::Fog FogFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::Fog result;

      std::string type = _sdf->Get<std::string>("type");
      if (type == "linear")
        result.set_type(msgs::Fog::LINEAR);
      else if (type == "exp")
        result.set_type(msgs::Fog::EXPONENTIAL);
      else if (type == "exp2")
        result.set_type(msgs::Fog::EXPONENTIAL2);
      else if (type == "none")
        result.set_type(msgs::Fog::NONE);
      else
        gzthrow(std::string("Unknown fog type[") + type + "]");

      result.mutable_color()->CopyFrom(
          Convert(_sdf->Get<ignition::math::Color>("color")));

      result.set_density(_sdf->Get<double>("density"));
      result.set_start(_sdf->Get<double>("start"));
      result.set_end(_sdf->Get<double>("end"));
      return result;
    }

    /////////////////////////////////////////////////
    msgs::Scene SceneFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::Scene result;

      Init(result, "scene");

      if (_sdf->HasElement("grid"))
        result.set_grid(_sdf->Get<bool>("grid"));
      else
        result.set_grid(true);

      if (_sdf->HasElement("origin_visual"))
        result.set_origin_visual(_sdf->Get<bool>("origin_visual"));
      else
        result.set_origin_visual(true);

      if (_sdf->HasElement("ambient"))
        result.mutable_ambient()->CopyFrom(
            Convert(_sdf->Get<ignition::math::Color>("ambient")));

      if (_sdf->HasElement("background"))
      {
        result.mutable_background()->CopyFrom(
            Convert(_sdf->Get<ignition::math::Color>("background")));
      }

      if (_sdf->HasElement("sky"))
      {
        msgs::Sky *skyMsg = result.mutable_sky();
        skyMsg->set_time(_sdf->GetElement("sky")->Get<double>("time"));
        skyMsg->set_sunrise(_sdf->GetElement("sky")->Get<double>("sunrise"));
        skyMsg->set_sunset(_sdf->GetElement("sky")->Get<double>("sunset"));

        if (_sdf->GetElement("sky")->HasElement("clouds"))
        {
          sdf::ElementPtr cloudsElem =
            _sdf->GetElement("sky")->GetElement("clouds");
          skyMsg->set_wind_speed(cloudsElem->Get<double>("speed"));
          skyMsg->set_wind_direction(cloudsElem->Get<double>("direction"));
          skyMsg->set_humidity(cloudsElem->Get<double>("humidity"));
          skyMsg->set_mean_cloud_size(cloudsElem->Get<double>("mean_size"));
          msgs::Set(skyMsg->mutable_cloud_ambient(),
                    cloudsElem->Get<ignition::math::Color>("ambient"));
        }
      }

      if (_sdf->HasElement("fog"))
        result.mutable_fog()->CopyFrom(FogFromSDF(_sdf->GetElement("fog")));

      if (_sdf->HasElement("shadows"))
        result.set_shadows(_sdf->Get<bool>("shadows"));

      return result;
    }

    /////////////////////////////////////////////////
    msgs::Sensor SensorFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::Sensor result;
      std::string type = _sdf->Get<std::string>("type");
      result.set_name(_sdf->Get<std::string>("name"));
      result.set_type(type);

      if (_sdf->HasElement("always_on"))
        result.set_always_on(_sdf->Get<bool>("always_on"));

      if (_sdf->HasElement("update_rate"))
        result.set_update_rate(_sdf->Get<double>("update_rate"));

      if (_sdf->HasElement("pose"))
      {
        msgs::Set(result.mutable_pose(),
            _sdf->Get<ignition::math::Pose3d>("pose"));
      }

      if (_sdf->HasElement("visualize"))
        result.set_visualize(_sdf->Get<bool>("visualize"));

      if (_sdf->HasElement("topic"))
        result.set_topic(_sdf->Get<std::string>("topic"));

      if (type == "camera" || type == "depth")
      {
        result.mutable_camera()->CopyFrom(
            msgs::CameraSensorFromSDF(_sdf->GetElement("camera")));
      }
      else if (type == "ray" || type == "gpu_ray")
      {
        result.mutable_ray()->CopyFrom(msgs::RaySensorFromSDF(
            _sdf->GetElement("ray")));
      }
      else if (type == "contact")
      {
        result.mutable_contact()->CopyFrom(
          msgs::ContactSensorFromSDF(_sdf->GetElement("contact")));
      }
      else if (type == "gps")
      {
        result.mutable_gps()->CopyFrom(
          msgs::GPSSensorFromSDF(_sdf->GetElement("gps")));
      }
      else if (type == "logical_camera")
      {
        result.mutable_logical_camera()->CopyFrom(
          msgs::LogicalCameraSensorFromSDF(_sdf->GetElement("logical_camera")));
      }
      else if (type == "imu")
      {
        result.mutable_imu()->CopyFrom(
          msgs::IMUSensorFromSDF(_sdf->GetElement("imu")));
      }
      else
      {
        gzwarn << "Conversion of sensor type[" << type << "] not supported."
          << std::endl;
      }

      return result;
    }

    /////////////////////////////////////////////////
    msgs::IMUSensor IMUSensorFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::IMUSensor result;

      std::array<std::string, 2> senses =
        {{"angular_velocity", "linear_acceleration"}};

      std::array<std::string, 3> dimensions = {{"x", "y", "z"}};

      for (auto const &sense : senses)
      {
        if (_sdf->HasElement(sense))
        {
          auto senseElem = _sdf->GetElement(sense);
          for (auto const &dim : dimensions)
          {
            if (senseElem->HasElement(dim))
            {
              auto dimElem = senseElem->GetElement(dim);
              auto noiseElem = dimElem->GetElement("noise");
              msgs::SensorNoise *noiseMsg;

              if (sense == "angular_velocity")
              {
                if (dim == "x")
                {
                  noiseMsg = result.mutable_angular_velocity()->
                    mutable_x_noise();
                }
                else if (dim == "y")
                {
                  noiseMsg = result.mutable_angular_velocity()->
                    mutable_y_noise();
                }
                else
                {
                  noiseMsg = result.mutable_angular_velocity()->
                    mutable_z_noise();
                }
              }
              else
              {
                if (dim == "x")
                {
                  noiseMsg = result.mutable_linear_acceleration()->
                    mutable_x_noise();
                }
                else if (dim == "y")
                {
                  noiseMsg = result.mutable_linear_acceleration()->
                    mutable_y_noise();
                }
                else
                {
                  noiseMsg = result.mutable_linear_acceleration()->
                    mutable_z_noise();
                }
              }

              noiseMsg->CopyFrom(SensorNoiseFromSDF(noiseElem));
            }
          }
        }
      }

      return result;
    }

    /////////////////////////////////////////////////
    msgs::CameraSensor CameraSensorFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::CameraSensor result;

      result.set_horizontal_fov(_sdf->Get<double>("horizontal_fov"));

      result.mutable_image_size()->set_x(
          _sdf->GetElement("image")->Get<int>("width"));
      result.mutable_image_size()->set_y(
          _sdf->GetElement("image")->Get<int>("height"));

      if (_sdf->GetElement("image")->HasElement("format"))
      {
        result.set_image_format(
            _sdf->GetElement("image")->Get<std::string>("format"));
      }

      result.set_near_clip(_sdf->GetElement("clip")->Get<double>("near"));
      result.set_far_clip(_sdf->GetElement("clip")->Get<double>("far"));

      if (_sdf->HasElement("save"))
      {
        result.set_save_enabled(_sdf->GetElement("save")->Get<bool>("enabled"));
        result.set_save_path(
            _sdf->GetElement("save")->Get<std::string>("path"));
      }

      if (_sdf->HasElement("distortion"))
      {
        sdf::ElementPtr distElem = _sdf->GetElement("distortion");
        msgs::Distortion *distortionMsg = result.mutable_distortion();

        if (distElem->HasElement("k1"))
          distortionMsg->set_k1(distElem->Get<double>("k1"));

        if (distElem->HasElement("k2"))
          distortionMsg->set_k2(distElem->Get<double>("k2"));

        if (distElem->HasElement("k3"))
          distortionMsg->set_k3(distElem->Get<double>("k3"));

        if (distElem->HasElement("p1"))
          distortionMsg->set_p1(distElem->Get<double>("p1"));

        if (distElem->HasElement("p2"))
          distortionMsg->set_p2(distElem->Get<double>("p2"));

        if (distElem->HasElement("center"))
        {
          distortionMsg->mutable_center()->set_x(
              distElem->Get<ignition::math::Vector2d>("center").X());
          distortionMsg->mutable_center()->set_y(
              distElem->Get<ignition::math::Vector2d>("center").Y());
        }
      }

      return result;
    }

    /////////////////////////////////////////////////
    msgs::RaySensor RaySensorFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::RaySensor result;
      sdf::ElementPtr rangeElem = _sdf->GetElement("range");
      sdf::ElementPtr scanElem = _sdf->GetElement("scan");
      sdf::ElementPtr hscanElem = scanElem->GetElement("horizontal");
      sdf::ElementPtr vscanElem = scanElem->GetElement("vertical");

      result.set_horizontal_samples(hscanElem->Get<int>("samples"));
      result.set_horizontal_resolution(hscanElem->Get<double>("resolution"));
      result.set_horizontal_min_angle(hscanElem->Get<double>("min_angle"));
      result.set_horizontal_max_angle(hscanElem->Get<double>("max_angle"));

      result.set_vertical_samples(vscanElem->Get<int>("samples"));
      result.set_vertical_resolution(vscanElem->Get<double>("resolution"));
      result.set_vertical_min_angle(vscanElem->Get<double>("min_angle"));
      result.set_vertical_max_angle(vscanElem->Get<double>("max_angle"));

      result.set_range_min(rangeElem->Get<double>("min"));
      result.set_range_max(rangeElem->Get<double>("max"));
      result.set_range_resolution(rangeElem->Get<double>("resolution"));

      return result;
    }

    /////////////////////////////////////////////////
    msgs::ContactSensor ContactSensorFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::ContactSensor result;
      result.set_collision_name(_sdf->Get<std::string>("collision"));
      return result;
    }

    /////////////////////////////////////////////////
    msgs::GPSSensor GPSSensorFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::GPSSensor result;

      // The two types of sensing
      std::array<std::string, 2> sensing =
        {{"position_sensing", "velocity_sensing"}};

      // The two dimensions for each of sensing types.
      std::array<std::string, 2> dimensions = {{"horizontal", "vertical"}};

      // Process each sensing
      for (auto const &sense : sensing)
      {
        // Make sure the element exists
        if (_sdf->HasElement(sense))
        {
          auto senseElem = _sdf->GetElement(sense);

          // Process each dimension
          for (auto const &dim : dimensions)
          {
            if (senseElem->HasElement(dim))
            {
              auto dimElem = senseElem->GetElement(dim);

              // Add noise
              if (dimElem->HasElement("noise"))
              {
                auto noiseElem = dimElem->GetElement("noise");
                msgs::SensorNoise *noiseMsg;

                if (sense == "position_sensing")
                {
                  if (dim == "horizontal")
                  {
                    noiseMsg = result.mutable_position()->
                      mutable_horizontal_noise();
                  }
                  else
                  {
                    noiseMsg = result.mutable_position()->
                      mutable_vertical_noise();
                  }
                }
                else
                {
                  if (dim == "horizontal")
                  {
                    noiseMsg = result.mutable_velocity()->
                      mutable_horizontal_noise();
                  }
                  else
                  {
                    noiseMsg = result.mutable_velocity()->
                      mutable_vertical_noise();
                  }
                }

                noiseMsg->CopyFrom(SensorNoiseFromSDF(noiseElem));
              }
            }
          }
        }
      }

      return result;
    }

    /////////////////////////////////////////////////
    msgs::LogicalCameraSensor LogicalCameraSensorFromSDF(sdf::ElementPtr _sdf)
    {
      msgs::LogicalCameraSensor result;
      result.set_near_clip(_sdf->Get<double>("near"));
      result.set_far_clip(_sdf->Get<double>("far"));
      result.set_horizontal_fov(_sdf->Get<double>("horizontal_fov"));
      result.set_aspect_ratio(_sdf->Get<double>("aspect_ratio"));
      return result;
    }

    /////////////////////////////////////////////////
    sdf::ElementPtr LightToSDF(const msgs::Light &_msg, sdf::ElementPtr _sdf)
    {
      sdf::ElementPtr lightSDF;

      if (_sdf)
      {
        lightSDF = _sdf;
      }
      else
      {
        lightSDF.reset(new sdf::Element);
        sdf::initFile("light.sdf", lightSDF);
      }

      lightSDF->GetAttribute("name")->Set(_msg.name());

      if (_msg.has_type() && _msg.type() == msgs::Light::POINT)
        lightSDF->GetAttribute("type")->Set("point");
      else if (_msg.has_type() && _msg.type() == msgs::Light::SPOT)
        lightSDF->GetAttribute("type")->Set("spot");
      else if (_msg.has_type() && _msg.type() == msgs::Light::DIRECTIONAL)
        lightSDF->GetAttribute("type")->Set("directional");

      if (_msg.has_pose())
      {
        lightSDF->GetElement("pose")->Set(ConvertIgn(_msg.pose()));
      }

      if (_msg.has_diffuse())
      {
        lightSDF->GetElement("diffuse")->Set(msgs::Convert(_msg.diffuse()));
      }

      if (_msg.has_specular())
      {
        lightSDF->GetElement("specular")->Set(msgs::Convert(_msg.specular()));
      }

      if (_msg.has_direction())
      {
        lightSDF->GetElement("direction")->Set(ConvertIgn(_msg.direction()));
      }

      if (_msg.has_attenuation_constant())
      {
        sdf::ElementPtr elem = lightSDF->GetElement("attenuation");
        elem->GetElement("constant")->Set(_msg.attenuation_constant());
      }

      if (_msg.has_attenuation_linear())
      {
        sdf::ElementPtr elem = lightSDF->GetElement("attenuation");
        elem->GetElement("linear")->Set(_msg.attenuation_linear());
      }

      if (_msg.has_attenuation_quadratic())
      {
        sdf::ElementPtr elem = lightSDF->GetElement("attenuation");
        elem->GetElement("quadratic")->Set(_msg.attenuation_quadratic());
      }

      if (_msg.has_range())
      {
        sdf::ElementPtr elem = lightSDF->GetElement("attenuation");
        elem->GetElement("range")->Set(_msg.range());
      }

      if (_msg.has_cast_shadows())
        lightSDF->GetElement("cast_shadows")->Set(_msg.cast_shadows());

      if (_msg.has_spot_inner_angle())
      {
        sdf::ElementPtr elem = lightSDF->GetElement("spot");
        elem->GetElement("inner_angle")->Set(_msg.spot_inner_angle());
      }

      if (_msg.has_spot_outer_angle())
      {
        sdf::ElementPtr elem = lightSDF->GetElement("spot");
        elem->GetElement("outer_angle")->Set(_msg.spot_outer_angle());
      }

      if (_msg.has_spot_falloff())
      {
        sdf::ElementPtr elem = lightSDF->GetElement("spot");
        elem->GetElement("falloff")->Set(_msg.spot_falloff());
      }
      return lightSDF;
    }

    /////////////////////////////////////////////////
    sdf::ElementPtr SensorNoiseToSDF(const msgs::SensorNoise &_msg,
        sdf::ElementPtr _sdf)
    {
      sdf::ElementPtr noiseSDF;

      if (_sdf)
      {
        noiseSDF = _sdf;
      }
      else
      {
        noiseSDF.reset(new sdf::Element);
        sdf::initFile("noise.sdf", noiseSDF);
      }

      if (_msg.type() == msgs::SensorNoise::NONE)
      {
        noiseSDF->GetAttribute("type")->Set("none");
      }
      else if (_msg.type() == msgs::SensorNoise::GAUSSIAN)
      {
        noiseSDF->GetAttribute("type")->Set("gaussian");
      }
      else if (_msg.type() == msgs::SensorNoise::GAUSSIAN_QUANTIZED)
      {
        noiseSDF->GetAttribute("type")->Set("gaussian_quantized");
      }

      if (_msg.has_mean())
        noiseSDF->GetElement("mean")->Set(_msg.mean());

      if (_msg.has_stddev())
        noiseSDF->GetElement("stddev")->Set(_msg.stddev());

      if (_msg.has_bias_mean())
        noiseSDF->GetElement("bias_mean")->Set(_msg.bias_mean());

      if (_msg.has_bias_stddev())
        noiseSDF->GetElement("bias_stddev")->Set(_msg.bias_stddev());

      if (_msg.has_precision())
        noiseSDF->GetElement("precision")->Set(_msg.precision());

      return noiseSDF;
    }

    /////////////////////////////////////////////////
    sdf::ElementPtr GPSSensorToSDF(const msgs::GPSSensor &_msg,
        sdf::ElementPtr _sdf)
    {
      sdf::ElementPtr gpsSDF;

      if (_sdf)
      {
        gpsSDF = _sdf;
      }
      else
      {
        gpsSDF.reset(new sdf::Element);
        sdf::initFile("gps.sdf", gpsSDF);
      }

      if (_msg.has_position())
      {
        if (_msg.position().has_horizontal_noise())
        {
          auto noiseElem = gpsSDF->GetElement("position_sensing")->GetElement(
              "horizontal")->GetElement("noise");
          noiseElem->PrintValues("  ");
          SensorNoiseToSDF(_msg.position().horizontal_noise(), noiseElem);
        }

        if (_msg.position().has_vertical_noise())
        {
          auto noiseElem = gpsSDF->GetElement("position_sensing")->GetElement(
              "vertical")->GetElement("noise");
          SensorNoiseToSDF(_msg.position().vertical_noise(), noiseElem);
        }
      }

      if (_msg.has_velocity())
      {
        if (_msg.velocity().has_horizontal_noise())
        {
          auto noiseElem = gpsSDF->GetElement("velocity_sensing")->GetElement(
              "horizontal")->GetElement("noise");
          SensorNoiseToSDF(_msg.velocity().horizontal_noise(), noiseElem);
        }

        if (_msg.velocity().has_vertical_noise())
        {
          auto noiseElem = gpsSDF->GetElement("velocity_sensing")->GetElement(
              "vertical")->GetElement("noise");
          SensorNoiseToSDF(_msg.velocity().vertical_noise(), noiseElem);
        }
      }

      return gpsSDF;
    }

    /////////////////////////////////////////////////
    sdf::ElementPtr IMUSensorToSDF(const msgs::IMUSensor &_msg,
        sdf::ElementPtr _sdf)
    {
      sdf::ElementPtr imuSDF;

      if (_sdf)
      {
        imuSDF = _sdf;
      }
      else
      {
        imuSDF.reset(new sdf::Element);
        sdf::initFile("imu.sdf", imuSDF);
      }

      if (_msg.has_angular_velocity())
      {
        if (_msg.angular_velocity().has_x_noise())
        {
          auto noiseElem = imuSDF->GetElement("angular_velocity")->GetElement(
              "x")->GetElement("noise");
          SensorNoiseToSDF(_msg.angular_velocity().x_noise(), noiseElem);
        }

        if (_msg.angular_velocity().has_y_noise())
        {
          auto noiseElem = imuSDF->GetElement("angular_velocity")->GetElement(
              "y")->GetElement("noise");
          SensorNoiseToSDF(_msg.angular_velocity().y_noise(), noiseElem);
        }

        if (_msg.angular_velocity().has_z_noise())
        {
          auto noiseElem = imuSDF->GetElement("angular_velocity")->GetElement(
              "z")->GetElement("noise");
          SensorNoiseToSDF(_msg.angular_velocity().z_noise(), noiseElem);
        }
      }

      if (_msg.has_linear_acceleration())
      {
        if (_msg.linear_acceleration().has_x_noise())
        {
          auto noiseElem = imuSDF->GetElement(
              "linear_acceleration")->GetElement("x")->GetElement("noise");
          SensorNoiseToSDF(_msg.linear_acceleration().x_noise(), noiseElem);
        }

        if (_msg.linear_acceleration().has_y_noise())
        {
          auto noiseElem = imuSDF->GetElement(
              "linear_acceleration")->GetElement("y")->GetElement("noise");
          SensorNoiseToSDF(_msg.linear_acceleration().y_noise(), noiseElem);
        }

        if (_msg.linear_acceleration().has_z_noise())
        {
          auto noiseElem = imuSDF->GetElement(
              "linear_acceleration")->GetElement("z")->GetElement("noise");
          SensorNoiseToSDF(_msg.linear_acceleration().z_noise(), noiseElem);
        }
      }

      return imuSDF;
    }

    /////////////////////////////////////////////////
    sdf::ElementPtr LogicalCameraSensorToSDF(
        const msgs::LogicalCameraSensor &_msg, sdf::ElementPtr _sdf)
    {
      sdf::ElementPtr logicalSDF;

      if (_sdf)
      {
        logicalSDF = _sdf;
      }
      else
      {
        logicalSDF.reset(new sdf::Element);
        sdf::initFile("logical_camera.sdf", logicalSDF);
      }

      logicalSDF->GetElement("horizontal_fov")->Set(_msg.horizontal_fov());
      logicalSDF->GetElement("aspect_ratio")->Set(_msg.aspect_ratio());
      logicalSDF->GetElement("near")->Set(_msg.near_clip());
      logicalSDF->GetElement("far")->Set(_msg.far_clip());

      return logicalSDF;
    }

    /////////////////////////////////////////////////
    sdf::ElementPtr CameraSensorToSDF(const msgs::CameraSensor &_msg,
        sdf::ElementPtr _sdf)
    {
      sdf::ElementPtr cameraSDF;

      if (_sdf)
      {
        cameraSDF = _sdf;
      }
      else
      {
        cameraSDF.reset(new sdf::Element);
        sdf::initFile("camera.sdf", cameraSDF);
      }

      if (_msg.has_horizontal_fov())
      {
        cameraSDF->GetElement("horizontal_fov")->Set(_msg.horizontal_fov());
      }
      if (_msg.has_image_size())
      {
        sdf::ElementPtr imageElem = cameraSDF->GetElement("image");
        imageElem->GetElement("width")->Set(_msg.image_size().x());
        imageElem->GetElement("height")->Set(_msg.image_size().y());
      }
      if (_msg.has_image_format())
      {
        sdf::ElementPtr imageElem = cameraSDF->GetElement("image");
        imageElem->GetElement("format")->Set(_msg.image_format());
      }
      if (_msg.has_near_clip() || _msg.has_far_clip())
      {
        sdf::ElementPtr clipElem = cameraSDF->GetElement("clip");
        if (_msg.has_near_clip())
          clipElem->GetElement("near")->Set(_msg.near_clip());
        if (_msg.has_far_clip())
          clipElem->GetElement("far")->Set(_msg.far_clip());
      }

      if (_msg.has_distortion())
      {
        msgs::Distortion distortionMsg = _msg.distortion();
        sdf::ElementPtr distortionElem =
            cameraSDF->GetElement("distortion");

        if (distortionMsg.has_center())
        {
          distortionElem->GetElement("center")->Set(
              ConvertIgn(distortionMsg.center()));
        }
        if (distortionMsg.has_k1())
        {
          distortionElem->GetElement("k1")->Set(distortionMsg.k1());
        }
        if (distortionMsg.has_k2())
        {
          distortionElem->GetElement("k2")->Set(distortionMsg.k2());
        }
        if (distortionMsg.has_k3())
        {
          distortionElem->GetElement("k3")->Set(distortionMsg.k3());
        }
        if (distortionMsg.has_p1())
        {
          distortionElem->GetElement("p1")->Set(distortionMsg.p1());
        }
        if (distortionMsg.has_p2())
        {
          distortionElem->GetElement("p2")->Set(distortionMsg.p2());
        }
      }
      return cameraSDF;
    }


    /////////////////////////////////////////////////
    sdf::ElementPtr CollisionToSDF(const msgs::Collision &_msg,
        sdf::ElementPtr _sdf)
    {
      sdf::ElementPtr collisionSDF;

      if (_sdf)
      {
        collisionSDF = _sdf;
      }
      else
      {
        collisionSDF.reset(new sdf::Element);
        sdf::initFile("collision.sdf", collisionSDF);
      }

      if (_msg.has_name())
        collisionSDF->GetAttribute("name")->Set(_msg.name());
      if (_msg.has_laser_retro())
        collisionSDF->GetElement("laser_retro")->Set(_msg.laser_retro());
      if (_msg.has_max_contacts())
        collisionSDF->GetElement("max_contacts")->Set(_msg.max_contacts());
      if (_msg.has_pose())
        collisionSDF->GetElement("pose")->Set(ConvertIgn(_msg.pose()));
      if (_msg.has_geometry())
      {
        sdf::ElementPtr geomElem = collisionSDF->GetElement("geometry");
        geomElem = GeometryToSDF(_msg.geometry(), geomElem);
      }
      if (_msg.has_surface())
      {
        sdf::ElementPtr surfaceElem = collisionSDF->GetElement("surface");
        surfaceElem = SurfaceToSDF(_msg.surface(), surfaceElem);
      }

      return collisionSDF;
    }

    /////////////////////////////////////////////////
    sdf::ElementPtr LinkToSDF(const msgs::Link &_msg,
        sdf::ElementPtr _sdf)
    {
      sdf::ElementPtr linkSDF;

      if (_sdf)
      {
        linkSDF = _sdf;
      }
      else
      {
        linkSDF.reset(new sdf::Element);
        sdf::initFile("link.sdf", linkSDF);
      }

      if (_msg.has_name())
        linkSDF->GetAttribute("name")->Set(_msg.name());
      if (_msg.has_gravity())
        linkSDF->GetElement("gravity")->Set(_msg.gravity());
      if (_msg.has_self_collide())
        linkSDF->GetElement("self_collide")->Set(_msg.self_collide());
      if (_msg.has_kinematic())
        linkSDF->GetElement("kinematic")->Set(_msg.kinematic());
      if (_msg.has_enable_wind())
        linkSDF->GetElement("enable_wind")->Set(_msg.enable_wind());
      if (_msg.has_pose())
        linkSDF->GetElement("pose")->Set(ConvertIgn(_msg.pose()));
      if (_msg.has_inertial())
      {
        sdf::ElementPtr inertialElem = linkSDF->GetElement("inertial");
        inertialElem = InertialToSDF(_msg.inertial(), inertialElem);
      }
      if (_msg.collision_size() > 0)
        while (linkSDF->HasElement("collision"))
          linkSDF->GetElement("collision")->RemoveFromParent();
      for (int i = 0; i < _msg.collision_size(); ++i)
      {
        sdf::ElementPtr collisionElem = linkSDF->AddElement("collision");
        collisionElem = CollisionToSDF(_msg.collision(i), collisionElem);
      }
      if (_msg.visual_size() > 0)
        while (linkSDF->HasElement("visual"))
          linkSDF->GetElement("visual")->RemoveFromParent();
      for (int i = 0; i < _msg.visual_size(); ++i)
      {
        sdf::ElementPtr visualElem = linkSDF->AddElement("visual");
        visualElem = VisualToSDF(_msg.visual(i), visualElem);
      }

      /// \todo LinkToSDF currently does not convert sensor and projector data

      return linkSDF;
    }

    /////////////////////////////////////////////////
    sdf::ElementPtr InertialToSDF(const msgs::Inertial &_msg,
        sdf::ElementPtr _sdf)
    {
      sdf::ElementPtr inertialSDF;

      if (_sdf)
      {
        inertialSDF = _sdf;
      }
      else
      {
        inertialSDF.reset(new sdf::Element);
        sdf::initFile("inertial.sdf", inertialSDF);
      }

      if (_msg.has_mass())
        inertialSDF->GetElement("mass")->Set(_msg.mass());
      if (_msg.has_pose())
        inertialSDF->GetElement("pose")->Set(ConvertIgn(_msg.pose()));

      sdf::ElementPtr inertiaSDF = inertialSDF->GetElement("inertia");
      if (_msg.has_ixx())
        inertiaSDF->GetElement("ixx")->Set(_msg.ixx());
      if (_msg.has_ixy())
        inertiaSDF->GetElement("ixy")->Set(_msg.ixy());
      if (_msg.has_ixz())
        inertiaSDF->GetElement("ixz")->Set(_msg.ixz());
      if (_msg.has_iyy())
        inertiaSDF->GetElement("iyy")->Set(_msg.iyy());
      if (_msg.has_iyz())
        inertiaSDF->GetElement("iyz")->Set(_msg.iyz());
      if (_msg.has_izz())
        inertiaSDF->GetElement("izz")->Set(_msg.izz());

      return inertialSDF;
    }

    /////////////////////////////////////////////////
    sdf::ElementPtr SurfaceToSDF(const msgs::Surface &_msg,
        sdf::ElementPtr _sdf)
    {
      sdf::ElementPtr surfaceSDF;

      if (_sdf)
      {
        surfaceSDF = _sdf;
      }
      else
      {
        surfaceSDF.reset(new sdf::Element);
        sdf::initFile("surface.sdf", surfaceSDF);
      }

      if (_msg.has_friction())
      {
        msgs::Friction friction = _msg.friction();
        sdf::ElementPtr frictionElem = surfaceSDF->GetElement("friction");
        sdf::ElementPtr physicsEngElem = frictionElem->GetElement("ode");
        if (friction.has_mu())
          physicsEngElem->GetElement("mu")->Set(friction.mu());
        if (friction.has_mu2())
          physicsEngElem->GetElement("mu2")->Set(friction.mu2());
        if (friction.has_fdir1())
        {
          physicsEngElem->GetElement("fdir1")->Set(
              ConvertIgn(friction.fdir1()));
        }
        if (friction.has_slip1())
          physicsEngElem->GetElement("slip1")->Set(friction.slip1());
        if (friction.has_slip2())
          physicsEngElem->GetElement("slip2")->Set(friction.slip2());

        if (friction.has_torsional())
        {
          msgs::Friction::Torsional torsional = friction.torsional();
          sdf::ElementPtr torsionalElem = frictionElem->GetElement("torsional");

          if (torsional.has_coefficient())
          {
            torsionalElem->GetElement("coefficient")->Set(
                torsional.coefficient());
          }
          if (torsional.has_patch_radius())
          {
            torsionalElem->GetElement("patch_radius")->Set(
                torsional.patch_radius());
          }
          if (torsional.has_surface_radius())
          {
            torsionalElem->GetElement("surface_radius")->Set(
                torsional.surface_radius());
          }
          if (torsional.has_use_patch_radius())
          {
            torsionalElem->GetElement("use_patch_radius")->Set(
                torsional.use_patch_radius());
          }
          if (torsional.has_ode())
          {
            msgs::Friction::Torsional::ODE ode = torsional.ode();
            sdf::ElementPtr odeElem = torsionalElem->GetElement("ode");
            if (ode.has_slip())
            {
              odeElem->GetElement("slip")->Set(ode.slip());
            }
          }
        }
      }
      sdf::ElementPtr bounceElem = surfaceSDF->GetElement("bounce");
      if (_msg.has_restitution_coefficient())
      {
        bounceElem->GetElement("restitution_coefficient")->Set(
            _msg.restitution_coefficient());
      }
      if (_msg.has_bounce_threshold())
      {
        bounceElem->GetElement("threshold")->Set(
            _msg.bounce_threshold());
      }

      sdf::ElementPtr contactElem = surfaceSDF->GetElement("contact");

      if (_msg.has_collide_without_contact())
      {
        contactElem->GetElement("collide_without_contact")->Set(
            _msg.collide_without_contact());
      }
      if (_msg.has_collide_without_contact_bitmask())
      {
        contactElem->GetElement("collide_without_contact_bitmask")->Set(
            _msg.collide_without_contact_bitmask());
      }
      if (_msg.has_collide_bitmask())
      {
        contactElem->GetElement("collide_bitmask")->Set(
            _msg.collide_bitmask());
      }

      sdf::ElementPtr odeElem = contactElem->GetElement("ode");
      sdf::ElementPtr bulletElem = contactElem->GetElement("bullet");
      if (_msg.has_soft_cfm())
      {
        odeElem->GetElement("soft_cfm")->Set(_msg.soft_cfm());
        bulletElem->GetElement("soft_cfm")->Set(_msg.soft_cfm());
      }
      if (_msg.has_soft_erp())
      {
        odeElem->GetElement("soft_erp")->Set(_msg.soft_erp());
        bulletElem->GetElement("soft_erp")->Set(_msg.soft_erp());
      }
      if (_msg.has_kp())
      {
        odeElem->GetElement("kp")->Set(_msg.kp());
        bulletElem->GetElement("kp")->Set(_msg.kp());
      }
      if (_msg.has_kd())
      {
        odeElem->GetElement("kd")->Set(_msg.kd());
        bulletElem->GetElement("kd")->Set(_msg.kd());
      }
      if (_msg.has_max_vel())
      {
        odeElem->GetElement("max_vel")->Set(_msg.max_vel());
      }
      if (_msg.has_min_depth())
      {
        odeElem->GetElement("min_depth")->Set(_msg.min_depth());
      }

      return surfaceSDF;
    }

    /////////////////////////////////////////////////
    sdf::ElementPtr GeometryToSDF(const msgs::Geometry &_msg,
        sdf::ElementPtr _sdf)
    {
      sdf::ElementPtr geometrySDF;

      if (_sdf)
      {
        geometrySDF = _sdf;
      }
      else
      {
        geometrySDF.reset(new sdf::Element);
        sdf::initFile("geometry.sdf", geometrySDF);
      }

      if (!_msg.has_type())
        return geometrySDF;

      if (_msg.type() == msgs::Geometry::BOX &&
          _msg.has_box())
      {
        sdf::ElementPtr geom = geometrySDF->GetElement("box");
        msgs::BoxGeom boxGeom = _msg.box();
        if (boxGeom.has_size())
          geom->GetElement("size")->Set(ConvertIgn(boxGeom.size()));
      }
      else if (_msg.type() == msgs::Geometry::CYLINDER &&
          _msg.has_cylinder())
      {
        sdf::ElementPtr geom = geometrySDF->GetElement("cylinder");
        msgs::CylinderGeom cylinderGeom = _msg.cylinder();
        if (cylinderGeom.has_radius())
          geom->GetElement("radius")->Set(cylinderGeom.radius());
        if (cylinderGeom.has_length())
          geom->GetElement("length")->Set(cylinderGeom.length());
      }
      else if (_msg.type() == msgs::Geometry::SPHERE &&
          _msg.has_sphere())
      {
        sdf::ElementPtr geom = geometrySDF->GetElement("sphere");
        msgs::SphereGeom sphereGeom = _msg.sphere();
        if (sphereGeom.has_radius())
          geom->GetElement("radius")->Set(sphereGeom.radius());
      }
      else if (_msg.type() == msgs::Geometry::PLANE &&
          _msg.has_plane())
      {
        sdf::ElementPtr geom = geometrySDF->GetElement("plane");
        msgs::PlaneGeom planeGeom = _msg.plane();
        if (planeGeom.has_normal())
        {
          geom->GetElement("normal")->Set(ConvertIgn(planeGeom.normal()));
        }
        if (planeGeom.has_size())
          geom->GetElement("size")->Set(ConvertIgn(planeGeom.size()));
        if (planeGeom.has_d())
          gzerr << "sdformat doesn't have Plane.d variable" << std::endl;
      }
      else if (_msg.type() == msgs::Geometry::IMAGE &&
          _msg.has_image())
      {
        sdf::ElementPtr geom = geometrySDF->GetElement("image");
        msgs::ImageGeom imageGeom = _msg.image();
        if (imageGeom.has_scale())
          geom->GetElement("scale")->Set(imageGeom.scale());
        if (imageGeom.has_height())
          geom->GetElement("height")->Set(imageGeom.height());
        if (imageGeom.has_uri())
          geom->GetElement("uri")->Set(imageGeom.uri());
        if (imageGeom.has_threshold())
          geom->GetElement("threshold")->Set(imageGeom.threshold());
        if (imageGeom.has_granularity())
          geom->GetElement("granularity")->Set(imageGeom.granularity());
      }
      else if (_msg.type() == msgs::Geometry::HEIGHTMAP &&
          _msg.has_heightmap())
      {
        sdf::ElementPtr geom = geometrySDF->GetElement("heightmap");
        msgs::HeightmapGeom heightmapGeom = _msg.heightmap();
        if (heightmapGeom.has_size())
        {
          geom->GetElement("size")->Set(ConvertIgn(heightmapGeom.size()));
        }
        if (heightmapGeom.has_origin())
        {
          geom->GetElement("pos")->Set(ConvertIgn(heightmapGeom.origin()));
        }
        if (heightmapGeom.has_sampling())
        {
          // check if old version of sdformat is in use
          if (geom->HasElementDescription("sampling"))
          {
            geom->GetElement("sampling")->Set(heightmapGeom.sampling());
          }
        }
        if (heightmapGeom.has_use_terrain_paging())
        {
          geom->GetElement("use_terrain_paging")->Set(
              heightmapGeom.use_terrain_paging());
        }
        if (heightmapGeom.texture_size() > 0)
          while (geom->HasElement("texture"))
            geom->GetElement("texture")->RemoveFromParent();
        for (int i = 0; i < heightmapGeom.texture_size(); ++i)
        {
          gazebo::msgs::HeightmapGeom_Texture textureMsg =
              heightmapGeom.texture(i);
          sdf::ElementPtr textureElem = geom->AddElement("texture");
          textureElem->GetElement("diffuse")->Set(textureMsg.diffuse());
          textureElem->GetElement("normal")->Set(textureMsg.normal());
          textureElem->GetElement("size")->Set(textureMsg.size());
        }
        if (heightmapGeom.blend_size() > 0)
          while (geom->HasElement("blend"))
            geom->GetElement("blend")->RemoveFromParent();
        for (int i = 0; i < heightmapGeom.blend_size(); ++i)
        {
          gazebo::msgs::HeightmapGeom_Blend blendMsg =
              heightmapGeom.blend(i);
          sdf::ElementPtr blendElem = geom->AddElement("blend");
          blendElem->GetElement("min_height")->Set(blendMsg.min_height());
          blendElem->GetElement("fade_dist")->Set(blendMsg.fade_dist());
        }
        if (heightmapGeom.has_filename())
          geom->GetElement("uri")->Set(heightmapGeom.filename());
      }
      else if (_msg.type() == msgs::Geometry::MESH &&
          _msg.has_mesh())
      {
        sdf::ElementPtr geom = geometrySDF->GetElement("mesh");
        msgs::MeshGeom meshGeom = _msg.mesh();
        geom = msgs::MeshToSDF(meshGeom, geom);
      }
      else if (_msg.type() == msgs::Geometry::POLYLINE &&
          _msg.polyline_size() > 0)
      {
        if (_msg.polyline_size() > 0)
          while (geometrySDF->HasElement("polyline"))
            geometrySDF->GetElement("polyline")->RemoveFromParent();

        for (int j = 0; j < _msg.polyline_size(); ++j)
        {
          sdf::ElementPtr polylineElem = geometrySDF->AddElement("polyline");
          if (_msg.polyline(j).has_height())
            polylineElem->GetElement("height")->Set(_msg.polyline(j).height());
          for (int i = 0; i < _msg.polyline(j).point_size(); ++i)
          {
            sdf::ElementPtr pointElem = polylineElem->AddElement("point");
            pointElem->Set(ConvertIgn(_msg.polyline(j).point(i)));
          }
        }
      }
      else
      {
        gzerr << "Unrecognized geometry type" << std::endl;
      }
      return geometrySDF;
    }

    /////////////////////////////////////////////////
    sdf::ElementPtr MeshToSDF(const msgs::MeshGeom &_msg, sdf::ElementPtr _sdf)
    {
      sdf::ElementPtr meshSDF;

      if (_sdf)
      {
        meshSDF = _sdf;
      }
      else
      {
        meshSDF.reset(new sdf::Element);
        sdf::initFile("mesh_shape.sdf", meshSDF);
      }

      if (_msg.has_filename())
        meshSDF->GetElement("uri")->Set(_msg.filename());

      if (_msg.has_submesh())
      {
        sdf::ElementPtr submeshElem = meshSDF->GetElement("submesh");
        submeshElem->GetElement("name")->Set(_msg.submesh());
        if (_msg.has_center_submesh())
          submeshElem->GetElement("center")->Set(_msg.center_submesh());
      }
      if (_msg.has_scale())
      {
        meshSDF->GetElement("scale")->Set(ConvertIgn(_msg.scale()));
      }
      return meshSDF;
    }

    /////////////////////////////////////////////////
    sdf::ElementPtr PluginToSDF(const msgs::Plugin &_msg, sdf::ElementPtr _sdf)
    {
      sdf::ElementPtr pluginSDF;

      if (_sdf)
      {
        pluginSDF = _sdf;
      }
      else
      {
        pluginSDF.reset(new sdf::Element);
        sdf::initFile("plugin.sdf", pluginSDF);
      }

      // Use the SDF parser to read all the inner xml.
      std::string tmp = "<sdf version='" + std::string(SDF_VERSION) + "'>";
      tmp += "<plugin name='" + _msg.name() + "' filename='" +
        _msg.filename() + "'>";
      tmp += _msg.innerxml();
      tmp += "</plugin></sdf>";

      sdf::readString(tmp, pluginSDF);

      return pluginSDF;
    }

    ////////////////////////////////////////////////////////
    void AddLinkGeom(Model &_model, const Geometry &_geom)
    {
      _model.add_link();
      int linkCount = _model.link_size();
      auto link = _model.mutable_link(linkCount-1);
      {
        std::ostringstream linkName;
        linkName << "link_" << linkCount;
        link->set_name(linkName.str());
      }

      {
        link->add_collision();
        auto collision = link->mutable_collision(0);
        collision->set_name("collision");
        *(collision->mutable_geometry()) = _geom;
      }

      {
        link->add_visual();
        auto visual = link->mutable_visual(0);
        visual->set_name("visual");
        *(visual->mutable_geometry()) = _geom;

        auto script = visual->mutable_material()->mutable_script();
        script->add_uri();
        script->set_uri(0, "file://media/materials/scripts/gazebo.material");
        script->set_name("Gazebo/Grey");
      }
    }

    ////////////////////////////////////////////////////////
    void AddBoxLink(Model &_model, const double _mass,
                    const ignition::math::Vector3d &_size)
    {
      Geometry geometry;
      geometry.set_type(Geometry_Type_BOX);
      Set(geometry.mutable_box()->mutable_size(), _size);

      AddLinkGeom(_model, geometry);
      int linkCount = _model.link_size();
      auto link = _model.mutable_link(linkCount-1);

      ignition::math::MassMatrix3d m;
      if (!m.SetFromBox(_mass, _size))
      {
        gzerr << "Error computing inertia, not setting" << std::endl;
      }
      else
      {
        msgs::Set(link->mutable_inertial(), m);
      }
    }

    ////////////////////////////////////////////////////////
    void AddCylinderLink(Model &_model,
                         const double _mass,
                         const double _radius,
                         const double _length)
    {
      Geometry geometry;
      geometry.set_type(Geometry_Type_CYLINDER);
      geometry.mutable_cylinder()->set_radius(_radius);
      geometry.mutable_cylinder()->set_length(_length);

      AddLinkGeom(_model, geometry);
      int linkCount = _model.link_size();
      auto link = _model.mutable_link(linkCount-1);

      ignition::math::MassMatrix3d m;
      if (!m.SetFromCylinderZ(_mass, _length, _radius))
      {
        gzerr << "Error computing inertia, not setting" << std::endl;
      }
      else
      {
        msgs::Set(link->mutable_inertial(), m);
      }
    }

    ////////////////////////////////////////////////////////
    void AddSphereLink(Model &_model, const double _mass,
                       const double _radius)
    {
      Geometry geometry;
      geometry.set_type(Geometry_Type_SPHERE);
      geometry.mutable_sphere()->set_radius(_radius);

      AddLinkGeom(_model, geometry);
      int linkCount = _model.link_size();
      auto link = _model.mutable_link(linkCount-1);

      ignition::math::MassMatrix3d m;
      if (!m.SetFromSphere(_mass, _radius))
      {
        gzerr << "Error computing inertia, not setting" << std::endl;
      }
      else
      {
        msgs::Set(link->mutable_inertial(), m);
      }
    }

    ////////////////////////////////////////////////////////
    sdf::ElementPtr ModelToSDF(const msgs::Model &_msg, sdf::ElementPtr _sdf)
    {
      sdf::ElementPtr modelSDF;

      if (_sdf)
      {
        modelSDF = _sdf;
      }
      else
      {
        modelSDF.reset(new sdf::Element);
        sdf::initFile("model.sdf", modelSDF);
      }

      if (_msg.has_name())
        modelSDF->GetAttribute("name")->Set(_msg.name());
      // ignore the id field, since it's not used in sdformat
      if (_msg.has_is_static())
        modelSDF->GetElement("static")->Set(_msg.is_static());
      if (_msg.has_enable_wind())
        modelSDF->GetElement("enable_wind")->Set(_msg.enable_wind());
      if (_msg.has_pose())
        modelSDF->GetElement("pose")->Set(msgs::ConvertIgn(_msg.pose()));

      if (_msg.joint_size() > 0)
        while (modelSDF->HasElement("joint"))
          modelSDF->GetElement("joint")->RemoveFromParent();
      for (int i = 0; i < _msg.joint_size(); ++i)
      {
        sdf::ElementPtr jointElem = modelSDF->AddElement("joint");
        jointElem = JointToSDF(_msg.joint(i), jointElem);
      }

      if (_msg.plugin_size() > 0)
      {
        while (modelSDF->HasElement("plugin"))
          modelSDF->GetElement("plugin")->RemoveFromParent();
      }

      for (int i = 0; i < _msg.plugin_size(); ++i)
      {
        sdf::ElementPtr pluginElem = modelSDF->AddElement("plugin");
        pluginElem = PluginToSDF(_msg.plugin(i), pluginElem);
      }

      if (_msg.link_size())
        while (modelSDF->HasElement("link"))
          modelSDF->GetElement("link")->RemoveFromParent();
      for (int i = 0; i < _msg.link_size(); ++i)
      {
        sdf::ElementPtr linkElem = modelSDF->AddElement("link");
        linkElem = LinkToSDF(_msg.link(i), linkElem);
      }

      // ignore the deleted field, since it's not used in sdformat
      if (_msg.visual_size() > 0)
      {
        // model element in SDF cannot store visuals,
        // so ignore them for now
        gzerr << "Model visuals not yet parsed" << std::endl;
      }
      // ignore the scale field, since it's not used in sdformat

      return modelSDF;
    }

    ////////////////////////////////////////////////////////
    sdf::ElementPtr JointToSDF(const msgs::Joint &_msg, sdf::ElementPtr _sdf)
    {
      sdf::ElementPtr jointSDF;

      if (_sdf)
      {
        jointSDF = _sdf;
      }
      else
      {
        jointSDF.reset(new sdf::Element);
        sdf::initFile("joint.sdf", jointSDF);
      }

      if (_msg.has_name())
        jointSDF->GetAttribute("name")->Set(_msg.name());
      if (_msg.has_type())
        jointSDF->GetAttribute("type")->Set(ConvertJointType(_msg.type()));
      // ignore the id field, since it's not used in sdformat
      // ignore the parent_id field, since it's not used in sdformat
      // ignore the child_id field, since it's not used in sdformat
      // ignore the angle field, since it's not used in sdformat
      if (_msg.has_parent())
        jointSDF->GetElement("parent")->Set(_msg.parent());
      if (_msg.has_child())
        jointSDF->GetElement("child")->Set(_msg.child());
      if (_msg.has_pose())
        jointSDF->GetElement("pose")->Set(ConvertIgn(_msg.pose()));
      if (_msg.has_axis1())
        AxisToSDF(_msg.axis1(), jointSDF->GetElement("axis"));
      if (_msg.has_axis2())
        AxisToSDF(_msg.axis2(), jointSDF->GetElement("axis2"));

      auto odePhysicsElem = jointSDF->GetElement("physics")->GetElement("ode");
      if (_msg.has_cfm())
        odePhysicsElem->GetElement("cfm")->Set(_msg.cfm());
      if (_msg.has_bounce())
        odePhysicsElem->GetElement("bounce")->Set(_msg.bounce());
      if (_msg.has_velocity())
        odePhysicsElem->GetElement("velocity")->Set(_msg.velocity());
      if (_msg.has_fudge_factor())
        odePhysicsElem->GetElement("fudge_factor")->Set(_msg.fudge_factor());

      {
        auto limitElem = odePhysicsElem->GetElement("limit");
        if (_msg.has_limit_cfm())
          limitElem->GetElement("cfm")->Set(_msg.limit_cfm());
        if (_msg.has_limit_erp())
          limitElem->GetElement("erp")->Set(_msg.limit_erp());
      }

      {
        auto suspensionElem = odePhysicsElem->GetElement("suspension");
        if (_msg.has_suspension_cfm())
          suspensionElem->GetElement("cfm")->Set(_msg.suspension_cfm());
        if (_msg.has_suspension_erp())
          suspensionElem->GetElement("erp")->Set(_msg.suspension_erp());
      }

      // gearbox joint message fields
      if (_msg.has_gearbox())
      {
        msgs::Joint::Gearbox gearboxMsg = _msg.gearbox();
        if (gearboxMsg.has_gearbox_reference_body())
        {
          jointSDF->GetElement("gearbox_reference_body")->Set(
              gearboxMsg.gearbox_reference_body());
        }
        if (gearboxMsg.has_gearbox_ratio())
        {
          jointSDF->GetElement("gearbox_ratio")->Set(
              gearboxMsg.gearbox_ratio());
        }
      }

      // screw joint message field
      if (_msg.has_screw())
      {
        msgs::Joint::Screw screwMsg = _msg.screw();
        if (screwMsg.has_thread_pitch())
          jointSDF->GetElement("thread_pitch")->Set(screwMsg.thread_pitch());
      }

      /// \todo JointToSDF currently does not convert sensor data

      return jointSDF;
    }

    ////////////////////////////////////////////////////////
    void AxisToSDF(const msgs::Axis &_msg, sdf::ElementPtr _sdf)
    {
      if (_msg.has_xyz())
        _sdf->GetElement("xyz")->Set(ConvertIgn(_msg.xyz()));

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
      if (!_msg.xyz_expressed_in().empty())
      {
        _sdf->GetElement("xyz")
            ->GetAttribute("expressed_in")
            ->Set(_msg.xyz_expressed_in());
      }
      else if (_msg.use_parent_model_frame())
      {
        _sdf->GetElement("xyz")->GetAttribute("expressed_in")->Set("__model__");
      }
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

      {
        auto dynamicsElem = _sdf->GetElement("dynamics");
        if (_msg.has_damping())
          dynamicsElem->GetElement("damping")->Set(_msg.damping());
        if (_msg.has_friction())
          dynamicsElem->GetElement("friction")->Set(_msg.friction());
      }

      {
        auto limitElem = _sdf->GetElement("limit");
        if (_msg.has_limit_lower())
          limitElem->GetElement("lower")->Set(_msg.limit_lower());
        if (_msg.has_limit_upper())
          limitElem->GetElement("upper")->Set(_msg.limit_upper());
        if (_msg.has_limit_effort())
          limitElem->GetElement("effort")->Set(_msg.limit_effort());
        if (_msg.has_limit_velocity())
          limitElem->GetElement("velocity")->Set(_msg.limit_velocity());
      }
    }

    /////////////////////////////////////////////
    msgs::SensorNoise SensorNoiseFromSDF(sdf::ElementPtr _elem)
    {
      msgs::SensorNoise result;

      auto noiseType = _elem->Get<std::string>("type");

      if (noiseType == "none")
        result.set_type(msgs::SensorNoise::NONE);
      else if (noiseType == "gaussian")
        result.set_type(msgs::SensorNoise::GAUSSIAN);
      else if (noiseType == "gaussian_quantized")
        result.set_type(msgs::SensorNoise::GAUSSIAN_QUANTIZED);
      else
      {
        gzerr << "Invalid sensor noise type["
          << noiseType << "]. Using 'none'.\n";

        result.set_type(msgs::SensorNoise::NONE);
      }

      if (_elem->HasElement("mean"))
        result.set_mean(_elem->Get<double>("mean"));

      if (_elem->HasElement("stddev"))
        result.set_stddev(_elem->Get<double>("stddev"));

      if (_elem->HasElement("bias_mean"))
        result.set_bias_mean(_elem->Get<double>("bias_mean"));

      if (_elem->HasElement("bias_stddev"))
        result.set_bias_stddev(_elem->Get<double>("bias_stddev"));

      if (_elem->HasElement("precision"))
        result.set_precision(_elem->Get<double>("precision"));

      return result;
    }

    /////////////////////////////////////////////
    ignition::msgs::Color ConvertIgnMsg(const msgs::Color &_msg)
    {
      ignition::msgs::Color result;

      result.set_r(_msg.r());
      result.set_g(_msg.g());
      result.set_b(_msg.b());
      result.set_a(_msg.a());

      return result;
    }

    /////////////////////////////////////////////
    msgs::Color ConvertIgnMsg(const ignition::msgs::Color &_msg)
    {
      msgs::Color result;
      result.set_r(_msg.r());
      result.set_g(_msg.g());
      result.set_b(_msg.b());
      result.set_a(_msg.a());

      return result;
    }

    /////////////////////////////////////////////////
    ignition::msgs::Material::ShaderType ConvertIgnMsg(
        const msgs::Material::ShaderType &_type)
    {
      auto result = ignition::msgs::Material::VERTEX;

      if (_type == msgs::Material::VERTEX)
      {
        result = ignition::msgs::Material::VERTEX;
      }
      else if (_type == msgs::Material::PIXEL)
      {
        result = ignition::msgs::Material::PIXEL;
      }
      else if (_type == msgs::Material::NORMAL_MAP_OBJECT_SPACE)
      {
        result = ignition::msgs::Material::NORMAL_MAP_OBJECT_SPACE;
      }
      else if (_type == msgs::Material::NORMAL_MAP_TANGENT_SPACE)
      {
        result = ignition::msgs::Material::NORMAL_MAP_TANGENT_SPACE;
      }
      else
      {
        gzerr << "Unrecognized ShaderType, returning VERTEX" << std::endl;
      }
      return result;
    }

    /////////////////////////////////////////////////
    msgs::Material::ShaderType ConvertIgnMsg(
        const ignition::msgs::Material::ShaderType &_type)
    {
      auto result = msgs::Material::VERTEX;

      if (_type == ignition::msgs::Material::VERTEX)
      {
        result = msgs::Material::VERTEX;
      }
      else if (_type == ignition::msgs::Material::PIXEL)
      {
        result = msgs::Material::PIXEL;
      }
      else if (_type == ignition::msgs::Material::NORMAL_MAP_OBJECT_SPACE)
      {
        result = msgs::Material::NORMAL_MAP_OBJECT_SPACE;
      }
      else if (_type == ignition::msgs::Material::NORMAL_MAP_TANGENT_SPACE)
      {
        result = msgs::Material::NORMAL_MAP_TANGENT_SPACE;
      }
      else
      {
        gzerr << "Unrecognized ShaderType, returning VERTEX" << std::endl;
      }
      return result;
    }

    /////////////////////////////////////////////////
    ignition::msgs::Material::Script ConvertIgnMsg(
        const msgs::Material::Script &_script)
    {
      ignition::msgs::Material::Script result;

      for (auto s : _script.uri())
      {
        result.add_uri(s);
      }

      if (_script.has_name())
        result.set_name(_script.name());

      return result;
    }

    /////////////////////////////////////////////////
    msgs::Material::Script ConvertIgnMsg(
        const ignition::msgs::Material::Script &_script)
    {
      msgs::Material::Script result;

      for (auto s : _script.uri())
      {
        result.add_uri(s);
      }

      result.set_name(_script.name());

      return result;
    }

    /////////////////////////////////////////////
    ignition::msgs::Material ConvertIgnMsg(const msgs::Material &_msg)
    {
      ignition::msgs::Material result;

      if (_msg.has_script())
        result.mutable_script()->CopyFrom(ConvertIgnMsg(_msg.script()));
      if (_msg.has_shader_type())
        result.set_shader_type(ConvertIgnMsg(_msg.shader_type()));
      if (_msg.has_normal_map())
        result.set_normal_map(_msg.normal_map());
      if (_msg.has_ambient())
        result.mutable_ambient()->CopyFrom(ConvertIgnMsg(_msg.ambient()));
      if (_msg.has_diffuse())
        result.mutable_diffuse()->CopyFrom(ConvertIgnMsg(_msg.diffuse()));
      if (_msg.has_specular())
        result.mutable_specular()->CopyFrom(ConvertIgnMsg(_msg.specular()));
      if (_msg.has_emissive())
        result.mutable_emissive()->CopyFrom(ConvertIgnMsg(_msg.emissive()));
      if (_msg.has_lighting())
        result.set_lighting(_msg.lighting());

      return result;
    }

    /////////////////////////////////////////////
    msgs::Material ConvertIgnMsg(const ignition::msgs::Material &_msg)
    {
      msgs::Material result;

      if (_msg.has_script())
        result.mutable_script()->CopyFrom(ConvertIgnMsg(_msg.script()));
      result.set_shader_type(ConvertIgnMsg(_msg.shader_type()));
      result.set_normal_map(_msg.normal_map());
      if (_msg.has_ambient())
        result.mutable_ambient()->CopyFrom(ConvertIgnMsg(_msg.ambient()));
      if (_msg.has_diffuse())
        result.mutable_diffuse()->CopyFrom(ConvertIgnMsg(_msg.diffuse()));
      if (_msg.has_specular())
        result.mutable_specular()->CopyFrom(ConvertIgnMsg(_msg.specular()));
      if (_msg.has_emissive())
        result.mutable_emissive()->CopyFrom(ConvertIgnMsg(_msg.emissive()));
      result.set_lighting(_msg.lighting());

      return result;
    }
  }
}
