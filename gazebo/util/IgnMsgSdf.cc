/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <ignition/msgs/Utility.hh>
#include "gazebo/util/IgnMsgSdf.hh"

namespace gazebo
{
  namespace util
  {
    /////////////////////////////////////////////////
    sdf::ElementPtr Convert(const ignition::msgs::Plugin &_msg,
        sdf::ElementPtr _sdf)
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
      std::string tmp = "<sdf version='" SDF_VERSION "'>";
      tmp += "<plugin name='" + _msg.name() + "' filename='" +
        _msg.filename() + "'>";
      tmp += _msg.innerxml();
      tmp += "</plugin></sdf>";

      sdf::readString(tmp, pluginSDF);

      return pluginSDF;
    }

    /////////////////////////////////////////////////
    template<>
    ignition::msgs::Plugin Convert(const sdf::ElementPtr _sdf)
    {
      ignition::msgs::Plugin result;

      if (_sdf->GetName() != "plugin")
      {
        gzerr << "Tried to convert SDF [" << _sdf->GetName() <<
            "] into [plugin]" << std::endl;
        return result;
      }

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
    sdf::ElementPtr Convert(const ignition::msgs::Visual &_msg,
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
      {
        visualSDF->GetElement("pose")->Set(
            ignition::msgs::Convert(_msg.pose()));
      }

      // Load the geometry
      if (_msg.has_geometry())
      {
        sdf::ElementPtr geomElem = visualSDF->GetElement("geometry");
        geomElem = Convert(_msg.geometry(), geomElem);
      }

      /// Load the material
      if (_msg.has_material())
      {
        sdf::ElementPtr materialElem = visualSDF->GetElement("material");
        materialElem = Convert(_msg.material(), materialElem);
      }

      // Set plugins of the visual
      for (int i = 0; i < _msg.plugin_size(); ++i)
      {
        sdf::ElementPtr pluginElem = visualSDF->AddElement("plugin");
        pluginElem = Convert(_msg.plugin(i), pluginElem);
      }

      return visualSDF;

    }

    /////////////////////////////////////////////////
    template<>
    ignition::msgs::Visual Convert(const sdf::ElementPtr _sdf)
    {
      ignition::msgs::Visual result;

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
        geomMsg->CopyFrom(Convert<ignition::msgs::Geometry>(
            _sdf->GetElement("geometry")));
      }

      /// Load the material
      if (_sdf->HasElement("material"))
      {
        auto matMsg = result.mutable_material();
        matMsg->CopyFrom(Convert<ignition::msgs::Material>(
            _sdf->GetElement("material")));
      }

      // Set the origin of the visual
      if (_sdf->HasElement("pose"))
      {
        ignition::msgs::Set(result.mutable_pose(),
            _sdf->Get<ignition::math::Pose3d>("pose"));
      }

      // Set plugins of the visual
      if (_sdf->HasElement("plugin"))
      {
        auto pluginElem = _sdf->GetElement("plugin");
        while (pluginElem)
        {
          auto pluginMsg = result.add_plugin();
          pluginMsg->CopyFrom(Convert<ignition::msgs::Plugin>(pluginElem));
          pluginElem = pluginElem->GetNextElement("plugin");
        }
      }

      return result;
    }

    /////////////////////////////////////////////////
    sdf::ElementPtr Convert(const ignition::msgs::Material &_msg,
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
        auto scriptElem = materialSDF->GetElement("script");
        auto script = _msg.script();

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
        shaderElem->GetAttribute("type")->Set(Convert(_msg.shader_type()));
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
    template<>
    ignition::msgs::Material Convert(const sdf::ElementPtr _sdf)
    {
      ignition::msgs::Material result;

      if (_sdf->GetName() != "material")
      {
        gzerr << "Tried to convert SDF [" << _sdf->GetName() <<
            "] into [material]" << std::endl;
        return result;
      }

      if (_sdf->HasElement("script"))
      {
        auto scriptElem = _sdf->GetElement("script");
        result.mutable_script()->set_name(
            scriptElem->Get<std::string>("name"));

        auto uriElem = scriptElem->GetElement("uri");
        while (uriElem)
        {
          result.mutable_script()->add_uri(uriElem->Get<std::string>());
          uriElem = uriElem->GetNextElement("uri");
        }
      }

      if (_sdf->HasElement("lighting"))
      {
        result.set_lighting(_sdf->Get<bool>("lighting"));
      }

      if (_sdf->HasElement("shader"))
      {
        auto shaderElem = _sdf->GetElement("shader");

        result.set_shader_type(
            Convert<ignition::msgs::Material::ShaderType>(
            shaderElem->Get<std::string>("type")));

        if (shaderElem->HasElement("normal_map"))
        {
          result.set_normal_map(
              shaderElem->GetElement("normal_map")->Get<std::string>());
        }
      }

      if (_sdf->HasElement("ambient"))
      {
        result.mutable_ambient()->CopyFrom(Convert(
            _sdf->Get<common::Color>("ambient")));
      }
      if (_sdf->HasElement("diffuse"))
      {
        result.mutable_diffuse()->CopyFrom(Convert(
            _sdf->Get<common::Color>("diffuse")));
      }
      if (_sdf->HasElement("specular"))
      {
        result.mutable_specular()->CopyFrom(Convert(
            _sdf->Get<common::Color>("specular")));
      }
      if (_sdf->HasElement("emissive"))
      {
        result.mutable_emissive()->CopyFrom(Convert(
            _sdf->Get<common::Color>("emissive")));
      }

      return result;
    }

    /////////////////////////////////////////////////
    std::string Convert(const ignition::msgs::Material::ShaderType &_type)
    {
      std::string result;
      switch (_type)
      {
        case ignition::msgs::Material::VERTEX:
        {
          result = "vertex";
          break;
        }
        case ignition::msgs::Material::PIXEL:
        {
          result = "pixel";
          break;
        }
        case ignition::msgs::Material::NORMAL_MAP_OBJECT_SPACE:
        {
          result = "normal_map_object_space";
          break;
        }
        case ignition::msgs::Material::NORMAL_MAP_TANGENT_SPACE:
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
    template<>
    ignition::msgs::Material::ShaderType Convert(
        const std::string &_str)
    {
      auto result = ignition::msgs::Material::VERTEX;
      if (_str == "vertex")
      {
        result = ignition::msgs::Material::VERTEX;
      }
      else if (_str == "pixel")
      {
        result = ignition::msgs::Material::PIXEL;
      }
      else if (_str == "normal_map_object_space")
      {
        result = ignition::msgs::Material::NORMAL_MAP_OBJECT_SPACE;
      }
      else if (_str == "normal_map_tangent_space")
      {
        result = ignition::msgs::Material::NORMAL_MAP_TANGENT_SPACE;
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
    sdf::ElementPtr Convert(const ignition::msgs::Geometry &_msg,
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

      if (_msg.type() == ignition::msgs::Geometry::BOX &&
          _msg.has_box())
      {
        sdf::ElementPtr geom = geometrySDF->GetElement("box");
        ignition::msgs::BoxGeom boxGeom = _msg.box();
        if (boxGeom.has_size())
        {
          geom->GetElement("size")->Set(
              ignition::msgs::Convert(boxGeom.size()));
        }
      }
      else if (_msg.type() == ignition::msgs::Geometry::CYLINDER &&
          _msg.has_cylinder())
      {
        sdf::ElementPtr geom = geometrySDF->GetElement("cylinder");
        ignition::msgs::CylinderGeom cylinderGeom = _msg.cylinder();
        if (cylinderGeom.has_radius())
          geom->GetElement("radius")->Set(cylinderGeom.radius());
        if (cylinderGeom.has_length())
          geom->GetElement("length")->Set(cylinderGeom.length());
      }
      else if (_msg.type() == ignition::msgs::Geometry::SPHERE &&
          _msg.has_sphere())
      {
        sdf::ElementPtr geom = geometrySDF->GetElement("sphere");
        ignition::msgs::SphereGeom sphereGeom = _msg.sphere();
        if (sphereGeom.has_radius())
          geom->GetElement("radius")->Set(sphereGeom.radius());
      }
      else if (_msg.type() == ignition::msgs::Geometry::PLANE &&
          _msg.has_plane())
      {
        sdf::ElementPtr geom = geometrySDF->GetElement("plane");
        ignition::msgs::PlaneGeom planeGeom = _msg.plane();
        if (planeGeom.has_normal())
        {
          geom->GetElement("normal")->Set(
              ignition::msgs::Convert(planeGeom.normal()));
        }
        if (planeGeom.has_size())
        {
          geom->GetElement("size")->Set(
              ignition::msgs::Convert(planeGeom.size()));
        }
        if (planeGeom.has_d())
          gzerr << "sdformat doesn't have Plane.d variable" << std::endl;
      }
      else if (_msg.type() == ignition::msgs::Geometry::IMAGE &&
          _msg.has_image())
      {
        sdf::ElementPtr geom = geometrySDF->GetElement("image");
        ignition::msgs::ImageGeom imageGeom = _msg.image();
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
      else if (_msg.type() == ignition::msgs::Geometry::HEIGHTMAP &&
          _msg.has_heightmap())
      {
        sdf::ElementPtr geom = geometrySDF->GetElement("heightmap");
        ignition::msgs::HeightmapGeom heightmapGeom = _msg.heightmap();
        if (heightmapGeom.has_size())
        {
          geom->GetElement("size")->Set(
              ignition::msgs::Convert(heightmapGeom.size()));
        }
        if (heightmapGeom.has_origin())
        {
          geom->GetElement("pos")->Set(
              ignition::msgs::Convert(heightmapGeom.origin()));
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
          ignition::msgs::HeightmapGeom_Texture textureMsg =
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
          ignition::msgs::HeightmapGeom_Blend blendMsg =
              heightmapGeom.blend(i);
          sdf::ElementPtr blendElem = geom->AddElement("blend");
          blendElem->GetElement("min_height")->Set(blendMsg.min_height());
          blendElem->GetElement("fade_dist")->Set(blendMsg.fade_dist());
        }
        if (heightmapGeom.has_filename())
          geom->GetElement("uri")->Set(heightmapGeom.filename());
      }
      else if (_msg.type() == ignition::msgs::Geometry::MESH &&
          _msg.has_mesh())
      {
        auto geom = geometrySDF->GetElement("mesh");
        auto meshGeom = _msg.mesh();
        geom = Convert(meshGeom, geom);
      }
      else if (_msg.type() == ignition::msgs::Geometry::POLYLINE &&
          _msg.polyline_size() > 0)
      {
        if (_msg.polyline_size() > 0)
        {
          while (geometrySDF->HasElement("polyline"))
            geometrySDF->GetElement("polyline")->RemoveFromParent();
        }

        for (int j = 0; j < _msg.polyline_size(); ++j)
        {
          sdf::ElementPtr polylineElem = geometrySDF->AddElement("polyline");
          if (_msg.polyline(j).has_height())
            polylineElem->GetElement("height")->Set(_msg.polyline(j).height());
          for (int i = 0; i < _msg.polyline(j).point_size(); ++i)
          {
            sdf::ElementPtr pointElem = polylineElem->AddElement("point");
            pointElem->Set(ignition::msgs::Convert(_msg.polyline(j).point(i)));
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
    template<>
    ignition::msgs::Geometry Convert(const sdf::ElementPtr _sdf)
    {
      ignition::msgs::Geometry result;

      if (_sdf->GetName() != "geometry")
      {
        gzerr << "Cannot create a geometry message from an "
          << _sdf->GetName() << " SDF element.\n";
        return result;
      }

      // Load the geometry
      auto geomElem = _sdf->GetFirstElement();
      if (!geomElem)
      {
        gzerr << "Invalid geometry element." << std::endl;
        return result;
      }

      if (geomElem->GetName() == "box")
      {
        result.set_type(ignition::msgs::Geometry::BOX);
        ignition::msgs::Set(result.mutable_box()->mutable_size(),
            geomElem->Get<ignition::math::Vector3d>("size"));
      }
      else if (geomElem->GetName() == "cylinder")
      {
        result.set_type(ignition::msgs::Geometry::CYLINDER);
        result.mutable_cylinder()->set_radius(
            geomElem->Get<double>("radius"));
        result.mutable_cylinder()->set_length(
            geomElem->Get<double>("length"));
      }
      else if (geomElem->GetName() == "sphere")
      {
        result.set_type(ignition::msgs::Geometry::SPHERE);
        result.mutable_sphere()->set_radius(
            geomElem->Get<double>("radius"));
      }
      else if (geomElem->GetName() == "plane")
      {
        result.set_type(ignition::msgs::Geometry::PLANE);
        ignition::msgs::Set(result.mutable_plane()->mutable_normal(),
            geomElem->Get<ignition::math::Vector3d>("normal"));
        ignition::msgs::Set(result.mutable_plane()->mutable_size(),
            geomElem->Get<ignition::math::Vector2d>("size"));
      }
      else if (geomElem->GetName() == "polyline")
      {
        auto polylineElem = geomElem;
        result.set_type(ignition::msgs::Geometry::POLYLINE);
        while (polylineElem)
        {
          auto polylineMsg = result.add_polyline();
          polylineMsg->set_height(polylineElem->Get<double>("height"));
          auto pointElem = polylineElem->GetElement("point");
          while (pointElem)
          {
             auto point = pointElem->Get<ignition::math::Vector2d>();
             pointElem = pointElem->GetNextElement("point");
             auto ptMsg = polylineMsg->add_point();
             ignition::msgs::Set(ptMsg, point);
          }
          polylineElem = polylineElem->GetNextElement("polyline");
        }
      }
      else if (geomElem->GetName() == "image")
      {
        result.set_type(ignition::msgs::Geometry::IMAGE);
        result.mutable_image()->set_scale(
            geomElem->Get<double>("scale"));
        result.mutable_image()->set_height(
            geomElem->Get<double>("height"));
        result.mutable_image()->set_uri(
            geomElem->Get<std::string>("uri"));
      }
      else if (geomElem->GetName() == "heightmap")
      {
        result.set_type(ignition::msgs::Geometry::HEIGHTMAP);
        ignition::msgs::Set(result.mutable_heightmap()->mutable_size(),
            geomElem->Get<ignition::math::Vector3d>("size"));
        ignition::msgs::Set(result.mutable_heightmap()->mutable_origin(),
            geomElem->Get<ignition::math::Vector3d>("pos"));

        auto textureElem = geomElem->GetElement("texture");
        while (textureElem)
        {
          auto tex = result.mutable_heightmap()->add_texture();
          tex->set_diffuse(textureElem->Get<std::string>("diffuse"));
          tex->set_normal(textureElem->Get<std::string>("normal"));
          tex->set_size(textureElem->Get<double>("size"));
          textureElem = textureElem->GetNextElement("texture");
        }

        auto blendElem = geomElem->GetElement("blend");
        while (blendElem)
        {
          auto blend = result.mutable_heightmap()->add_blend();
          blend->set_min_height(blendElem->Get<double>("min_height"));
          blend->set_fade_dist(blendElem->Get<double>("fade_dist"));
          blendElem = blendElem->GetNextElement("blend");
        }

        // Set if the rendering engine uses terrain paging
        bool useTerrainPaging = geomElem->Get<bool>("use_terrain_paging");
        result.mutable_heightmap()->set_use_terrain_paging(useTerrainPaging);
      }
      else if (geomElem->GetName() == "mesh")
      {
        result.set_type(ignition::msgs::Geometry::MESH);
        result.mutable_mesh()->CopyFrom(
            Convert<ignition::msgs::MeshGeom>(geomElem));
      }
      else if (geomElem->GetName() == "empty")
      {
        result.set_type(ignition::msgs::Geometry::EMPTY);
      }
      else
      {
        gzerr << "Invalid geometry [" << geomElem->GetName() << "]"
            << std::endl;
        return result;
      }

      return result;
    }

    /////////////////////////////////////////////////
    sdf::ElementPtr Convert(const ignition::msgs::MeshGeom &_msg,
        sdf::ElementPtr _sdf)
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
        auto submeshElem = meshSDF->GetElement("submesh");
        submeshElem->GetElement("name")->Set(_msg.submesh());
        if (_msg.has_center_submesh())
          submeshElem->GetElement("center")->Set(_msg.center_submesh());
      }
      if (_msg.has_scale())
      {
        meshSDF->GetElement("scale")->Set(
            ignition::msgs::Convert(_msg.scale()));
      }
      return meshSDF;
    }

    /////////////////////////////////////////////////
    template<>
    ignition::msgs::MeshGeom Convert(const sdf::ElementPtr _sdf)
    {
      ignition::msgs::MeshGeom result;

      if (_sdf->GetName() != "mesh")
      {
        gzerr << "Cannot create a mesh message from an "
          << _sdf->GetName() << " SDF element.\n";
        return result;
      }

      ignition::msgs::Set(result.mutable_scale(),
          _sdf->Get<ignition::math::Vector3d>("scale"));

      result.set_filename(_sdf->Get<std::string>("uri"));

      if (_sdf->HasElement("submesh"))
      {
        auto submeshElem = _sdf->GetElement("submesh");
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

    /////////////////////////////////////////////
    ignition::msgs::Color Convert(const gazebo::common::Color &_c)
    {
      ignition::msgs::Color result;
      result.set_r(_c.r);
      result.set_g(_c.g);
      result.set_b(_c.b);
      result.set_a(_c.a);
      return result;
    }

    /////////////////////////////////////////////
    gazebo::common::Color Convert(const ignition::msgs::Color &_c)
    {
      return gazebo::common::Color(_c.r(), _c.g(), _c.b(), _c.a());
    }
  }
}

