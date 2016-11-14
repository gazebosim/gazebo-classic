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

    /////////////////////////////////////////////////
    sdf::ElementPtr Convert(const ignition::msgs::CameraSensor &_msg,
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
        auto imageElem = cameraSDF->GetElement("image");
        imageElem->GetElement("width")->Set(_msg.image_size().x());
        imageElem->GetElement("height")->Set(_msg.image_size().y());
      }
      if (_msg.has_image_format())
      {
        auto imageElem = cameraSDF->GetElement("image");
        imageElem->GetElement("format")->Set(_msg.image_format());
      }
      if (_msg.has_near_clip() || _msg.has_far_clip())
      {
        auto clipElem = cameraSDF->GetElement("clip");
        if (_msg.has_near_clip())
          clipElem->GetElement("near")->Set(_msg.near_clip());
        if (_msg.has_far_clip())
          clipElem->GetElement("far")->Set(_msg.far_clip());
      }

      if (_msg.has_distortion())
      {
        auto distortionMsg = _msg.distortion();
        auto distortionElem = cameraSDF->GetElement("distortion");

        if (distortionMsg.has_center())
        {
          distortionElem->GetElement("center")->Set(
              ignition::msgs::Convert(distortionMsg.center()));
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
    template<>
    ignition::msgs::CameraSensor Convert(const sdf::ElementPtr _sdf)
    {
      ignition::msgs::CameraSensor result;

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
        auto distElem = _sdf->GetElement("distortion");
        auto distortionMsg = result.mutable_distortion();

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
    template<>
    ignition::msgs::ContactSensor Convert(const sdf::ElementPtr _sdf)
    {
      ignition::msgs::ContactSensor result;
      result.set_collision_name(_sdf->Get<std::string>("collision"));
      return result;
    }

    /////////////////////////////////////////////////
    sdf::ElementPtr Convert(const ignition::msgs::GPSSensor &_msg,
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
          Convert(_msg.position().horizontal_noise(), noiseElem);
        }

        if (_msg.position().has_vertical_noise())
        {
          auto noiseElem = gpsSDF->GetElement("position_sensing")->GetElement(
              "vertical")->GetElement("noise");
          Convert(_msg.position().vertical_noise(), noiseElem);
        }
      }

      if (_msg.has_velocity())
      {
        if (_msg.velocity().has_horizontal_noise())
        {
          auto noiseElem = gpsSDF->GetElement("velocity_sensing")->GetElement(
              "horizontal")->GetElement("noise");
          Convert(_msg.velocity().horizontal_noise(), noiseElem);
        }

        if (_msg.velocity().has_vertical_noise())
        {
          auto noiseElem = gpsSDF->GetElement("velocity_sensing")->GetElement(
              "vertical")->GetElement("noise");
          Convert(_msg.velocity().vertical_noise(), noiseElem);
        }
      }

      return gpsSDF;
    }

    /////////////////////////////////////////////////
    template<>
    ignition::msgs::GPSSensor Convert(const sdf::ElementPtr _sdf)
    {
      ignition::msgs::GPSSensor result;

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
                ignition::msgs::SensorNoise *noiseMsg;

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

                noiseMsg->CopyFrom(Convert<ignition::msgs::SensorNoise>(
                    noiseElem));
              }
            }
          }
        }
      }

      return result;
    }

    /////////////////////////////////////////////////
    sdf::ElementPtr Convert(const ignition::msgs::IMUSensor &_msg,
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
          Convert(_msg.angular_velocity().x_noise(), noiseElem);
        }

        if (_msg.angular_velocity().has_y_noise())
        {
          auto noiseElem = imuSDF->GetElement("angular_velocity")->GetElement(
              "y")->GetElement("noise");
          Convert(_msg.angular_velocity().y_noise(), noiseElem);
        }

        if (_msg.angular_velocity().has_z_noise())
        {
          auto noiseElem = imuSDF->GetElement("angular_velocity")->GetElement(
              "z")->GetElement("noise");
          Convert(_msg.angular_velocity().z_noise(), noiseElem);
        }
      }

      if (_msg.has_linear_acceleration())
      {
        if (_msg.linear_acceleration().has_x_noise())
        {
          auto noiseElem = imuSDF->GetElement(
              "linear_acceleration")->GetElement("x")->GetElement("noise");
          Convert(_msg.linear_acceleration().x_noise(), noiseElem);
        }

        if (_msg.linear_acceleration().has_y_noise())
        {
          auto noiseElem = imuSDF->GetElement(
              "linear_acceleration")->GetElement("y")->GetElement("noise");
          Convert(_msg.linear_acceleration().y_noise(), noiseElem);
        }

        if (_msg.linear_acceleration().has_z_noise())
        {
          auto noiseElem = imuSDF->GetElement(
              "linear_acceleration")->GetElement("z")->GetElement("noise");
          Convert(_msg.linear_acceleration().z_noise(), noiseElem);
        }
      }

      return imuSDF;
    }

    /////////////////////////////////////////////////
    template<>
    ignition::msgs::IMUSensor Convert(const sdf::ElementPtr _sdf)
    {
      ignition::msgs::IMUSensor result;

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
              ignition::msgs::SensorNoise *noiseMsg;

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

              noiseMsg->CopyFrom(Convert<ignition::msgs::SensorNoise>(
                  noiseElem));
            }
          }
        }
      }

      return result;
    }

    /////////////////////////////////////////////////
    sdf::ElementPtr Convert(const ignition::msgs::LogicalCameraSensor &_msg,
        sdf::ElementPtr _sdf)
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
    template<>
    ignition::msgs::LogicalCameraSensor Convert(const sdf::ElementPtr _sdf)
    {
      ignition::msgs::LogicalCameraSensor result;
      result.set_near_clip(_sdf->Get<double>("near"));
      result.set_far_clip(_sdf->Get<double>("far"));
      result.set_horizontal_fov(_sdf->Get<double>("horizontal_fov"));
      result.set_aspect_ratio(_sdf->Get<double>("aspect_ratio"));
      return result;
    }

    /////////////////////////////////////////////////
    template<>
    ignition::msgs::RaySensor Convert(const sdf::ElementPtr _sdf)
    {
      ignition::msgs::RaySensor result;
      auto rangeElem = _sdf->GetElement("range");
      auto scanElem = _sdf->GetElement("scan");
      auto hscanElem = scanElem->GetElement("horizontal");
      auto vscanElem = scanElem->GetElement("vertical");

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
    sdf::ElementPtr Convert(const ignition::msgs::SensorNoise &_msg,
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

      if (_msg.type() == ignition::msgs::SensorNoise::NONE)
      {
        noiseSDF->GetAttribute("type")->Set("none");
      }
      else if (_msg.type() == ignition::msgs::SensorNoise::GAUSSIAN)
      {
        noiseSDF->GetAttribute("type")->Set("gaussian");
      }
      else if (_msg.type() == ignition::msgs::SensorNoise::GAUSSIAN_QUANTIZED)
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
    template<>
    ignition::msgs::SensorNoise Convert(const sdf::ElementPtr _sdf)
    {
      ignition::msgs::SensorNoise result;

      auto noiseType = _sdf->Get<std::string>("type");

      if (noiseType == "none")
        result.set_type(ignition::msgs::SensorNoise::NONE);
      else if (noiseType == "gaussian")
        result.set_type(ignition::msgs::SensorNoise::GAUSSIAN);
      else if (noiseType == "gaussian_quantized")
        result.set_type(ignition::msgs::SensorNoise::GAUSSIAN_QUANTIZED);
      else
      {
        gzerr << "Invalid sensor noise type["
          << noiseType << "]. Using 'none'.\n";

        result.set_type(ignition::msgs::SensorNoise::NONE);
      }

      if (_sdf->HasElement("mean"))
        result.set_mean(_sdf->Get<double>("mean"));

      if (_sdf->HasElement("stddev"))
        result.set_stddev(_sdf->Get<double>("stddev"));

      if (_sdf->HasElement("bias_mean"))
        result.set_bias_mean(_sdf->Get<double>("bias_mean"));

      if (_sdf->HasElement("bias_stddev"))
        result.set_bias_stddev(_sdf->Get<double>("bias_stddev"));

      if (_sdf->HasElement("precision"))
        result.set_precision(_sdf->Get<double>("precision"));

      return result;
    }

    /////////////////////////////////////////////////
    template<>
    ignition::msgs::Sensor Convert(const sdf::ElementPtr _sdf)
    {
      ignition::msgs::Sensor result;
      std::string type = _sdf->Get<std::string>("type");
      result.set_name(_sdf->Get<std::string>("name"));
      result.set_type(type);

      if (_sdf->HasElement("always_on"))
        result.set_always_on(_sdf->Get<bool>("always_on"));

      if (_sdf->HasElement("update_rate"))
        result.set_update_rate(_sdf->Get<double>("update_rate"));

      if (_sdf->HasElement("pose"))
      {
        ignition::msgs::Set(result.mutable_pose(),
            _sdf->Get<ignition::math::Pose3d>("pose"));
      }

      if (_sdf->HasElement("visualize"))
        result.set_visualize(_sdf->Get<bool>("visualize"));

      if (_sdf->HasElement("topic"))
        result.set_topic(_sdf->Get<std::string>("topic"));

      if (type == "camera")
      {
        result.mutable_camera()->CopyFrom(
            Convert<ignition::msgs::CameraSensor>(_sdf->GetElement("camera")));
      }
      else if (type == "ray")
      {
        result.mutable_ray()->CopyFrom(Convert<ignition::msgs::RaySensor>(
            _sdf->GetElement("ray")));
      }
      else if (type == "contact")
      {
        result.mutable_contact()->CopyFrom(
            Convert<ignition::msgs::ContactSensor>(
            _sdf->GetElement("contact")));
      }
      else if (type == "gps")
      {
        result.mutable_gps()->CopyFrom(
            Convert<ignition::msgs::GPSSensor>(_sdf->GetElement("gps")));
      }
      else if (type == "logical_camera")
      {
        result.mutable_logical_camera()->CopyFrom(
            Convert<ignition::msgs::LogicalCameraSensor>(
            _sdf->GetElement("logical_camera")));
      }
      else if (type == "imu")
      {
        result.mutable_imu()->CopyFrom(
          Convert<ignition::msgs::IMUSensor>(_sdf->GetElement("imu")));
      }
      else
      {
        gzwarn << "Conversion of sensor type[" << type << "] not supported."
          << std::endl;
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
