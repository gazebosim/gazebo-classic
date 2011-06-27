#include <boost/tokenizer.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "common/Console.hh"
#include "sdf/parser/parser.hh"

namespace sdf
{

bool getPlugins(TiXmlElement *_parentXml, 
                std::map<std::string, boost::shared_ptr<Plugin> > &_plugins)
{
  // Get all plugins 
  for (TiXmlElement* pluginXml = _parentXml->FirstChildElement("plugin"); 
      pluginXml; pluginXml = pluginXml->NextSiblingElement("plugin"))
  {
    boost::shared_ptr<Plugin> plugin;
    plugin.reset(new Plugin);

    if (initXml(pluginXml, plugin))
    {
      if (_plugins.find(plugin->name.GetValue()) != _plugins.end())
      {
        gzerr << "plugin '" << plugin->name.GetValue() << "' is not unique.\n";
        plugin.reset();
        return false;
      }
      else
      {
        _plugins.insert(make_pair(plugin->name.GetValue(),plugin));
      }
    }
    else
    {
      gzerr << "plugin xml is not initialized correctly\n";
      plugin.reset();
      return false;
    }
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Base> &_base)
{
  boost::char_separator<char> openSep("{");
  boost::char_separator<char> commaSep(",");

  std::list<TiXmlElement*> elements;

  // Remove all white space
  boost::erase_all(_base->xmlTree, " ");

  TiXmlElement *element = _config;
  elements.push_back(_config);

  boost::tokenizer<boost::char_separator<char> > openTok(_base->xmlTree, openSep);
  BOOST_FOREACH(std::string str1, openTok)
  {
    int index = str1.find(":");
    std::string name = str1.substr(0, index);
    str1.replace(0,index+1,"");

    if (elements.back()->Value() != name)
    {
      element = elements.back()->FirstChildElement(name);
      if (element)
        elements.push_back(element);
    }

    if (elements.size() > 0 && elements.back()->Value() == name)
    {
      boost::tokenizer<boost::char_separator<char> > tok2(str1, commaSep);
      BOOST_FOREACH(std::string str2, tok2)
      {
        TiXmlElement *currentElement = elements.back();
        while (str2.rfind("}") != std::string::npos)
        {
          str2.erase(str2.size()-1,1);
          elements.pop_back();
        }
        Param *param = Param::Find(_base->parameters, str2);
        if (param)
        {
          if (!param->Set(currentElement->Attribute( str2.c_str() )))
          {
            gzerr << "Unable to parse key[" << param->GetKey() << "] from string[" << str2 << "].\n";
            return false;
          }
        }
        else
        {
          gzerr << "Interface is missing key[" << str2 << "]\n";
          return false;
        }
      }
    }
  }

  return true;
}



bool initXml(TiXmlElement *_config, boost::shared_ptr<Sensor> &_sensor)
{
  _sensor->Clear();

  /// Get all the plugins
  getPlugins(_config, _sensor->plugins);
  boost::shared_ptr<Base> base = boost::shared_static_cast<Base>(_sensor);
  return initXml(_config, base);
}


bool initXml(TiXmlElement *_config, boost::shared_ptr<Material> &_material)
{
  _material->Clear();

  boost::shared_ptr<Base> base = boost::shared_static_cast<Base>(_material);
  return initXml(_config, base);
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Inertial> &_inertial)
{
  _inertial->Clear();
  boost::shared_ptr<Base> base = boost::shared_static_cast<Base>(_inertial);
  return initXml(_config, base);
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Collision> &_collision)
{
  _collision->Clear();
  boost::shared_ptr<Base> base = boost::shared_static_cast<Base>(_collision);
  return initXml(_config, base);
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Surface> &_surface)
{
  _surface->Clear();
  TiXmlElement *friction = _config->FirstChildElement("friction");
  if (friction)
  {
    _surface->frictions.clear();

    for (TiXmlElement* frictionXml = friction->FirstChildElement(); 
        frictionXml; frictionXml = frictionXml->NextSiblingElement())
    {
      if (std::string(frictionXml->Value()) == "ode")
      {
        boost::shared_ptr<ODEFriction> odeFriction( new ODEFriction );
        boost::shared_ptr<Base> base = boost::shared_static_cast<Base>(odeFriction);
        if (!initXml(frictionXml, base))
        {
          gzerr << "Unable to parse ODE Friction element\n";
          return false;
        }
        _surface->frictions.push_back( odeFriction );
      }

    }
  }

  TiXmlElement *contact = _config->FirstChildElement("contact");
  if (contact)
  {
    _surface->contacts.clear();

    for (TiXmlElement* contactXml = contact->FirstChildElement(); 
        contactXml; contactXml = contactXml->NextSiblingElement())
    {
      if (std::string(contactXml->Value()) == "ode")
      {
        boost::shared_ptr<ODESurfaceContact> odeContact( new ODESurfaceContact );
        boost::shared_ptr<Base> base = boost::shared_static_cast<Base>(odeFriction);
        if (!initXml(contactXml, base))
        {
          gzerr << "Unable to parse ODE Contact element\n";
          return false;
        }
        _surface->contacts.push_back( odeContact );
      }
    }
  }

  return true;
}

bool initXml(TiXmlElement *_config, boost::shared_ptr<Link> &_link)
{
  _link->Clear();

  // Multiple Visuals (optional)
  for (TiXmlElement* visXml = _config->FirstChildElement("visual"); 
      visXml; visXml = visXml->NextSiblingElement("visual"))
  {
    boost::shared_ptr<Visual> vis;
    vis.reset(new Visual);

    if (initXml(visXml,vis))
    {
      //  add Visual to the vector
      _link->visuals.push_back(vis);
    }
    else
    {
      gzerr << "Could not parse visual element for Link '" 
        << _link->name << "'\n";
      vis.reset();
      return false;
    }
  }

  // Multiple Collisions (optional)
  for (TiXmlElement* colXml = _config->FirstChildElement("collision"); 
      colXml; colXml = colXml->NextSiblingElement("collision"))
  {
    boost::shared_ptr<Collision> col;
    col.reset(new Collision);

    if (initXml(colXml,col))
    {
      // group exists, add Collision to the vector in the map
      _link->collisions.push_back(col);
    }
    else
    {
      gzerr << "Could not parse collision element for Link '" 
        <<  _link->name << "'\n";
      col.reset();
      return false;
    }
  }

  // Get all sensor elements
  for (TiXmlElement* sensorXml = _config->FirstChildElement("sensor"); 
      sensorXml; sensorXml = sensorXml->NextSiblingElement("sensor"))
  {
    std::string sensorType = sensorXml->Attribute("type");

    boost::shared_ptr<Sensor> sensor;
    if ( sensorType == "camera")
      sensor.reset(new Camera);
    else if (sensorType == "ray")
      sensor.reset(new Ray);
    else if (sensorType == "contact")
      sensor.reset(new Contact);
    else
    {
      gzerr << "Unknown sensor type[" << sensorType << "]\n";
      return false;
    }

    if (initXml(sensorXml,sensor))
    {
      if (_link->GetSensor(sensor->name.GetValue()))
      {
        gzerr << "sensor '" << sensor->name << "' is not unique.\n";
        sensor.reset();
        return false;
      }
      else
      {
        _link->sensors.insert(make_pair(sensor->name.GetValue(),sensor));
      }
    }
    else
    {
      gzerr << "sensor xml is not initialized correctly\n";
      sensor.reset();
      return false;
    }
  }

  boost::shared_ptr<Base> base = boost::shared_static_cast<Base>(_link);
  return initXml(_config, base);
}

////////////////////////////////////////////////////////////////////////////////
bool initFile(const std::string &_filename, boost::shared_ptr<Model> &_model)
{
  TiXmlDocument xmlDoc;
  xmlDoc.LoadFile(_filename);

  return initDoc(&xmlDoc,_model);
}

////////////////////////////////////////////////////////////////////////////////
bool initString(const std::string &_xmlString, boost::shared_ptr<Model> &_model)
{
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(_xmlString.c_str());

  return initDoc(&xmlDoc, _model);
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Load Model from TiXMLDocument
bool initDoc(TiXmlDocument *_xmlDoc, boost::shared_ptr<Model> &_model)
{
  if (!_xmlDoc)
  {
    gzerr << "Could not parse the xml\n";
    return false;
  }

  TiXmlElement *modelXml = _xmlDoc->FirstChildElement("model");
  if (!modelXml)
  {
    gzerr << "Could not find the 'model' element in the xml file\n";
    return false;
  }

  return initXml(modelXml, _model);
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Load Model from TiXMLElement
bool initXml(TiXmlElement *_xml, boost::shared_ptr<Model> &_model)
{

  if (!_xml) 
    return false;

  _model->Clear();

  // Get all Link elements
  for (TiXmlElement* linkXml = _xml->FirstChildElement("link"); 
      linkXml; linkXml = linkXml->NextSiblingElement("link"))
  {
    boost::shared_ptr<Link> link;
    link.reset(new Link);

    if (initXml(linkXml,link))
    {
      if (_model->GetLink(link->name.GetValue()))
      {
        gzerr << "link '" << link->name << "' is not unique.\n";
        link.reset();
        return false;
      }
      else
      {
        _model->links.insert(make_pair(link->name.GetValue(),link));
      }
    }
    else
    {
      gzerr << "link xml is not initialized correctly\n";
      link.reset();
      return false;
    }
  }

  if (_model->links.empty())
  {
    gzerr << "No link elements found in xml file\n";
    return false;
  }

  // Get all Joint elements
  for (TiXmlElement* jointXml = _xml->FirstChildElement("joint"); 
      jointXml; jointXml = jointXml->NextSiblingElement("joint"))
  {
    boost::shared_ptr<Joint> joint;
    joint.reset(new Joint);

    if (initXml(jointXml,joint))
    {
      if (_model->GetJoint(joint->name.GetValue()))
      {
        gzerr << "joint '" << joint->name << "' is not unique.\n";
        joint.reset();
        return false;
      }
      else
      {
        _model->joints.insert(make_pair(joint->name.GetValue(),joint));
      }
    }
    else
    {
      gzerr << "joint xml is not initialized correctly\n";
      joint.reset();
      return false;
    }
  }

  /// Get all the plugins
  getPlugins(_xml, _model->plugins);

  return true;
}


////////////////////////////////////////////////////////////////////////////////
bool initFile(const std::string &_filename, boost::shared_ptr<World> &_world)
{
  TiXmlDocument xmlDoc;
  xmlDoc.LoadFile(_filename);

  return initDoc(&xmlDoc, _world);
}

////////////////////////////////////////////////////////////////////////////////
bool initString(const std::string &_xmlString, boost::shared_ptr<World> &_world)
{
  TiXmlDocument xmlDoc;
  xmlDoc.Parse(_xmlString.c_str());

  return initDoc(&xmlDoc,_world);
}

////////////////////////////////////////////////////////////////////////////////
bool initDoc(TiXmlDocument *_xmlDoc, boost::shared_ptr<World> &_world)
{
  if (!_xmlDoc)
  {
    gzerr << "Could not parse the xml\n";
    return false;
  }

  TiXmlElement *worldXml = _xmlDoc->FirstChildElement("world");
  if (!worldXml)
  {
    gzerr << "Could not find the 'world' element in the xml file\n";
    return false;
  }

  return initXml(worldXml, _world);
}

////////////////////////////////////////////////////////////////////////////////
bool initXml(TiXmlElement *_worldXml, boost::shared_ptr<World> &_world)
{
  if (!_worldXml) 
  {
    gzerr << "Error: World XML is NULL\n";
    return false;
  }

  _world->Clear();

  // Get all light elements
  for (TiXmlElement* lightXml = _worldXml->FirstChildElement("light"); 
      lightXml; lightXml = lightXml->NextSiblingElement("light"))
  {
    boost::shared_ptr<Light> light;
    light.reset(new Light);

    if (initXml(lightXml,light))
    {
      _world->lights.push_back(light);
    }
    else
    {
      gzerr << "light xml is not initialized correctly\n";
      light.reset();
      return false;
    }
  }


  // Get all model elements
  for (TiXmlElement* modelXml = _worldXml->FirstChildElement("model"); 
      modelXml; modelXml = modelXml->NextSiblingElement("model"))
  {
    boost::shared_ptr<Model> model;
    model.reset(new Model);

    if (initXml(modelXml,model))
    {
      if (_world->GetModel(model->name.GetValue()))
      {
        gzerr << "model '" << model->name << "' is not unique.\n";
        model.reset();
        return false;
      }
      else
      {
        _world->models.insert(make_pair(model->name.GetValue(),model));
      }
    }
    else
    {
      gzerr << "model xml is not initialized correctly\n";
      model.reset();
      return false;
    }
  }

  // Get all Joint elements
  for (TiXmlElement* jointXml = _worldXml->FirstChildElement("joint"); 
      jointXml; jointXml = jointXml->NextSiblingElement("joint"))
  {
    boost::shared_ptr<Joint> joint;
    joint.reset(new Joint);

    if (initXml(jointXml,joint))
    {
      if (_world->GetJoint(joint->name.GetValue()))
      {
        gzerr << "joint '" << joint->name << "s' is not unique.\n";
        joint.reset();
        return false;
      }
      else
      {
        _world->joints.insert(make_pair(joint->name.GetValue(),joint));
      }
    }
    else
    {
      gzerr << "joint xml is not initialized correctly\n";
      joint.reset();
      return false;
    }
  }

  /// Get all the plugins
  getPlugins(_worldXml, _world->plugins);

  return true;
}

/*

bool saveXml(const std::string &filename, const boost::shared_ptr<World> &_world)
{
  TiXmlDocument doc;

  TiXmlDeclaration *decl = new TiXmlDeclaration( "1.0", "", "" );
  doc.LinkEndChild(decl);

  TiXmlElement *root = new TiXmlElement("gazebo");
  doc.LinkEndChild(root);
  root->SetAttribute("version","1.0");

  TiXmlElement *worldElement = new TiXmlElement("world");
  root->LinkEndChild(worldElement);

  worldElement->SetAttribute(_world->name.GetKey(), 
                             _world->name.GetAsString());

  saveXml(worldElement, _world->scene); 
  saveXml(worldElement, _world->physics); 

  // Save the models
  std::map<std::string, boost::shared_ptr<Model> >::const_iterator miter;
  for (miter = _world->models.begin(); miter != _world->models.end(); miter++)
  {
    saveXml(worldElement, miter->second); 
  }

  // Save the joints
  std::map<std::string, boost::shared_ptr<Joint> >::const_iterator jiter;
  for (jiter = _world->joints.begin(); jiter != _world->joints.end(); jiter++)
  {
    saveXml(worldElement, jiter->second); 
  }

  // Save the plugins
  std::map<std::string, boost::shared_ptr<Plugin> >::const_iterator piter;
  for (piter = _world->plugins.begin(); piter != _world->plugins.end(); piter++)
  {
    saveXml(worldElement, piter->second); 
  }

  TiXmlPrinter printer;
  printer.SetIndent("  ");

  doc.Accept(&printer);

  std::fstream out(filename.c_str(), std::ios::out);
  out << printer.CStr();
  out.close();

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Scene> &_scene)
{
  TiXmlElement *sceneNode = new TiXmlElement("scene");
  _parent->LinkEndChild( sceneNode );

  TiXmlElement *ambientNode = new TiXmlElement("ambient");
  sceneNode->LinkEndChild( ambientNode );
  ambientNode->SetAttribute( _scene->ambientColor.GetKey(),
                             _scene->ambientColor.GetAsString() );

  TiXmlElement *backgroundNode = new TiXmlElement("background");
  sceneNode->LinkEndChild( backgroundNode );
  backgroundNode->SetAttribute( _scene->backgroundColor.GetKey(),
                                _scene->backgroundColor.GetAsString() );


  TiXmlElement *skyNode = new TiXmlElement("sky");
  backgroundNode->LinkEndChild( skyNode );
  skyNode->SetAttribute( _scene->skyMaterial.GetKey(),
                         _scene->skyMaterial.GetAsString());

  // Save shadows
  TiXmlElement *shadowNode = new TiXmlElement("shadows");
  sceneNode->LinkEndChild( shadowNode );
  shadowNode->SetAttribute( _scene->shadowEnabled.GetKey(),
                            _scene->shadowEnabled.GetAsString());
  shadowNode->SetAttribute( _scene->shadowColor.GetKey(),
                            _scene->shadowColor.GetAsString());
  shadowNode->SetAttribute( _scene->shadowType.GetKey(),
                            _scene->shadowType.GetAsString());

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Physics> &_physics)
{
  TiXmlElement *physicsNode = new TiXmlElement("physics");
  _parent->LinkEndChild( physicsNode );

  physicsNode->SetAttribute( _physics->type.GetKey(), 
                             _physics->type.GetAsString() );

  TiXmlElement *gravityNode = new TiXmlElement("gravity");
  physicsNode->LinkEndChild( gravityNode );
  gravityNode->SetAttribute( _physics->gravity.GetKey(),
                             _physics->gravity.GetAsString() );

  if (_physics->type.GetAsString() == "ode")
  {
    boost::shared_ptr<OpenDynamicsEngine> openDynamicsEngine = boost::shared_static_cast<OpenDynamicsEngine>(_physics->engine);
    saveXml( physicsNode, openDynamicsEngine );
  }

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<OpenDynamicsEngine> &_engine)
{
  TiXmlElement *odeNode = new TiXmlElement("ode");
  _parent->LinkEndChild( odeNode );

  TiXmlElement *solverNode = new TiXmlElement("solver");
  odeNode->LinkEndChild( solverNode );

  solverNode->SetAttribute( _engine->solverType.GetKey(),
                            _engine->solverType.GetAsString() );
  solverNode->SetAttribute( _engine->dt.GetKey(),
                            _engine->dt.GetAsString() );
  solverNode->SetAttribute( _engine->iters.GetKey(),
                            _engine->iters.GetAsString() );
  solverNode->SetAttribute( _engine->sor.GetKey(),
                            _engine->sor.GetAsString() );


  TiXmlElement *constraintsNode = new TiXmlElement("constraints");
  odeNode->LinkEndChild( constraintsNode );
  constraintsNode->SetAttribute( _engine->cfm.GetKey(),
                                 _engine->cfm.GetAsString() );
  constraintsNode->SetAttribute( _engine->erp.GetKey(),
                                 _engine->erp.GetAsString() );
  constraintsNode->SetAttribute( _engine->contactMaxCorrectingVel.GetKey(),
                                 _engine->contactMaxCorrectingVel.GetAsString() );
  constraintsNode->SetAttribute( _engine->contactSurfaceLayer.GetKey(),
                                 _engine->contactSurfaceLayer.GetAsString() );

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Joint> &_joint)
{
  TiXmlElement *jointNode = new TiXmlElement("joint");
  _parent->LinkEndChild( jointNode );

  jointNode->SetAttribute(_joint->name.GetKey(),
                          _joint->name.GetAsString());
  jointNode->SetAttribute(_joint->type.GetKey(),
                          _joint->type.GetAsString());

  TiXmlElement *parentNode = new TiXmlElement("parent");
  jointNode->LinkEndChild(parentNode);
  parentNode->SetAttribute( _joint->parentLinkName.GetKey(),
                            _joint->parentLinkName.GetAsString() );

  TiXmlElement *childNode = new TiXmlElement("child");
  jointNode->LinkEndChild(childNode);
  childNode->SetAttribute( _joint->childLinkName.GetKey(),
                           _joint->childLinkName.GetAsString() );

  jointNode->SetAttribute(_joint->origin.GetKey(),
                          _joint->origin.GetAsString());

  // Axis 1
  TiXmlElement *axisNode = new TiXmlElement("axis");
  jointNode->LinkEndChild(axisNode);
  axisNode->SetAttribute( _joint->axis.GetKey(), _joint->axis.GetAsString() );

  // Axis 2
  TiXmlElement *axis2Node = new TiXmlElement("axis2");
  jointNode->LinkEndChild(axis2Node);
  axis2Node->SetAttribute( _joint->axis2.GetKey(), _joint->axis2.GetAsString());

  // Dynamics
  if (_joint->dynamics)
  {
    TiXmlElement *dynamicsNode = new TiXmlElement("dynamics");
    jointNode->LinkEndChild(dynamicsNode);
    dynamicsNode->SetAttribute( _joint->dynamics->damping.GetKey(),
        _joint->dynamics->damping.GetAsString() );
    dynamicsNode->SetAttribute( _joint->dynamics->friction.GetKey(),
        _joint->dynamics->friction.GetAsString() );
  }

  // Limit
  if (_joint->limits)
  {
    TiXmlElement *limitNode = new TiXmlElement("limit");
    jointNode->LinkEndChild(limitNode);
    limitNode->SetAttribute( _joint->limits->lower.GetKey(),
        _joint->limits->lower.GetAsString() );
    limitNode->SetAttribute( _joint->limits->upper.GetKey(),
        _joint->limits->upper.GetAsString() );
    limitNode->SetAttribute( _joint->limits->effort.GetKey(),
        _joint->limits->effort.GetAsString() );
    limitNode->SetAttribute( _joint->limits->velocity.GetKey(),
        _joint->limits->velocity.GetAsString() );
  }

  return true;
}

bool saveXml(TiXmlElement *_parent, const gazebo::math::Vector3 &_vec)
{
  std::ostringstream stream;
  stream << _vec;
  _parent->SetAttribute("xyz", stream.str());
  return true;
}

bool saveXml(TiXmlElement *_parent, const gazebo::math::Quaternion &_rot)
{
  std::ostringstream stream;
  stream << _rot;
  _parent->SetAttribute("rot", stream.str());
  return true;
}


//bool saveXml(TiXmlElement *_parent, const ParamT<gazebo::math::Pose,true> &_pose)
//{
//  TiXmlElement *originNode = new TiXmlElement(_pose.GetKey());
//  _parent->LinkEndChild( originNode );
//
//  saveXml(originNode, _pose.GetValue());
//
//  return true;
//}
//
//bool saveXml(TiXmlElement *_parent, const ParamT<gazebo::math::Pose,false> &_pose)
//{
//  TiXmlElement *originNode = new TiXmlElement(_pose.GetKey());
//  _parent->LinkEndChild( originNode );
//
//  saveXml(originNode, _pose.GetValue());
//
//  return true;
//}

//bool saveXml(TiXmlElement *_parent, const gazebo::math::Pose &_pose)
//{
//  std::ostringstream poseStream;
//  poseStream << _pose;
//  _parent->SetAttribute("pose", poseStream.str());
//  return true;
//}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Model> &_model)
{
  TiXmlElement *modelNode = new TiXmlElement("model");
  _parent->LinkEndChild( modelNode );


  modelNode->SetAttribute( _model->name.GetKey(),
                           _model->name.GetAsString() );

  // Save the links
  std::map<std::string, boost::shared_ptr<Link> >::const_iterator liter;
  for (liter = _model->links.begin(); liter != _model->links.end(); liter++)
  {
    saveXml(modelNode, liter->second);
  }

  // Save the joints
  std::map<std::string, boost::shared_ptr<Joint> >::const_iterator jiter;
  for (jiter = _model->joints.begin(); jiter != _model->joints.end(); jiter++)
  {
    saveXml(modelNode, jiter->second);
  }

  // Save the plugins
  std::map<std::string, boost::shared_ptr<Plugin> >::const_iterator piter;
  for (piter = _model->plugins.begin(); piter != _model->plugins.end(); piter++)
  {
    saveXml(modelNode, piter->second);
  }

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Link> &_link)
{
  TiXmlElement *linkNode = new TiXmlElement("link");
  _parent->LinkEndChild( linkNode );

  linkNode->SetAttribute(_link->name.GetKey(),
                         _link->name.GetAsString());
  linkNode->SetAttribute(_link->selfCollide.GetKey(),
                         _link->selfCollide.GetAsString());
  linkNode->SetAttribute(_link->gravity.GetKey(),
                         _link->gravity.GetAsString());

  // Save interial
  saveXml(linkNode, _link->inertial);

  // Save visuals
  std::vector<boost::shared_ptr<Visual> >::const_iterator viter;
  for (viter = _link->visuals.begin(); viter != _link->visuals.end(); viter++)
  {
    saveXml(linkNode, *viter);
  }

  // Save collisions
  std::vector<boost::shared_ptr<Collision> >::const_iterator citer;
  for (citer = _link->collisions.begin(); citer != _link->collisions.end(); citer++)
  {
    saveXml(linkNode, *citer);
  }

  // Save sensors
  std::map<std::string, boost::shared_ptr<Sensor> >::const_iterator siter;
  for (siter = _link->sensors.begin(); siter != _link->sensors.end(); siter++)
  {
    saveXml(linkNode, siter->second);
  }

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Plugin> &_plugin)
{
  TiXmlElement *pluginNode = new TiXmlElement("plugin");
  _parent->LinkEndChild( pluginNode );
  pluginNode->SetAttribute(_plugin->name.GetKey(),
                           _plugin->name.GetAsString());
  pluginNode->SetAttribute(_plugin->filename.GetKey(),
                           _plugin->filename.GetAsString());

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Visual> &_visual)
{
  TiXmlElement *visualNode = new TiXmlElement("visual");
  _parent->LinkEndChild( visualNode );
  visualNode->SetAttribute( _visual->castShadows.GetKey(),
                            _visual->castShadows.GetAsString());
  visualNode->SetAttribute( _visual->name.GetKey(),
                            _visual->name.GetAsString());

  visualNode->SetAttribute( _visual->origin.GetKey(),
                            _visual->origin.GetAsString() );

  saveXml(visualNode, _visual->geometry);
  saveXml(visualNode, _visual->material);

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Geometry> &_geom)
{
  TiXmlElement *shapeNode = NULL;
  TiXmlElement *geomNode = new TiXmlElement("geometry");
  _parent->LinkEndChild( geomNode );

  if (_geom->type == Geometry::PLANE)
  {
    boost::shared_ptr<Plane> shape = boost::shared_static_cast<Plane>(_geom);
    shapeNode = new TiXmlElement("plane");
    shapeNode->SetAttribute(shape->normal.GetKey(),shape->normal.GetAsString());
  }
  else if (_geom->type == Geometry::SPHERE)
  {
    boost::shared_ptr<Sphere> shape = boost::shared_static_cast<Sphere>(_geom);
    shapeNode = new TiXmlElement("sphere");
    shapeNode->SetAttribute(shape->radius.GetKey(),shape->radius.GetAsString());
  }
  else if (_geom->type == Geometry::BOX)
  {
    boost::shared_ptr<Box> shape = boost::shared_static_cast<Box>(_geom);
    shapeNode = new TiXmlElement("box");
    std::ostringstream stream;
    stream << shape->size;
    shapeNode->SetAttribute("size", stream.str());
  }
  else if (_geom->type == Geometry::CYLINDER)
  {
    boost::shared_ptr<Cylinder> shape = boost::shared_static_cast<Cylinder>(_geom);
    shapeNode = new TiXmlElement("cylinder");
    shapeNode->SetAttribute(shape->radius.GetKey(),shape->radius.GetAsString());
    shapeNode->SetAttribute(shape->length.GetKey(),shape->length.GetAsString());
  }
  else if (_geom->type == Geometry::MESH)
  {
    boost::shared_ptr<Mesh> shape = boost::shared_static_cast<Mesh>(_geom);
    shapeNode = new TiXmlElement("mesh");
    shapeNode->SetAttribute(shape->filename.GetKey(),
                            shape->filename.GetAsString());
    shapeNode->SetAttribute(shape->scale.GetKey(),
                            shape->scale.GetAsString());
  }
  else
  {
    gzerr << "Unknown geometry type[" << _geom->type << "]\n";
    return false;
  }

  geomNode->LinkEndChild( shapeNode );

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Material> &_mat)
{
  TiXmlElement *matNode = new TiXmlElement("material");
  _parent->LinkEndChild( matNode );
  matNode->SetAttribute( _mat->script.GetKey(), _mat->script.GetAsString());

  TiXmlElement *colorNode = new TiXmlElement("color");
  colorNode->SetAttribute( _mat->color.GetKey(), _mat->color.GetAsString());

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Collision> &_collision)
{
  TiXmlElement *collisionNode = new TiXmlElement("collision");
  _parent->LinkEndChild( collisionNode );
  collisionNode->SetAttribute( _collision->name.GetKey(),
                               _collision->name.GetAsString());

  collisionNode->SetAttribute( _collision->origin.GetKey(),
                               _collision->origin.GetAsString());

  saveXml(collisionNode, _collision->geometry);

  if (_collision->surface)
    saveXml(collisionNode, _collision->surface);

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Sensor> &_sensor)
{
  TiXmlElement *sensorNode = new TiXmlElement("sensor");
  _parent->LinkEndChild( sensorNode );

  sensorNode->SetAttribute( _sensor->name.GetKey(),
                            _sensor->name.GetAsString() );
  sensorNode->SetAttribute( _sensor->type.GetKey(),
                            _sensor->type.GetAsString() );
  sensorNode->SetAttribute( _sensor->alwaysOn.GetKey(),
                            _sensor->alwaysOn.GetAsString() );
  sensorNode->SetAttribute( _sensor->updateRate.GetKey(),
                            _sensor->updateRate.GetAsString() );
  sensorNode->SetAttribute( _sensor->origin.GetKey(),
                            _sensor->origin.GetAsString() );

  if ((*_sensor->type) == "camera")
  {
    boost::shared_ptr<Camera> camera = boost::shared_static_cast<Camera>(_sensor);
    saveXml( sensorNode, camera);
  }
  else if ( (*_sensor->type) == "ray")
  {
    boost::shared_ptr<Ray> ray = boost::shared_static_cast<Ray>(_sensor);
    saveXml( sensorNode, ray);
  }
  else if ( (*_sensor->type) != "contact")
  {
    gzerr << "Unknown sensor type[" << _sensor->type << "]\n";
    return false;
  }

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Camera> &_camera)
{
  TiXmlElement *hfovNode = new TiXmlElement("horizontal_fov");
  _parent->LinkEndChild( hfovNode );
  hfovNode->SetAttribute( _camera->horizontalFov.GetKey(),
                          _camera->horizontalFov.GetAsString() );

  TiXmlElement *imageNode = new TiXmlElement("image");
  _parent->LinkEndChild( imageNode );
  imageNode->SetAttribute( _camera->imageWidth.GetKey(),
                           _camera->imageWidth.GetAsString() );
  imageNode->SetAttribute( _camera->imageHeight.GetKey(),
                           _camera->imageHeight.GetAsString() );

  TiXmlElement *clipNode = new TiXmlElement("clip");
  _parent->LinkEndChild( clipNode );
  clipNode->SetAttribute( _camera->clipNear.GetKey(),
                          _camera->clipNear.GetAsString() );
  clipNode->SetAttribute( _camera->clipFar.GetKey(),
                          _camera->clipFar.GetAsString() );

  TiXmlElement *saveNode = new TiXmlElement("save");
  _parent->LinkEndChild( saveNode );
  saveNode->SetAttribute( _camera->saveEnabled.GetKey(),
                          _camera->saveEnabled.GetAsString() );
  saveNode->SetAttribute( _camera->savePath.GetKey(),
                          _camera->savePath.GetAsString() );

  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Ray> &_ray)
{
  TiXmlElement *scanNode = new TiXmlElement("scan");
  _parent->LinkEndChild( scanNode );
  scanNode->SetAttribute( _ray->display.GetKey(),
                          _ray->display.GetAsString() );

  TiXmlElement *horizontalNode = new TiXmlElement("horizontal");
  _parent->LinkEndChild( horizontalNode );
  horizontalNode->SetAttribute( _ray->horizontalSamples.GetKey(),
                                _ray->horizontalSamples.GetAsString() );
  horizontalNode->SetAttribute( _ray->horizontalResolution.GetKey(),
                                _ray->horizontalResolution.GetAsString() );
  horizontalNode->SetAttribute( _ray->horizontalMinAngle.GetKey(),
                                _ray->horizontalMinAngle.GetAsString() );
  horizontalNode->SetAttribute( _ray->horizontalMaxAngle.GetKey(),
                                _ray->horizontalMaxAngle.GetAsString() );

  TiXmlElement *verticalNode = new TiXmlElement("vertical");
  _parent->LinkEndChild( verticalNode );
  verticalNode->SetAttribute( _ray->verticalSamples.GetKey(),
                              _ray->verticalSamples.GetAsString() );
  verticalNode->SetAttribute( _ray->verticalResolution.GetKey(),
                              _ray->verticalResolution.GetAsString() );
  verticalNode->SetAttribute( _ray->verticalMinAngle.GetKey(),
                              _ray->verticalMinAngle.GetAsString() );
  verticalNode->SetAttribute( _ray->verticalMaxAngle.GetKey(),
                              _ray->verticalMaxAngle.GetAsString() );

  TiXmlElement *rangeNode = new TiXmlElement("range");
  _parent->LinkEndChild( rangeNode );
  rangeNode->SetAttribute( _ray->rangeMin.GetKey(),
                           _ray->rangeMin.GetAsString() );
  rangeNode->SetAttribute( _ray->rangeMax.GetKey(),
                           _ray->rangeMax.GetAsString() );
  rangeNode->SetAttribute( _ray->rangeResolution.GetKey(),
                           _ray->rangeResolution.GetAsString() );
  return true;
}

bool saveXml(TiXmlElement *_parent, const boost::shared_ptr<Inertial> &_inertial)
{
  TiXmlElement *inertialNode = new TiXmlElement("inertial");
  _parent->LinkEndChild( inertialNode );
  inertialNode->SetAttribute(_inertial->mass.GetKey(),
                             _inertial->mass.GetAsString());
  inertialNode->SetAttribute(_inertial->origin.GetKey(),
                             _inertial->origin.GetAsString());

  TiXmlElement *inertiaNode = new TiXmlElement("inertia");
  inertialNode->LinkEndChild(inertiaNode);
  inertiaNode->SetAttribute( _inertial->ixx.GetKey(),
                             _inertial->ixx.GetAsString() );
  inertiaNode->SetAttribute( _inertial->ixy.GetKey(),
                             _inertial->ixy.GetAsString() );
  inertiaNode->SetAttribute( _inertial->ixz.GetKey(),
                             _inertial->ixz.GetAsString() );
  inertiaNode->SetAttribute( _inertial->iyy.GetKey(),
                             _inertial->iyy.GetAsString() );
  inertiaNode->SetAttribute( _inertial->iyz.GetKey(),
                             _inertial->iyz.GetAsString() );
  inertiaNode->SetAttribute( _inertial->izz.GetKey(),
                             _inertial->izz.GetAsString() );

  return true;
}

bool saveXml(TiXmlElement *_parent, boost::shared_ptr<Surface> &_surface)
{
  TiXmlElement *surfaceNode = new TiXmlElement("surface");
  _parent->LinkEndChild( surfaceNode );

  TiXmlElement *bounceNode = new TiXmlElement("bounce");
  surfaceNode->LinkEndChild( bounceNode );
  bounceNode->SetAttribute( _surface->bounceRestCoeff.GetKey(),
                            _surface->bounceRestCoeff.GetAsString() );
  bounceNode->SetAttribute( _surface->bounceThreshold.GetKey(),
                            _surface->bounceThreshold.GetAsString() );

  if (_surface->frictions.size() > 0)
  {
    TiXmlElement *frictionNode = new TiXmlElement("friction");
    surfaceNode->LinkEndChild( frictionNode );

    std::vector< boost::shared_ptr<Friction> >::const_iterator fiter;
    for (fiter = _surface->frictions.begin(); fiter != _surface->frictions.end(); fiter++)
    {
      if ((*fiter)->type == Friction::ODE)
      {
        boost::shared_ptr<ODEFriction> odeFriction = boost::shared_static_cast<ODEFriction>(*fiter);
        saveXml(frictionNode, odeFriction);
      }
    }
  }

  if (_surface->contacts.size() > 0)
  {
    TiXmlElement *contactNode = new TiXmlElement("contact");
    surfaceNode->LinkEndChild( contactNode );

    std::vector< boost::shared_ptr<SurfaceContact> >::const_iterator citer;
    for (citer = _surface->contacts.begin(); citer != _surface->contacts.end(); citer++)
    {
      if ((*citer)->type == SurfaceContact::ODE)
      {
        boost::shared_ptr<ODESurfaceContact> odeContact = boost::shared_static_cast<ODESurfaceContact>(*citer);
        saveXml(contactNode, odeContact);
      }
    }
  }

  return true;
}

bool saveXml(TiXmlElement *_parent, boost::shared_ptr<ODEFriction> &_friction)
{
  TiXmlElement *odeNode = new TiXmlElement("ode");
  _parent->LinkEndChild( odeNode );

  odeNode->SetAttribute( _friction->mu.GetKey(),
                         _friction->mu.GetAsString() );
  odeNode->SetAttribute( _friction->mu2.GetKey(),
                         _friction->mu2.GetAsString() );
  odeNode->SetAttribute( _friction->fdir1.GetKey(),
                         _friction->fdir1.GetAsString() );
  odeNode->SetAttribute( _friction->slip1.GetKey(),
                         _friction->slip1.GetAsString() );
  odeNode->SetAttribute( _friction->slip2.GetKey(),
                         _friction->slip2.GetAsString() );
  return true;
}

bool saveXml(TiXmlElement *_parent, boost::shared_ptr<ODESurfaceContact> &_contact)
{
  TiXmlElement *odeNode = new TiXmlElement("ode");
  _parent->LinkEndChild( odeNode );

  odeNode->SetAttribute( _contact->softCFM.GetKey(),
                         _contact->softCFM.GetAsString() );
  odeNode->SetAttribute( _contact->kp.GetKey(),
                         _contact->kp.GetAsString() );
  odeNode->SetAttribute( _contact->kd.GetKey(),
                         _contact->kd.GetAsString() );
  odeNode->SetAttribute( _contact->maxVel.GetKey(),
                         _contact->maxVel.GetAsString() );
  odeNode->SetAttribute( _contact->minDepth.GetKey(),
                         _contact->minDepth.GetAsString() );

  return true;
}
*/

}
