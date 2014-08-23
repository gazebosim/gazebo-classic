/* Desc: System plugin for rendering the particles from Fluidix
 * Author: Andrei Haidu
 */

#include "FluidVisPlugin.hh"

using namespace gazebo;


//////////////////////////////////////////////////
FluidVisPlugin::FluidVisPlugin()
{
}

//////////////////////////////////////////////////
FluidVisPlugin::~FluidVisPlugin()
{
}

//////////////////////////////////////////////////
void FluidVisPlugin::Load(int _argc, char ** _argv)
{
	// check for the given arguments
	for (unsigned int i = 0; i < _argc; ++i)
	{
		// look for '--type' characters
		if(std::string(_argv[i]) == "--type")
		{
			// set the next argument as the tyoe of visualization
			this->visualizationType = _argv[++i];
		}

	}

	// is no correct value given. set default value to visualization
	if (this->visualizationType != "sphere")
	{
		this->visualizationType = "point";
	}

	// output
	std::cout << "Visualization options: " << std::endl;
	std::cout << "   --type  \t\t Particle visualization type: point (default), sphere." << std::endl;
	std::cout << "Currently selected: " << this->visualizationType << std::endl;
}

//////////////////////////////////////////////////
void  FluidVisPlugin::Init()
{
	this->node = transport::NodePtr(new transport::Node());

	this->newFluidMsgReceived = false;

	// Event to check that the rendering engine is loaded
    this->updateConnection = event::Events::ConnectPreRender(
            boost::bind(&FluidVisPlugin::InitAtRenderEvent, this));
}

//////////////////////////////////////////////////
void FluidVisPlugin::InitAtRenderEvent()
{
	// Initialize node only after the rendering engine has been loaded
	this->node->Init();

	// subscribe to the fluid topic
	this->fluidSub = this->node->Subscribe("~/fluid_pos",
			&FluidVisPlugin::OnFluidMsg, this);

	// subscribe to the fluid obj topic
	this->fluidObjSub = this->node->Subscribe("~/fluid_obj_pos",
			&FluidVisPlugin::OnFluidObjMsg, this);

	this->manager = gui::get_active_camera()->GetScene()->GetManager();


	// Continue calling update for every rendering event
	if (this->visualizationType == "sphere")
	{
		this->updateConnection = event::Events::ConnectPreRender(
				boost::bind(&FluidVisPlugin::RenderAsSpheresUpdate, this));
	}
	else
	{
		this->updateConnection = event::Events::ConnectPreRender(
				boost::bind(&FluidVisPlugin::RenderAsPointsUpdate, this));
	}
}

/////////////////////////////////////////////////
void FluidVisPlugin::RenderAsPointsUpdate()
{
	// render fluid if new message received
	if (this->newFluidMsgReceived)
	{
//		std::cout << "OnUpdate: rendering fluid.." << std::endl;

		// render fluid particles
		FluidVisPlugin::RenderParticles(this->fluidParticlePositions, "fluid1");
		this->newFluidMsgReceived = false;
	}

	// render fluid obj if new message received
	if (this->newFluidObjMsgReceived)
	{
//		std::cout << "OnUpdate: rendering objects.." << std::endl;

		// render object particles
		FluidVisPlugin::RenderObjectsParticles(this->objNameToPos_M);

		this->newFluidObjMsgReceived = false;
	}
}

/////////////////////////////////////////////////
void FluidVisPlugin::RenderAsSpheresUpdate()
{
	// render fluid if new message received
	if (this->newFluidMsgReceived)
	{
//		std::cout << "OnUpdate: rendering fluid.." << std::endl;

		// render fluid particles
		FluidVisPlugin::RenderParticlesAsEntities(this->fluidParticlePositions, "fluid1");

		this->newFluidMsgReceived = false;
	}

	// render fluid obj if new message received
	if (this->newFluidObjMsgReceived)
	{
//		std::cout << "OnUpdate: rendering objects.." << std::endl;

		// render object particles
		FluidVisPlugin::RenderObjectsParticles(this->objNameToPos_M);

		this->newFluidObjMsgReceived = false;
	}
}

/////////////////////////////////////////////////
void FluidVisPlugin::OnFluidMsg(
		const boost::shared_ptr<msgs::Fluid const> &_msg)
{
//	std::cout << "OnFluidMsg: msg received.." << std::endl;

	this->newFluidMsgReceived = true;

	// if message name is fluid
	if(_msg->name() == "fluid1")
	{
		// resize position vector regarding to the msg size
		this->fluidParticlePositions.resize(_msg->position_size());

		for (int i = 0; i < _msg->position_size(); i++)
		{
			this->fluidParticlePositions[i] = Ogre::Vector3(
					_msg->position(i).x(),_msg->position(i).y(),_msg->position(i).z());
		}
	}
}

/////////////////////////////////////////////////
void FluidVisPlugin::OnFluidObjMsg(
		const boost::shared_ptr<msgs::Fluid const> &_msg)
{
//	std::cout << "OnFluidObjMsg: msg received.. with name:" << _msg->name() << std::endl;

	this->newFluidObjMsgReceived = true;

	// check if object is in the map
	if (this->objNameToPos_M.find(_msg->name()) != this->objNameToPos_M.end())
	{
		// resize position vector regarding to the msg size
		this->objNameToPos_M.at(_msg->name()).resize(_msg->position_size());

		for (int i = 0; i < _msg->position_size(); i++)
		{
			this->objNameToPos_M.at(_msg->name())[i] = Ogre::Vector3(
					_msg->position(i).x(),_msg->position(i).y(),_msg->position(i).z());
		}
	}
	else
	{
		// the object has not been mapped yet
		this->objNameToPos_M[_msg->name()] = std::vector<Ogre::Vector3>(_msg->position_size());

		for (int i = 0; i < _msg->position_size(); i++)
		{
			this->objNameToPos_M.at(_msg->name())[i] = Ogre::Vector3(
					_msg->position(i).x(),_msg->position(i).y(),_msg->position(i).z());
		}
	}
}

/////////////////////////////////////////////////
void FluidVisPlugin::RenderParticles(std::vector<Ogre::Vector3> &_particles, std::string _name)
{
	Ogre::SceneNode *sceneNode = NULL;
	Ogre::ManualObject *obj = NULL;
	bool attached = false;

	if (this->manager->hasManualObject(_name))
	{
		sceneNode = this->manager->getSceneNode(_name);
		obj = this->manager->getManualObject(_name);
		attached = true;
	}
	else
	{
		sceneNode = this->manager->getRootSceneNode()->createChildSceneNode(_name);
		obj = this->manager->createManualObject(_name);
	}

	sceneNode->setVisible(true);
	obj->setVisible(true);

	obj->clear();

	// OT_POINT_LIST = 1, A list of points, 1 vertex per point
    // OT_LINE_LIST = 2, A list of lines, 2 vertices per line
    // OT_LINE_STRIP = 3, A strip of connected lines, 1 vertex per line plus 1 start vertex
    // OT_TRIANGLE_LIST = 4, A list of triangles, 3 vertices per triangle
	// OT_TRIANGLE_STRIP = 5, A strip of triangles, 3 vertices for the first triangle, and 1 per triangle after that
	// OT_TRIANGLE_FAN = 6, A fan of triangles, 3 vertices for the first triangle, and 1 per triangle after that
	obj->begin("Gazebo/Red", Ogre::RenderOperation::OT_POINT_LIST);

	for (int i = 0; i < _particles.size(); i++)
	{
		obj->position(_particles[i]);
	}

	obj->end();

	if (!attached)
		sceneNode->attachObject(obj);
}

/////////////////////////////////////////////////
void FluidVisPlugin::RenderObjectsParticles(
		std::map<std::string, std::vector<Ogre::Vector3> > &_name_to_pos_map)
{
	// loop through all the objects
	for (std::map<std::string, std::vector<Ogre::Vector3> >::iterator map_iter= _name_to_pos_map.begin();
			map_iter != _name_to_pos_map.end(); ++map_iter)
	{
		Ogre::SceneNode *sceneNode = NULL;
		Ogre::ManualObject *obj = NULL;
		bool attached = false;

		if (this->manager->hasManualObject(map_iter->first))
		{
			sceneNode = this->manager->getSceneNode(map_iter->first);
			obj = this->manager->getManualObject(map_iter->first);
			attached = true;
		}
		else
		{
			sceneNode = this->manager->getRootSceneNode()->createChildSceneNode(map_iter->first);
			obj = this->manager->createManualObject(map_iter->first);
		}

		sceneNode->setVisible(true);
		obj->setVisible(true);

		obj->clear();

		// OT_POINT_LIST = 1, A list of points, 1 vertex per point
	    // OT_LINE_LIST = 2, A list of lines, 2 vertices per line
	    // OT_LINE_STRIP = 3, A strip of connected lines, 1 vertex per line plus 1 start vertex
	    // OT_TRIANGLE_LIST = 4, A list of triangles, 3 vertices per triangle
		// OT_TRIANGLE_STRIP = 5, A strip of triangles, 3 vertices for the first triangle, and 1 per triangle after that
		// OT_TRIANGLE_FAN = 6, A fan of triangles, 3 vertices for the first triangle, and 1 per triangle after that
		obj->begin("Gazebo/Red", Ogre::RenderOperation::OT_POINT_LIST);

		for (int i = 0; i < map_iter->second.size(); i++)
		{
			obj->position(map_iter->second[i]);
		}

		obj->end();

		if (!attached)
			sceneNode->attachObject(obj);
	}
}

/////////////////////////////////////////////////
void FluidVisPlugin::RenderParticlesAsEntities(std::vector<Ogre::Vector3> &_particles, std::string _name)
{
	//std::cout << "OnUpdate: Rendering new positions.." << std::endl;
	for (int i = 0; i < _particles.size(); i++)
	{
		Ogre::SceneNode *sceneNode = NULL;
		Ogre::Entity *entity = NULL;
		bool attached = false;
		std::ostringstream name_ss;
		std::string name;

		name_ss << _name << "_" << i;
		name = name_ss.str();


		if(this->manager->hasEntity(name))
		{
			sceneNode = this->manager->getSceneNode(name);
			entity = this->manager->getEntity(name);
			attached = true;
		}
		else
		{
			sceneNode = this->manager->getRootSceneNode()->createChildSceneNode(name);
			entity = this->manager->createEntity(name,
					Ogre::SceneManager::PT_SPHERE);
			entity->setMaterialName("Gazebo/Red");
		}

		sceneNode->setVisible(true);
		entity->setVisible(true);

		sceneNode->setScale(0.0001, 0.0001, 0.0001);

		sceneNode->setPosition(_particles[i]);

		if (!attached)
		{
			sceneNode->attachObject(entity);
		}
	}
}

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(FluidVisPlugin)
