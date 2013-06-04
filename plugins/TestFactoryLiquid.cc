#include "gazebo/gazebo.hh"
#include "physics/physics.hh"
#include "common/Plugin.hh"
#include "transport/transport.hh"
#include <math.h>

#define PI 3.14159265

namespace gazebo
{
  class FactoryLiquid : public WorldPlugin
  {
  	public: virtual ~FactoryLiquid()
  	{

  	}
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
    	math::Vector3 p3, init_pos;
    	std::stringstream xml;
    	int spawned, level;
    	unsigned int nr_spheres;
    	double mass, radius, mu, mu2, slip1, slip2, cfm, erp, kp, kd, bounce, inertia;

    	///////////////////////////////////////////////////////////////////////////////////
    	/////// SDF PARAMETERS

    	////////////// Get nr of spheres
        if (!_sdf->HasElement("nr_spheres"))
        {
      	  std::cout << "Missing parameter <nr_spheres> in FactoryLiquid, default to 0" << std::endl;
      	  nr_spheres = 0;
        }
        else nr_spheres = _sdf->GetElement("nr_spheres")->GetValueUInt();

    	////////////// Set up the initial position parameter
        if (!_sdf->HasElement("init_pos"))
        {
      	  std::cout << "Missing parameter <init_pos> in FactoryLiquid, default to 0 0 0" << std::endl;
      	  init_pos.x = 0.0;
      	  init_pos.y = 0.0;
      	  init_pos.z = 0.0;
        }
        else init_pos = _sdf->GetElement("init_pos")->GetValueVector3();

    	////////////// Set up liquid sphere mass
    	if (!_sdf->HasElement("mass"))
    	{
    		std::cout << "Missing parameter <mass> in FactoryLiquid, default to 0 0 0" << std::endl;
    		mass = 0.001;
    	}
    	else mass = _sdf->GetElement("mass")->GetValueDouble();

    	////////////// mu
        if (!_sdf->HasElement("mu"))
        {
      	  std::cout << "Missing parameter <mu> in FactoryLiquid, default to 0" << std::endl;
      	  mu = 0;
        }
        else mu = _sdf->GetElement("mu")->GetValueDouble();

    	////////////// mu2
        if (!_sdf->HasElement("mu2"))
        {
      	  std::cout << "Missing parameter <mu2> in FactoryLiquid, default to 0" << std::endl;
      	  mu2 = 0;
        }
        else mu2 = _sdf->GetElement("mu2")->GetValueDouble();

    	////////////// slip1
        if (!_sdf->HasElement("slip1"))
        {
      	  std::cout << "Missing parameter <slip1> in FactoryLiquid, default to 0" << std::endl;
      	slip1 = 0;
        }
        else slip1 = _sdf->GetElement("slip1")->GetValueDouble();

    	////////////// slip2
        if (!_sdf->HasElement("slip2"))
        {
      	  std::cout << "Missing parameter <slip2> in FactoryLiquid, default to 0" << std::endl;
      	slip2 = 0;
        }
        else slip2 = _sdf->GetElement("slip2")->GetValueDouble();

    	////////////// cfm
        if (!_sdf->HasElement("cfm"))
        {
      	  std::cout << "Missing parameter <cfm> in FactoryLiquid, default to 0" << std::endl;
      	  cfm = 0;
        }
        else cfm = _sdf->GetElement("cfm")->GetValueDouble();

    	////////////// erp
        if (!_sdf->HasElement("erp"))
        {
      	  std::cout << "Missing parameter <erp> in FactoryLiquid, default to 0" << std::endl;
      	  erp = 0;
        }
        else erp = _sdf->GetElement("erp")->GetValueDouble();

    	////////////// kp
        if (!_sdf->HasElement("kp"))
        {
      	  std::cout << "Missing parameter <kp> in FactoryLiquid, default to 0" << std::endl;
      	kp = 0;
        }
        else kp = _sdf->GetElement("kp")->GetValueDouble();

    	////////////// kd
        if (!_sdf->HasElement("kd"))
        {
      	  std::cout << "Missing parameter <kd> in FactoryLiquid, default to 0" << std::endl;
      	kd = 0;
        }
        else kd = _sdf->GetElement("kd")->GetValueDouble();

    	////////////// bounce
    	if (!_sdf->HasElement("bounce"))
    	{
    		std::cout << "Missing parameter <bounce> in FactoryLiquid, default to 0" << std::endl;
    		bounce = 0;
    	}
    	else bounce = _sdf->GetElement("bounce")->GetValueDouble();

    	////////////// inertia
        if (!_sdf->HasElement("inertia"))
        {
      	  std::cout << "Missing parameter <inertia> in FactoryLiquid, default to 0" << std::endl;
      	inertia = 0;
        }
        else inertia = _sdf->GetElement("inertia")->GetValueDouble();

        //print parameters
		std::printf("**Liquid parameters:"
				" \n mu: %f"
				" \n mu2: %f"
				" \n slip1: %f"
				" \n slip2: %f"
				" \n cfm: %f"
				" \n erp: %f"
				" \n kp: %f"
				" \n kd: %f \n",
				mu, mu2, slip1, slip2, cfm, erp, kp, kd);

        /////// END SDF PARAMETERS
    	///////////////////////////////////////////////////////////////////////////////////

    	level = 0;
    	spawned = 0;
    	radius = 0.005;

    	p3.x = 0.0;
    	p3.y = 0.0;
    	p3.z = 0.0;


    	///////////////////////////////////////////////////////////////////////////////////
    	///////////////////////////////////////////////////////////////////////////////////
    	//////////////////////////// START XML LIQUID
    	xml << "<?xml version='1.0'?>\n";
    	xml << "<sdf version='1.4'>\n";
    	xml << "<model name='liquid_spheres'>\n";
    	xml << "\t<static>false</static>\n";
    	xml << "\t<pose>" << init_pos.x << " " << init_pos.y << " " << init_pos.z << " 0 0 0 </pose>\n";

    	for (unsigned int i=0; i<nr_spheres; i++)
    	{
    		p3 = FactoryLiquid::part_position(i, radius, spawned, level);
    		xml << "\t\t<link name='sphere_link_" << i << "'>\n";
        	xml << "\t\t\t<self_collide>true</self_collide>\n";
    		xml << "\t\t\t<pose>" << p3.x << " " << p3.y << " " << p3.z << " 0 0 0</pose>\n";

    		xml << "\t\t\t<inertial>\n";
    		xml << "\t\t\t\t<pose> 0 0 0 0 0 0 </pose>\n";
    		xml << "\t\t\t\t<inertia>\n";
    	    xml << "\t\t\t\t\t<ixx>" << inertia << "</ixx>\n";
    	    xml << "\t\t\t\t\t<ixy>0</ixy>\n";
    	    xml << "\t\t\t\t\t<ixz>0</ixz>\n";
    	    xml << "\t\t\t\t\t<iyy>" << inertia << "</iyy>\n";
    	    xml << "\t\t\t\t\t<iyz>0</iyz>\n";
    	    xml << "\t\t\t\t\t<izz>" << inertia << "</izz>\n";
    		xml << "\t\t\t\t</inertia>\n";
    		xml << "\t\t\t\t<mass>" << mass << "</mass>\n";
    		xml << "\t\t\t</inertial>\n";

    		xml << "\t\t\t<collision name='collision_" << i << "'>\n";
    		xml << "\t\t\t\t<geometry>\n";
    		xml << "\t\t\t\t\t<sphere>\n";
    		xml << "\t\t\t\t\t\t<radius>" << radius << "</radius>\n";
    		xml << "\t\t\t\t\t</sphere>\n";
    		xml << "\t\t\t\t</geometry>\n";
    		xml << "\t\t\t\t<surface>\n";
    		xml << "\t\t\t\t\t<friction>\n";
    		xml << "\t\t\t\t\t\t<ode>\n";
    	    xml << "\t\t\t\t\t\t\t<mu>" << mu << "</mu>\n";
    	    xml << "\t\t\t\t\t\t\t<mu2>" << mu2 << "</mu2>\n";
    	    xml << "\t\t\t\t\t\t\t<fdir1>0.0 0.0 0.0</fdir1>\n";
    	    xml << "\t\t\t\t\t\t\t<slip1>" << slip1 << "</slip1>\n";
    	    xml << "\t\t\t\t\t\t\t<slip2>" << slip2 << "</slip2>\n";
    		xml << "\t\t\t\t\t\t</ode>\n";
    		xml << "\t\t\t\t\t</friction>\n";
    		xml << "\t\t\t\t\t<bounce>\n";
    		xml << "\t\t\t\t\t\t<restitution_coefficient>" << bounce << "</restitution_coefficient>\n";
    		xml << "\t\t\t\t\t\t<threshold>10000.0</threshold>\n";
    		xml << "\t\t\t\t\t</bounce>\n";
    		xml << "\t\t\t\t\t<contact>\n";
    		xml << "\t\t\t\t\t\t<ode>\n";
    	    xml << "\t\t\t\t\t\t\t<soft_cfm>" << cfm << "</soft_cfm>\n";
    	    xml << "\t\t\t\t\t\t\t<soft_erp>" << erp << "</soft_erp>\n";
    	    xml << "\t\t\t\t\t\t\t<kp>" << kp << "</kp>\n";
    	    xml << "\t\t\t\t\t\t\t<kd>" << kd << "</kd>\n";
    	    xml << "\t\t\t\t\t\t\t<max_vel>100.0</max_vel>\n";
    	    xml << "\t\t\t\t\t\t\t<min_depth>0.001</min_depth>\n";
    		xml << "\t\t\t\t\t\t</ode>\n";
            xml << "\t\t\t\t\t</contact>\n";
    		xml << "\t\t\t\t</surface>\n";
    		xml << "\t\t\t</collision>\n";

    		xml << "\t\t\t<visual name='sphere_visual_" << i << "'>\n";
    		xml << "\t\t\t\t<geometry>\n";
    		xml << "\t\t\t\t\t<sphere>\n";
    		xml << "\t\t\t\t\t\t<radius>" << radius << "</radius>\n";
    		xml << "\t\t\t\t\t</sphere>\n";
    		xml << "\t\t\t\t</geometry>\n";
    		xml << "\t\t\t\t<material>\n";
    		xml << "\t\t\t\t\t<script>\n";
    		xml << "\t\t\t\t\t\t<uri>file://media/materials/scripts/gazebo.material</uri>\n";
    		xml << "\t\t\t\t\t\t<name>Gazebo/Red</name>\n";
    		xml << "\t\t\t\t\t</script>\n";
    		xml << "\t\t\t\t</material>\n";
    		xml << "\t\t\t</visual>\n";
    		xml << "\t\t</link>\n";
    	}


    	/* plugin is a WorldPlugin now, it is loaded in the .world file  */
    	//xml << "<plugin name='dynamic_joint_plugin' filename='libdynamic_joint_plugin.so'/>\n";

		xml << "</model>\n";
		xml << "</gazebo>\n";

		/////////////////////////////////////////
    	//std::cout << xml.str() << "\n";

        sdf::SDF sphereSDF;
        sphereSDF.SetFromString(xml.str());


        _parent->InsertModelSDF(sphereSDF);

    }

    public: math::Vector3 part_position(int i, double radius, int& spawned, int& level)
    {
		math::Vector3 v3;
		int ii, index_c, c_crt, max_c_in_c;
		double size, R;
		ii = i - spawned;
		size = radius * 2;

		v3.z = level * size;
		if (ii != 0)
		{
			index_c = ii-1;
			c_crt = 1;

			while (index_c >= (6*c_crt))
			{
				index_c -= 6*c_crt;
				c_crt++;
			}
			max_c_in_c = c_crt * 6;
			R = c_crt * size;

			if((index_c == (max_c_in_c - 1)) && ((2*R) + (size) >= 0.025))
			{
				spawned = i+1;
				level++;
			}

			v3.x = R * cos((double) index_c * 2 * PI / max_c_in_c);
			v3.y = R * sin((double) index_c * 2 * PI / max_c_in_c);
		}

		return v3;
    }
};

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(FactoryLiquid)
}
