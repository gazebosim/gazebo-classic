#include <iostream>
#include <math.h>
#include <boost/shared_ptr.hpp>
#include <sdf/sdf.hh>
#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_dynamic_io.hpp>

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"
#include "collision_map_request.pb.h"
#include "collision_map_grid.h"


namespace gazebo
{
typedef const boost::shared_ptr<const collision_map_creator_msgs::msgs::CollisionMapRequest> CollisionMapRequestPtr;
class CollisionMapCreator : public WorldPlugin
{
    transport::NodePtr node;
    transport::PublisherPtr imagePub;
    transport::SubscriberPtr commandSubscriber;
    physics::WorldPtr world;

public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
        node = transport::NodePtr(new transport::Node());
        world = _parent;
        // Initialize the node with the world name
        node->Init(world->GetName());
        std::cout << "Subscribing to: " << "~/collision_map/command" << std::endl;
        commandSubscriber = node->Subscribe("~/collision_map/command", &CollisionMapCreator::create, this);
        imagePub = node->Advertise<msgs::Image>("~/collision_map/image");
    }

public: void create(CollisionMapRequestPtr &msg)
    {
        std::cout << "Received message" << std::endl;

        std::cout << "Creating collision map with corners at (" <<
                     msg->upperleft().x() << ", " << msg->upperleft().y() << "), (" <<
                     msg->upperright().x() << ", " << msg->upperright().y() << "), (" <<
                     msg->lowerright().x() << ", " << msg->lowerright().y() << "), (" <<
                     msg->lowerleft().x() << ", " << msg->lowerleft().y() << ") with collision projected from z = " <<
                     msg->height() << "\nResolution = " << msg->resolution() << " m\n" <<
                     "Occupied spaces will be filled with: " << msg->threshold() << std::endl;
        // double z = msg->height();
        double dX_vertical = msg->upperleft().x() - msg->lowerleft().x();
        double dY_vertical = msg->upperleft().y() - msg->lowerleft().y();
        double mag_vertical = sqrt(dX_vertical * dX_vertical + dY_vertical * dY_vertical);
        dX_vertical = msg->resolution() * dX_vertical / mag_vertical;
        dY_vertical = msg->resolution() * dY_vertical / mag_vertical;

        // double step_s = msg->resolution();

        double dX_horizontal = msg->upperright().x() - msg->upperleft().x();
        double dY_horizontal = msg->upperright().y() - msg->upperleft().y();
        double mag_horizontal = sqrt(dX_horizontal * dX_horizontal + dY_horizontal * dY_horizontal);
        dX_horizontal = msg->resolution() * dX_horizontal / mag_horizontal;
        dY_horizontal = msg->resolution() * dY_horizontal / mag_horizontal;



        int count_vertical = mag_vertical / msg->resolution();
        int count_horizontal = mag_horizontal / msg->resolution();

        if (count_vertical == 0 || count_horizontal == 0)
        {
            std::cout << "Image has a zero dimensions, check coordinates" << std::endl;
            return;
        }
        double x,y;

        double dist;
        std::string entityName;
        math::Vector3 start, end;
        start.z = msg->height();
        end.z = 0.001;

        gazebo::physics::PhysicsEnginePtr engine = world->GetPhysicsEngine();
        engine->InitForThread();
        gazebo::physics::RayShapePtr ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
              engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

        std::cout << "Rasterizing model and checking collisions" << std::endl;
        CollisionMapGrid grid = CollisionMapGrid(
              msg->upperleft().x(), count_horizontal, msg->upperleft().y(), count_vertical);

        for (int i = 0; i < count_vertical; ++i)
        {
            std::cout << "Percent complete: " << i * 100.0 / count_vertical << std::endl;
            x = i * dX_vertical + msg->lowerleft().x();
            y = i * dY_vertical + msg->lowerleft().y();
            for (int j = 0; j < count_horizontal; ++j)
            {
                x += dX_horizontal;
                y += dY_horizontal;

                start.x = end.x= x;
                start.y = end.y = y;
                ray->SetPoints(start, end);
                ray->GetIntersection(dist, entityName);
                if (!entityName.empty())
                {
                    grid.fill(i, j);
                }
            }
        }

        std::cout << "Completed calculations." << std::endl;
        if (!msg->filename().empty())
	{
	  grid.print(msg->filename());
	}
    }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(CollisionMapCreator)
}
