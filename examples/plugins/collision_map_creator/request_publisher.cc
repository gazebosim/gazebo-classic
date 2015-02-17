#include <iostream>
#include <math.h>
#include <deque>
#include <sdf/sdf.hh>

#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/msgs/msgs.hh"

#include "collision_map_request.pb.h"
#include "vector2d.pb.h"

using namespace std;

bool createVectorArray(const char * vectorString,
                       deque<gazebo::msgs::Vector2d*> corners)
{
    deque<gazebo::msgs::Vector2d*>::iterator it;

    string cornersStr = vectorString;
    size_t opening=0;
    size_t closing=0;
    for (it = corners.begin(); it != corners.end(); ++it)
    {
        opening = cornersStr.find('(', closing);
        closing = cornersStr.find(')', opening);
        if (opening == string::npos || closing == string::npos)
        {
            std::cout << "Poorly formed string: " << cornersStr << std::endl;
            std::cout << "( found at: " << opening << " ) found at: " << closing << std::endl;
            return false;
        }
        string oneCornerStr = cornersStr.substr(opening + 1, closing - opening - 1);
        size_t commaLoc = oneCornerStr.find(",");
        string x = oneCornerStr.substr(0,commaLoc);
        string y = oneCornerStr.substr(commaLoc + 1, oneCornerStr.length() - commaLoc);
        (*it)->set_x(atof(x.c_str()));
        (*it)->set_y(atof(y.c_str()));
    }
    return true;
}

int main(int argc, char * argv[])
{
    if (argc > 4)
    {
        collision_map_creator_msgs::msgs::CollisionMapRequest request;
        deque<gazebo::msgs::Vector2d*> corners;

        corners.push_back(request.mutable_upperleft());
        corners.push_back(request.mutable_upperright());
        corners.push_back(request.mutable_lowerright());
        corners.push_back(request.mutable_lowerleft());

        if (!createVectorArray(argv[1],corners))
        {
            return -1;
        }

        request.set_height(atof(argv[2]));
        request.set_resolution(atof(argv[3]));
        request.set_filename(argv[4]);

        if (argc == 6)
        {
            request.set_threshold(atoi(argv[6]));
        }

        gazebo::transport::init();
        gazebo::transport::run();
        gazebo::transport::NodePtr node(new gazebo::transport::Node());
        node->Init("default");

        std::cout << "Request: " <<
                     " UL.x: " << request.upperleft().x() <<
                     " UL.y: " << request.upperleft().y() <<
                     " UR.x: " << request.upperright().x() <<
                     " UR.y: " << request.upperright().y() <<
                     " LR.x: " << request.lowerright().x() <<
                     " LR.y: " << request.lowerright().y() <<
                     " LL.x: " << request.lowerleft().x() <<
                     " LL.y: " << request.lowerleft().y() <<
                     " Height: " << request.height() <<
                     " Resolution: " << request.resolution() <<
                     " Filename: " << request.filename() <<
                     " Threshold: " << request.threshold() << std::endl;

        gazebo::transport::PublisherPtr imagePub =
                node->Advertise<collision_map_creator_msgs::msgs::CollisionMapRequest>(
                                                            "~/collision_map/command");
        imagePub->WaitForConnection();
        imagePub->Publish(request);

	std::cout << "Request received!\n\n";

        gazebo::transport::fini();
        return 0;
    }
    return -1;
}
