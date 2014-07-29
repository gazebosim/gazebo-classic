/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef _ROAD2D_HH_
#define _ROAD2D_HH_

#include <string>
#include <vector>
#include <list>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/transport/TransportTypes.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Spline.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class Road Road.hh rendering/rendering.hh
    /// \brief Used to render a strip of road.
    class GAZEBO_VISIBLE Road2d
    {
      /// \brief Constructor
      public: Road2d();

      /// \brief Destructor
      public: virtual ~Road2d();

      /// \brief Load the visual using a parent visual.
      /// \param[in] _parent Pointer to the parent visual.
      public: void Load(VisualPtr _parent);

      /// \brief Process all received messages
      private: void PreRender();

      /// \brief Recieve a road msg
      private: void OnRoadMsg(ConstRoadPtr &_msg);

      /// \brief A road segment
      private: class Segment : public Ogre::SimpleRenderable
               {
                 /// \brief Load the road segment from message data.
                 /// \param[in] _msg The robot data.
                 public: void Load(msgs::Road _msg);


                 /// \internal
                 /// \brief Implementation of Ogre::SimpleRenderable
                 public: virtual Ogre::Real getBoundingRadius() const;

                 /// \internal
                 /// \brief Implementation of Ogre::SimpleRenderable
                 public: virtual Ogre::Real getSquaredViewDepth(
                             const Ogre::Camera* cam) const;

                 /// \brief Name of the road.
                 public: std::string name;

                 /// \brief Point that make up the middle of the road.
                 public: std::vector<math::Vector3> points;

                 /// \brief Width of the road.
                 public: double width;

                 /// \brief Texture of the road
                 public: std::string texture;
               };

      /// \def RoadMsgs_L
      /// \brief List of road messages
      typedef std::list<boost::shared_ptr<msgs::Road const> > RoadMsgs_L;

      /// \brief List of messages to process.
      private: RoadMsgs_L msgs;

      /// \brief All the road segments.
      private: std::vector<Road2d::Segment*> segments;

      /// \brief The parent visual.
      private: VisualPtr parent;

      /// \brief Handles communication.
      private: transport::NodePtr node;

      /// \brief Subscribes to the road message topic.
      private: transport::SubscriberPtr sub;

      /// \brief All the event connections.
      private: std::vector<event::ConnectionPtr> connections;
    };
    /// \}
  }
}
#endif
