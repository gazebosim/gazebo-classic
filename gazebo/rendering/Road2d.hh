/*
 * Copyright 2011 Nate Koenig
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

namespace gazebo
{
  namespace rendering
  {
    class Road2d
    {
      /// \brief Constructor
      public: Road2d();

      /// \brief Destructor
      public: virtual ~Road2d();

      public: void Load(VisualPtr _parent);

      /// \brief Process all received messages
      private: void PreRender();


      /// \brief Recieve a road msg
      private: void OnRoadMsg(ConstRoadPtr &_msg);

      private: class Segment : public Ogre::SimpleRenderable
               {
                 public: void Load(msgs::Road _msg);

                 /// \brief Implementation of Ogre::SimpleRenderable
                 public: virtual Ogre::Real getBoundingRadius(void) const;

                 /// \brief Implementation of Ogre::SimpleRenderable
                 public: virtual Ogre::Real getSquaredViewDepth(
                             const Ogre::Camera* cam) const;

                 public: std::string name;
                 public: std::vector<math::Vector3> points;
                 public: double width;
               };


      typedef std::list<boost::shared_ptr<msgs::Road const> > RoadMsgs_L;
      private: RoadMsgs_L msgs;
      private: std::vector<Road2d::Segment*> segments;
      private: VisualPtr parent;

      private: transport::NodePtr node;
      private: transport::SubscriberPtr sub;

      private: std::vector<event::ConnectionPtr> connections;
    };
  }
}
#endif
