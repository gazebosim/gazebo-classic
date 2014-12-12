/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#ifndef _INREGIONEVENTSOURCE_HH_
#define _INREGIONEVENTSOURCE_HH_

#include <map>
#include <string>
#include <vector>

#include "EventSource.hh"

namespace gazebo
{
  /// \brief A Volume base class with an inclusion query PointInVoume.
  ///  The default implementation is a simple axis aligned bounding box.
  class Volume
  {
    /// \brief dtor
    public: virtual ~Volume();

    /// \brief Checks if a point lies inside the box
    /// \param[in] _p point
    public: virtual bool PointInVolume(const math::Vector3 &_p) const;

    /// \brief The volume extent
    public: math::Vector3 min, max;
  };

  typedef boost::shared_ptr<Volume> VolumePtr;

  /// \brief A region, made of a list of volumes
  class Region
  {
    /// \brief Load from a world file (inside a SimEvent plugin element)
    /// \param[in] _sdf the region element
    public: void Load(const sdf::ElementPtr &_sdf);

    /// \brief Checks if a point lies inside the region
    /// \param[in] _p point
    public: bool PointInRegion(const math::Vector3 &p) const;

    /// \brief name of the region (as defined in the world file)
    public: std::string name;

    /// \brief The list of volumes inside this region
    public: std::vector<VolumePtr> volumes;
  };

  typedef boost::shared_ptr<Region> RegionPtr;

  /// \brief convenience function to print a region to the console
  /// \param[in] _out the output stream
  /// \param[in] _region the instance to write out
  std::ostream& operator << (std::ostream &_out, const Region &_region);

  /// \brief The event generator class
  class  InRegionEventSource: public EventSource
  {
    /// \brief Constructor
    /// \param[in] _pub the publisher for the SimEvents
    /// \param[in] _world Pointer to the world.
    /// \param[in] _regions dictionary of regions in the world
    public: InRegionEventSource(transport::PublisherPtr _pub,
                                physics::WorldPtr _world,
                                const std::map<std::string, RegionPtr>
                                                              &_regions);

    /// \brief Initialize the event
    public: virtual void Init();

    /// \brief Called every simulation step
    public: void Update();

    /// \brief Loads the full name of the model and the region from the world
    /// file.
    /// \param[in] _sdf
    public: virtual void Load(const sdf::ElementPtr &_sdf);

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief The model used for the in region check.
    private: std::string modelName;

    /// \brief A pointer to the model
    /// looked up at initialization, to avoid doing lookups during updates
    private: physics::ModelPtr model;

    /// \brief The region used for the in region check.
    private: std::string regionName;

    /// \brief The region pointer
    private: RegionPtr region;

    /// \brief A map of region names to region pointers.
    private: const std::map<std::string, RegionPtr> &regions;

    /// \brief true if the model is currently inside the region
    private: bool isInside;
  };
}


#endif
