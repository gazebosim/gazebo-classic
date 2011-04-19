/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#ifndef GAZEBO_TRANSPORT_HH
#define GAZEBO_TRANSPORT_HH

#include <boost/bind.hpp>

#include "transport/TransportTypes.hh"
#include "transport/SubscribeOptions.hh"
#include "transport/TopicManager.hh"

namespace gazebo
{
  namespace transport
  {

    /// Get the hostname and port of the master from the GAZEBO_MASTER_URI
    /// environment variable
    /// \param master_host The hostname of the master is set to this param
    /// \param master_port The port of the master is set to this param
    /// \return False if the GAZEBO_MASTER_URI was not found
    bool get_master_uri(std::string &master_host, unsigned short &master_port);

    /// \brief Initialize the transport system
    /// \param master_host The hostname or IP of the master. Leave empty to
    ///                    use pull address from the GAZEBO_MASTER_URI env var.
    /// \param master_port The port  of the master. Leave empty to
    ///                    use pull address from the GAZEBO_MASTER_URI env var.
    void init(const std::string &master_host="", unsigned short master_port=0);

    /// \brief Set the global topic namespace
    void set_topic_namespace(const std::string &space);

    /// \brief Adverise a topic
    template<typename M>
    transport::PublisherPtr advertise(const std::string &topic)
    {
      return transport::TopicManager::Instance()->Advertise<M>(topic);
    }

    /// \brief Subscribe to a topic, and return data on the callback
    template<typename M, typename T>
    transport::SubscriberPtr subscribe(const std::string &topic,
        void(T::*fp)(const boost::shared_ptr<M const> &), T *obj)
    {
      SubscribeOptions ops;
      ops.template Init<M>(topic, boost::bind(fp, obj, _1));
      return transport::TopicManager::Instance()->Subscribe(ops);
    }

    /// \brief Subscribe to a topic, and return data on the callback
    template<typename M>
    transport::SubscriberPtr subscribe(const std::string &topic,
        void(*fp)(const boost::shared_ptr<M const> &))
    {
      SubscribeOptions ops;
      ops.template Init<M>(topic, fp);
      return transport::TopicManager::Instance()->Subscribe(ops);
    }

  }
}

#endif
