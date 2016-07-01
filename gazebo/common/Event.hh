/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_COMMON_EVENT_HH_
#define GAZEBO_COMMON_EVENT_HH_

#include <atomic>
#include <functional>
#include <list>
#include <map>
#include <memory>
#include <mutex>

#include <gazebo/gazebo_config.h>
#include <gazebo/common/Time.hh>
#include <gazebo/common/CommonTypes.hh>
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \ingroup gazebo_event
  /// \brief Event namespace
  namespace event
  {
    /// \addtogroup gazebo_event Events
    /// \{

    /// \class Event Event.hh common/common.hh
    /// \brief Base class for all events
    class GZ_COMMON_VISIBLE Event
    {
      /// \brief Constructor
      public: Event();

      /// \brief Destructor
      public: virtual ~Event();

      /// \brief Disconnect
      /// \param[in] _c A pointer to a connection
      /// \deprecated Use event::~Connection to disconnect
      public: virtual void Disconnect(ConnectionPtr _c)
              GAZEBO_DEPRECATED(8.0) = 0;

      /// \brief Disconnect
      /// \param[in] _id Integer ID of a connection
      public: virtual void Disconnect(int _id) = 0;

      /// \brief Get whether this event has been signaled.
      /// \return True if the event has been signaled.
      /// \deprecated See bool Signaled() const;
      public: bool GetSignaled() const GAZEBO_DEPRECATED(8.0);

      /// \brief Get whether this event has been signaled.
      /// \return True if the event has been signaled.
      public: bool Signaled() const;

      /// \brief Set whether this event has been signaled.
      /// \param[in] _sig True if the event has been signaled.
      public: void SetSignaled(const bool _sig);

      /// \brief True if the event has been signaled.
      private: bool signaled;
    };

    /// \brief A class that encapsulates a connection.
    class GZ_COMMON_VISIBLE Connection
    {
      /// \brief Constructor.
      /// \param[in] _e Event pointer to connect with.
      /// \param[in] _i Unique id.
      public: Connection(Event *_e, const int _i);

      /// \brief Destructor.
      public: ~Connection();

      /// \brief Get the id of this connection.
      /// \return The id of this connection.
      /// \deprecated See const Id() const;
      public: int GetId() const GAZEBO_DEPRECATED(8.0);

      /// \brief Get the id of this connection.
      /// \return The id of this connection.
      public: int Id() const;

      /// \brief the event for this connection
      private: Event *event = nullptr;

      /// \brief the id set in the constructor
      private: int id = -1;

      /// \brief set during the constructor
      private: common::Time creationTime;

      /// \brief Friend class.
      public: template<typename T> friend class EventT;
    };

    /// \brief A class for event processing.
    template<typename T>
    class EventT : public Event
    {
      /// \brief Constructor.
      public: EventT();

      /// \brief Destructor.
      public: virtual ~EventT();

      /// \brief Connect a callback to this event.
      /// \param[in] _subscriber Pointer to a callback function.
      /// \return A Connection object, which will automatically call
      /// Disconnect when it goes out of scope.
      public: ConnectionPtr Connect(const std::function<T> &_subscriber);

      /// \brief Disconnect a callback to this event.
      /// \param[in] _c The connection to disconnect.
      /// \deprecated Use event::~Connection to disconnect
      public: virtual void Disconnect(ConnectionPtr _c)
              GAZEBO_DEPRECATED(8.0);

      /// \brief Disconnect a callback to this event.
      /// \param[in] _id The id of the connection to disconnect.
      public: virtual void Disconnect(int _id);

      /// \brief Get the number of connections.
      /// \return Number of connection to this Event.
      public: unsigned int ConnectionCount() const;

      /// \brief Access the signal.
      public: void operator()()
              {this->Signal();}

      /// \brief Signal the event with one parameter.
      /// \param[in] _p the parameter.
      public: template< typename P >
              void operator()(const P &_p)
      {
        this->Signal(_p);
      }

      /// \brief Signal the event with two parameters.
      /// \param[in] _p1 the first parameter.
      /// \param[in] _p2 the second parameter.
      public: template< typename P1, typename P2 >
              void operator()(const P1 &_p1, const P2 &_p2)
      {
        this->Signal(_p1, _p2);
      }

      /// \brief Signal the event with three parameters.
      /// \param[in] _p1 the first parameter.
      /// \param[in] _p2 the second parameter.
      /// \param[in] _p3 the second parameter.
      public: template< typename P1, typename P2, typename P3 >
              void operator()(const P1 &_p1, const P2 &_p2, const P3 &_p3)
      {
        this->Signal(_p1, _p2, _p3);
      }

      /// \brief Signal the event with four parameters.
      /// \param[in] _p1 the first parameter.
      /// \param[in] _p2 the second parameter.
      /// \param[in] _p3 the second parameter.
      /// \param[in] _p4 the first parameter.
      public: template< typename P1, typename P2, typename P3, typename P4 >
              void operator()(const P1 &_p1, const P2 &_p2, const P3 &_p3,
                              const P4 &_p4)
      {
        this->Signal(_p1, _p2, _p3, _p4);
      }

      /// \brief Signal the event with five parameters.
      /// \param[in] _p1 the first parameter.
      /// \param[in] _p2 the second parameter.
      /// \param[in] _p3 the second parameter.
      /// \param[in] _p4 the first parameter.
      /// \param[in] _p5 the fift parameter.
      public: template< typename P1, typename P2, typename P3, typename P4,
                        typename P5 >
              void operator()(const P1 &_p1, const P2 &_p2, const P3 &_p3,
                              const P4 &_p4, const P5 &_p5)
      {
        this->Signal(_p1, _p2, _p3, _p4, _p5);
      }

      /// \brief Signal the event with six parameters.
      /// \param[in] _p1 the first parameter.
      /// \param[in] _p2 the second parameter.
      /// \param[in] _p3 the second parameter.
      /// \param[in] _p4 the first parameter.
      /// \param[in] _p5 the fift parameter.
      /// \param[in] _p6 the sixt parameter.
      public: template< typename P1, typename P2, typename P3, typename P4,
                        typename P5, typename P6 >
              void operator()(const P1 &_p1, const P2 &_p2, const P3 &_p3,
                              const P4 &_p4, const P5 &_p5, const P6 &_p6)
      {
        this->Signal(_p1, _p2, _p3, _p4, _p5, _p6);
      }

      /// \brief Signal the event with seven parameters.
      /// \param[in] _p1 the first parameter.
      /// \param[in] _p2 the second parameter.
      /// \param[in] _p3 the second parameter.
      /// \param[in] _p4 the first parameter.
      /// \param[in] _p5 the fifth parameter.
      /// \param[in] _p6 the sixth parameter.
      /// \param[in] _p7 the seventh parameter.
      public: template< typename P1, typename P2, typename P3, typename P4,
                        typename P5, typename P6, typename P7 >
              void operator()(const P1 &_p1, const P2 &_p2, const P3 &_p3,
                              const P4 &_p4, const P5 &_p5, const P6 &_p6,
                              const P7 &_p7)
      {
        this->Signal(_p1, _p2, _p3, _p4, _p5, _p6, _p7);
      }

      /// \brief Signal the event with eight parameters.
      /// \param[in] _p1 the first parameter.
      /// \param[in] _p2 the second parameter.
      /// \param[in] _p3 the second parameter.
      /// \param[in] _p4 the first parameter.
      /// \param[in] _p5 the fifth parameter.
      /// \param[in] _p6 the sixth parameter.
      /// \param[in] _p7 the seventh parameter.
      /// \param[in] _p8 the eighth parameter.
      public: template< typename P1, typename P2, typename P3, typename P4,
                        typename P5, typename P6, typename P7, typename P8 >
              void operator()(const P1 &_p1, const P2 &_p2, const P3 &_p3,
                              const P4 &_p4, const P5 &_p5, const P6 &_p6,
                              const P7 &_p7, const P8 &_p8)
      {
        this->Signal(_p1, _p2, _p3, _p4, _p5, _p6, _p7, _p8);
      }

      /// \brief Signal the event with nine parameters.
      /// \param[in] _p1 the first parameter.
      /// \param[in] _p2 the second parameter.
      /// \param[in] _p3 the second parameter.
      /// \param[in] _p4 the first parameter.
      /// \param[in] _p5 the fifth parameter.
      /// \param[in] _p6 the sixth parameter.
      /// \param[in] _p7 the seventh parameter.
      /// \param[in] _p8 the eighth parameter.
      /// \param[in] _p9 the ninth parameter.
      public: template< typename P1, typename P2, typename P3, typename P4,
                        typename P5, typename P6, typename P7, typename P8,
                        typename P9 >
              void operator()(const P1 &_p1, const P2 &_p2, const P3 &_p3,
                              const P4 &_p4, const P5 &_p5, const P6 &_p6,
                              const P7 &_p7, const P8 &_p8, const P9 &_p9)
      {
        this->Signal(_p1, _p2, _p3, _p4, _p5, _p6, _p7, _p8, _p9);
      }

      /// \brief Signal the event with ten parameters.
      /// \param[in] _p1 the first parameter.
      /// \param[in] _p2 the second parameter.
      /// \param[in] _p3 the second parameter.
      /// \param[in] _p4 the first parameter.
      /// \param[in] _p5 the fifth parameter.
      /// \param[in] _p6 the sixth parameter.
      /// \param[in] _p7 the seventh parameter.
      /// \param[in] _p8 the eighth parameter.
      /// \param[in] _p9 the ninth parameter.
      /// \param[in] _p10 the tenth parameter.
      public: template< typename P1, typename P2, typename P3, typename P4,
                        typename P5, typename P6, typename P7, typename P8,
                        typename P9, typename P10 >
              void operator()(const P1 &_p1, const P2 &_p2, const P3 &_p3,
                              const P4 &_p4, const P5 &_p5, const P6 &_p6,
                              const P7 &_p7, const P8 &_p8, const P9 &_p9,
                              const P10 &_p10)
      {
        this->Signal(_p1, _p2, _p3, _p4, _p5, _p6, _p7, _p8, _p9, _p10);
      }

      /// \brief Signal the event for all subscribers.
      public: void Signal()
      {
        this->Cleanup();

        this->SetSignaled(true);
        for (const auto &iter: this->connections)
        {
          if (iter.second->on)
            iter.second->callback();
        }
      }

      /// \brief Signal the event with one parameter.
      /// \param[in] _p parameter.
      public: template< typename P >
              void Signal(const P &_p)
      {
        this->Cleanup();

        this->SetSignaled(true);
        for (const auto &iter: this->connections)
        {
          if (iter.second->on)
            iter.second->callback(_p);
        }
      }

      /// \brief Signal the event with two parameter.
      /// \param[in] _p1 the first parameter.
      /// \param[in] _p2 the second parameter.
      public: template< typename P1, typename P2 >
              void Signal(const P1 &_p1, const P2 &_p2)
      {
        this->Cleanup();

        this->SetSignaled(true);
        for (const auto &iter: this->connections)
        {
          if (iter.second->on)
            iter.second->callback(_p1, _p2);
        }
      }

      /// \brief Signal the event with three parameter.
      /// \param[in] _p1 the first parameter.
      /// \param[in] _p2 the second parameter.
      /// \param[in] _p3 the second parameter.
      public: template< typename P1, typename P2, typename P3 >
              void Signal(const P1 &_p1, const P2 &_p2, const P3 &_p3)
      {
        this->Cleanup();

        this->SetSignaled(true);
        for (const auto &iter: this->connections)
        {
          if (iter.second->on)
            iter.second->callback(_p1, _p2, _p3);
        }
      }

      /// \brief Signal the event with four parameter.
      /// \param[in] _p1 the first parameter.
      /// \param[in] _p2 the second parameter.
      /// \param[in] _p3 the second parameter.
      /// \param[in] _p4 the first parameter.
      public: template<typename P1, typename P2, typename P3, typename P4>
              void Signal(const P1 &_p1, const P2 &_p2, const P3 &_p3,
                          const P4 &_p4)
      {
        this->Cleanup();

        this->SetSignaled(true);
        for (const auto &iter: this->connections)
        {
          if (iter.second->on)
            iter.second->callback(_p1, _p2, _p3, _p4);
        }
      }

      /// \brief Signal the event with five parameter.
      /// \param[in] _p1 the first parameter.
      /// \param[in] _p2 the second parameter.
      /// \param[in] _p3 the second parameter.
      /// \param[in] _p4 the first parameter.
      /// \param[in] _p5 the fifth parameter.
      public: template<typename P1, typename P2, typename P3, typename P4,
                       typename P5>
              void Signal(const P1 &_p1, const P2 &_p2, const P3 &_p3,
                          const P4 &_p4, const P5 &_p5)
      {
        this->Cleanup();

        this->SetSignaled(true);
        for (const auto &iter: this->connections)
        {
          if (iter.second->on)
            iter.second->callback(_p1, _p2, _p3, _p4, _p5);
        }
      }

      /// \brief Signal the event with six parameter.
      /// \param[in] _p1 the first parameter.
      /// \param[in] _p2 the second parameter.
      /// \param[in] _p3 the second parameter.
      /// \param[in] _p4 the first parameter.
      /// \param[in] _p5 the fifth parameter.
      /// \param[in] _p6 the sixth parameter.
      public: template<typename P1, typename P2, typename P3, typename P4,
                       typename P5, typename P6>
              void Signal(const P1 &_p1, const P2 &_p2, const P3 &_p3,
                  const P4 &_p4, const P5 &_p5, const P6 &_p6)
      {
        this->Cleanup();

        this->SetSignaled(true);
        for (const auto &iter: this->connections)
        {
          if (iter.second->on)
            iter.second->callback(_p1, _p2, _p3, _p4, _p5, _p6);
        }
      }

      /// \brief Signal the event with seven parameter.
      /// \param[in] _p1 the first parameter.
      /// \param[in] _p2 the second parameter.
      /// \param[in] _p3 the second parameter.
      /// \param[in] _p4 the first parameter.
      /// \param[in] _p5 the fifth parameter.
      /// \param[in] _p6 the sixth parameter.
      /// \param[in] _p7 the seventh parameter.
      public: template<typename P1, typename P2, typename P3, typename P4,
                       typename P5, typename P6, typename P7>
              void Signal(const P1 &_p1, const P2 &_p2, const P3 &_p3,
                  const P4 &_p4, const P5 &_p5, const P6 &_p6, const P7 &_p7)
      {
        this->Cleanup();

        this->SetSignaled(true);
        for (const auto &iter: this->connections)
        {
          if (iter.second->on)
            iter.second->callback(_p1, _p2, _p3, _p4, _p5, _p6, _p7);
        }
      }

      /// \brief Signal the event with eight parameter.
      /// \param[in] _p1 the first parameter.
      /// \param[in] _p2 the second parameter.
      /// \param[in] _p3 the second parameter.
      /// \param[in] _p4 the first parameter.
      /// \param[in] _p5 the fifth parameter.
      /// \param[in] _p6 the sixth parameter.
      /// \param[in] _p7 the seventh parameter.
      /// \param[in] _p8 the eighth parameter.
      public: template<typename P1, typename P2, typename P3, typename P4,
                       typename P5, typename P6, typename P7, typename P8>
              void Signal(const P1 &_p1, const P2 &_p2, const P3 &_p3,
                  const P4 &_p4, const P5 &_p5, const P6 &_p6, const P7 &_p7,
                  const P8 &_p8)
      {
        this->Cleanup();

        this->SetSignaled(true);
        for (const auto &iter: this->connections)
        {
          if (iter.second->on)
          {
            iter.second->callback(_p1, _p2, _p3, _p4, _p5, _p6, _p7, _p8);
          }
        }
      }

      /// \brief Signal the event with nine parameter.
      /// \param[in] _p1 the first parameter.
      /// \param[in] _p2 the second parameter.
      /// \param[in] _p3 the second parameter.
      /// \param[in] _p4 the first parameter.
      /// \param[in] _p5 the fifth parameter.
      /// \param[in] _p6 the sixth parameter.
      /// \param[in] _p7 the seventh parameter.
      /// \param[in] _p8 the eighth parameter.
      /// \param[in] _p9 the ninth parameter.
      public: template< typename P1, typename P2, typename P3, typename P4,
                        typename P5, typename P6, typename P7, typename P8,
                        typename P9 >
              void Signal(const P1 &_p1, const P2 &_p2, const P3 &_p3,
                  const P4 &_p4, const P5 &_p5, const P6 &_p6, const P7 &_p7,
                  const P8 &_p8, const P9 &_p9)
      {
        this->Cleanup();

        this->SetSignaled(true);
        for (const auto &iter: this->connections)
        {
          if (iter.second->on)
          {
            iter.second->callback(
                _p1, _p2, _p3, _p4, _p5, _p6, _p7, _p8, _p9);
          }
        }
      }

      /// \brief Signal the event with ten parameter.
      /// \param[in] _p1 the first parameter.
      /// \param[in] _p2 the second parameter.
      /// \param[in] _p3 the second parameter.
      /// \param[in] _p4 the first parameter.
      /// \param[in] _p5 the fifth parameter.
      /// \param[in] _p6 the sixth parameter.
      /// \param[in] _p7 the seventh parameter.
      /// \param[in] _p8 the eighth parameter.
      /// \param[in] _p9 the ninth parameter.
      /// \param[in] _p10 the tenth parameter.
      public: template< typename P1, typename P2, typename P3, typename P4,
                        typename P5, typename P6, typename P7, typename P8,
                        typename P9, typename P10 >
              void Signal(const P1 &_p1, const P2 &_p2, const P3 &_p3,
                  const P4 &_p4, const P5 &_p5, const P6 &_p6, const P7 &_p7,
                  const P8 &_p8, const P9 &_p9, const P10 &_p10)
      {
        this->Cleanup();

        this->SetSignaled(true);
        for (const auto &iter: this->connections)
        {
          if (iter.second->on)
          {
            iter.second->callback(
                _p1, _p2, _p3, _p4, _p5, _p6, _p7, _p8, _p9, _p10);
          }
        }
      }

      /// \internal
      /// \brief Removes queued connections.
      /// We assume that this function is called from a Signal function.
      private: void Cleanup();

      /// \brief A private helper class used in maintaining connections.
      private: class EventConnection
      {
        /// \brief Constructor
        public: EventConnection(const bool _on, std::function<T> _cb)
                : callback(_cb)
        {
          // Windows Visual Studio 2012 does not have atomic_bool constructor,
          // so we have to set "on" using operator=
          this->on = _on;
        }

        /// \brief On/off value for the event callback
        public: std::atomic_bool on;

        /// \brief Callback function
        public: std::function<T> callback;
      };

      /// \def EvtConnectionMap
      /// \brief Event Connection map typedef.
      typedef std::map<int, std::unique_ptr<EventConnection>> EvtConnectionMap;

      /// \brief Array of connection callbacks.
      private: EvtConnectionMap connections;

      /// \brief A thread lock.
      private: std::mutex mutex;

      /// \brief List of connections to remove
      private: std::list<typename EvtConnectionMap::const_iterator>
              connectionsToRemove;
    };

    /// \brief Constructor.
    template<typename T>
    EventT<T>::EventT()
    : Event()
    {
    }

    /// \brief Destructor. Deletes all the associated connections.
    template<typename T>
    EventT<T>::~EventT()
    {
      this->connections.clear();
    }

    /// \brief Adds a connection.
    /// \param[in] _subscriber the subscriber to connect.
    template<typename T>
    ConnectionPtr EventT<T>::Connect(const std::function<T> &_subscriber)
    {
      int index = 0;
      if (!this->connections.empty())
      {
        auto const &iter = this->connections.rbegin();
        index = iter->first + 1;
      }
      this->connections[index].reset(new EventConnection(true, _subscriber));
      return ConnectionPtr(new Connection(this, index));
    }

    /// \brief Removes a connection.
    /// \param[in] _c the connection.
    template<typename T>
    void EventT<T>::Disconnect(ConnectionPtr _c)
    {
      if (!_c)
        return;

      this->Disconnect(_c->Id());
      _c->event = nullptr;
      _c->id = -1;
    }

    /// \brief Get the number of connections.
    /// \return Number of connections.
    template<typename T>
    unsigned int EventT<T>::ConnectionCount() const
    {
      return this->connections.size();
    }

    /// \brief Removes a connection.
    /// \param[in] _id the connection index.
    template<typename T>
    void EventT<T>::Disconnect(int _id)
    {
      // Find the connection
      auto const &it = this->connections.find(_id);

      if (it != this->connections.end())
      {
        it->second->on = false;
        this->connectionsToRemove.push_back(it);
      }
    }

    /////////////////////////////////////////////
    template<typename T>
    void EventT<T>::Cleanup()
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      // Remove all queue connections.
      for (auto &conn : this->connectionsToRemove)
        this->connections.erase(conn);
      this->connectionsToRemove.clear();
    }
    /// \}
  }
}
#endif
