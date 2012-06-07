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

#ifndef __EVENT_HH__
#define __EVENT_HH__

#include <iostream>
#include <vector>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include "common/Time.hh"
#include "common/CommonTypes.hh"
#include "gazebo_config.h"

namespace gazebo
{
  /// \ingroup gazebo_event
  /// \brief Event namespace
  namespace event
  {
    /// \addtogroup gazebo_event Events
    /// \{
    /// \brief Base class for all events
    class Event
    {
      /// \brief Constructor
      public: virtual ~Event() {}

      /// \brief Disconnect 
      /// \param _c A pointer to a connection
      public: virtual void Disconnect(ConnectionPtr _c) = 0;

      /// \brief Disconnect 
      /// \param _id Integer ID of a connection
      public: virtual void Disconnect(int _id) = 0;
    };

    /// \brief A class that encapsulates a connection
    class Connection
    {
      /// \brief Constructor
      public: Connection() :event(NULL), id(-1) {}

      /// \brief Constructor
      /// \param _e Event pointer to connect with
      /// \param _i Unique id
      public: Connection(Event *_e, int _i);

      /// \brief Destructor
      public: ~Connection();

      /// \brief Get the id of this connection
      /// \return The id of this connection
      public: int GetId() const;

      private: Event *event;
      private: int id;

      private: static int counter;

      private: common::Time creationTime;
      public: template<typename T> friend class EventT;
    };

    /// \brief An class for event processing
    template< typename T>
    class EventT : public Event
    {
      /// \brief Destructor
      public: virtual ~EventT();

      /// \brief Connect a callback to this event
      /// \param _subscriber Pointer to a callback function
      /// \return A Connection object, which will automatically call
      ///         Disconnect when it goes out of scope
      public: ConnectionPtr Connect(const boost::function<T> &_subscriber);

      /// \brief Disconnect a callback to this event
      /// \param _c The connection to disconnect
      public: virtual void Disconnect(ConnectionPtr _c);

      /// \brief Disconnect a callback to this event
      /// \param _id The id of the connection to disconnect
      public: virtual void Disconnect(int _id);

      /// \brief Access the signal
      public: void operator()()
              {this->Signal();}

      /// \brief Signal the event
      public: void Signal()
              {
                for (unsigned int i = 0; i < connections.size(); i++)
                {
                  (*this->connections[i])();
                }
              }

      /// \brief Signal the event with one parameter
      public: template< typename P >
              void operator()(const P &p)
              { this->Signal(p); }

      /// \brief Signal the event with two parameters
      public: template< typename P1, typename P2 >
              void operator()(const P1 &p1, const P2 &p2)
              { this->Signal(p1, p2); }

      /// \brief Signal the event with three parameters
      public: template< typename P1, typename P2, typename P3 >
              void operator()(const P1 &p1, const P2 &p2, const P3 &p3)
              { this->Signal(p1, p2, p3); }

      /// \brief Signal the event with four parameters
      public: template< typename P1, typename P2, typename P3, typename P4 >
              void operator()(const P1 &p1, const P2 &p2, const P3 &p3,
                              const P4 &p4)
              { this->Signal(p1, p2, p3, p4); }

      /// \brief Signal the event with five parameters
      public: template< typename P1, typename P2, typename P3, typename P4,
                        typename P5 >
              void operator()(const P1 &p1, const P2 &p2, const P3 &p3,
                              const P4 &p4, const P5 &p5)
              { this->Signal(p1, p2, p3, p4, p5); }

      /// \brief Signal the event with six parameters
      public: template< typename P1, typename P2, typename P3, typename P4,
                        typename P5, typename P6 >
              void operator()(const P1 &p1, const P2 &p2, const P3 &p3,
                              const P4 &p4, const P5 &p5, const P6 &p6)
              { this->Signal(p1, p2, p3, p4, p5, p6); }

      /// \brief Signal the event with seven parameters
      public: template< typename P1, typename P2, typename P3, typename P4,
                        typename P5, typename P6, typename P7 >
              void operator()(const P1 &p1, const P2 &p2, const P3 &p3,
                              const P4 &p4, const P5 &p5, const P6 &p6,
                              const P7 &p7)
              { this->Signal(p1, p2, p3, p4, p5, p6, p7); }

      /// \brief Signal the event with eight parameters
      public: template< typename P1, typename P2, typename P3, typename P4,
                        typename P5, typename P6, typename P7, typename P8 >
              void operator()(const P1 &p1, const P2 &p2, const P3 &p3,
                              const P4 &p4, const P5 &p5, const P6 &p6,
                              const P7 &p7, const P8 &p8)
              { this->Signal(p1, p2, p3, p4, p5, p6, p7, p8); }

      /// \brief Signal the event with nine parameters
      public: template< typename P1, typename P2, typename P3, typename P4,
                        typename P5, typename P6, typename P7, typename P8,
                        typename P9 >
              void operator()(const P1 &p1, const P2 &p2, const P3 &p3,
                              const P4 &p4, const P5 &p5, const P6 &p6,
                              const P7 &p7, const P8 &p8, const P9 &p9)
              { this->Signal(p1, p2, p3, p4, p5, p6, p7, p8, p9); }

      /// \brief Signal the event with ten parameters
      public: template< typename P1, typename P2, typename P3, typename P4,
                        typename P5, typename P6, typename P7, typename P8,
                        typename P9, typename P10 >
              void operator()(const P1 &p1, const P2 &p2, const P3 &p3,
                              const P4 &p4, const P5 &p5, const P6 &p6,
                              const P7 &p7, const P8 &p8, const P9 &p9,
                              const P10 &p10)
              { this->Signal(p1, p2, p3, p4, p5, p6, p7, p8, p9, p10); }

      /// \brief Signal the event with one parameter
      public: template< typename P >
              void Signal(const P &p)
              {
                for (unsigned int i = 0; i < connections.size(); i++)
                {
                  (*this->connections[i])(p);
                }
              }

      /// \brief Signal the event with two parameter
      public: template< typename P1, typename P2 >
              void Signal(const P1 &p1, const P2 &p2)
              {
                for (unsigned int i = 0; i < connections.size(); i++)
                {
                  (*this->connections[i])(p1, p2);
                }
              }

      /// \brief Signal the event with three parameter
      public: template< typename P1, typename P2, typename P3 >
              void Signal(const P1 &p1, const P2 &p2, const P3 &p3)
              {
                for (unsigned int i = 0; i < connections.size(); i++)
                {
                  (*this->connections[i])(p1, p2, p3);
                }
              }

      /// \brief Signal the event with four parameter
      public: template<typename P1, typename P2, typename P3, typename P4>
              void Signal(const P1 &p1, const P2 &p2, const P3 &p3,
                          const P4 &p4)
              {
                for (unsigned int i = 0; i < connections.size(); i++)
                {
                  (*this->connections[i])(p1, p2, p3, p4);
                }
              }

      /// \brief Signal the event with five parameter
      public: template<typename P1, typename P2, typename P3, typename P4,
                       typename P5>
              void Signal(const P1 &p1, const P2 &p2, const P3 &p3,
                          const P4 &p4, const P5 &p5)
              {
                for (unsigned int i = 0; i < connections.size(); i++)
                {
                  (*this->connections[i])(p1, p2, p3, p4, p5);
                }
              }


      /// \brief Signal the event with six parameter
      public: template<typename P1, typename P2, typename P3, typename P4,
                       typename P5, typename P6>
              void Signal(const P1 &p1, const P2 &p2, const P3 &p3,
                  const P4 &p4, const P5 &p5, const P6 &p6)
              {
                for (unsigned int i = 0; i < connections.size(); i++)
                {
                  (*this->connections[i])(p1, p2, p3, p4, p5, p6);
                }
              }

      /// \brief Signal the event with seven parameter
      public: template<typename P1, typename P2, typename P3, typename P4,
                       typename P5, typename P6, typename P7>
              void Signal(const P1 &p1, const P2 &p2, const P3 &p3,
                  const P4 &p4, const P5 &p5, const P6 &p6, const P7 &p7)
              {
                for (unsigned int i = 0; i < connections.size(); i++)
                {
                  (*this->connections[i])(p1, p2, p3, p4, p5, p6, p7);
                }
              }

      /// \brief Signal the event with eight parameter
      public: template<typename P1, typename P2, typename P3, typename P4,
                       typename P5, typename P6, typename P7, typename P8>
              void Signal(const P1 &p1, const P2 &p2, const P3 &p3,
                  const P4 &p4, const P5 &p5, const P6 &p6, const P7 &p7,
                  const P8 &p8)
              {
                for (unsigned int i = 0; i < connections.size(); i++)
                {
                  (*this->connections[i])(p1, p2, p3, p4, p5, p6, p7, p8);
                }
              }

      /// \brief Signal the event with nine parameter
      public: template< typename P1, typename P2, typename P3, typename P4,
                        typename P5, typename P6, typename P7, typename P8,
                        typename P9 >
              void Signal(const P1 &p1, const P2 &p2, const P3 &p3,
                  const P4 &p4, const P5 &p5, const P6 &p6, const P7 &p7,
                  const P8 &p8, const P9 &p9)
              {
                for (unsigned int i = 0; i < connections.size(); i++)
                {
                  (*this->connections[i])(p1, p2, p3, p4, p5, p6, p7, p8, p9);
                }
              }

      /// \brief Signal the event with ten parameter
      public: template< typename P1, typename P2, typename P3, typename P4,
                        typename P5, typename P6, typename P7, typename P8,
                        typename P9, typename P10 >
              void Signal(const P1 &p1, const P2 &p2, const P3 &p3,
                  const P4 &p4, const P5 &p5, const P6 &p6, const P7 &p7,
                  const P8 &p8, const P9 &p9, const P10 &p10)
              {
                for (unsigned int i = 0; i < connections.size(); i++)
                {
                  (*this->connections[i])(p1, p2, p3, p4, p5,
                      p6, p7, p8, p9, p10);
                }
              }

      private: std::vector<boost::function<T> *> connections;
      private: std::vector<int> connectionIds;
      private: boost::mutex lock;
    };

    template<typename T>
    EventT<T>::~EventT()
    {
      for (unsigned int i = 0; i < this->connections.size(); i++)
        delete this->connections[i];
      this->connections.clear();
      this->connectionIds.clear();
    }

    template<typename T>
    ConnectionPtr EventT<T>::Connect(const boost::function<T> &_subscriber)
    {
      int index = this->connections.size();
      this->connections.push_back(new boost::function<T>(_subscriber));
      this->connectionIds.push_back(index);
      return ConnectionPtr(new Connection(this, index));
    }

    template<typename T>
    void EventT<T>::Disconnect(ConnectionPtr c)
    {
      this->Disconnect(c->GetId());
      c->event = NULL;
      c->id = -1;
    }

    template<typename T>
    void EventT<T>::Disconnect(int _id)
    {
      // search for index of the connection based on id
      for (unsigned int i = 0; i < this->connectionIds.size(); i++)
      {
        if (_id == this->connectionIds[i])
        {
          this->connectionIds.erase(this->connectionIds.begin()+i);
          this->connections.erase(this->connections.begin()+i);
          break;
        }
      }
    }
    /// \}
  }
}
#endif
